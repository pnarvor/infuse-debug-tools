#include "OdometryExtractor.hpp"

using namespace std;

namespace Infuse
{

OdometryExtractor::OdometryExtractor(const boost::filesystem::path& bagPrefixPath, const std::string& inputPoseTopic, const std::string& outputDeltaTopic, const std::string& outputAttitudeTopic, const std::string& outputPath, const std::list<std::string>& topicsToCopy) :
    inputPoseTopic_(inputPoseTopic),
    outputDeltaTopic_(outputDeltaTopic),
    outputAttitudeTopic_(outputAttitudeTopic),
    outputPath_(outputPath),
    outputDelta_(0),
    outputAttitude_(0)
{
    this->findBagPaths(bagPrefixPath);
}

void OdometryExtractor::extract()
{
    this->computeOutputData();
    this->writeBags();
}

// static member functions
void OdometryExtractor::findBagPaths(const boost::filesystem::path& bagsPrefixPath)
{
    std::string bagsPrefix = bagsPrefixPath.filename().string();
    std::string bagsPath = bagsPrefixPath.parent_path().string();

    if(!boost::filesystem::is_directory(bagsPath))
        throw invalid_argument("Coud not retrieve valid bag folder");

    std::vector<boost::filesystem::path> v;
    for(auto&& x : boost::filesystem::directory_iterator(bagsPath))
        v.push_back(x.path());
    std::sort(v.begin(), v.end());

    bagsFilenames_.clear();
    for(auto&& x : v)
    {
        if(boost::algorithm::starts_with(x.filename().string(), bagsPrefix))
            bagsFilenames_.push_back(x.filename().string());
    }

    if(bagsFilenames_.size() == 0)
        throw invalid_argument("No bag files found at specified location.");

    bagsRootPath_ = bagsPath;
}

void OdometryExtractor::computeOutputData()
{
    outputDelta_.resize(bagsFilenames_.size());
    outputAttitude_.resize(bagsFilenames_.size());
    currentOutputDelta_ = outputDelta_.begin();
    currentOutputAttitude_ = outputAttitude_.begin();
    gotFirstPose_ = false;
    timeSpentMoving_ = 0.0;

    rosbag::Bag bagIn;
    for(auto name = bagsFilenames_.begin(); name != bagsFilenames_.end(); name++)
    {
        cout << "Reading " << bagsRootPath_ + "/" + *name << "... ";
        bagIn.open(bagsRootPath_ + "/" + *name);
        rosbag::View view(bagIn, rosbag::TopicQuery(inputPoseTopic_));

        infuse_msgs::asn1_bitstream::ConstPtr m;
        for(auto it = view.begin(); it != view.end(); it++)
        {
            m = it->instantiate<infuse_msgs::asn1_bitstream>();
            if(!m)
                throw runtime_error("Error reading rosbag : asn1_bitstream instantiation error.");

            this->processMessage(*m);
        }

        currentOutputDelta_++;
        currentOutputAttitude_++;

        cout << "Done !" << endl;
        bagIn.close();
    }
}

void OdometryExtractor::createOutputDir() const
{
    std::string path(bagsRootPath_ + "/" + outputPath_);
    if(boost::filesystem::exists(path))
    {
        if(!boost::filesystem::is_directory(path))
            throw runtime_error(std::string("The path \"") + path + "\" already exists and is not a directory.");
    }
    else
        boost::filesystem::create_directory(bagsRootPath_ + "/" + outputPath_);

    // Empty directory ?
}

void OdometryExtractor::writeBags() const
{
    rosbag::Bag bagOut;
    
    this->createOutputDir();
    for(int i = 0; i < bagsFilenames_.size(); i++)
    {
        int count = 0;
        bagOut.open(bagsRootPath_ + "/" + outputPath_ + "/" + bagsFilenames_[i], rosbag::bagmode::Write);
        for(auto msg = outputAttitude_[i].begin(); msg != outputAttitude_[i].end(); msg++)
        {
            bagOut.write(outputAttitudeTopic_, msg->first, msg->second);
            count++;
        }
        for(auto msg = outputDelta_[i].begin(); msg != outputDelta_[i].end(); msg++)
        {
            bagOut.write(outputDeltaTopic_, msg->first, msg->second);
            count++;
        }
        bagOut.close();
        cout << count << " messages written in " << outputPath_ + "/" + bagsFilenames_[i] << endl;
    }
}

PositionManager::Pose OdometryExtractor::decodePose(const infuse_msgs::asn1_bitstream& bstream)
{
    PositionManager::Pose res;
    asn1SccTransformWithCovariance asnPose;
    uint8_t buf[asn1SccTransformWithCovariance_REQUIRED_BYTES_FOR_ENCODING];
    BitStream asnBitStream;

    memcpy(buf, bstream.data.data(), bstream.data.size());
    BitStream_AttachBuffer(&asnBitStream, buf, bstream.data.size());

    int errorCode;
    if(!asn1SccTransformWithCovariance_Decode(&asnPose, &asnBitStream, &errorCode))
        throw runtime_error(std::string("Could not decode asn pose, error code : ") + std::to_string(errorCode) + ".");

    fromASN1SCC(asnPose, res);

    return res;
}

infuse_msgs::asn1_bitstream OdometryExtractor::encodeDeltaPose(const PositionManager::Pose pose)
{
    infuse_msgs::asn1_bitstream res;
    res.type = std::string("TransformWithCovariance");

    asn1SccTransformWithCovariance asnPose;
    uint8_t buf[asn1SccTransformWithCovariance_REQUIRED_BYTES_FOR_ENCODING];
    BitStream asnBitStream;

    toASN1SCC(pose, asnPose);
    asnPose.metadata.dataEstimated.arr[0] = 1;
    asnPose.metadata.dataEstimated.arr[1] = 1;
    asnPose.metadata.dataEstimated.arr[2] = 1;
    asnPose.metadata.dataEstimated.arr[3] = 0;
    asnPose.metadata.dataEstimated.arr[4] = 0;
    asnPose.metadata.dataEstimated.arr[5] = 0;
    asnPose.metadata.dataEstimated.arr[6] = 0;

    BitStream_Init(&asnBitStream, buf, asn1SccTransformWithCovariance_REQUIRED_BYTES_FOR_ENCODING);

    int errorCode;
    if(!asn1SccTransformWithCovariance_Encode(&asnPose, &asnBitStream, &errorCode, TRUE))
        throw runtime_error(std::string("Could not encode asn pose, error code : ") + std::to_string(errorCode) + ".");

    res.data.assign(buf, buf + asnBitStream.count);

    return res;
}

infuse_msgs::asn1_bitstream OdometryExtractor::encodeAttitude(const PositionManager::Pose pose)
{
    infuse_msgs::asn1_bitstream res;
    res.type = std::string("TransformWithCovariance");

    asn1SccTransformWithCovariance asnPose;
    uint8_t buf[asn1SccTransformWithCovariance_REQUIRED_BYTES_FOR_ENCODING];
    BitStream asnBitStream;

    toASN1SCC(pose, asnPose);
    asnPose.metadata.dataEstimated.arr[0] = 0;
    asnPose.metadata.dataEstimated.arr[1] = 0;
    asnPose.metadata.dataEstimated.arr[2] = 0;
    asnPose.metadata.dataEstimated.arr[3] = 1;
    asnPose.metadata.dataEstimated.arr[4] = 1;
    asnPose.metadata.dataEstimated.arr[5] = 1;
    asnPose.metadata.dataEstimated.arr[6] = 1;

    BitStream_Init(&asnBitStream, buf, asn1SccTransformWithCovariance_REQUIRED_BYTES_FOR_ENCODING);

    int errorCode;
    if(!asn1SccTransformWithCovariance_Encode(&asnPose, &asnBitStream, &errorCode, TRUE))
        throw runtime_error(std::string("Could not encode asn pose, error code : ") + std::to_string(errorCode) + ".");

    res.data.assign(buf, buf + asnBitStream.count);

    return res;
}

void OdometryExtractor::processMessage(const infuse_msgs::asn1_bitstream& message)
{
    infuse_msgs::asn1_bitstream outputMsg;
    
    PositionManager::Pose newPose = OdometryExtractor::decodePose(message);

    // computing absolute attitude
    PositionManager::Pose attitude(newPose);
    attitude._tr.transform.translation = base::Vector3d::Zero();
    attitude._tr.transform.cov = base::Matrix6d::Zero();
    attitude._tr.transform.cov(0,0) = 0.03;                      // 10 degrees standard deviation
    attitude._tr.transform.cov(1,1) = 0.03;                      // 10 degrees standard deviation
    attitude._tr.transform.cov(2,2) = 2.35e-11 * timeSpentMoving_*timeSpentMoving_; // 1 degree deviation each hour
    
    outputMsg = OdometryExtractor::encodeAttitude(attitude);
    outputMsg.header = message.header;
    currentOutputAttitude_->insert(std::pair<ros::Time, infuse_msgs::asn1_bitstream>(outputMsg.header.stamp, outputMsg));
    
    if(!gotFirstPose_)
    {
        lastPose_ = newPose;
        gotFirstPose_ = true;
        return;
    }
    
    // computing delta odometry
    PositionManager::Pose dp(newPose._child, newPose._child);
    dp._parentTime = lastPose_._childTime;
    dp._childTime = newPose._childTime;

    base::Matrix3d rot0 = lastPose_._tr.transform.orientation.toRotationMatrix();
    dp._tr.transform.translation = rot0.inverse()*(newPose._tr.transform.translation - lastPose_._tr.transform.translation);
    dp._tr.transform.orientation = lastPose_._tr.transform.orientation.inverse()*newPose._tr.transform.orientation;
    lastPose_ = newPose;

    double dt = (dp._childTime - dp._parentTime) / 1000000.0;
    double v = dp._tr.transform.translation.norm() / dt;
    Eigen::AngleAxis<double> angleAxis(dp._tr.transform.orientation);
    double w = angleAxis.angle() / dt;

    angleAxis.angle() *= 0.5;
    base::Matrix3d rotRK2 = base::Quaterniond(angleAxis).toRotationMatrix();

    if(fabs(v) > 0.01 || fabs(w) > 0.017)
        timeSpentMoving_ += dt;

    base::Matrix3d Sig0, Sig;
    Sig0 = base::Matrix3d::Zero();
    Sig0(0,0) = (0.1*v)*(0.1*v);
    Sig = rotRK2*Sig0*rotRK2.transpose();

    dp._tr.transform.cov = base::Matrix6d::Zero();
    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            dp._tr.transform.cov(i,j) = Sig(i,j);
        }
    }

    // exporting delta
    outputMsg = OdometryExtractor::encodeDeltaPose(dp);
    outputMsg.header = message.header;
    currentOutputDelta_->insert(std::pair<ros::Time, infuse_msgs::asn1_bitstream>(outputMsg.header.stamp, outputMsg));
    
    //cout << "Time spend moving : " << timeSpentMoving_ << endl;
}

};

