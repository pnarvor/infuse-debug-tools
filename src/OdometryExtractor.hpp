#ifndef DEF_INFUSE_ODOMETRY_EXTRACTOR_HPP
#define DEF_INFUSE_ODOMETRY_EXTRACTOR_HPP

#include <iostream>
#include <exception>
#include <algorithm>
#include <random>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string/predicate.hpp>

#include <infuse_msgs/asn1_bitstream.h>
#include <infuse_asn1_types/TransformWithCovariance.h>
#include <infuse_asn1_conversions/asn1_base_conversions.hpp>
#include <infuse_asn1_conversions/asn1_pom_conversions.hpp>

namespace Infuse
{

class Filter1
{
    protected:

    double G_;
    double alpha_;
    double last_;

    public:

    Filter1(double fcn) :
        G_(fcn / (fcn + 2.0)),
        alpha_((fcn - 2.0) / (fcn + 2.0)),
        last_(0.0)
    {}

    void apply(std::vector<double>& v)
    {
        last_ = 0.0;
        for(double& value : v)
        {
            double tmp = value;
            value = G_*value - alpha_*last_;
            last_ = tmp;
        }
    }
};

class NoiseGenerator
{
    public:

    bool noiseLess_;
    std::normal_distribution<double> dist_;
    Filter1 filter_;
    std::default_random_engine generator_;

    public:

    NoiseGenerator(double stddev, double fc = 1.0) :
        filter_(fc)
    {
        if(stddev <= 0)
        {
            noiseLess_ = true;
            dist_ = std::normal_distribution<double>(0.0, 0.0);
        }
        else
        {
            noiseLess_ = false;
            dist_ = std::normal_distribution<double>(0.0, stddev);
        }
    }

    void generate(std::vector<double>& noise, int N)
    {
        noise.resize(N);
        if(noiseLess_)
        {
            for(double& value : noise)
                value = 0.0;
            return;
        }

        for(double& value : noise)
            value = dist_(generator_);
        //filter_.apply(noise);
    }

    double get()
    {
        if(noiseLess_)
            return 0.0;
        return dist_(generator_);
    }
};

class OdometryExtractor
{
    protected:

    typedef std::map<ros::Time, infuse_msgs::asn1_bitstream> TopicMessages;

    std::string inputPoseTopic_;
    std::string outputDeltaTopic_;
    std::string outputAttitudeTopic_;

    std::string outputPath_;
    std::list<std::string> topicsToCopy_;

    std::string bagsRootPath_;
    std::vector<std::string> bagsFilenames_;

    std::vector<TopicMessages> outputDelta_; // each elements contains all messages of a single ouput bag
    std::vector<TopicMessages>::iterator currentOutputDelta_;
    std::vector<TopicMessages> outputAttitude_; // each elements contains all messages of a single ouput bag
    std::vector<TopicMessages>::iterator currentOutputAttitude_;

    // Memory for pose processing
    PositionManager::Pose lastPose_;
    bool gotFirstPose_;
    double timeSpentMoving_;

    public:

    NoiseGenerator noiseGenDelta_;
    NoiseGenerator noiseGenAttitude_;

    public:

    OdometryExtractor(const boost::filesystem::path& bagPrefixPath, const std::string& inputPoseTopic, const std::string& outputDeltaTopic, const std::string& outputAttitudeTopic, const std::string& outputPath = "delta", const std::list<std::string>& topicsToCopy = std::list<std::string>());
    void extract();
    void computeOutputData(); // Fill outputData (generate delta poses to be saved in output bags
    void addNoise();
    void writeBags() const;

    protected:

    void findBagPaths(const boost::filesystem::path& bagPrefixPath);
    void createOutputDir() const;

    // static functions
    protected:

    static PositionManager::Pose decodePose(const infuse_msgs::asn1_bitstream& bstream, std::string& producerId);
    static infuse_msgs::asn1_bitstream encodeDeltaPose(const PositionManager::Pose pose, std::string& producerId);
    static infuse_msgs::asn1_bitstream encodeAttitude(const PositionManager::Pose pose, std::string& producerId);

    static PositionManager::Pose computeDeltaPose(const PositionManager::Pose& p0, const PositionManager::Pose& p1);

    void processMessage(const infuse_msgs::asn1_bitstream& bstream);
};

};

#endif //DEF_INFUSE_ODOMETRY_EXTRACTOR_HPP
