#ifndef DEF_INFUSE_ODOMETRY_EXTRACTOR_HPP
#define DEF_INFUSE_ODOMETRY_EXTRACTOR_HPP

#include <iostream>
#include <exception>
#include <algorithm>

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

    OdometryExtractor(const boost::filesystem::path& bagPrefixPath, const std::string& inputPoseTopic, const std::string& outputDeltaTopic, const std::string& outputAttitudeTopic, const std::string& outputPath = "delta", const std::list<std::string>& topicsToCopy = std::list<std::string>());
    void extract();
    void computeOutputData(); // Fill outputData (generate delta poses to be saved in output bags
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
