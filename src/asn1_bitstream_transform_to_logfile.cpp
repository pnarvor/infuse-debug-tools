#include <map>
#include <memory>
#include <fstream>

#include <ros/ros.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <infuse_msgs/asn1_bitstream.h>
#include <infuse_asn1_types/TransformWithCovariance.h>

#include <infuse_debug_tools/LogTopic.h>

#include "asn1_bitstream_transform_processer.hpp"
#include "asn1_bitstream_logger.hpp"


namespace infuse_debug_tools
{

class ASN1BitstreamTransformToLogfile
{
public:
  ASN1BitstreamTransformToLogfile();
  ~ASN1BitstreamTransformToLogfile();

  bool log_topic(infuse_debug_tools::LogTopic::Request  &req,
                 infuse_debug_tools::LogTopic::Response &res);

  void pose_callback(const infuse_msgs::asn1_bitstream::Ptr& msg, std::ofstream& ofs);

private:
  bool register_log_callback(const std::string &topic, const std::string &logfile);

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::ServiceServer connect_pose_srv_;
  std::map<std::string,ros::Subscriber> sub_map_;
  std::map<std::string,std::ofstream> ofs_map_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
};


ASN1BitstreamTransformToLogfile::ASN1BitstreamTransformToLogfile() :
  private_nh_{"~"},
  connect_pose_srv_{private_nh_.advertiseService("log_topic", &ASN1BitstreamTransformToLogfile::log_topic, this)}
{
  std::string topics_str;
  std::string logfiles_str;
  if (private_nh_.getParam("topics", topics_str) and private_nh_.getParam("logfiles", logfiles_str)) {
    // Split strings into a vectors
    std::vector<std::string> topics;
    {
      std::stringstream ss(topics_str);
      std::istream_iterator<std::string> begin(ss), eos; // end-of-stream
      topics.assign(begin, eos);
    }
    std::vector<std::string> logfiles;
    {
      std::stringstream ss(logfiles_str);
      std::istream_iterator<std::string> begin(ss), eos; // end-of-stream
      logfiles.assign(begin, eos);
    }
    if (topics.size() != logfiles.size()) {
      ROS_ERROR_STREAM("You should inform the same number of topics and logfiles by ROS parameters! (#topics = " << topics.size() << ", #logfiles = " << logfiles.size() << ")");
    } else {
      for(auto t_it = topics.begin(), l_it = logfiles.begin();
          t_it != topics.end(), l_it != logfiles.end();
          t_it++, l_it++) {
        try {
          register_log_callback(*t_it, *l_it);
        } catch (...) {
          ROS_INFO_STREAM("ERROR: Could not log to topic " << *t_it << " to file " << *l_it);
        }          
      }
    }
  }
}

ASN1BitstreamTransformToLogfile::~ASN1BitstreamTransformToLogfile()
{
  for (auto & x : ofs_map_)
    x.second.close();
}

bool ASN1BitstreamTransformToLogfile::register_log_callback(const std::string &topic, const std::string &logfile)
{
  ofs_map_[topic] = std::ofstream(logfile);
  // Write header
  std::vector<std::string> entries{ASN1BitstreamLogger::GetTransformWithCovarianceLogEntries()};
  unsigned int index = 1;
  for (auto entry : entries) {
    ofs_map_[topic] << "# " << index << " - " << entry << '\n';
    index++;  
  }
  // ofs_map_[topic] << "#parent_time child_time x y z qw qx qy qz q_norm roll pitch yaw" << '\n';
  boost::function<void (const infuse_msgs::asn1_bitstream::Ptr&)> callback = 
    boost::bind(&ASN1BitstreamTransformToLogfile::pose_callback, this, _1, boost::ref(ofs_map_[topic]));
  // Create subscriber
  sub_map_[topic] = nh_.subscribe<infuse_msgs::asn1_bitstream>(topic, 100, callback);
}

bool ASN1BitstreamTransformToLogfile::log_topic(infuse_debug_tools::LogTopic::Request  &req,
                                                infuse_debug_tools::LogTopic::Response &res)
{
  try {
    register_log_callback(req.topic, req.logfile);
    res.success = true;
    return true;
  } catch (...) {
    std::stringstream ss;
    ss << "Unknown exception when registering log callback for topic " << req.topic << " to file " << req.logfile;
    ROS_INFO_STREAM(ss.str());
    res.success = false;
    res.message = ss.str();
    return false;
  }
}

void ASN1BitstreamTransformToLogfile::pose_callback(const infuse_msgs::asn1_bitstream::Ptr& msg, std::ofstream& ofs)
{
  // Initialize
  asn1SccTransformWithCovariance asn1Transform;
  asn1SccTransformWithCovariance_Initialize(&asn1Transform);

  // Decode
  flag res;
  int errorCode;
  BitStream bstream;
  BitStream_AttachBuffer(&bstream, msg->data.data(), msg->data.size());
  res = asn1SccTransformWithCovariance_Decode(&asn1Transform, &bstream, &errorCode);
  if (not res) {
    ROS_INFO("Error decoding asn1SccTransformWithCovariance! Error: %d", errorCode);
    return;
  }

  ASN1BitstreamLogger::LogTransformWithCovariance(asn1Transform, ofs);

}

} // namespace infuse_debug_tools

int main(int argc, char **argv)
{
  ros::init(argc, argv, "asn1_bitstream_transform_to_logfile");

  infuse_debug_tools::ASN1BitstreamTransformToLogfile asn1_bitstream_transform_to_logfile;

  ros::spin();

  return 0;
}