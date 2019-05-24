#ifndef IMAGEPAIRMATCHER_H
#define IMAGEPAIRMATCHER_H

#include <string>
#include <vector>
#include <memory>
#include <fstream>
#include <cmath>

#include <infuse_msgs/asn1_bitstream.h>
#include <infuse_asn1_types/Frame.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <infuse_asn1_conversions/asn1_base_conversions.hpp>
#include <infuse_asn1_conversions/asn1_pcl_conversions.hpp>

#if WITH_XIMGPROC
#include <opencv2/ximgproc.hpp>
#endif

#include <boost/filesystem.hpp>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

namespace infuse_debug_tools {

class ImagePairMatcher {

public:
  typedef pcl::PointXYZI Point;
  typedef pcl::PointCloud<Point> PointCloud;
  typedef pcl::PointXYZRGB ColoredPoint;
  typedef pcl::PointCloud<ColoredPoint> ColoredPointCloud;
  typedef pcl::visualization::PointCloudColorHandler<Point> ColorHandler;
  typedef ColorHandler::Ptr ColorHandlerPtr;
  typedef ColorHandler::ConstPtr ColorHandlerConstPtr;

  enum class ColorMode {
    kBlueToRed,
    kGreenToMagenta,
    kWhiteToRed,
    kGrayOrRed,
    kRainbow
  };

public:

  struct stereoBMParams
  {
      /**
       * @brief Pre-Filter Type
       * 0 = PREFILTER_NORMALIZED_RESPONSE
       * 1 = PREFILTER_XSOBEL
       */
      int pre_filter_type;

      /**
       * @brief Size of the Pre-Filter
       * It must be an odd number >= 5 & <= 255
       */
      int pre_filter_size;

      /**
       * @brief Texture Threshold
       */
      int texture_threshold;
  };

  struct stereoSGBMParams
  {
      /**
       * @brief The first parameter controlling the disparity smoothness.
       */
      int P1;

      /**
       * @brief The second parameter controlling the disparity smoothness.
       * The larger the values are, the smoother the disparity is.
       * P1 is the penalty on the disparity change by plus or minus 1 between neighbor pixels.
       * P2 is the penalty on the disparity change by more than 1 between neighbor pixels.
       * The algorithm requires P2 > P1 .
       * See stereo_match.cpp sample where some reasonably good P1 and P2 values are shown
       * (like 8*number_of_image_channels*blockSize*blockSize
       * and 32*number_of_image_channels*blockSize*blockSize , respectively)
       */
      int P2;

      /**
       * @brief Algorithm mode.
       * 0 = MODE_SGBM
       * 1 = MODE_HH
       * 2 = MODE_SGBM_3WAY
       * 3 = MODE_HH4
       * Set it to StereoSGBM::MODE_HH to run the full-scale two-pass dynamic programming algorithm.
       * It will consume O(W*H*numDisparities) bytes, which is large for 640x480 stereo and huge for HD-size pictures.
       */
      int mode;
  };

  struct stereoMatcherParams
  {
      /**
       * @brief Algorithm to be used
       * 0 = Block Matching (BM)
       * 1 = Semi Global Block Matching (SGBM)
       */
      int algorithm;

      /**
       * @brief Minimum possible disparity value.
       * Normally, it is zero but sometimes rectification algorithms can shift images, so this parameter needs to be adjusted accordingly.
       */
      int min_disparity;

      /**
       * @brief Maximum disparity minus minimum disparity.
       * The value is always greater than zero.
       * In the current implementation, this parameter must be divisible by 16.
       */
      int num_disparities;

      /**
       * @brief Matched block size. It must be an odd number >=1 .
       * Normally, it should be somewhere in the 3..11 range.
       */
      int block_size;

      /**
       * @brief Maximum size of smooth disparity regions to consider their noise speckles and invalidate.
       * Set it to 0 to disable speckle filtering. Otherwise, set it somewhere in the 50-200 range.
       */
      int speckle_window_size;

      /**
       * @brief Maximum disparity variation within each connected component.
       * If you do speckle filtering, set the parameter to a positive value, it will be implicitly multiplied by 16.
       * Normally, 1 or 2 is good enough.
       */
      int speckle_range;

      /**
       * @brief Maximum allowed difference (in integer pixel units) in the left-right disparity check.
       * Set it to a non-positive value to disable the check.
       */
      int disp12_max_diff;

      /**
       * @brief Truncation value for the prefiltered image pixels.
       * The algorithm first computes x-derivative at each pixel and clips its value by [-preFilterCap, preFilterCap] interval.
       * The result values are passed to the Birchfield-Tomasi pixel cost function.
       */
      int pre_filter_cap;

      /**
       * @brief Margin in percentage by which the best (minimum) computed cost function value should "win" the second best value to consider the found match correct.
       * Normally, a value within the 5-15 range is good enough.
       */
      int uniqueness_ratio;

      stereoBMParams bm_params;
      stereoSGBMParams sgbm_params;
  };

  struct filterParams
  {
      /**
       * @brief Set to true to filter the disparity map
       */
      bool use_filter;

      /**
       * @brief Filtering with confidence requires two disparity maps (for the left and right views) and is approximately two times slower.
       * However, quality is typically significantly better.
       */
      bool use_confidence;

      /** @brief DepthDiscontinuityRadius is a parameter used in confidence computation.
       * It defines the size of low-confidence regions around depth discontinuities.
       */
      int depth_discontinuity_radius;

      /** @brief Lambda is a parameter defining the amount of regularization during filtering.
       * Larger values force filtered disparity map edges to adhere more to source image edges.
       * Typical value is 8000.
       */
      double lambda;

      /** @brief LRCthresh is a threshold of disparity difference used in left-right-consistency check during
       * confidence map computation. The default value of 24 (1.5 pixels) is virtually always good enough.
       */
      int lrc_thresh;

      /** @brief SigmaColor is a parameter defining how sensitive the filtering process is to source image edges.
       * Large values can lead to disparity leakage through low-contrast edges.
       * Small values can make the filter too sensitive to noise and textures in the source image.
       * Typical values range from 0.8 to 2.0.
       */
      double sigma_color;
  };

  struct StereoRectificationParams
  {
      /**
       * @brief Degradation ratio to be applied over the x-axis
       */
      int xratio;

      /**
       * @brief Degradation ratio to be applied over the y-axis
       */
      int yratio;

      /**
       * @brief Free scaling parameter
       * If it is -1, the function performs the default scaling.
       * Otherwise, the parameter should be between 0 and 1.
       * Scaling=0 means that the rectified images are zoomed and shifted so that only valid pixels are visible (no black areas after rectification).
       * Scaling=1 means that the rectified image is decimated and shifted so that all the pixels from the original images from the cameras are retained in the rectified images (no source image pixels are lost).
       * Obviously, any intermediate value yields an intermediate result between those two extreme cases.
       */
      double scaling;

      /**
       * @brief Path to the calibration file
       * The calibration file should be named "sensorIDLeft-sensorIDRight.yml"
       * If this file is located in "/path/to/calibration/sensorIDLeft-sensorIDRight.yaml",
       * then this parameter should be "/path/to/calibration"
       */
      std::string calibration_file_path;
  };

  struct StereoMatchingParams
  {
      stereoMatcherParams stereoMatcher;
      #if WITH_XIMGPROC
          filterParams filter;
      #endif
  };

  StereoMatchingParams matching_parameters_;
  StereoRectificationParams rect_parameters_;

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
  ImagePairMatcher(const std::string &output_dir, const std::vector<std::string> &bag_paths, const std::string &image_topic, const std::string & img_extension, infuse_debug_tools::ImagePairMatcher::StereoMatchingParams & matching_parameters, infuse_debug_tools::ImagePairMatcher::StereoRectificationParams & rect_parameters, bool extractPointClouds = false);
  void Match();

private:
  void ProcessImagePair(const infuse_msgs::asn1_bitstream::Ptr& msg);
  void ProcessImage(asn1SccFrame & asn1_frame, boost::filesystem::path data_dir);
  void ProcessStereoMatching(asn1SccFramePair& in_frame_pair, asn1SccFrame& out_raw_disparity, asn1SccFrame& out_color_disparity, std::vector<std::string> & metadata_values);
  void ProcessStereoRectification(asn1SccFramePair& in_original_stereo_pair, asn1SccFramePair& out_rectified_stereo_pair, cv::Mat & out_rect_left, cv::Mat & out_rect_right);

private:
  //! Directory where to put the dataset
  boost::filesystem::path output_dir_;
  //! Strings with paths to the bags to be processed
  std::vector<std::string> bag_paths_;
  //! Name of the topics with the images
  std::string image_topic_;
  //! Directories where to put the image files (set on Extract())
  boost::filesystem::path disparity_dir_;
  boost::filesystem::path disparity_data_dir_;
  boost::filesystem::path colored_disparity_data_dir_;
  boost::filesystem::path rect_left_data_dir_;
  boost::filesystem::path rect_right_data_dir_;
  boost::filesystem::path disparity_metadata_dir_;
  //! Variable used to decode the ASN1 message into.
  std::unique_ptr<asn1SccFramePair> asn1_frame_pair_ptr_;
  //! Variable used to point to output raw disparity image.
  std::unique_ptr<asn1SccFrame> out_raw_disparity_ptr_;
  //! Variable used to point to output colored disparity image.
  std::unique_ptr<asn1SccFrame> out_color_disparity_ptr_;

  //! Number of characters to be used when creating pcd filename
  unsigned int length_img_filename_;
  //! Counter for the number of images files written. Used to create the pcd filename
  size_t image_count_;
  //! Max value for image_count_, computed from length_img_filename_ in the constructor
  size_t image_max_;
  //! Extension used to save the image files
  std::string img_extension_;
  //! Stream used to write the disparity metadata file
  std::ofstream disparity_metadata_ofs_;

  cv::Ptr<cv::StereoBM> _bm;
  cv::Ptr<cv::StereoSGBM> _sgbm;
  #if WITH_XIMGPROC
          int _algorithm;
          bool _use_confidence;
          cv::Ptr<cv::ximgproc::DisparityWLSFilter> _filter;
          cv::Ptr<cv::StereoMatcher> _right_matcher;
  #endif


  //! Variable used to decode the ASN1 message into.
  std::unique_ptr<asn1SccFramePair> asn1_rect_out_frame_pair_ptr_;

  //! Sensors id
  std::string _sensor_id_left;
  std::string _sensor_id_right;
  //! Path of the calibration file
  std::string _calibration_file_path;

  //! Degradation ratio to be applied over the x-axis
  int _xratio;
  //! Degradation ratio to be applied over the y-axis
  int _yratio;
  //! Free scaling parameter
  double _scaling;

  //! Check initialisation
  bool _initialized;

  //! First output map for image left rectification
  cv::Mat _lmapx;
  //! Second output map for image left rectification
  cv::Mat _lmapy;
  //! First output map for image right rectification
  cv::Mat _rmapx;
  //! Second output map for image left rectification
  cv::Mat _rmapy;

  cv::Mat1d _PLeft;
  cv::Mat1d _PRight;
  double _baseline;


  // Starting from here, code for PointClpudExtraction /////////////////////////
  cv::Mat1d Q_; // computed by cv::stereoRectify, saved for use in 
                // reprojectImageTo3D for computing pointClouds
  cv::Mat image3D_;       // StereoPoint cloud in cv::Mat format to be converted in PCD format
  cv::Mat pointCloudLum_; // image used to fill the luminance part in the pcl type
  void savePointCloud(const cv::Mat& disparity,
                      const cv::Mat& lum,
                      const cv::Mat1d& Q);
  PointCloud::Ptr disparityToPointCloud(const cv::Mat& disparity,
                                        const cv::Mat& lum,
                                        const cv::Mat1d& Q);
  Eigen::Affine3d ConvertAsn1PoseToEigen(const asn1SccTransformWithCovariance& asn1_pose);
  std::tuple<float,float,float,float,float,float> FindMinMax(const PointCloud& cloud);
  void ColorPointCloud(ColoredPointCloud & colored_cloud);
  Eigen::Affine3f ComputeSensorPoseInFixedFrame(const asn1SccFramePair& asn1_frame_pair);

  template<typename PointT>
  void SetCloudSensorPose(const Eigen::Affine3f & pose, pcl::PointCloud<PointT> &pcl_cloud)
  {
    pcl_cloud.sensor_origin_ << pose.translation(), 0.0;
    pcl_cloud.sensor_orientation_ = Eigen::Quaternionf(pose.rotation());
  }

  boost::filesystem::path pcd_dir_;
  boost::filesystem::path pcd_img_dir_;

  unsigned int length_pcd_filename_;
  //! Counter for the number of pcd files written. Used to create the pcd filename
  size_t pcd_count_;
  //! Max value for pcd_count_, computed from length_pcd_filename_ in the constructor
  size_t pcd_max_;
  //! Stream used to write the metadata file
  std::ofstream metadata_ofs_;
  //! Store if we want to extract pngs using PCLVisualizer
  bool extract_pcl_pngs_;
  //! Viewer used to render pngs to be dumped
  boost::shared_ptr<pcl::visualization::PCLVisualizer> pcl_viewer_;
  //! Point size for PNG extraction
  double point_size_;
  //! Directory where to put the png files (set on Extract())
  boost::filesystem::path png_dir_;
  //! Store if we want to compute min max z
  bool compute_min_max_z_;
  //! Store min and max z used to create color lookup table
  double min_z_, max_z_;
  //! Color mode used when dumping png views
  ColorMode color_mode_;
};

// Functions used to convert color modes from/to streams
std::istream& operator>>(std::istream& in, ImagePairMatcher::ColorMode& color_mode);
std::ostream& operator<<(std::ostream& out, const ImagePairMatcher::ColorMode& color_mode);

} // infuse_debug_tools

#endif // IMAGEPAIRMATCHER_H
