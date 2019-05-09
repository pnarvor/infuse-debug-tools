#include "ImagePairMatcher.hpp"

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <infuse_asn1_conversions/asn1_base_conversions.hpp>
#include <infuse_asn1_conversions/asn1_pcl_conversions.hpp>

#include <boost/progress.hpp>

#include <opencv2/opencv.hpp>

#include "asn1_bitstream_logger.hpp"

namespace bfs = boost::filesystem;

namespace infuse_debug_tools {

ImagePairMatcher::ImagePairMatcher(const std::string &output_dir, const std::vector<std::string> &bag_paths, const std::string &image_topic, const std::string & img_extension, infuse_debug_tools::ImagePairMatcher::StereoMatchingParams & matching_parameters, infuse_debug_tools::ImagePairMatcher::StereoRectificationParams & rect_parameters)
  : output_dir_{output_dir},
    bag_paths_{bag_paths},
    image_topic_{image_topic},
    img_extension_{img_extension},
    matching_parameters_{matching_parameters},
    rect_parameters_{rect_parameters},
    asn1_frame_pair_ptr_{std::make_unique<asn1SccFramePair>()},
    asn1_rect_out_frame_pair_ptr_{std::make_unique<asn1SccFramePair>()},
    out_raw_disparity_ptr_{std::make_unique<asn1SccFrame>()},
    out_color_disparity_ptr_{std::make_unique<asn1SccFrame>()},
    length_img_filename_{5},
    image_count_{0},
    image_max_{std::stoul(std::string("1") + std::string(length_img_filename_, '0')) - 1},
    length_pcd_filename_{5},
    pcd_count_{0},
    pcd_max_{std::stoul(std::string("1") + std::string(length_pcd_filename_, '0')) - 1},
    //extract_pcl_pngs_{false},
    extract_pcl_pngs_{true},
    pcl_viewer_{nullptr},
    point_size_{1},
    compute_min_max_z_{true},
    min_z_{0},
    max_z_{0},
    color_mode_{ColorMode::kRainbow}
{
    if (extract_pcl_pngs_)
    {
        pcl_viewer_.reset(new pcl::visualization::PCLVisualizer ("3D Viewer"));
        pcl_viewer_->setBackgroundColor (255, 255, 255);
        pcl_viewer_->initCameraParameters ();
    }
}


void ImagePairMatcher::Match()
{
  // Vector of topics used to create a view on the bag
  std::vector<std::string> topics = {image_topic_};

  // Get the number of messages to process
  size_t n_images = 0;
  for (auto bag_path : bag_paths_) {
    rosbag::Bag bag(bag_path); // bagmode::Read by default
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    n_images += view.size();
    bag.close();
  }

  // Stop here if there's nothing on the topic
  if (n_images == 0) {
    std::cout << "Warning: Nothing to extract on topic " << image_topic_ << std::endl;
    return;
  }

  // Makes sure the output dir does not already exists
  if (bfs::exists(output_dir_)) {
    std::stringstream ss;
    if (bfs::is_directory(output_dir_))
      ss << "A directory named \"" << output_dir_.string() << "\" already exists. Please remove it or choose another directory to output the point clouds.";
    else if (bfs::is_regular_file(output_dir_))
      ss << "A regular file named \"" << output_dir_.string() << "\" already exists. Please remove this file or choose another directory name to output the point clouds.";
    else
      ss << "\"" << output_dir_.string() << "\" already exists. Please remove it or choose another directory name to output the point clouds.";
    throw std::runtime_error(ss.str());
  }

  // Lambda function that creates a directory (or subdir inside dir if specified).
  auto lambda_create_subdir = [](bfs::path dir, std::string subdir = "") -> bfs::path {
    bfs::path dirpath = dir / subdir;
    bool dir_created = bfs::create_directory(dirpath);
    if (not dir_created) {
      std::stringstream ss;
      ss << "Could not create \"" << dirpath.string() << "\" directory.";
      throw std::runtime_error(ss.str());
    }
    return dirpath;
  };
  // Create output dir
  lambda_create_subdir(output_dir_);
  // Create subdirs
  disparity_data_dir_         = lambda_create_subdir(output_dir_,   "data");
  colored_disparity_data_dir_ = lambda_create_subdir(output_dir_,   "data_colored");
  rect_left_data_dir_         = lambda_create_subdir(output_dir_,   "rect_left");
  rect_right_data_dir_        = lambda_create_subdir(output_dir_,   "rect_right");
  disparity_metadata_dir_     = lambda_create_subdir(output_dir_,   "metadata");
  pcd_dir_                    = lambda_create_subdir(output_dir_,   "point_cloud_data");
  pcd_img_dir_                = lambda_create_subdir(output_dir_,   "point_cloud_pngs");

  // Fill metadata names vector
  std::vector<std::string> metadata_names;
  metadata_names.push_back("left_timestamp");
  metadata_names.push_back("right_timestamp");
  metadata_names.push_back("number_paired_pixels");
  metadata_names.push_back("percentage_of_paired_pixels");

  // Write dataformat files. The rationalle of keeping the dataformat
  // separated from the metadata is that this way it is possible to associate
  // the cloud number with the line in the metadata file.
  // We do it using a lambda function.
  auto lambda_create_dataformat_file = [](bfs::path dir, std::string file_prefix, const std::vector<std::string> & entries) -> void {
    std::ofstream dataformat_ofs((dir / (file_prefix + "dataformat.txt")).string());
    unsigned int index = 1;
    for (auto entry : entries) {
      dataformat_ofs << "# " << std::setw(2) << index << " - " << entry << '\n';
      index++;
    }
    dataformat_ofs.close();
  };
  lambda_create_dataformat_file(output_dir_, "disparity_", metadata_names);

  // Setup metadata file
  disparity_metadata_ofs_.open((output_dir_ / "disparity_all_metadata.txt").string());

  // Setup progress display
  std::cout << "Extracting " << n_images << " image pairs to " << output_dir_.string() << "/...";
  boost::progress_display show_progress( n_images );

  // Loop over bags
  for (auto bag_path : bag_paths_) {
    rosbag::Bag bag(bag_path); // bagmode::Read by default
    // Create a view of the bag with the selected topics only
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    try {
      // Loop over messages in each view
      for (rosbag::MessageInstance const m: view) {
        infuse_msgs::asn1_bitstream::Ptr i = m.instantiate<infuse_msgs::asn1_bitstream>();
        if (i != nullptr) {
          ProcessImagePair(i);
          ++show_progress; // Update progress display
        } else throw std::runtime_error("Could not instantiate an infuse_msgs::asn1_bitstream message!");
      } // for msgs in view
    } catch (...) {
      // Assure the bags are closed if something goes wrong and re-trhow
      bag.close();
      throw;
    }

    bag.close();
  } // for bags

  // metadata_ofs_.close();

}

void ImagePairMatcher::ProcessImagePair(const infuse_msgs::asn1_bitstream::Ptr& msg)
{
  // Guard against overflow on the filename numbers
  if (image_count_ > image_max_)
    throw std::runtime_error("Overflow on the image filename counter. Please increase the number of characters to be used to compose the filename");

  // Initialize asn1 point cloud to be sure we have clean object.
  asn1SccFramePair_Initialize(asn1_frame_pair_ptr_.get());

  // Decode
  flag res;
  int errorCode;
  BitStream bstream;
  BitStream_AttachBuffer(&bstream, msg->data.data(), msg->data.size());
  res = asn1SccFramePair_Decode(asn1_frame_pair_ptr_.get(), &bstream, &errorCode);
  if (not res) {
    std::stringstream ss;
    ss << "Error decoding asn1SccFramePair! Error: " << errorCode << "\n";
    throw std::runtime_error(ss.str());
  }

  // Process stereo matching
  std::vector<std::string> metadata_values;
  ProcessStereoMatching(*asn1_frame_pair_ptr_, *out_raw_disparity_ptr_, *out_color_disparity_ptr_, metadata_values);

  // Write images
  // ProcessImage(*out_raw_disparity_ptr_,              disparity_data_dir_);
  // ProcessImage(*out_color_disparity_ptr_,            colored_disparity_data_dir_);
  ProcessImage(*out_color_disparity_ptr_,            disparity_data_dir_);
  ProcessImage(asn1_rect_out_frame_pair_ptr_->left,  rect_left_data_dir_);
  ProcessImage(asn1_rect_out_frame_pair_ptr_->right, rect_right_data_dir_);

  // Write all_metadata files
  for(std::vector<int>::size_type i = 0; i != metadata_values.size(); i++) {
      disparity_metadata_ofs_ << metadata_values.at(i) << " ";
  }
  disparity_metadata_ofs_ << '\n';

  // Lambda function to dump metadata from map
  auto lambda_log_metadata_on_file = [this](bfs::path metadata_dir, std::vector<std::string> metadata_values) -> void {
    // Compose output filename
    std::string filename = std::to_string(this->image_count_);
    filename = std::string(this->length_img_filename_ - filename.length(), '0') + filename + ".txt";
    bfs::path metadata_path = metadata_dir / filename;

    // Write the metadata file
    std::ofstream img_metadata_ofs(metadata_path.string());
    for(std::vector<int>::size_type i = 0; i != metadata_values.size(); i++) {
        img_metadata_ofs << metadata_values.at(i) << " ";
    }
    img_metadata_ofs.close();

  };
  // Dump metadata of frames in a separated files
  lambda_log_metadata_on_file(disparity_metadata_dir_, metadata_values);

  image_count_++;
}

void ImagePairMatcher::ProcessImage(asn1SccFrame & asn1_frame, boost::filesystem::path data_dir)
{
  // Bind to the pointer inside the asn1 variable
  cv::Mat img( asn1_frame.data.rows, asn1_frame.data.cols,
    CV_MAKETYPE((int)(asn1_frame.data.depth), asn1_frame.data.channels),
    asn1_frame.data.data.arr, asn1_frame.data.rowSize);

  // Compose output filename
  std::string img_filename = std::to_string(image_count_);
  img_filename = std::string(length_img_filename_ - img_filename.length(), '0') + img_filename + "." + img_extension_;
  bfs::path img_path = data_dir / img_filename;

  // save the file
  cv::imwrite(img_path.string(), img);
}

void ImagePairMatcher::ProcessStereoMatching(asn1SccFramePair& in_frame_pair, asn1SccFrame& out_raw_disparity, asn1SccFrame& out_color_disparity, std::vector<std::string> & metadata_values)
{
    cv::Mat rect_left, rect_right;
    cv::Mat disparity;

    ProcessStereoRectification(in_frame_pair, *asn1_rect_out_frame_pair_ptr_, rect_left, rect_right);

    // Using Algorithm StereoBM
    if(matching_parameters_.stereoMatcher.algorithm == 0)
    {
        cout << "Using StereoBM" << endl;
        if(_bm.empty())
        {
            _bm = cv::StereoBM::create(matching_parameters_.stereoMatcher.num_disparities, matching_parameters_.stereoMatcher.block_size);
        }

        _bm->setBlockSize(matching_parameters_.stereoMatcher.block_size);
        _bm->setDisp12MaxDiff(matching_parameters_.stereoMatcher.disp12_max_diff);
        _bm->setMinDisparity(matching_parameters_.stereoMatcher.min_disparity);
        _bm->setNumDisparities(matching_parameters_.stereoMatcher.num_disparities);
        _bm->setPreFilterCap(matching_parameters_.stereoMatcher.pre_filter_cap);
        _bm->setPreFilterSize(matching_parameters_.stereoMatcher.bm_params.pre_filter_size);
        _bm->setPreFilterType(matching_parameters_.stereoMatcher.bm_params.pre_filter_type);
        _bm->setSpeckleRange(matching_parameters_.stereoMatcher.speckle_range);
        _bm->setSpeckleWindowSize(matching_parameters_.stereoMatcher.speckle_window_size);
        _bm->setTextureThreshold(matching_parameters_.stereoMatcher.bm_params.texture_threshold);
        _bm->setUniquenessRatio(matching_parameters_.stereoMatcher.uniqueness_ratio);

        _bm->compute(rect_left, rect_right, disparity);



    }
    // Using Algorithm StereoSGBM
    else if(matching_parameters_.stereoMatcher.algorithm == 1)
    {
        auto& mp = matching_parameters_;
        cout << "Using StereoSGBM" << endl;
        cout<<endl<<"min_disparity;     : "<<mp.stereoMatcher.min_disparity;
        cout<<endl<<"num_disparities;   : "<<mp.stereoMatcher.num_disparities; 
        cout<<endl<<"block_size;        : "<<mp.stereoMatcher.block_size;
        cout<<endl<<"sgbm_params.P1;    : "<<mp.stereoMatcher.sgbm_params.P1; 
        cout<<endl<<"sgbm_params.P2;    : "<<mp.stereoMatcher.sgbm_params.P2; 
        cout<<endl<<"disp12_max_diff;   : "<<mp.stereoMatcher.disp12_max_diff; 
        cout<<endl<<"pre_filter_cap;    : "<<mp.stereoMatcher.pre_filter_cap;
        cout<<endl<<"uniqueness_ratio;  : "<<mp.stereoMatcher.uniqueness_ratio; 
        cout<<endl<<"speckle_window_siz : "<<mp.stereoMatcher.speckle_window_size; 
        cout<<endl<<"speckle_range;     : "<<mp.stereoMatcher.speckle_range;
        cout<<endl<<"sgbm_params.mode;  : "<<mp.stereoMatcher.sgbm_params.mode;
        if(_sgbm.empty())
        {
            _sgbm = cv::StereoSGBM::create(
            matching_parameters_.stereoMatcher.min_disparity, 
            matching_parameters_.stereoMatcher.num_disparities, 
            matching_parameters_.stereoMatcher.block_size, 
            matching_parameters_.stereoMatcher.sgbm_params.P1, 
            matching_parameters_.stereoMatcher.sgbm_params.P2, 
            matching_parameters_.stereoMatcher.disp12_max_diff, 
            matching_parameters_.stereoMatcher.pre_filter_cap, 
            matching_parameters_.stereoMatcher.uniqueness_ratio, 
            matching_parameters_.stereoMatcher.speckle_window_size, 
            matching_parameters_.stereoMatcher.speckle_range, 
            matching_parameters_.stereoMatcher.sgbm_params.mode);
        }

        _sgbm->setBlockSize(matching_parameters_.stereoMatcher.block_size);
        _sgbm->setDisp12MaxDiff(matching_parameters_.stereoMatcher.disp12_max_diff);
        _sgbm->setMinDisparity(matching_parameters_.stereoMatcher.min_disparity);
        _sgbm->setMode(matching_parameters_.stereoMatcher.sgbm_params.mode);
        _sgbm->setNumDisparities(matching_parameters_.stereoMatcher.num_disparities);
        _sgbm->setP1(matching_parameters_.stereoMatcher.sgbm_params.P1);
        _sgbm->setP2(matching_parameters_.stereoMatcher.sgbm_params.P2);
        _sgbm->setPreFilterCap(matching_parameters_.stereoMatcher.pre_filter_cap);
        _sgbm->setSpeckleRange(matching_parameters_.stereoMatcher.speckle_range);
        _sgbm->setSpeckleWindowSize(matching_parameters_.stereoMatcher.speckle_window_size);
        _sgbm->setUniquenessRatio(matching_parameters_.stereoMatcher.uniqueness_ratio);

        _sgbm->compute(rect_left, rect_right, disparity);

    }

#if WITH_XIMGPROC
//#if 0
    
    cout << "Using WITH_XIMPROG" << endl;
    bool reset_filter = false;
    bool reset_matcher = false;
    if(matching_parameters_.stereoMatcher.algorithm != _algorithm)
    {
        _algorithm = matching_parameters_.stereoMatcher.algorithm;
        reset_filter = true;
        reset_matcher = true;
    }
    else if(matching_parameters_.filter.use_confidence != _use_confidence)
    {
        _use_confidence = matching_parameters_.filter.use_confidence;
        reset_filter = true;
    }

    if(matching_parameters_.filter.use_filter)
    {
        cv::Mat disparity_filtered;

        if(matching_parameters_.filter.use_confidence
                ){
            cv::Mat disparity_right;
            if(_right_matcher.empty() || reset_matcher)
            {
                switch(_algorithm)
                {
                    case 0:
                        _right_matcher = cv::ximgproc::createRightMatcher(_bm);
                        break;
                    case 1:
                        _right_matcher = cv::ximgproc::createRightMatcher(_sgbm);
                        break;
                }
            }

            _right_matcher->compute(rect_left, rect_right, disparity_right);

            if(_filter.empty() || reset_filter)
            {
                switch(_algorithm)
                {
                    case 0:
                        _filter = cv::ximgproc::createDisparityWLSFilter(_bm);
                        break;
                    case 1:
                        _filter = cv::ximgproc::createDisparityWLSFilter(_sgbm);
                        break;
                }
            }

            _filter->setDepthDiscontinuityRadius(matching_parameters_.filter.depth_discontinuity_radius);
            _filter->setLambda(matching_parameters_.filter.lambda);
            _filter->setLRCthresh(matching_parameters_.filter.lrc_thresh);
            _filter->setSigmaColor(matching_parameters_.filter.sigma_color);

            _filter->filter(disparity, rect_left, disparity_filtered, disparity_right);
        }
        else
        {
            if(_filter.empty() || reset_filter)
            {
                _filter = cv::ximgproc::createDisparityWLSFilterGeneric(false);
            }

            _filter->setDepthDiscontinuityRadius(matching_parameters_.filter.depth_discontinuity_radius);
            _filter->setLambda(matching_parameters_.filter.lambda);
            _filter->setSigmaColor(matching_parameters_.filter.sigma_color);

            _filter->filter(disparity, rect_left, disparity_filtered);
        }

        disparity = disparity_filtered;
    }
#endif

    // at this point disparity image is in cv::Mat disparity variable
    // cv::Mat rect_left, rect_right
    this->savePointCloud(disparity, rect_left, Q_);

    // Convert Mat to ASN.1
    out_raw_disparity.metadata.msgVersion = frame_Version;
    out_raw_disparity.metadata = in_frame_pair.left.metadata;
    out_raw_disparity.intrinsic = in_frame_pair.left.intrinsic;
    out_raw_disparity.extrinsic = in_frame_pair.left.extrinsic;

    out_raw_disparity.metadata.mode = asn1Sccmode_UNDEF;
    out_raw_disparity.metadata.pixelModel = asn1Sccpix_DISP;
    out_raw_disparity.metadata.errValues.arr[0].type = asn1Sccerror_UNDEFINED;
    out_raw_disparity.metadata.errValues.arr[0].value = -16.0;
    out_raw_disparity.metadata.errValues.nCount = 1;

    double min_disp, max_disp;
    cv::minMaxLoc(disparity, &min_disp, &max_disp);
    out_raw_disparity.metadata.pixelCoeffs.arr[0] = 16.0;
    out_raw_disparity.metadata.pixelCoeffs.arr[1] = 0.0;
    out_raw_disparity.metadata.pixelCoeffs.arr[2] = in_frame_pair.baseline;
    out_raw_disparity.metadata.pixelCoeffs.arr[3] = max_disp;
    out_raw_disparity.metadata.pixelCoeffs.arr[4] = min_disp;

    out_raw_disparity.data.msgVersion = array3D_Version;
    out_raw_disparity.data.channels = static_cast<asn1SccT_UInt32>(disparity.channels());
    out_raw_disparity.data.rows = static_cast<asn1SccT_UInt32>(disparity.rows);
    out_raw_disparity.data.cols = static_cast<asn1SccT_UInt32>(disparity.cols);
    out_raw_disparity.data.depth = static_cast<asn1SccArray3D_depth_t>(disparity.depth());
    out_raw_disparity.data.rowSize = disparity.step[0];
    out_raw_disparity.data.data.nCount =  static_cast<int>(out_raw_disparity.data.rows * out_raw_disparity.data.rowSize);
    memcpy(out_raw_disparity.data.data.arr, disparity.data, static_cast<size_t>(out_raw_disparity.data.data.nCount));

    // Convert the filtered image as a cv::Mat for display
    cv::Mat filtered =  cv::Mat(static_cast<int>(out_raw_disparity.data.rows), static_cast<int>(out_raw_disparity.data.cols),
                                CV_MAKETYPE(static_cast<int>(out_raw_disparity.data.depth), static_cast<int>(out_raw_disparity.data.channels)),
                                out_raw_disparity.data.data.arr, out_raw_disparity.data.rowSize);

    // Apply a colormap
    cv::Mat filteredDisparityColor;
    double min,    max;
    cv::minMaxLoc(filtered, &min, &max);
    filtered.convertTo(filtered, CV_8U, 255 / (max - min), -255.0 * min / (max - min));
    cv::Mat mask = filtered > 0;
    cv::applyColorMap(filtered, filtered, 2);
    filtered.copyTo(filteredDisparityColor, mask);

    // Convert Mat to ASN.1
    out_color_disparity.metadata.msgVersion = frame_Version;
    out_color_disparity.metadata = in_frame_pair.left.metadata;
    out_color_disparity.intrinsic = in_frame_pair.left.intrinsic;
    out_color_disparity.extrinsic = in_frame_pair.left.extrinsic;

    out_color_disparity.metadata.mode = asn1Sccmode_UNDEF;
    out_color_disparity.metadata.pixelModel = asn1Sccpix_DISP;
    out_color_disparity.metadata.errValues.arr[0].type = asn1Sccerror_UNDEFINED;
    out_color_disparity.metadata.errValues.arr[0].value = -16.0;
    out_color_disparity.metadata.errValues.nCount = 1;

    double min_color_disp, max_color_disp;
    cv::minMaxLoc(filteredDisparityColor, &min_color_disp, &max_color_disp);
    out_color_disparity.metadata.pixelCoeffs.arr[0] = 16.0;
    out_color_disparity.metadata.pixelCoeffs.arr[1] = 0.0;
    out_color_disparity.metadata.pixelCoeffs.arr[2] = in_frame_pair.baseline;
    out_color_disparity.metadata.pixelCoeffs.arr[3] = max_color_disp;
    out_color_disparity.metadata.pixelCoeffs.arr[4] = min_color_disp;

    out_color_disparity.data.msgVersion = array3D_Version;
    out_color_disparity.data.channels = static_cast<asn1SccT_UInt32>(filteredDisparityColor.channels());
    out_color_disparity.data.rows = static_cast<asn1SccT_UInt32>(filteredDisparityColor.rows);
    out_color_disparity.data.cols = static_cast<asn1SccT_UInt32>(filteredDisparityColor.cols);
    out_color_disparity.data.depth = static_cast<asn1SccArray3D_depth_t>(filteredDisparityColor.depth());
    out_color_disparity.data.rowSize = filteredDisparityColor.step[0];
    out_color_disparity.data.data.nCount =  static_cast<int>(out_color_disparity.data.rows * out_color_disparity.data.rowSize);
    memcpy(out_color_disparity.data.data.arr, filteredDisparityColor.data, static_cast<size_t>(out_color_disparity.data.data.nCount));


    // Get the number of paired pixels and calculate percentage
    double nb_paired = 0;
    double percentage;
    for (int x = 0; x<disparity.rows; x++)
    {
        for (int y = 0; y<disparity.cols; y++)
        {
            // Accesssing values of each pixel
            if ((disparity.at<int16_t>(x, y)/16) > 0)
                nb_paired += 1;
        }
    }

    percentage = (nb_paired/(disparity.rows * disparity.cols))*100;

    metadata_values.push_back(std::to_string(in_frame_pair.left.metadata.timeStamp.microseconds));
    metadata_values.push_back(std::to_string(in_frame_pair.right.metadata.timeStamp.microseconds));
    metadata_values.push_back(std::to_string(static_cast<int>(nb_paired)));
    metadata_values.push_back(std::to_string(percentage));

}

void ImagePairMatcher::ProcessStereoRectification(asn1SccFramePair& in_original_stereo_pair, asn1SccFramePair& out_rectified_stereo_pair, cv::Mat & out_rect_left, cv::Mat & out_rect_right){
    cv::Mat in_left(static_cast<int>(in_original_stereo_pair.left.data.rows), static_cast<int>(in_original_stereo_pair.left.data.cols), CV_MAKETYPE(static_cast<int>(in_original_stereo_pair.left.data.depth), static_cast<int>(in_original_stereo_pair.left.data.channels)), in_original_stereo_pair.left.data.data.arr, in_original_stereo_pair.left.data.rowSize);
    cv::Mat in_right(static_cast<int>(in_original_stereo_pair.right.data.rows), static_cast<int>(in_original_stereo_pair.right.data.cols), CV_MAKETYPE(static_cast<int>(in_original_stereo_pair.right.data.depth), static_cast<int>(in_original_stereo_pair.right.data.channels)), in_original_stereo_pair.right.data.data.arr, in_original_stereo_pair.right.data.rowSize);

    // Generate correction maps if needed
    if( std::string(reinterpret_cast<char const *>(in_original_stereo_pair.left.intrinsic.sensorId.arr)) != _sensor_id_left ||
            std::string(reinterpret_cast<char const *>(in_original_stereo_pair.right.intrinsic.sensorId.arr)) != _sensor_id_right ||
            rect_parameters_.calibration_file_path != _calibration_file_path ||
            rect_parameters_.xratio != _xratio ||
            rect_parameters_.yratio != _yratio ||
            rect_parameters_.scaling != _scaling){

        _sensor_id_left = std::string(reinterpret_cast<char const *>(in_original_stereo_pair.left.intrinsic.sensorId.arr));
        _sensor_id_right = std::string(reinterpret_cast<char const *>(in_original_stereo_pair.right.intrinsic.sensorId.arr));
        _calibration_file_path = rect_parameters_.calibration_file_path;
        _xratio = rect_parameters_.xratio;
        _yratio = rect_parameters_.yratio;
        _scaling = rect_parameters_.scaling;

        //cv::FileStorage fs( _calibration_file_path + "/" + _sensor_id_left + std::string("-") + _sensor_id_right + ".yml", cv::FileStorage::READ );
        cv::FileStorage fs( _calibration_file_path, cv::FileStorage::READ );
        if( fs.isOpened() ){
            cv::Size image_size;
            cv::Mat1d camera_matrix_L;
            cv::Mat1d camera_matrix_R;
            cv::Mat1d dist_coeffs_L;
            cv::Mat1d dist_coeffs_R;
            cv::Mat1d R;
            cv::Mat1d T;

            fs["image_width"]  >> image_size.width;
            fs["image_height"] >> image_size.height;

            fs["camera_matrix_1"] >> camera_matrix_L;
            fs["distortion_coefficients_1"] >> dist_coeffs_L;

            fs["camera_matrix_2"] >> camera_matrix_R;
            fs["distortion_coefficients_2"] >> dist_coeffs_R;

            fs["rotation_matrix"] >> R;
            fs["translation_coefficients"] >> T;

            cv::Size newSize;
            newSize.width = image_size.width / _xratio;
            newSize.height = image_size.height / _yratio;

            cv::Mat1d RLeft;
            cv::Mat1d RRight;
            //cv::Mat1d Q;

            cv::stereoRectify(camera_matrix_L, dist_coeffs_L, camera_matrix_R, dist_coeffs_R, image_size, R, T, RLeft, RRight, _PLeft, _PRight, Q_, CV_CALIB_ZERO_DISPARITY, _scaling, newSize);

            std::cout << "Pleft :\n" << _PLeft << std::endl;
            std::cout << "Pright :\n" << _PRight << std::endl;
            std::cout << "Q :\n" << Q_ << std::endl << std::endl;

            cv::initUndistortRectifyMap(camera_matrix_L, dist_coeffs_L, RLeft, _PLeft, newSize, CV_32F, _lmapx, _lmapy);
            cv::initUndistortRectifyMap(camera_matrix_R, dist_coeffs_R, RRight, _PRight, newSize, CV_32F, _rmapx, _rmapy);

            _baseline = 1.0 / Q_.at<double>(3,2);

            _initialized = true;
        }
        else{
            _initialized = false;
            //std::cerr << "Can't open the calibration file: " << _calibration_file_path + "/" + _sensor_id_left + std::string("-") + _sensor_id_right + ".yml" << std::endl;
            std::cerr << "Can't open the calibration file: "
                      << _calibration_file_path << std::endl;
        }
    }

    if( _initialized ){
        cv::remap(in_left, out_rect_left, _lmapx, _lmapy, cv::INTER_LINEAR);
        cv::remap(in_right, out_rect_right, _rmapx, _rmapy, cv::INTER_LINEAR);

        // Getting image pair
        out_rectified_stereo_pair.msgVersion = frame_Version;
        out_rectified_stereo_pair.baseline = _baseline;

        // Left image
        {
            asn1SccFrame & img = out_rectified_stereo_pair.left;

            // init the structure
            img.msgVersion = frame_Version;

            img.intrinsic = in_original_stereo_pair.left.intrinsic;

            Eigen::Map<Eigen::Matrix3d>(img.intrinsic.cameraMatrix.arr[0].arr, 3, 3) = Eigen::Map<Eigen::Matrix3d, 0, Eigen::OuterStride<>>(reinterpret_cast<double *>(_PLeft.data), 3, 3, Eigen::OuterStride<>(4));

            Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 1>>(img.intrinsic.distCoeffs.arr, in_original_stereo_pair.left.intrinsic.distCoeffs.nCount, 1) = Eigen::Matrix<double, Eigen::Dynamic, 1>::Zero(in_original_stereo_pair.left.intrinsic.distCoeffs.nCount, 1);
            img.intrinsic.distCoeffs.nCount = in_original_stereo_pair.left.intrinsic.distCoeffs.nCount;

            img.intrinsic.cameraModel = asn1Scccam_PINHOLE;

            img.extrinsic = in_original_stereo_pair.left.extrinsic;
            img.metadata = in_original_stereo_pair.left.metadata;

            // Array3D
            {
                img.data.msgVersion = array3D_Version;
                img.data.rows = static_cast<asn1SccT_UInt32>(out_rect_left.rows);
                img.data.cols = static_cast<asn1SccT_UInt32>(out_rect_left.cols);
                img.data.channels = static_cast<asn1SccT_UInt32>(out_rect_left.channels());
                img.data.depth = static_cast<asn1SccArray3D_depth_t>(out_rect_left.depth());
                img.data.rowSize = static_cast<asn1SccT_UInt32>(out_rect_left.step[0]);
                img.data.data.nCount = static_cast<int>(img.data.rows * img.data.rowSize);
                memcpy(img.data.data.arr, out_rect_left.data, static_cast<size_t>(img.data.data.nCount));
            }
        }

        // Right image
        {
            asn1SccFrame & img = out_rectified_stereo_pair.right;

            // init the structure
            img.msgVersion = frame_Version;

            img.intrinsic = in_original_stereo_pair.right.intrinsic;

            Eigen::Map<Eigen::Matrix3d>(img.intrinsic.cameraMatrix.arr[0].arr, 3, 3) = Eigen::Map<Eigen::Matrix3d, 0, Eigen::OuterStride<>>(reinterpret_cast<double *>(_PRight.data), 3, 3, Eigen::OuterStride<>(4));

            Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 1>>(img.intrinsic.distCoeffs.arr, in_original_stereo_pair.right.intrinsic.distCoeffs.nCount, 1) = Eigen::Matrix<double, Eigen::Dynamic, 1>::Zero(in_original_stereo_pair.right.intrinsic.distCoeffs.nCount, 1);
            img.intrinsic.distCoeffs.nCount = in_original_stereo_pair.right.intrinsic.distCoeffs.nCount;

            img.intrinsic.cameraModel = asn1Scccam_PINHOLE;

            img.extrinsic = in_original_stereo_pair.right.extrinsic;
            img.metadata = in_original_stereo_pair.right.metadata;

            // Array3D
            {
                img.data.msgVersion = array3D_Version;
                img.data.rows = static_cast<asn1SccT_UInt32>(out_rect_right.rows);
                img.data.cols = static_cast<asn1SccT_UInt32>(out_rect_right.cols);
                img.data.channels = static_cast<asn1SccT_UInt32>(out_rect_right.channels());
                img.data.depth = static_cast<asn1SccArray3D_depth_t>(out_rect_right.depth());
                img.data.rowSize = static_cast<asn1SccT_UInt32>(out_rect_right.step[0]);
                img.data.data.nCount = static_cast<int>(img.data.rows * img.data.rowSize);
                memcpy(img.data.data.arr, out_rect_right.data, static_cast<size_t>(img.data.data.nCount));
            }
        }
    }
}

void ImagePairMatcher::savePointCloud(const cv::Mat& disparity,
                                      const cv::Mat& lum,
                                      const cv::Mat1d& Q)
{
    PointCloud::Ptr pcl_cloud_ptr = this->disparityToPointCloud(disparity,
                                                                lum, Q);
    Eigen::Affine3f T_fixed_sensor = this->ComputeSensorPoseInFixedFrame(*asn1_frame_pair_ptr_);
    this->SetCloudSensorPose(T_fixed_sensor, *pcl_cloud_ptr); // Associate global pose to data

    std::string pcd_filename = std::to_string(pcd_count_);
    pcd_filename = std::string(length_pcd_filename_ - pcd_filename.length(), '0') 
                 + pcd_filename + ".pcd";
    bfs::path pcd_path = pcd_dir_ / pcd_filename;

    // Save pcd
    bool pcd_binary_mode = true;
    cout << pcd_path.string() << endl;
    pcl::io::savePCDFile( pcd_path.string(), *pcl_cloud_ptr, pcd_binary_mode );
   
    // Handle PNG extraction
    if (extract_pcl_pngs_)
    {
        // Transform the points in the global frame for display (and set point_cloud pose to I)
        pcl::transformPointCloud(*pcl_cloud_ptr, *pcl_cloud_ptr, T_fixed_sensor);
        SetCloudSensorPose(Eigen::Affine3f::Identity(), *pcl_cloud_ptr);

        // Get a version of the point cloud that can be colored
        ColoredPointCloud::Ptr pcl_colored_cloud_ptr(new ColoredPointCloud());
        pcl::copyPointCloud(*pcl_cloud_ptr, *pcl_colored_cloud_ptr);

        // Compute useful transformations
        // Sensor frame that considers only yaw
        Eigen::Affine3f T_fixed_robot = ConvertAsn1PoseToEigen(
            asn1_frame_pair_ptr_->left.extrinsic.pose_fixedFrame_robotFrame).cast<float>();

        Eigen::Affine3f T_fixed_sensoryaw;
        //T_fixed_sensoryaw = Eigen::AngleAxis<float>(ASN1BitstreamLogger::Yaw(
        //    Eigen::Quaternionf(T_fixed_sensor.rotation())), Eigen::Vector3f::UnitZ());
        //T_fixed_sensoryaw.translation() = T_fixed_sensor.translation();
        T_fixed_sensoryaw = Eigen::AngleAxis<float>(ASN1BitstreamLogger::Yaw(
            Eigen::Quaternionf(T_fixed_robot.rotation())), Eigen::Vector3f::UnitZ());
        T_fixed_sensoryaw.translation() = T_fixed_robot.translation();
        // Camera pose, behind the robot and considering only sensor yaw
        // (makes camera less shaky). Note that since the velodyne is mounted
        // facing backwards, we preform a positive translation on sensor's X axis
        //Eigen::Affine3f T_fixed_camera = T_fixed_sensoryaw * Eigen::Translation<float,3>(45,0,20);
        Eigen::Affine3f T_fixed_camera = T_fixed_sensoryaw * Eigen::Translation<float,3>(5,10,5);

        // Add a new at current sensor pose. This creates a trail of frames
        //pcl_viewer_->addCoordinateSystem (1.0, T_fixed_sensor, "sensor_frame");
        pcl_viewer_->addCoordinateSystem (1.0, T_fixed_robot, "robot_frame");
        // Or alternativaly we can update the coordinate system, w/o leaving the trail
        // pcl_viewer_->updateCoordinateSystemPose ("sensor_frame", T_fixed_sensor);

        // Fill RGB values for each point
        ColorPointCloud(*pcl_colored_cloud_ptr);

        // Remove previous point cloud
        if(pcd_count_ != 0) {
          pcl_viewer_->removePointCloud("sample cloud");
        }

        // Add cloud
        pcl_viewer_->addPointCloud (pcl_colored_cloud_ptr, "sample cloud");
        pcl_viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size_, "sample cloud");

        // Put the camera behind the robot, looking at the sensor origin, and upwards
        pcl_viewer_->setCameraPosition (T_fixed_camera.translation()[0], // pos_x
                                        T_fixed_camera.translation()[1], // pos_y
                                        T_fixed_camera.translation()[2], // pos_z
                                        T_fixed_sensoryaw.translation()[0], // view_x
                                        T_fixed_sensoryaw.translation()[1], // view_y
                                        T_fixed_sensoryaw.translation()[2], // view_z
                                        0,  // up_x
                                        0,  // up_y
                                        1); // up_z

        // Render
        bool force_redraw = true;
        pcl_viewer_->spinOnce (1, force_redraw);

        // Save png
        bfs::path png_path =
            pcd_img_dir_ / bfs::path(pcd_path).filename().replace_extension(".png");
        pcl_viewer_->saveScreenshot(png_path.string());
    }

    pcd_count_++;
}

ImagePairMatcher::PointCloud::Ptr ImagePairMatcher::disparityToPointCloud(
    const cv::Mat& disparity, const cv::Mat& lum, const cv::Mat1d& Q)
{
    cv::Mat image3d;
    reprojectImageTo3D(disparity, image3d, Q, true);
    PointCloud::Ptr pcl_cloud_ptr(new PointCloud());

	fromASN1SCC(asn1_frame_pair_ptr_->left.metadata.timeStamp, pcl_cloud_ptr->header.stamp);
    if(!asn1_frame_pair_ptr_->left.extrinsic.hasFixedTransform)
        throw std::invalid_argument("A left image where not tagged with a transform !");
	fromASN1SCC(
        asn1_frame_pair_ptr_->left.extrinsic.pose_robotFrame_sensorFrame.metadata.childFrameId,
        pcl_cloud_ptr->header.frame_id);

	pcl_cloud_ptr->points.resize(image3d.rows * image3d.cols);
    pcl_cloud_ptr->height = image3d.rows;
    pcl_cloud_ptr->width  = image3d.cols;

    for(int w = 0; w < pcl_cloud_ptr->width; w++)
    {
        for(int h = 0; h < pcl_cloud_ptr->height; h++)
        {
            if(image3d.at<cv::Vec3f>(h,w)[2] >= 9999.9
               || std::isnan(image3d.at<cv::Vec3f>(h,w)[0])
               || std::isnan(image3d.at<cv::Vec3f>(h,w)[1])
               || std::isnan(image3d.at<cv::Vec3f>(h,w)[2])
               || h >= pcl_cloud_ptr->height - 4 || h < 4
               || w >= pcl_cloud_ptr->width  - 4 || w < 4)
            {
                (*pcl_cloud_ptr)(w,h).intensity = -1.0;
                (*pcl_cloud_ptr)(w,h).x = 0.0;
                (*pcl_cloud_ptr)(w,h).y = 0.0;
                (*pcl_cloud_ptr)(w,h).z = 0.0;
            }
            else
            {
                (*pcl_cloud_ptr)(w,h).x = image3d.at<cv::Vec3f>(h,w)[0];
                (*pcl_cloud_ptr)(w,h).y = image3d.at<cv::Vec3f>(h,w)[1];
                (*pcl_cloud_ptr)(w,h).z = image3d.at<cv::Vec3f>(h,w)[2];
                // /!\ Check the conversion values /!\ //
                switch(asn1_frame_pair_ptr_->left.data.depth)
                {
                    default:
                        (*pcl_cloud_ptr)(w,h).intensity = -1.0;
                        break;
                    case asn1Sccdepth_8U:
                        (*pcl_cloud_ptr)(w,h).intensity = lum.at<uint8_t>(h,w) / 255.0;
                        break;
                    case asn1Sccdepth_8S:
                        (*pcl_cloud_ptr)(w,h).intensity = lum.at<int8_t>(h,w) / 127.0;
                        break;
                    case asn1Sccdepth_16U:
                        (*pcl_cloud_ptr)(w,h).intensity = lum.at<uint16_t>(h,w) / 65536.0;
                        break;
                    case asn1Sccdepth_16S:
                        (*pcl_cloud_ptr)(w,h).intensity = lum.at<int16_t>(h,w) / 32767.0;
                        break;
                    case asn1Sccdepth_32S:
                        (*pcl_cloud_ptr)(w,h).intensity = lum.at<int32_t>(h,w) / 2147483647.0;
                        break;
                    case asn1Sccdepth_32F:
                        (*pcl_cloud_ptr)(w,h).intensity = lum.at<float>(h,w);
                        break;
                    case asn1Sccdepth_64F:
                        (*pcl_cloud_ptr)(w,h).intensity = lum.at<double>(h,w);
                        break;
                }
            }
        }
    }
    
    return pcl_cloud_ptr;
}

Eigen::Affine3f ImagePairMatcher::ComputeSensorPoseInFixedFrame(
    const asn1SccFramePair& asn1_frame_pair)
{
    Eigen::Affine3d T_fixed_robot = ConvertAsn1PoseToEigen(
        asn1_frame_pair.left.extrinsic.pose_fixedFrame_robotFrame);

    Eigen::Affine3d T_robot_sensor = ConvertAsn1PoseToEigen(
        asn1_frame_pair.left.extrinsic.pose_robotFrame_sensorFrame);

    return (T_fixed_robot * T_robot_sensor).cast<float>();
}

// (nearly) RAW COPIED FROM PointCloudExtractor ////////////////////////////////
Eigen::Affine3d ImagePairMatcher::ConvertAsn1PoseToEigen(const asn1SccTransformWithCovariance& asn1_pose)
{
    Eigen::Matrix3d m;
    m = Eigen::Quaterniond(asn1_pose.data.orientation.arr[3],
                           asn1_pose.data.orientation.arr[0],
                           asn1_pose.data.orientation.arr[1],
                           asn1_pose.data.orientation.arr[2]);

    Eigen::Affine3d eigen_pose;
    eigen_pose.linear() = m;
    eigen_pose.translation() << asn1_pose.data.translation.arr[0],
                                asn1_pose.data.translation.arr[1],
                                asn1_pose.data.translation.arr[2];
    return eigen_pose;
}

std::tuple<float,float,float,float,float,float> ImagePairMatcher::FindMinMax(
    const PointCloud& cloud)
{
  float min_x = cloud.points.at(0).x,
        max_x = cloud.points.at(0).x,
        min_y = cloud.points.at(0).y,
        max_y = cloud.points.at(0).y,
        min_z = cloud.points.at(0).z,
        max_z = cloud.points.at(0).z;

  for (auto & point : cloud.points)
  {
    if (min_x > point.x)
      min_x = point.x;

    if (max_x < point.x)
      max_x = point.x;

    if (min_y > point.y)
      min_y = point.y;

    if (max_y < point.y)
      max_y = point.y;

    if (min_z > point.z)
      min_z = point.z;

    if (max_z < point.z)
      max_z = point.z;
  }

  return std::make_tuple(min_x, max_x, min_y, max_y, min_z, max_z);
}

void ImagePairMatcher::ColorPointCloud(ColoredPointCloud & colored_cloud)
{
  double lut_scale = 255.0 / (max_z_ - min_z_);  // max is 255, min is 0

  if (min_z_ == max_z_)  // In case the cloud is flat
    lut_scale = 1.0;  // Avoid rounding error in boost

  for (auto & point : colored_cloud.points)
  {
    int value = boost::math::iround ( (point.z - min_z_) * lut_scale); // Round the number to the closest integer

    // Guard against outliers
    if (value > 255)
      value = 255;
    if (value < 0)
      value = 0;

    // Color the point
    switch (color_mode_)
    {
      case ColorMode::kBlueToRed:
        // Blue (= min) -> Red (= max)
        point.r = value;
        point.g = 0;
        point.b = 255 - value;
        break;
      case ColorMode::kGreenToMagenta:
        // Green (= min) -> Magenta (= max)
        point.r = value;
        point.g = 255 - value;
        point.b = value;
        break;
      case ColorMode::kWhiteToRed:
        // White (= min) -> Red (= max)
        point.r = 255;
        point.g = 255 - value;
        point.b = 255 - value;
        break;
      case ColorMode::kGrayOrRed:
        // Grey (< 128) / Red (> 128)
        if (value > 128)
        {
          point.r = 255;
          point.g = 0;
          point.b = 0;
        }
        else
        {
          point.r = 128;
          point.g = 128;
          point.b = 128;
        }
        break;
      default:
        // Blue -> Green -> Red (~ rainbow)
        point.r = value > 128 ? (value - 128) * 2 : 0;  // r[128] = 0, r[255] = 255
        point.g = value < 128 ? 2 * value : 255 - ( (value - 128) * 2);  // g[0] = 0, g[128] = 255, g[255] = 0
        point.b = value < 128 ? 255 - (2 * value) : 0;  // b[0] = 255, b[128] = 0
    } 
  }
}

std::istream& operator>>(std::istream& in, ImagePairMatcher::ColorMode& color_mode)
{
  using ColorMode = ImagePairMatcher::ColorMode;

  std::string token;
  in >> token;
  if (token == "blue2red")
    color_mode = ColorMode::kBlueToRed;
  else if (token == "green2magenta")
    color_mode = ColorMode::kGreenToMagenta;
  else if (token == "white2red")
    color_mode = ColorMode::kWhiteToRed;
  else if (token == "gray/red")
    color_mode = ColorMode::kGrayOrRed;
  else if (token == "rainbow")
    color_mode = ColorMode::kRainbow;
  else
      in.setstate(std::ios_base::failbit);
  return in;
}

std::ostream& operator<<(std::ostream& out, const ImagePairMatcher::ColorMode& color_mode)
{
  using ColorMode = ImagePairMatcher::ColorMode;

  if (color_mode == ColorMode::kBlueToRed)
    out << "blue2red";
  else if (color_mode == ColorMode::kGreenToMagenta)
    out << "green2magenta";
  else if (color_mode == ColorMode::kWhiteToRed)
    out << "white2red";
  else if (color_mode == ColorMode::kGrayOrRed)
    out << "gray/red";
  else if (color_mode == ColorMode::kRainbow)
    out << "rainbow";
  else
    out.setstate(std::ios_base::failbit);

  return out;
}

} // infuse_debug_tools

