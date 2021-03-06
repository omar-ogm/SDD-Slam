//
// Created by omar on 7/12/19.
//
#include "CVDifodo.h"
#include "mrpt/poses/CPose3D.h"

CVDifodo::CVDifodo() : mrpt::vision::CDifodo() {
  // DEFAULT ATTRIBUTES VALUES
  rows_orig = 480;
  cols_orig = 640;

  depth_pixel_scale = 1000;

  min_depth_value_filter = 0.5f;
  max_depth_value_filter = 4.5f;

  /**
   * The number of times we want to downsample the original resolution.
   * downsample = 1 means that no downsample will be made.
   * Check the original description from definitions in CDifodo.h
   */
  downsample = 1;

  ctf_levels = 5;

  rows_ctf = rows_orig / downsample;
  cols_ctf = cols_orig / downsample;

  fovh_degrees = 74.0f; //58.6;
  fovv_degrees = 62.0f;; //45.6;

  // NOTE: I dont know why but if set to true, the algorithm doesnt work...
  fast_pyramid = false;

  // Initialize the displacement pose
  cam_pose_displacement = mrpt::poses::CPose3D(mrpt::poses::UNINITIALIZED_POSE);
}

CVDifodo::~CVDifodo() {
}

void CVDifodo::loadConfiguration() {
  this->rows_orig = SD_SLAM::Config::Height();
  this->cols_orig = SD_SLAM::Config::Width();

  // Fixed value. Not used since the image that comes to DIFODO has been preprocessed to be at millimeters.
  this->depth_pixel_scale = 1;

  this->min_depth_value_filter = SD_SLAM::Config::difodo_min_depth_value_filter();
  this->max_depth_value_filter = SD_SLAM::Config::difodo_max_depth_value_filter();

  this->downsample = SD_SLAM::Config::difodo_downsample();
  this->ctf_levels = SD_SLAM::Config::difodo_ctf_levels();

  rows_ctf = rows_orig / downsample;
  cols_ctf = cols_orig / downsample;

  this->fovh_degrees = SD_SLAM::Config::fovh();
  this->fovv_degrees = SD_SLAM::Config::fovv();

  this->fast_pyramid = SD_SLAM::Config::difodo_fast_pyramid();

  /******************SET DIFODO INNER ATTRIBUTES VALUES*****************/
  this->fovh = M_PI * fovh_degrees / 180.0;
  this->fovv = M_PI * fovv_degrees / 180.0;

  // Resolution of the depth image that the algorithm will work with. After the downsampling.
  this->m_width = cols_orig / this->downsample;
  this->m_height = rows_orig / this->downsample;

  // Resolution of the finest level of the pyramid. Should be equal or lower than the image working resolution.
  if (rows_orig < rows_ctf) {
    ROS_ERROR("The rows_ctf (%d) cannot be more than those of the working depth image rows_orig/downsample = %d.",
              rows_ctf, this->m_height);
    // Set the maximum allowed by default
    this->rows = rows_orig / downsample;
  } else {
    this->rows = rows_ctf;
  }

  if (cols_orig < cols_ctf) {
    ROS_ERROR("The cols_ctf (%d) cannot be more than those of the working depth image cols_orig/downsample = %d.",
              cols_ctf, this->m_width);
    // Set the maximum allowed by default
    this->cols = cols_orig / downsample;
  } else {
    this->cols = cols_ctf;
  }

  /******************Resize Matrices and adjust parameters*****************/
  // *****Pyramid setup*****
  // Resize each matrix to have the same number of matrices than levels of pyramid
  const unsigned int pyr_levels =
          std::round(log((float) this->m_width / this->cols) / log(2.f)) + this->ctf_levels;
  this->depth.resize(pyr_levels);
  this->depth_old.resize(pyr_levels);
  this->depth_inter.resize(pyr_levels);
  this->depth_warped.resize(pyr_levels);
  this->xx.resize(pyr_levels);
  this->xx_inter.resize(pyr_levels);
  this->xx_old.resize(pyr_levels);
  this->xx_warped.resize(pyr_levels);
  this->yy.resize(pyr_levels);
  this->yy_inter.resize(pyr_levels);
  this->yy_old.resize(pyr_levels);
  this->yy_warped.resize(pyr_levels);
  this->transformations.resize(pyr_levels);

  // Resize each level (or matrix) of the pyramid to the desired (rows,cols) size
  for (unsigned int i = 0; i < pyr_levels; i++) {
    unsigned int s = pow(2.f, int(i));
    this->cols_i = this->m_width / s;
    this->rows_i = this->m_height / s;
    this->depth[i].resize(this->rows_i, this->cols_i);
    this->depth_inter[i].resize(this->rows_i, this->cols_i);
    this->depth_old[i].resize(this->rows_i, this->cols_i);
    this->depth[i].fill(0.0f);
    this->depth_old[i].fill(0.0f);
    this->xx[i].resize(this->rows_i, this->cols_i);
    this->xx_inter[i].resize(this->rows_i, this->cols_i);
    this->xx_old[i].resize(this->rows_i, this->cols_i);
    this->xx[i].fill(0.0f);
    this->xx_old[i].fill(0.0f);
    this->yy[i].resize(this->rows_i, this->cols_i);
    this->yy_inter[i].resize(rows_i, this->cols_i);
    this->yy_old[i].resize(this->rows_i, this->cols_i);
    this->yy[i].fill(0.0f);
    this->yy_old[i].fill(0.0f);
    this->transformations[i].resize(4, 4);

    if (this->cols_i <= this->cols) {
      this->depth_warped[i].resize(this->rows_i, this->cols_i);
      this->xx_warped[i].resize(this->rows_i, this->cols_i);
      this->yy_warped[i].resize(this->rows_i, this->cols_i);
    }
  }

  // Prepare the depth image for the resolution of the images after the downsample
  this->depth_wf.setSize(this->m_height, this->m_width);
}

void CVDifodo::loadInnerConfiguration() {
  // This method returns false if the value couldn be found, but we already have the default values in the constructor
  // so no need to check for it
  bool IS_TUM_DATASET = false;

  if (IS_TUM_DATASET) {
    // FOR TUM use this
    // Realsense D435 = 74.0f ; TUM dataset = 58.5;
    // Realsense D435 = 62.0f ; TUM dataset = 46.6;
    fovh_degrees = 58.5f;
    fovv_degrees = 46.6f;

    // Realsense D435 = 1000 ; TUM dataset = 1 (5000 in their website and also 16bit) for the rosbag it seems 32bit and
    // scale of 1
    depth_pixel_scale = 1;

  } else { // REALSENSE D435
    // Realsense D435 = 74.0f ; TUM dataset = 58.5;
    // Realsense D435 = 62.0f ; TUM dataset = 46.6;
    fovh_degrees = 74.0f;
    fovv_degrees = 62.0f;

    // Realsense D435 = 1000 ; TUM dataset = 1 (5000 in their website and also 16bit) for the rosbag it seems 32bit and
    // scale of 1
    // scale of 1 since this configuration parameter is already managed by SD-SLAM and the image get by difodo is a
    // opencv and already has been scaled to 1.
    depth_pixel_scale = 1;
  }

  rows_orig = 480;
  cols_orig = 640;

  min_depth_value_filter = 0.1f;
  max_depth_value_filter = 7.0f;

  /**
   * The number of times we want to downsample the original resolution.
   * downsample = 1 means that no downsample will be made.
   * Check the original description from definitions in CDifodo.h
   */
  downsample = 4;

  ctf_levels = 5;

  rows_ctf = rows_orig / downsample;
  cols_ctf = cols_orig / downsample;

  // NOTE: I dont know why but if set to true, the algorithm doesnt work...
  fast_pyramid = false;

  ROS_INFO_STREAM(std::endl <<
                            "---------------------------------------------------------" << std::endl <<
                            "         DIFODO CONFIGURATION PARAMETERS LOADED" << std::endl <<
                            "---------------------------------------------------------" << std::endl <<
                            "rows_orig: " << rows_orig << std::endl <<
                            "cols_orig: " << cols_orig << std::endl <<
                            "downsample: " << downsample << std::endl <<
                            "ctf_levels " << ctf_levels << std::endl <<
                            "fovh_degrees: " << fovh_degrees << std::endl <<
                            "fovv_degrees: " << fovv_degrees << std::endl <<
                            "fast_pyramid: " << fast_pyramid << std::endl);

  /******************SET DIFODO ATTRIBUTES VALUES*****************/
  this->fovh = M_PI * fovh_degrees / 180.0;
  this->fovv = M_PI * fovv_degrees / 180.0;

  // Resolution of the depth image that the algorithm will work with. After the downsampling.
  this->m_width = cols_orig / this->downsample;
  this->m_height = rows_orig / this->downsample;

  // Resolution of the finest level of the pyramid. Should be equal or lower than the image working resolution.
  if (rows_orig < rows_ctf) {
    ROS_ERROR("The rows_ctf (%d) cannot be more than those of the working depth image rows_orig/downsample = %d.",
              rows_ctf, this->m_height);
    // Set the maximum allowed by default
    this->rows = rows_orig / downsample;
  } else {
    this->rows = rows_ctf;
  }

  if (cols_orig < cols_ctf) {
    ROS_ERROR("The cols_ctf (%d) cannot be more than those of the working depth image cols_orig/downsample = %d.",
              cols_ctf, this->m_width);
    // Set the maximum allowed by default
    this->cols = cols_orig / downsample;
  } else {
    this->cols = cols_ctf;
  }

  /******************Resize Matrices and adjust parameters*****************/
  // *****Pyramid setup*****
  // Resize each matrix to have the same number of matrices than levels of pyramid
  const unsigned int pyr_levels =
          std::round(log((float) this->m_width / this->cols) / log(2.f)) + this->ctf_levels;
  this->depth.resize(pyr_levels);
  this->depth_old.resize(pyr_levels);
  this->depth_inter.resize(pyr_levels);
  this->depth_warped.resize(pyr_levels);
  this->xx.resize(pyr_levels);
  this->xx_inter.resize(pyr_levels);
  this->xx_old.resize(pyr_levels);
  this->xx_warped.resize(pyr_levels);
  this->yy.resize(pyr_levels);
  this->yy_inter.resize(pyr_levels);
  this->yy_old.resize(pyr_levels);
  this->yy_warped.resize(pyr_levels);
  this->transformations.resize(pyr_levels);

  // Resize each level (or matrix) of the pyramid to the desired (rows,cols) size
  for (unsigned int i = 0; i < pyr_levels; i++) {
    unsigned int s = pow(2.f, int(i));
    this->cols_i = this->m_width / s;
    this->rows_i = this->m_height / s;
    this->depth[i].resize(this->rows_i, this->cols_i);
    this->depth_inter[i].resize(this->rows_i, this->cols_i);
    this->depth_old[i].resize(this->rows_i, this->cols_i);
    this->depth[i].fill(0.0f);
    this->depth_old[i].fill(0.0f);
    this->xx[i].resize(this->rows_i, this->cols_i);
    this->xx_inter[i].resize(this->rows_i, this->cols_i);
    this->xx_old[i].resize(this->rows_i, this->cols_i);
    this->xx[i].fill(0.0f);
    this->xx_old[i].fill(0.0f);
    this->yy[i].resize(this->rows_i, this->cols_i);
    this->yy_inter[i].resize(rows_i, this->cols_i);
    this->yy_old[i].resize(this->rows_i, this->cols_i);
    this->yy[i].fill(0.0f);
    this->yy_old[i].fill(0.0f);
    this->transformations[i].resize(4, 4);

    if (this->cols_i <= this->cols) {
      this->depth_warped[i].resize(this->rows_i, this->cols_i);
      this->xx_warped[i].resize(this->rows_i, this->cols_i);
      this->yy_warped[i].resize(this->rows_i, this->cols_i);
    }
  }

  // Prepare the depth image for the resolution of the images after the downsample
  this->depth_wf.setSize(this->m_height, this->m_width);
}

void CVDifodo::loadFrame(cv::Mat &depth_frame) {
  this->depth_opencv_mat = depth_frame;
  this->loadFrame();
}

void CVDifodo::loadFrame() {
  // Check if resize is needed
  if (this->downsample != 1) {
    double resize_factor = 1.f / this->downsample;
    cv::resize(depth_opencv_mat, depth_opencv_mat, cv::Size(), resize_factor, resize_factor);
  }

  // Load in the depth image so the DIFODO algorithm can used it to process the algorithm
  cvMatToMRPTMat(this->depth_opencv_mat, this->depth_wf);
}

/**
 * There are probably better and FASTER ways to loop over a cv::Mat, see:
 * https://docs.opencv.org/2.4/doc/tutorials/core/how_to_scan_images/how_to_scan_images.html#howtoscanimagesopencv
 * https://answers.opencv.org/question/38/what-is-the-most-effective-way-to-access-cvmat-elements-in-a-loop/
 * https://kezunlin.me/post/61d55ab4/#toc-heading-17 (Best, updated with foreach for new c++ 11)
 * @param cv_ptr
 */
void CVDifodo::cvMatToMRPTMat(const cv::Mat &depthCVMat, mrpt::math::CMatrixFloat &depthMRPTMat) {
  for (uint row = 0; row < rows; row++) {
    for (uint col = 0; col < cols; col++) {
      //Esto hace una copia seguramente, habra que mirar las formas mas eficientes y en cualquier caso evitar la
      // copia sino obtener un puntero a la posicion de memoria u algo asi.
      // NOTE: The depth image from realsense comes in milimeters with 4 digits. 3215 will be 3.215 meteres
      // DIFODO uses meters in double precision.
      double depth_pixel_value = 0.0f;

      // 32 bit depth image since the conversion is made on the GrabRGBD image.
      depth_pixel_value = (1.0f / depth_pixel_scale) * depthCVMat.at<float>(row, col);

      // FILTER DEPTH VALUES
      if (min_depth_value_filter <= depth_pixel_value && depth_pixel_value <=
                                                         max_depth_value_filter) {  // Represents expected pre-REP logic and is the only necessary condition for most applications.
        // This is a valid measurement.
        depthMRPTMat(row, col) = depth_pixel_value;
      } else if (!isfinite(depth_pixel_value) && depth_pixel_value < 0) {
        // Object too close to measure.
        depthMRPTMat(row, col) = 0.0f;
      } else if (!isfinite(depth_pixel_value) && depth_pixel_value > 0) {
        // No objects detected in range.
        depthMRPTMat(row, col) = 0.0f;
      } else if (isnan(depth_pixel_value)) {
        // This is an erroneous, invalid, or missing measurement.
        depthMRPTMat(row, col) = 0.0f;
      } else {
        // The sensor reported these measurements as valid, but they are discarded per the limits defined by minimum_range and maximum_range.
        depthMRPTMat(row, col) = 0.0f;
      }

      // TODO: Crear una funcion de filtering o hacer aqui un filtrado de la imagen de profundidad en funcion de
      //  distancia minima y maxima (Setear esos valores a 0) Mirar como lo hace en los ejemplos de
      //  DifOdometry-camera y DifOdometr_Datasets
      //  Check limit values based on resolution of the depth images for the realsense. Ill create a realsense
      //  mode where if we are using the realsense D435 we know exactly which values to use, or maybe just add
      //  these tow limit values as configuration parameters. (Better to have a complex configuration file but
      //  well explain that having everythin on code.
      //  https://www.intel.com/content/dam/support/us/en/documents/emerging-technologies/intel-realsense-technology/Intel-RealSense-D400-Series-Datasheet.pdf
    }
  }
}

mrpt::poses::CPose3D CVDifodo::getDisplacementPoseInSDSLAMCoords() {
  // In the documentation roll is Z and yaw is X -> https://docs.mrpt.org/reference/devel/classmrpt_1_1poses_1_1_c_pose3_d.html#add560401daf3cf79cff8645c7cf0f62e
  // BUT in reality X == roll, Y == Pitch, Z == yaw in this case
  mrpt::poses::CPose3D dispInSDSLAM(this->cam_pose_displacement.y(),
                                    this->cam_pose_displacement.z(),
                                    this->cam_pose_displacement.x(),
                                    this->cam_pose_displacement.roll(),
                                    this->cam_pose_displacement.yaw(),
                                    this->cam_pose_displacement.pitch());

  return dispInSDSLAM;
}

void CVDifodo::execute_iteration() {
  // Estimation of the new pose
  this->odometryCalculation();

  // Get displacement or movement between poses, referenced to the cam_oldpose coordinate reference system
  cam_pose_displacement = this->cam_pose - this->cam_oldpose;
  ROS_DEBUG_STREAM("Displacement estimation from DIFODO" << cam_pose_displacement);
}