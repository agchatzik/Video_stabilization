#include <stdio.h>
#include <string.h>
#include <chrono>
#include <boost/foreach.hpp>
#include <math.h>
#include <vector>
#include <ctime>
#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/ximgproc.hpp>
#include "opencv2/core/utility.hpp"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Bool.h>

// PCL specific includes
#include <pcl/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/2d/convolution.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/console/parse.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#define WIDTH 320
#define HEIGHT 240
#define IMAGE_SIZE (HEIGHT * WIDTH)

using namespace cv;
using namespace std;
using namespace cv::ximgproc;

std::string stereo_dir = "/home/chatzikalymnios/Desktop/StereoPixelProc/resources/images/";
std::string storage_dir = "/home/chatzikalymnios/Desktop/StereoPixelProc/Storage/";

typedef pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudPtr;
typedef pcl::PointCloud<pcl::PointXYZ> pointCloud;

Ptr<StereoBM> stereo_matcher = StereoBM::create(16);


//Our Block Matching class


////Stereo Matrices
Mat leftImage =  Mat::zeros(240,320,CV_8U);
Mat rightImage =  Mat::zeros(240,320,CV_8U);

//Mat leftImage2 =  Mat::zeros(240,320,CV_8U);
//Mat rightImage2 =  Mat::zeros(240,320,CV_8U);

////////////////
String algo = "bm";
String filter = "wls_conf";

bool no_downscale = true;
bool landing_is_allowed = false;


int max_disp = 48;
double lambda = 8000.0;
double sigma  = 1.5;
double fbs_spatial = 16.0;
double fbs_luma = 8.0;
double fbs_chroma = 8.0;
double fbs_lambda = 128.0;
double vis_mult = 7.0;

struct edge_pixels {
    int row;
    int col;
};

int calculateDisparity( cv::Mat& disparity,cv::Mat& filtered_disp_vis);
void calculatePixelFlatness(const cv::Mat& dst, cv::Mat& Flatness, cv::Mat& Nearest_edge_pixel_index_matrix, vector<edge_pixels>& canny_pixels);
void calculateDepthConfidence(const cv::Mat& disparity, cv::Mat& comfMap);
void calculatePointCloud(const cv::Mat& disparity, pointCloudPtr& Point_Cloud);
void remove_outliers(pointCloudPtr&  cloud);
void calculateSteepness(pointCloudPtr&  cloud, cv::Mat& Steepness_matrix ,cv::Mat& DotProduct_matrix );

bool isLandingAllowed (Point maxLoc, pointCloudPtr&  cloud, cv::Mat& Flatness_Information_matrix ,cv::Mat& DotProduct_matrix, cv::Mat& Nearest_edge_pixel_index_matrix, vector<edge_pixels>& canny_pixels);

void calculateDeviation(const cv::Mat& disparity, cv::Mat& Deviation_map);
void hover_callback(const std_msgs::Bool::ConstPtr& msg);
