
#include "landing.h"
#include "std_msgs/Bool.h"

using namespace cv;
using namespace std;
using namespace cv::ximgproc;
using namespace std::chrono;

ros::Publisher landing_pub;
ros::Publisher landing_allowed_pub;
std_msgs::Bool allowed;
bool FLAG = false;
int countz = 0;
bool is_hover;
ros::Subscriber hoverSub;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudPtr;
typedef pcl::PointCloud<pcl::PointXYZ> pointCloud;

int image_counter = 0;
time_t now= time(0);
tm *ltm = localtime(&now);

pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pointCloud);

class LandingNode {
public:
    LandingNode (ros::NodeHandle &nh);
    ~LandingNode ();
    void proc_callback (const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight);

private:
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Subscriber<sensor_msgs::Image> *sub_left;
    message_filters::Subscriber<sensor_msgs::Image> *sub_right;   
    message_filters::Synchronizer<sync_pol> *sync_;
   
};

LandingNode::~LandingNode () {
    delete sub_left;
    delete sub_right;
    delete sync_;

}

void spinthread() {
    ros::spin();	
}

LandingNode::LandingNode( ros::NodeHandle &nh) {
    sub_left = new message_filters::Subscriber<sensor_msgs::Image> (nh, "/guidance/left/image_raw", 10);
    sub_right = new message_filters::Subscriber<sensor_msgs::Image> (nh, "/guidance/right/image_raw", 10);

    sync_ = new message_filters::Synchronizer<sync_pol> (sync_pol(10), *sub_left, *sub_right);

    sync_->registerCallback(boost::bind(&LandingNode::proc_callback, this, _1, _2));

    landing_pub = nh.advertise<sensor_msgs::PointCloud2>("landing_point", 10);
    landing_allowed_pub = nh.advertise<std_msgs::Bool>("landing_allowed", 10);
    boost::thread t = boost::thread(boost::bind(&spinthread));
}

void hover_callback(const std_msgs::Bool::ConstPtr& msg){

    is_hover = msg->data;
    
}


void LandingNode::proc_callback(const sensor_msgs::ImageConstPtr& msgLeft, const sensor_msgs::ImageConstPtr& msgRight) {

 //ROS_INFO("BEFORE is hover %d",is_hover);
 if(is_hover){
    cv_bridge::CvImagePtr img_ptr_right;
    try
    {
        img_ptr_right = cv_bridge::toCvCopy(msgRight, sensor_msgs::image_encodings::TYPE_8UC1); //CV_16SC1

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msgRight->encoding.c_str());
    }


    cv_bridge::CvImagePtr img_ptr_left;
    try
    {
        img_ptr_left = cv_bridge::toCvCopy(msgLeft, sensor_msgs::image_encodings::TYPE_8UC1); //CV_16SC1

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msgLeft->encoding.c_str());
    }

//
    leftImage = img_ptr_left->image;
    rightImage = img_ptr_right->image;

std::ostringstream image_distance;
            image_distance << "/home/ubuntu/Desktop/Drone_stereo_Images/images_left/Scene"<<1+ ltm->tm_min << 1+ ltm->tm_sec<<image_counter<<"_left.jpg";
            std::string copyOfStr = image_distance.str();

            imwrite(copyOfStr, leftImage );


            std::ostringstream image_distance2;
            image_distance2 << "/home/ubuntu/Desktop/Drone_stereo_Images/images_right/Scene"<<1+ ltm->tm_min << 1+ ltm->tm_sec<<image_counter<<"_right.jpg";
            std::string copyOfStr2 = image_distance2.str();

            imwrite(copyOfStr2, rightImage );

      image_counter ++;

//    imshow("left_image", leftImage);
//    imshow("right_image", rightImage);
 //   cv::waitKey(30);

    if (leftImage.empty() || rightImage.empty()) {
        cout << "Image not Loaded: Incorrect filepath" << endl;
        system("pause");
    }

    /// matrices initializetion
    cv::Mat disparity = Mat::zeros(240, 320, CV_8U);
    cv::Mat disparity_vis;
    cv::Mat DepthConfidence = Mat::zeros(240, 320, CV_32F);
    cv::Mat Flatness_Information_matrix = Mat::zeros(240, 320, CV_32F);
    cv::Mat Nearest_edge_pixel_index_matrix = Mat::zeros(240, 320, CV_32SC1);
    cv::Mat TotalScore = Mat::zeros(240, 320, CV_32F);
    cv::Mat Steepness_matrix = Mat::zeros(240, 320, CV_32F);
    cv::Mat DotProduct_matrix = Mat::zeros(240, 320, CV_32F);

    cv::Mat Deviation_matrix = Mat::zeros(240, 320, CV_32F);

    vector<edge_pixels> canny_pixels;
    cout<<"1"<<endl;
    /// run disparity function
    calculateDisparity(disparity, disparity_vis);
    //sleep(10);

cout<<"2"<<endl;
    /// Estimate confidence in Depth information
    //calculateDepthConfidence(disparity, DepthConfidence);
cout<<"3"<<endl;

    /// run frame flatness function
    auto start = high_resolution_clock::now();
    calculatePixelFlatness(disparity_vis, Flatness_Information_matrix, Nearest_edge_pixel_index_matrix,
                           canny_pixels);
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop - start);
    cout << "Pixel flateness time: " << duration.count() << endl;
cout<<"4"<<endl;
    /// make point cloud pointer
    pointCloudPtr Point_Cloud(new pointCloud);

    /// run point cloud function
    calculatePointCloud(disparity, Point_Cloud);
cout<<"5"<<endl;
    // sleep(10);

    /// remove_point cloud outliers
    //remove_outliers(Point_Cloud);
cout<<"6"<<endl;
    calculateSteepness(Point_Cloud, Steepness_matrix, DotProduct_matrix);
    //cv::imshow("Steepness_matrix", Steepness_matrix);
cout<<"7"<<endl;
    calculateDeviation(disparity, Deviation_matrix);

    ///Define score weights
    float w1 = 0.0;
    float w2 = 0.275;
    float w3 = 0.45;
    float w4 = 0.275;

    /// add Matrices to take final score
   // cv::add(w1*DepthConfidence, w2*Flatness_Information_matrix, TotalScore);

    cv::add(w3*Steepness_matrix, w2*Flatness_Information_matrix, TotalScore);

    //cv::add(w3*Steepness_matrix, TotalScore, TotalScore);

    cv::add(w4*Deviation_matrix, TotalScore, TotalScore);

    /// find max in total score

    //double minVal, maxVal;
    Point minLoc, maxLoc;

    minMaxLoc(TotalScore, nullptr , nullptr, &minLoc, &maxLoc);
 

    //landing_pub.publish(output);

    // update if new landing site is suitable sor landing	
    landing_is_allowed = isLandingAllowed(maxLoc, Point_Cloud, Flatness_Information_matrix,DotProduct_matrix,
                                    Nearest_edge_pixel_index_matrix, canny_pixels);

    //landing_is_allowed = false ;
    //if(countz>-1) landing_is_allowed = true ;
    //countz++;

    cout << "Landing Allowed: "<< landing_is_allowed <<endl;
    /// Publish point
    if(landing_is_allowed){
	int pc_landing_index2 = (maxLoc.y) * (WIDTH - max_disp) + (maxLoc.x - max_disp);

	output->header.frame_id = "my_frame";
	output->height = output->width = 1;
	output->points.emplace_back(
	    pcl::PointXYZ(Point_Cloud->points.at(pc_landing_index2).x, Point_Cloud->points.at(pc_landing_index2).y,
		          Point_Cloud->points.at(pc_landing_index2).z));
    }
    /// ROS Message update
    allowed.data = landing_is_allowed;

    //sleep for a sec to take the right hover msg
    ros::Duration(1.5).sleep();
 }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "Landing_algo");
    ros::start();

    ros::NodeHandle nh;
    hoverSub =  nh.subscribe("/is_hover", 10, &hover_callback);
    LandingNode Node(nh );

    //allowed.data = false;

    ros::Rate loop_rate(10); // 10 hz
    while (ros::ok())
    {
       // ros::spinOnce();

       if(landing_is_allowed) landing_pub.publish(output);
       if(FLAG) {
	landing_allowed_pub.publish(allowed);
	FLAG = false;
       }
       loop_rate.sleep();
    }
}


int calculateDisparity(cv::Mat& filtered_disp, cv::Mat& filtered_disp_vis){

    int wsize = 15; //default window size for BM on full-sized views

    //Stereo Matrices
    Mat left = leftImage;
    Mat right = rightImage;

    Mat left_for_matcher, right_for_matcher;
    Mat left_disp,right_disp;

    filtered_disp = Mat::zeros(HEIGHT, WIDTH, CV_16S);

    Mat solved_disp,solved_filtered_disp;
    Mat conf_map = Mat(left.rows,left.cols,CV_8U);
    conf_map = Scalar(255);
    Rect ROI;
    Ptr<DisparityWLSFilter> wls_filter;
    double matching_time, filtering_time;

    if(max_disp<=0 || max_disp%16!=0)
    {
        cout<<"Incorrect max_disparity value: it should be positive and divisible by 16";
        return -1;
    }
    if(wsize<=0 || wsize%2!=1)
    {
        cout<<"Incorrect window_size value: it should be positive and odd";
        return -1;
    }

    if(filter=="wls_conf") // filtering with confidence (significantly better quality than wls_no_conf)
    {
        if (!no_downscale) {
            // downscale the views to speed-up the matching stage, as we will need to compute both left
            // and right disparity maps for confidence map computation
            //! [downscale]
            max_disp /= 2;
            if (max_disp % 16 != 0)
                max_disp += 16 - (max_disp % 16);
            resize(left, left_for_matcher, Size(), 0.5, 0.5, CV_INTER_LINEAR);
            resize(right, right_for_matcher, Size(), 0.5, 0.5, CV_INTER_LINEAR);
            //! [downscale]
        } else {
            left_for_matcher = left.clone();
            right_for_matcher = right.clone();
        }

        if(algo=="bm")
        {
            //! [matching]
            Ptr<StereoBM> left_matcher = StereoBM::create(max_disp,wsize);
            wls_filter = createDisparityWLSFilter(left_matcher);
            Ptr<StereoMatcher> right_matcher = createRightMatcher(left_matcher);

            //cvtColor(left_for_matcher,  left_for_matcher,  COLOR_BGR2GRAY);
            //cvtColor(right_for_matcher, right_for_matcher, COLOR_BGR2GRAY);

            matching_time = (double)getTickCount();
            left_matcher-> compute(left_for_matcher, right_for_matcher,left_disp);
            right_matcher->compute(right_for_matcher,left_for_matcher, right_disp);
            matching_time = ((double)getTickCount() - matching_time)/getTickFrequency();
            //! [matching]
        }
        else if(algo=="sgbm")
        {
            Ptr<StereoSGBM> left_matcher  = StereoSGBM::create(0,max_disp,wsize);
            left_matcher->setP1(24*wsize*wsize);
            left_matcher->setP2(96*wsize*wsize);
            left_matcher->setPreFilterCap(63);
            left_matcher->setMode(StereoSGBM::MODE_SGBM_3WAY);
            wls_filter = createDisparityWLSFilter(left_matcher);
            Ptr<StereoMatcher> right_matcher = createRightMatcher(left_matcher);

            matching_time = (double)getTickCount();
            left_matcher-> compute(left_for_matcher, right_for_matcher,left_disp);
            right_matcher->compute(right_for_matcher,left_for_matcher, right_disp);
            matching_time = ((double)getTickCount() - matching_time)/getTickFrequency();
        }
        else
        {
            cout<<"Unsupported algorithm";
            return -1;
        }
    }

    //! [filtering_wls]
    wls_filter->setLambda(lambda);
    wls_filter->setSigmaColor(sigma);
    filtering_time = (double)getTickCount();
    wls_filter->filter(left_disp,left,filtered_disp,right_disp);
    filtering_time = ((double)getTickCount() - filtering_time)/getTickFrequency();
    //! [filtering_wls]

    conf_map = wls_filter->getConfidenceMap();

    Mat left_disp_resized;
    resize(left_disp,left_disp_resized,left.size());

    // Get the ROI that was used in the last filter call:
    ROI = wls_filter->getROI();
    if(!no_downscale)
    {
        // upscale raw disparity and ROI back for a proper comparison:
        resize(left_disp,left_disp,Size(),2.0,2.0);
        left_disp = left_disp*2.0;
        //  left_disp_resized = left_disp_resized*2.0;
        ROI = Rect(ROI.x*2,ROI.y*2,ROI.width*2,ROI.height*2);
    }
    //! [visualization]
    Mat raw_disp_vis = Mat::zeros(HEIGHT, WIDTH, CV_8U);
    getDisparityVis(left_disp,raw_disp_vis,vis_mult);
    //namedWindow("raw disparity", WINDOW_AUTOSIZE);
    //imshow("raw disparity", raw_disp_vis);

    ///// write for storage
    //imwrite( storage_dir + "raw_disparity/Image12.jpg", raw_disp_vis );

    filtered_disp_vis = Mat::zeros(HEIGHT, WIDTH, CV_8U);
    getDisparityVis(filtered_disp,filtered_disp_vis,vis_mult);
    //namedWindow("filtered disparity", WINDOW_AUTOSIZE);
    //imshow("filtered disparity", filtered_disp_vis);
    return 0;
}

void calculateDepthConfidence(const cv::Mat& disparity, cv::Mat& DepthConfidence)
{
    double min, max;
    Mat matrix(240,320,CV_32F);

    for (int i =0 ; i < disparity.rows; i++ )
        for (int j = max_disp ; j < disparity.cols; j++ )

            matrix.at<float>(i,j) = powf(disparity.at<short>(i,j),2.0);

    cv::minMaxLoc(matrix, &min, &max);

     for(int i = 7; i < HEIGHT-7; i++) {
        for (int j = max_disp+7; j < WIDTH-7; j++) {
            if(disparity.at<short>(i,j)<0) DepthConfidence.at<float>(i,j)=0;
            else DepthConfidence.at<float>(i,j) = (1.0 + ((matrix.at<float>(i,j)-powf(max,2.0 ))));
            //cout << "Depth conf : "<< DepthConfidence.at<float>(i,j)<<endl;
        }
    }

    /// Normalize between range [0 - 1]
    double minVal, maxVal;

    minMaxLoc(DepthConfidence, &minVal, &maxVal,nullptr,nullptr );

    for(int i = 7; i < HEIGHT-7; i++) {
        for (int j = max_disp+7; j < WIDTH-7; j++) {
            DepthConfidence.at<float>(i, j) = (DepthConfidence.at<float>(i, j) - minVal)/ (maxVal - minVal);
            //cout << DepthConfidence.at<float>(i, j) << endl;
        }
    }


}

void calculatePixelFlatness(const cv::Mat& disparity, cv::Mat& Flatness_Information_matrix , cv::Mat& Nearest_edge_pixel_index_matrix, vector<edge_pixels>& canny_pixels){

    ///////canny parameters
    Mat CannyEdges, detected_edges;
    const int ratio = 11;
    int lowThreshold = 100;
    //const int max_lowThreshold = 200;
    const int kernel_size = 7;
    const char* window_name = "Edge Map";

    /// euclidean distance
    //detected_edges = disparity ;
    blur(disparity, detected_edges, Size(3,3) );
    Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );
    CannyEdges = Scalar::all(0);
    disparity.copyTo( CannyEdges, detected_edges);
    //imshow( window_name, CannyEdges );

    //imwrite( storage_dir + "Canny_edges/Image18.jpg", CannyEdges );
    // Collect none zero pixels in a structure

    int k  = 0;
    for(int i = 0; i < HEIGHT; i++) {
	   canny_pixels.push_back(edge_pixels());
           canny_pixels[k].row = i;
           canny_pixels[k].col = WIDTH-1;
           k++;
    }
   for(int i = 0; i < HEIGHT; i++) {
	   canny_pixels.push_back(edge_pixels());
           canny_pixels[k].row = i;
           canny_pixels[k].col = max_disp;
           k++;
    }
    for(int i = 0; i < WIDTH; i++) {
	   canny_pixels.push_back(edge_pixels());
           canny_pixels[k].row = 0;
           canny_pixels[k].col = i;
           k++;
   }
   for(int i = 0; i < WIDTH; i++) {
	   canny_pixels.push_back(edge_pixels());
           canny_pixels[k].row = HEIGHT-1;
           canny_pixels[k].col = i;
           k++;
    }

    for(int i = 1; i < HEIGHT-1; i++) {
        for (int j = max_disp+1; j < WIDTH-1; j++) {

            if((static_cast<int>(CannyEdges.at<uchar>(i, j)) !=0)  ){
                canny_pixels.push_back(edge_pixels());
                canny_pixels[k].row = i;
                canny_pixels[k].col = j;
                k++;
            }
            //   cout<< (static_cast<int>(dst.at<uchar>(i,j))) <<endl;
        }
    }
    // Estimate flatness information matrix

    float euc_dist;
    //vector<float> distances;

    float min_dist ;
    int min_index;
    int count = 0;
    int minElementIndex;
    for(int i = 0; i < CannyEdges.rows; i++) {
        for (int j = max_disp; j < CannyEdges.cols; j++) {

          
             if((static_cast<int>(CannyEdges.at<uchar>(i, j)) ==0)){
                min_dist = 100000.0;
                for (int k = 0; k < canny_pixels.size(); k+=4) {
                    if((abs(canny_pixels[k].row-i)+abs(canny_pixels[k].col-j))> min_dist+15) continue;
                    Point2f a(canny_pixels[k].row, canny_pixels[k].col); 
		    Point2f b(i,j);
                    euc_dist = cv::norm(a-b);
                    if (euc_dist < min_dist) {

                        min_dist = euc_dist;
                        min_index = k;
                    }
                    //distances.push_back(euc_dist);
                }

                // min_dist = *min_element(distances.begin(), distances.end());
                // minElementIndex = std::min_element(distances.begin(), distances.end()) - distances.begin();
                Nearest_edge_pixel_index_matrix.at<int>(i, j) = min_index;
                Flatness_Information_matrix.at<float>(i, j) = min_dist;
                //  Flatness_Information_matrix[i][j] = min_dist;
            }
            count++;
            //cout << count <<"\t "<< Flatness_Information_matrix.at<float>(i,j) << endl;
        }
    }


    /// Normalize between range [0 - 1]
    double minVal, maxVal;

    minMaxLoc(Flatness_Information_matrix, &minVal, &maxVal, nullptr, nullptr );

    for(int i = 0; i < HEIGHT; i++) {
        for (int j = max_disp; j < WIDTH; j++) {
            Flatness_Information_matrix.at<float>(i, j) = (Flatness_Information_matrix.at<float>(i, j) - minVal)/ (maxVal - minVal);
        }
    }
    // clean distance vector
    // distances.erase(distances.begin(),distances.end());
}

void calculatePointCloud(const cv::Mat& disparity, pcl::PointCloud<pcl::PointXYZ>::Ptr& Cloud) {

//    //////////// constant defined by the dataset creator
//    double constant = 200.0f;
//    double MM_PER_M = 1000.0f;
//
 

    ///store pointcloud file
    //ofstream myfile;
    //myfile.open(storage_dir + "/Pointcloud/pointcloud.txt");

    float x, y, z;
    //////////////////////////////// new

    /// units in mm
    double factor = 6.7431;
    double factor2 = 6.3655;
    double mm2metres = 1000.0;
    double fx = 246.927; //mm
    double fy = 246.927;
    double cx = 162.796;// 425.807709;
    double cy = 123.928;// 395.494293;
    double b =  15;

    int index = 0;

    for (int v = 0; v < HEIGHT; v++) {
        for (int u = max_disp; u < WIDTH; u++) {

            double dis = disparity.at<short>(v, u);

            double depth = ((fx * b) /(dis*factor2));

            z = depth;
            x = (u - cx) * z / fx;
            y = (v - cy) * z / fy;

            Cloud->points.emplace_back(pcl::PointXYZ(x, y, z));

            index ++;
        }
    }

    //cout << "PointCloud size: " << Cloud->points.size() << endl;

}

void remove_outliers(pointCloudPtr&  cloud){
    /// Remove outliers from point cloud

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_outliers (new pcl::PointCloud<pcl::PointXYZ>);

    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor(true) ;

    // Count time needed for ouliers removal
    //auto start = high_resolution_clock::now();

    sor.setInputCloud (cloud); // or msg
    sor.setMeanK (8);
    sor.setStddevMulThresh (1.0);
    sor.filter (*cloud_filtered);

    //auto stop = high_resolution_clock::now();
    //auto duration = duration_cast<milliseconds>(stop - start);

    //std::cout << "Filtering time: " << duration.count() << std::endl;

    std::cerr << "Cloud after filtering: " << std::endl;
    std::cerr << cloud_filtered->points.size() << std::endl;

    /// store to pcd file
    cloud_filtered->width = 1;
    cloud_filtered->height = cloud_filtered->points.size();
    //pcl::io::savePCDFileASCII("LandingPC.pcd", *cloud_filtered);
    //std::cerr << "Saved " << cloud_filtered->points.size () << " data points to test_pcd.pcd." << std::endl;
}

void calculateSteepness(pointCloudPtr&  cloud, cv::Mat& Steepness_matrix ,cv::Mat& DotProduct_matrix ){

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);

    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

    // Use all neighbors in a sphere
    // e of radius 3cm (0.03)
   // ne.setRadiusSearch (0.5);
    ne.setKSearch (60);
    // Compute the features
    ne.compute (*cloud_normals);

    // cloud_normals->points.size () should have the same size as the input cloud->points.size ()*
    pcl::PointCloud<pcl::Normal> Normals = *cloud_normals;

    // calculate normals dot product with Z axis and each pixel's steepness score
    pcl::Normal Znormal(0.0f,0.0f,-1.0f); //////// To dot vgainei arnitiko me meion sto z paei thetiko

    float theta;
    float dotProduct;
    float n;

    std::vector<float> Steepness;
    std::vector<float> DotProducts;
    float th = 15.0f;

    /// store normals
//    ofstream normals_file;
//    normals_file.open(storage_dir + "/Pointcloud/normals.txt");
//    normals_file << "x\t" << "y\t" << "z\t" <<endl;
    for (auto normal : Normals){
        //normals_file << fixed << setprecision(4) << normal.normal_x <<"\t" << normal.normal_y <<"\t"<< normal.normal_z <<endl;

        dotProduct = normal.normal_x * Znormal.normal_x + normal.normal_y * Znormal.normal_y + normal.normal_z * Znormal.normal_z;

        DotProducts.push_back(dotProduct);
        //cout << "DOT : "<<dotProduct<< endl;
        theta =  acos(dotProduct)*180/M_PI;
        //cout <<"theta :"<<theta << endl;
        n = exp(-(pow(theta,2)/ (2*pow(th,2))));
        Steepness.push_back(n);
    }
    //normals_file.close();
    //printf("Normals size: %d \n",Normals.size());

    //printf("steepness score: %f \n", Steepness.at(0));
    //printf("Steepness Score size : %d \n",Steepness.size());

    /// Display for check purpose
    //printf( "pixel normal at 0 : %f, %f, %f \n", Normals.at(0).normal_x,  Normals.at(0).normal_y,  Normals.at(0).normal_z);
    float dot = Normals.at(0).normal_x * Znormal.normal_x + Normals.at(0).normal_y * Znormal.normal_y + Normals.at(0).normal_z * Znormal.normal_z;
    //printf ("dot product at 0 : %f \n", dot);

    /// Make steepness matrix

    int indice = 0;
    for (int i = 0; i < HEIGHT; i++) {                
        for (int j = max_disp; j < WIDTH; j++) {

            //cout << "indice No : "<< indice <<endl;
            if (isnan(Steepness.at(indice))) Steepness_matrix.at<float>(i,j) = 0.0f; ///
            else Steepness_matrix.at<float>(i,j) =  Steepness.at(indice) ;

	    DotProduct_matrix.at<float>(i,j) = DotProducts.at(indice);	
            indice ++;
        }
    }

  /// Normalize between range [0 - 1]
    double minVal, maxVal;

    minMaxLoc(Steepness_matrix, &minVal, &maxVal, nullptr, nullptr );

    for(int i = 0; i < HEIGHT; i++) {
        for (int j = max_disp; j < WIDTH; j++) {
            Steepness_matrix.at<float>(i, j) = (Steepness_matrix.at<float>(i, j) - minVal)/ (maxVal - minVal);
        }
    }
   
    // remove all point from point cloud
    //cloud.clear();

}

bool isLandingAllowed (Point maxLoc, pointCloudPtr&  cloud, cv::Mat& Flatness_Information_matrix ,cv::Mat& DotProduct_matrix, cv::Mat& Nearest_edge_pixel_index_matrix, vector<edge_pixels>& canny_pixels){

    float UAV_size = 0.6f;

    /// find the closest edge to landing point pixel
    int closest_edge_index =  Nearest_edge_pixel_index_matrix.at<int>(maxLoc.y,maxLoc.x);

    int pc_edge_index = (canny_pixels[closest_edge_index].row)*(WIDTH-max_disp)+(canny_pixels[closest_edge_index].col-max_disp);

    /// world coordinates of landing point
    int pc_landing_index = (maxLoc.y)*(WIDTH-max_disp)+(maxLoc.x-max_disp);

    ///edge point
    Point2f a( cloud->points.at(pc_edge_index).x, cloud->points.at(pc_edge_index).y);
    ///landing point
    Point2f b(cloud->points.at(pc_landing_index).x,cloud->points.at(pc_landing_index).y);
    double dist = cv::norm(a-b);
    //cout<< "Distance to nearest depth change point : "<<  dist <<endl;

   
    //cout<< "POINT : "<<"x "<< cloud->points.at(pc_landing_index).x <<"y " << cloud->points.at(pc_landing_index).y<<"z "<<cloud->points.at(pc_landing_index).z<<endl;

    float lp_dot = DotProduct_matrix.at<float>(maxLoc.y, maxLoc.x);

    FLAG = true;

    if ((dist > 1.3*UAV_size)&&(lp_dot > 0.966)) return true;
    else return false;
}

void calculateDeviation(const cv::Mat& disparity, cv::Mat& Deviation_matrix) {
    int win = 5;
    int ww = win/2;
    vector<float> vec;
    Scalar mean, stddev;
    for (int v = 9; v < HEIGHT-9; v++) {
        for (int u = max_disp+9; u < WIDTH-9; u++) {
            for (int p = -ww; p < ww+1; p++) {

                vec.emplace_back(disparity.at<short>(v + p, u + p));
            }
            meanStdDev(vec, mean, stddev);
            Deviation_matrix.at<float>(v,u) = exp(-stddev[0]);
            vec.erase(vec.begin(), vec.end());
        }
    }

    /// Normalize between range [0 - 1]
    double minVal, maxVal;
    minMaxLoc(Deviation_matrix, &minVal, &maxVal,nullptr,nullptr );

    for(int i = ww; i < HEIGHT-ww; i++) {
        for (int j = max_disp+ww; j < WIDTH-ww; j++) {
            Deviation_matrix.at<float>(i, j) = (Deviation_matrix.at<float>(i, j) - minVal)/ (maxVal - minVal);
            //cout << Flatness_Information_matrix.at<float>(i, j) << endl;
        }
    }
}
