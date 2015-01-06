#include<ros/ros.h>
#include<ros/package.h>
#include<message_filters/subscriber.h>
#include<message_filters/synchronizer.h>
#include<message_filters/sync_policies/approximate_time.h>

#include<sensor_msgs/Image.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/image_encodings.h>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<stdlib.h>
#include<sensor_msgs/PointCloud2.h>
#include<pcl/conversions.h>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl/PCLPointCloud2.h>
#include<pcl_conversions/pcl_conversions.h>
#include<pcl/io/pcd_io.h>
#include<visualization_msgs/Marker.h>

#include"geometry_msgs/PoseWithCovarianceStamped.h"
#include"geometry_msgs/PoseStamped.h"
#include"geometry_msgs/Pose2D.h"
#include<geometry_msgs/PoseWithCovarianceStamped.h>
#include<geometry_msgs/PoseStamped.h>
#include<geometry_msgs/Pose2D.h>
#include<tf/transform_broadcaster.h>
#include "tf_conversions/tf_eigen.h"

using namespace sensor_msgs;
using namespace message_filters;
using namespace cv;
using namespace std;
namespace enc = sensor_msgs::image_encodings;
static const char WINDOW[] = "Image Window";

class HumanDetection{

	public:
		HumanDetection();

		//target rectangle center cordinates;
		float rect_center_x, rect_center_y;

		//kinect_pose variables;
		float kinect_pose_x,kinect_pose_y,kinect_pose_z;

		//lower and upper limit for HSV;
		int hue_1,hue_2,saturation_1,saturation_2,value_1,value_2;

		//amcl_pose variable ?
		tf::Transform robo_world;
		tf::Transform camera_robo;
		tf::Transform person_camera;
		tf::Transform final_transform;
		void applyTransform();
		//final pose wrt world
		geometry_msgs::Pose2D msg2;



	private:
		//ros nodehandle
		ros::NodeHandle nh_;

		//subscribe to get kinect depth and rgb image
		ros::Subscriber rgb_image;
		ros::Subscriber depth_image;
		ros::Subscriber get_amcl_pose;

		//final pose wrt world
		ros::Publisher final_pose;


		//function to publish person's pose w.r.t kinect
		void imageCallback_(const ImageConstPtr& rgb_img );
		void depthCallback_(const PointCloud2ConstPtr& depth_img );
		void amclCallback_(const geometry_msgs::PoseWithCovarianceStampedConstPtr& data_in );
		//function to transform pose in kinect's frame to world frame
		void poseCallback_();

		ros::Time begin_ ;

};


HumanDetection::HumanDetection(){


	hue_1=100;hue_2=150;saturation_1=80;saturation_2=220;value_1=30;value_2=130;

	//synchronize here for rgb and depth images ?

	rgb_image = nh_.subscribe("/camera/rgb/image_color", 20, &HumanDetection::imageCallback_, this);
	depth_image = nh_.subscribe("/camera/depth/points", 20, &HumanDetection::depthCallback_, this);
	get_amcl_pose =nh_.subscribe("/amcl_pose", 20, &HumanDetection::amclCallback_, this);

	final_pose = nh_.advertise<geometry_msgs::Pose2D>("final_pose",5);



	//	message_filters::Subscriber<Image> rgb_image(nh_, "/camera/rgb/image_color", 1);
	//	message_filters::Subscriber<sensor_msgs::PointCloud2> depth_image(nh_,"/camera/depth/points", 1);
	//	message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> pose_sub(nh,"/amcl_pose",10);

	//	typedef sync_policies::ApproximateTime<Image,sensor_msgs::PointCloud2> MySyncPolicy;
	//	cout<<"1"<<endl;
	//Published Topics
	//	pub=nh.advertise<geometry_msgs::Pose2D>("person_pose", 10);
	//	marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	//	final_pose=nh.advertise<geometry_msgs::Pose2D>("final_pose",10);
	//	cout<<"2"<<endl;
	//	Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), rgb_image,depth_image);
	//	sync.registerCallback(boost::bind(&HumanDetection::imageCallback_, _1, _2 ,this));

}

int main(int argc,char **argv){

	//set hsv lower n upper values from argc n argv here
	ros::init(argc,argv,"human_detection");
	HumanDetection human_detection;
	//	ros::spinOnce();
	while(ros::ok())
	{
		//publisher
	human_detection.applyTransform();

		ros::spinOnce();
	}



}


	

void HumanDetection::applyTransform(){

	camera_robo.setOrigin( tf::Vector3(0.0,0.25,0.0) );
	camera_robo.setRotation( tf::Quaternion(0,0,0,1) );

	person_camera.setOrigin( tf::Vector3(kinect_pose_x,kinect_pose_y,kinect_pose_z) );
	person_camera.setRotation( tf::Quaternion(0,0,0,1) );

	final_transform = robo_world * camera_robo;
	final_transform = final_transform * person_camera ;

	tf::Vector3 person_posev;
	person_posev = final_transform.getOrigin();

	geometry_msgs::Point person_pose;
	person_pose.x = person_posev[0];
	person_pose.y = person_posev[1];
	person_pose.z = person_posev[2];
	/////////////////////////

	msg2.x=person_pose.x;	//Put the modified x-cordinate here
	msg2.y=person_pose.z;	//Put modifed z-cordinate here
//	msg2.theta=timenow;	//Time
	msg2.theta=0;
	cout<<msg2.x<<"\t"<<msg2.y<<"\n"<<endl;
	final_pose.publish(msg2);

}

void HumanDetection::amclCallback_(const geometry_msgs::PoseWithCovarianceStampedConstPtr& data_in){

	robo_world.setOrigin( tf::Vector3(data_in->pose.pose.position.x, data_in->pose.pose.position.y,data_in->pose.pose.position.z ));

	robo_world.setRotation( tf::Quaternion(data_in->pose.pose.orientation.x,data_in->pose.pose.orientation.y,data_in->pose.pose.orientation.z,data_in->pose.pose.orientation.w ));

}

void HumanDetection::depthCallback_( const PointCloud2ConstPtr& depth_img){

	//converting from poincloud to pointcloud2
	pcl::PCLPointCloud2 pcl_pc;
	pcl_conversions::toPCL(*depth_img,pcl_pc);
	pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::fromPCLPointCloud2(pcl_pc, cloud);

	float xcor,ycor,zcor; //These are the x,y,z cordinates of that pixel in 3-D
	//	int ll;
	//	ofstream outfile;

	//getting the x,y,z of the center from pointcloud
	int pp = rect_center_x + ((rect_center_y)*(cloud.width)); 
	zcor = (float)cloud.points[pp].z;
	ycor = (float)cloud.points[pp].y;
	xcor = (float)cloud.points[pp].x;
	//	ros::Time begin = ros::Time::now();
	//	ros::Duration current = begin-initial;
	//	float timenow=current.toSec();
//	cout<<xcor<<"\t"<<ycor<<"\t"<<zcor<<"\t"<<"\t"<<endl;
	//	outfile.open(tstamp,std::ios_base::app);
	//	if( (pcl_isfinite(xcor)) || (pcl_isfinite(xcor)) )
	//		outfile<<xcor<<","<<zcor<<","<<current<<endl;

	kinect_pose_x = xcor;
	kinect_pose_y = ycor;
	kinect_pose_z = zcor;
}


void HumanDetection::imageCallback_(const ImageConstPtr& rgb_img){
	//reading rgb image from subscriber 
	cv_bridge::CvImagePtr cv_ptr;
	namedWindow(WINDOW);
	try{
		cv_ptr = cv_bridge::toCvCopy(rgb_img, enc::BGR8);
	}
	catch (cv_bridge::Exception& e){
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	//converting image from rgb color space to hsv color space 
	Mat HSV;
	Mat src_gray;
	cvtColor(cv_ptr->image,HSV,CV_BGR2HSV);

	//filtering out a particular color range
	inRange(HSV,Scalar(hue_1,saturation_1,value_1),Scalar(hue_2,saturation_2,value_2),src_gray);

	//TO DO:how to adjust correct threshold values
	imshow(WINDOW,src_gray);
	int thresh = 180;
	int max_thresh = 255;
	RNG rng(12345);

	Mat threshold_output;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

	/// Detect edges using Threshold
	threshold( src_gray, threshold_output, thresh, 255, THRESH_BINARY );
	/// Find contours
	findContours( threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

	/// Find the rotated rectangles and ellipses for each contour
	vector<RotatedRect> minRect( contours.size() );
	vector<RotatedRect> minEllipse( contours.size() );

	for( int i = 0; i < contours.size(); i++ )
	{
		minRect[i] = minAreaRect( Mat(contours[i]) );
	}

	/// Draw contours + rotated rects and find the largest rectangle
	Mat drawing = Mat::zeros( threshold_output.size(), CV_8UC3 );
	double largest_area=0;
	int largest_cont_ind=0;
	for( int i = 0; i< contours.size(); i++ )
	{
		double a=contourArea( contours[i],false);
		if(a>largest_area)
		{
			largest_area=a;
			largest_cont_ind=i;
		}
	}
	Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
	Point2f rect_points[4]; minRect[largest_cont_ind].points( rect_points );

	for( int j = 0; j < 4; j++ )
	{
		line( drawing, rect_points[j], rect_points[(j+1)%4], color, 1, 8 );
	}

	//Following are the centre of largest rectangles found [center of t-shirt of Person Detected] in pixels of rgb image
	int centre_x,centre_y;
	centre_x=(rect_points[0].x+rect_points[2].x)/2;
	centre_y=(rect_points[0].y+rect_points[2].y)/2;

	Point pt;
	pt.x=centre_x;
	pt.y=centre_y;
	line( drawing, pt, pt, color, 1, 8 );
	imshow(WINDOW,src_gray);

	rect_center_x = centre_x;
	rect_center_y = centre_y;


}


