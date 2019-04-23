//Naveen Chandra R
//160432
//naveencr@iitk.ac.in

#include <ros/ros.h>
#include <stdlib.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

pcl::ModelCoefficients findPlane(pcl::PointXYZ p1, pcl::PointXYZ p2,
	pcl::PointXYZ p3)
{
	//////////
	// TODO //
	//////////

	// This function is supposed to find the equation of the plane passing
	// through 3 given points (p1, p2 and p3). You can access the co-ordinates of
	// the points as p1.x, p1.y, p1.z, etc. You need to return a variable named
	// 'plane' of type is pcl::ModelCoefficients. You need to assign the values
	// plane.values[0] through plane.values[3] that represent the co-efficients
	// a, b, c, d (resp.) in the equation of the plane ax + by + cz = d

	pcl::ModelCoefficients plane;
	plane.values.resize(4);
	double a=0.0,b=0.0,c=0.0,d=0.0,norm=0.0;

	// Find the first 3 co-efficients using the cross product (p2-p1)x(p3-p1)

	a = ((p2.y-p1.y)*(p3.z-p1.z) - (p3.y-p1.y)*(p2.z-p1.z));
	b = ((p3.x-p1.x)*(p2.z-p1.z) - (p2.x-p1.x)*(p3.z-p1.z));
	c = ((p2.x-p1.x)*(p3.y-p1.y) - (p3.x-p1.x)*(p2.y-p1.y));


	// Normalize the first 3 co-efficients (so you can use it as a unit vector later)
  	norm = sqrt((a*a)+(b*b)+(c*c));
	// Note that while dividing by the norm, you need to ensure that norm != 0.0
	if(norm != 0.0)
	{
		a = a/norm;
		b = b/norm;
		c = c/norm;
		d = (a*p1.x + b*p1.y + c*p1.z);
		plane.values[0]= a;
		plane.values[1]= b;
		plane.values[2]= c;
		plane.values[3]= d;
	}
	// (in that case, just output a random plane passing through the 3 collinear ponts)
  	// if(norm==0.0)
	// {
	// 	a = ((p2.y-p1.y)*(0.0-p1.z) - (0.0-p1.y)*(p2.z-p1.z));
	// 	b = ((0.0-p1.x)*(p2.z-p1.z) - (p2.x-p1.x)*(0.0-p1.z));
	// 	c = ((p2.x-p1.x)*(0.0-p1.y) - (0.0-p1.x)*(p2.y-p1.y));
	// 	norm = sqrt((a*a)+(b*b)+(c*c));
	// 		plane.values[0]= a = a/norm;
	// 		plane.values[1]= b = b/norm;
	// 		plane.values[2]= c = c/norm;
	// 		plane.values[3]= d = a*p1.x + b*p1.y + c*p1.z;
	// }
	// Find the 4th co-efficient by taking the dot product of one of the vectors
	// (points) with the plane normal found (the first 3 co-efficients)
	return plane;
}

void cloudCallback (const sensor_msgs::PointCloud2ConstPtr& input_cloud,
	int N, float threshold, int d_required, ros::Publisher& pub_segmented)
{
	// Copying message data
	pcl::PointCloud<pcl::PointXYZ> tc;					// temp_cloud
	pcl::fromROSMsg (*input_cloud, tc);
	pcl::PointCloud<pcl::PointXYZRGB> segmented_cloud;	// final output
	pcl::copyPointCloud(tc, segmented_cloud);

	//////////
	// TODO //
	//////////

	// This part of the code is supposed to find the best plane that fits the
	// data using RANSAC. The variable tc of type pcl::PointCloud contains all
	// the points, which can be accessed as tc.points[0] through tc.points[M-1]
	// where M = tc.size().

	int M = tc.size(), d_current, d_max = 0;
	
	pcl::ModelCoefficients plane, bestplane;

	srand(time(NULL));	// If you're using the rand() function, it is a good idea to seed it
  	int a=0,b=0,c=0;
	// Main RANSAC Algorithm:
	// For N iterations
	// 		find the plane passing through 3 random points from tc (using findPlane)
	// 		find the number of points, d_current whose distance from the plane is less than threshold
	//		if d_current > d_max, update d_max and bestplane
	for (int i=0; i<N; i++)
	{
		a = rand()%M;
	 	b = rand()%M;
	 	c = rand()%M;
		d_current = 0;
	 	plane = findPlane(tc.points[b], tc.points[a], tc.points[c]);
	 	
		for(int j=0; j<M; j++)
	 	{
		 	float dist=0;
		 	dist = abs((plane.values[0]*tc.points[j].x) +(plane.values[1]*tc.points[j].y) +(plane.values[2]*tc.points[j].z) - (plane.values[3]))/sqrt((plane.values[0]*plane.values[0])+(plane.values[1]*plane.values[1])+(plane.values[2]*plane.values[2]));
		 	if(dist<threshold)
				d_current++;
	 	}

  		if(d_current>d_max)
		{
			d_max= d_current;
			bestplane= plane;
		}

 	}

	// This part will mark the points in segmented_cloud as red or green
	// depending on whether they fit the model given by bestplane and threshold
	// It will then publish the output
	if (d_max >= d_required)
	{
		ROS_INFO_STREAM("Success: "<<d_max<<" inliers found");

		// Generating segmented point cloud
		for (int j=0; j<M; j++)
		{
			if (std::abs(
				tc.points[j].x*bestplane.values[0]+tc.points[j].y*bestplane.values[1]+
				tc.points[j].z*bestplane.values[2]-bestplane.values[3])
				 < threshold)
			{
				segmented_cloud.points[j].r = 0;
				segmented_cloud.points[j].g = 255;
				segmented_cloud.points[j].b = 0;
			}
			else
			{
				segmented_cloud.points[j].r = 255;
				segmented_cloud.points[j].g = 0;
				segmented_cloud.points[j].b = 0;
			}
		}

		// Converting to ROS message and publishing
		sensor_msgs::PointCloud2 output_cloud;
		pcl::toROSMsg(segmented_cloud, output_cloud);
		output_cloud.header.stamp = input_cloud->header.stamp;
		output_cloud.header.frame_id = input_cloud->header.frame_id;
		pub_segmented.publish(output_cloud);
	}
	else
	{
		ROS_INFO_STREAM("Failed: Maximum number of iterations exceeded\n"<<
			"(No fit with more than "<<d_max<<" inliers found)");
	}

	return;
}

int main(int argc, char **argv)
{
	// Initializing node
	ros::init(argc, argv, "cloud_segmenter_algo");
	ros::NodeHandle nh;
	ros::Time t;

	// Getting parameters
	float w, p, threshold;
	int d_required;
	nh.param("threshold", threshold, (float)0.03);	// threshold distance for considering a point as inlier
	nh.param("d_required", d_required, 100000);		// inlier data points required to assume successful fit
	nh.param("w", w, (float)0.30);					// inlier fraction
	nh.param("p", p, (float)0.98);					// required success probability
	int N = log(1-p)/log(1-w*w*w);					// s = 3 for a plane model => w^3

	// Creating subscriber and publisher objects
	ros::Publisher pub_segmented =
		nh.advertise<sensor_msgs::PointCloud2>("/segmented_output", 1);
	ros::Subscriber sub_raw =
		nh.subscribe<sensor_msgs::PointCloud2>("/cloud_raw", 1, boost::bind(&cloudCallback, _1,
		N, threshold, d_required, boost::ref(pub_segmented)));

	// All remaining processing is done within the callback function
	ros::spin();

	return 0;
}
