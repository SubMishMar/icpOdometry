#include <fstream>

#include <boost/version.hpp>
#include <boost/thread.hpp>
#if BOOST_VERSION >= 104100
    #include <boost/thread/future.hpp>
#endif // BOOST_VERSION >=  104100

#include "ros/ros.h"
#include "ros/console.h"
#include "pointmatcher/PointMatcher.h"
#include "pointmatcher/Timer.h"

#include "nabo/nabo.h"

#include "pointmatcher_ros/point_cloud.h"
#include "pointmatcher_ros/transform.h"
#include "pointmatcher_ros/get_params_from_server.h"
#include "pointmatcher_ros/ros_logger.h"

#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"
#include "tf_conversions/tf_eigen.h"
#include "tf/transform_listener.h"
#include "eigen_conversions/eigen_msg.h"

#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/String.h"

#include <sstream>

using namespace PointMatcherSupport;
typedef PointMatcher<float> PM;

class ICPOdometry {

	typedef PM::DataPoints DP;

    ros::NodeHandle& nh;

    // Subscribers
    ros::Subscriber cloudSub;

    // Publisher
    ros::Publisher posePub;

	// libpointmatcher
	PM::DataPointsFilters inputFilters;
    PM::ICPSequence icp;

    // Parameters
    int inputQueueSize;
    int minReadingPointCount;

public:
    ICPOdometry(ros::NodeHandle& nh);

protected:
    void gotCloud(const sensor_msgs::PointCloud2& cloudMsgIn);
    void processCloud(std::unique_ptr<DP> newPointCloud,
    				  const std::string& scannerFrame,
    				  const ros::Time& stamp,
    				  uint32_t seq);
    void loadExternalParameters();
};

void ICPOdometry::loadExternalParameters() {
	std::string configFilename;
	
	if (ros::param::get("~icpConfig", configFilename)) {
		std::ifstream ifs(configFilename.c_str());
		if (ifs.good()) {
			icp.loadFromYaml(ifs);
		} else {
			ROS_ERROR_STREAM("Using default config, Cannot load ICP config from YAML file" << configFilename);
			icp.setDefault();
		}
	} else {
		ROS_INFO_STREAM("No ICP config file given, using default");
		icp.setDefault();
	}

	if (ros::param::get("~inputFiltersConfig", configFilename)) {
		std::ifstream ifs(configFilename.c_str());
		if (ifs.good()) {
			inputFilters = PM::DataPointsFilters(ifs);
		} else {
			ROS_INFO_STREAM("Cannot load input filters config from YAML file " << configFilename);
		}
	} else {
		ROS_INFO_STREAM("No input filters config file given, not using these filters");
	}
}

void ICPOdometry::processCloud(std::unique_ptr<DP> newPointCloud,
							   const std::string& scannerFrame,
							   const ros::Time& stamp,
							   uint32_t seq) {

    timer t;
	
	// Convert point cloud
	const size_t goodCount(newPointCloud->features.cols());
	if (goodCount == 0) {
		ROS_ERROR("[ICP] I found no good points in the cloud");
		return;
	}

	// Dimension of the point cloud
	const int dimp1(newPointCloud->features.rows());

	if(!(newPointCloud->descriptorExists("stamps_Msec") && newPointCloud->descriptorExists("stamps_sec") && newPointCloud->descriptorExists("stamps_nsec"))) {
		const float Msec = round(stamp.sec/1e6);
		const float sec = round(stamp.sec - Msec*1e6);
		const float nsec = round(stamp.nsec);

		const PM::Matrix desc_Msec = PM::Matrix::Constant(1, goodCount, Msec);
		const PM::Matrix desc_sec = PM::Matrix::Constant(1, goodCount, sec);
		const PM::Matrix desc_nsec = PM::Matrix::Constant(1, goodCount, nsec);

		newPointCloud->addDescriptor("stamps_Msec", desc_Msec);
		newPointCloud->addDescriptor("stamps_sec", desc_sec);
		newPointCloud->addDescriptor("stamps_nsec", desc_nsec);
	}

	// Ensure a minimum amount of point before filtering
	int ptsCount = newPointCloud->getNbPoints();
	if(ptsCount < minReadingPointCount) {
		ROS_ERROR_STREAM("[ICP] Not enough points in newPointCloud: only " << ptsCount << " pts.");
		return;
	}

	{
		timer t; // Print how long the algo took!
		// Apply filters to incoming cloud, in scanner coordinates
		inputFilters.apply(*newPointCloud);
		ROS_INFO_STREAM("[ICP] Input filters took " << t.elapsed() << " [s]");
	}
}

void ICPOdometry::gotCloud(const sensor_msgs::PointCloud2& cloudMsgIn) {

    std::unique_ptr<DP> cloud(new DP(PointMatcher_ros::rosMsgToPointMatcherCloud<float>(cloudMsgIn)));
    processCloud(move(cloud), cloudMsgIn.header.frame_id, cloudMsgIn.header.stamp, cloudMsgIn.header.seq);
}

ICPOdometry::ICPOdometry(ros::NodeHandle& nh): 
nh(nh),
inputQueueSize(getParam<int>("inputQueueSize", 10)),
minReadingPointCount(getParam<int>("minReadingPointCount", 2000)) {
    cloudSub = nh.subscribe("cloud_in", inputQueueSize, &ICPOdometry::gotCloud, this);
    posePub = nh.advertise<geometry_msgs::PoseStamped>("icp_pose", 50, true);
    loadExternalParameters();
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "mapper");
    ros::NodeHandle n;
    ICPOdometry icpodom(n);
    ros::spin();

    return 0; 
}