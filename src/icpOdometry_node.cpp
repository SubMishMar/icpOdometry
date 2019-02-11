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

class ICPOdometry {
    ros::NodeHandle& nh;

    // Subscribers
    ros::Subscriber cloudSub;

    // Publisher
    ros::Publisher posePub;

    // Parameters
    int inputQueueSize;

public:
    ICPOdometry(ros::NodeHandle& nh);

protected:
    void gotCloud(const sensor_msgs::PointCloud2& cloudMsgIn);
};

void ICPOdometry::gotCloud(const sensor_msgs::PointCloud2& cloudMsgIn) {
    ROS_INFO_STREAM("Received PointCloud");
}

ICPOdometry::ICPOdometry(ros::NodeHandle& nh): 
nh(nh),
inputQueueSize(getParam<int>("inputQueueSize", 10)) {
    cloudSub = nh.subscribe("cloud_in", inputQueueSize, &ICPOdometry::gotCloud, this);
    posePub = nh.advertise<geometry_msgs::PoseStamped>("icp_pose", 50, true);
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "mapper");
    ros::NodeHandle n;
    ICPOdometry icpodom(n);
    ros::spin();

    return 0; 
}