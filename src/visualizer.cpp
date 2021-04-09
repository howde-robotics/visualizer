#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <jsk_rviz_plugins/OverlayText.h>
#include <sstream>
#include <vector>

#include <tf2_ros/transform_broadcaster.h>

#include "dragoon_messages/watchHeartbeat.h"
#include "dragoon_messages/watchStatus.h"
// #include "dragoon_messages/ObjectInfo.h"
#include "dragoon_messages/Objects.h"

std::vector<char> status;

void healthSubCB(const dragoon_messages::watchStatusConstPtr& msg)
{
    status.clear();
    for (auto stat : msg->status_array){
        status.push_back(stat);
    }
    
}

void objectPosesCB(const dragoon_messages::ObjectsConstPtr& msg) 
{
    static tf2_ros::TransformBroadcaster br;
    for (auto obj : msg->objects_info) {
        if (obj.Class == "person") {
            ROS_INFO_STREAM("PERSON DETECTED with prob: " << obj.probability);
            geometry_msgs::TransformStamped transformStamped;
            transformStamped.header.stamp = ros::Time::now();
            transformStamped.header.frame_id = "camera_depth_optical_frame";
            transformStamped.child_frame_id = "huamn";
            transformStamped.transform.translation.x = obj.pose.x;
            transformStamped.transform.translation.y = obj.pose.y;
            transformStamped.transform.translation.z = obj.pose.z;
            transformStamped.transform.rotation.x = 0;
            transformStamped.transform.rotation.y = 0;
            transformStamped.transform.rotation.z = 0;
            transformStamped.transform.rotation.w = 1;

            // br.sendTransform(transformStamped);
        }
    }
}

int main(int argc, char** argv)
{
ros::init(argc, argv, "visualizer");
ros::NodeHandle nh;
ros::Rate loop_rate(10);

ros::Subscriber health_sub = nh.subscribe("/health_status", 5, healthSubCB);

ros::Subscriber human_det_sub = nh.subscribe("/ObjectPoses", 5, objectPosesCB);

ros::Publisher overlay_pub = nh.advertise<jsk_rviz_plugins::OverlayText>("guiText", 10);
jsk_rviz_plugins::OverlayText text;
text = jsk_rviz_plugins::OverlayText();
// theta = counter % 255 / 255.0;
text.width = 400;
text.height = 1000;
text.left = 10;
text.top = 10;
text.text_size = 12;
text.line_width = 2;
text.font = "DejaVu Sans Mono";
// text.text = "FUCKING FUCKING FUCK GUFKCSJLDNSJDN";



ros::Publisher vis_pub = nh.advertise<visualization_msgs::Marker>( "dragoon_mesh", 0);
visualization_msgs::Marker marker;
marker.header.frame_id = "/base_link";
marker.header.stamp = ros::Time();
// marker.ns = "my_namespace";
marker.id = 0;
marker.type = visualization_msgs::Marker::MESH_RESOURCE;
marker.pose.position.x = -0.32;
marker.pose.position.y = 0.35;
marker.pose.position.z = -0.15;
marker.pose.orientation.x = 0.7071068;
marker.pose.orientation.y = 0.0;
marker.pose.orientation.z = 0.0;
marker.pose.orientation.w = 0.7071068;
marker.scale.x = 0.001;
marker.scale.y = 0.001;
marker.scale.z = 0.001;
marker.color.a = 1.0; // Don't forget to set the alpha!
marker.color.r = 1.0;
marker.color.g = 0.0;
marker.color.b = 0.0;
marker.mesh_resource = "package://visualizer/meshes/DRAGOON_SINGLE_bin.STL";
vis_pub.publish( marker );
int count = 0;
while(ros::ok()) {
    vis_pub.publish(marker);
    ++count;
    std::stringstream ss;
    ss << "Health Status" << std::endl;
    if(status.size() > 0) {
        ss << "LIDAR:\t" << (status.at(0) ? "<span style=\"color: green;\">OK</span>" :
                                        "<span style=\"color: red;\">BAD</span>" )
            << std::endl;
        ss << "Seek:\t" << (status.at(1) ? "<span style=\"color: green;\">OK</span>" :
                                        "<span style=\"color: red;\">BAD</span>" )
            << std::endl;
        ss << "Realsense RGB:\t" << (status.at(2) ? "<span style=\"color: green;\">OK</span>" :
                                        "<span style=\"color: red;\">BAD</span>" )
            << std::endl;

        ss << "Realsense Depth:\t" << (status.at(3) ? "<span style=\"color: green;\">OK</span>" :
                                        "<span style=\"color: red;\">BAD</span>" )
            << std::endl;

        ss << "Transformed IMU:\t" << (status.at(4) ? "<span style=\"color: green;\">OK</span>" :
                                        "<span style=\"color: red;\">BAD</span>" )
            << std::endl;
        ss << "Localize:\t" << (status.at(5) ? "<span style=\"color: green;\">OK</span>" :
                                        "<span style=\"color: red;\">BAD</span>" )
            << std::endl;
        ss << "Detection:\t" << (status.at(6) ? "<span style=\"color: green;\">OK</span>" :
                                        "<span style=\"color: red;\">BAD</span>" )
            << std::endl;
    } else {
        ss << "No Health Status to report" << std::endl;
    }
    
    text.text = ss.str();
    overlay_pub.publish(text);
    // ROS_WARN("test warning");
    ros::spinOnce();
    loop_rate.sleep();
}

}