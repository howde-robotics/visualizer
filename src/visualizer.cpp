#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <jsk_rviz_plugins/OverlayText.h>
#include <sstream>
#include <vector>

#include <tf2_ros/transform_listener.h>

#include "dragoon_messages/watchHeartbeat.h"
#include "dragoon_messages/watchStatus.h"

#include "wire_msgs/WorldEvidence.h"
#include "wire_msgs/WorldState.h"
#include "problib/conversions.h"


std::vector<char> status;

void healthSubCB(const dragoon_messages::watchStatusConstPtr& msg)
{
    status.clear();
    for (auto stat : msg->status_array){
        status.push_back(stat);
    }
    
}


ros::Publisher world_evidence_marker_pub_;
ros::Publisher world_state_marker_pub_;
double mixtureWeightThresh = 0.0;
/* Stolen from Visualizer.cpp in the wire_viz package
 * Get the most probable Gaussian from a pdf
 */

const pbl::Gaussian* getBestGaussian(const pbl::PDF& pdf, double min_weight) {
	if (pdf.type() == pbl::PDF::GAUSSIAN) {
		return pbl::PDFtoGaussian(pdf);
	} else if (pdf.type() == pbl::PDF::MIXTURE) {
		const pbl::Mixture* mix = pbl::PDFtoMixture(pdf);

		if (mix){
			const pbl::Gaussian* G_best = NULL;
			double w_best = min_weight;
			for(int i = 0; i < mix->components(); ++i) {
				const pbl::PDF& pdf = mix->getComponent(i);
				const pbl::Gaussian* G = pbl::PDFtoGaussian(pdf);
				double w = mix->getWeight(i);
				if (G && w > w_best) {
					G_best = G;
					w_best = w;
				}
			}
			return G_best;
		}
    }

	return NULL;
}

/*
 * World evidence callback. Creates a marker for each of the world model objects
 */
void worldEvidenceCallback(const wire_msgs::WorldEvidence::ConstPtr& msg) {

	// ID for marker administration
    int id = 0;
    // Create marker
    visualization_msgs::MarkerArray markers_msg;

	for (const wire_msgs::ObjectEvidence& obj_ev : msg->object_evidence) {
        for (auto prop : obj_ev.properties)
        {
            if (prop.attribute == "position")
            {
                pbl::PDF* pdf = pbl::msgToPDF(prop.pdf);
                const pbl::Gaussian* gauss = getBestGaussian(*pdf, mixtureWeightThresh);
                if (gauss) {
                    visualization_msgs::Marker marker;
                    const pbl::Vector& mean = gauss->getMean();
                    marker.ns = "detections/evidence";
                    marker.id = id;
                    marker.pose.position.x = mean(0);
                    marker.pose.position.y = mean(1);
                    marker.pose.position.z = mean(2);
                    marker.pose.orientation.w = 1;
                    marker.pose.orientation.y = 0;
                    marker.pose.orientation.x = 0;
                    marker.pose.orientation.z = 0;
                    marker.color.a = 0.5;
                    marker.color.g = 1.0;
                    marker.color.r = 1.0;
                    marker.header.stamp = msg->header.stamp;
                    marker.header.frame_id = "map";
                    marker.type = visualization_msgs::Marker::SPHERE;
                    marker.scale.x = 0.2;
                    marker.scale.y = 0.2;
                    marker.scale.z = 0.2;
                    markers_msg.markers.push_back(marker);
                    ++id;
                }
            }
        }

	}
    // // Publish marker
    world_evidence_marker_pub_.publish(markers_msg);
}




/*
 * World state callback. Creates a marker for each detection.
 */
void worldStateCallback(const wire_msgs::WorldState::ConstPtr& msg) {


    visualization_msgs::MarkerArray markers_msg;
    int id = 0;
	// Iterate over world state objects
	for (const wire_msgs::ObjectState& obj : msg->objects) {

		// Create marker


        for (auto prop : obj.properties)
        {
            if (prop.attribute == "position")
            {
                pbl::PDF* pdf = pbl::msgToPDF(prop.pdf);
                const pbl::Gaussian* gauss = getBestGaussian(*pdf, mixtureWeightThresh);
                if (gauss) {
                    visualization_msgs::Marker marker;
                    const pbl::Vector& mean = gauss->getMean();
                    marker.ns = "detections/state";
                    marker.id = id;
                    marker.pose.position.x = mean(0);
                    marker.pose.position.y = mean(1);
                    marker.pose.position.z = mean(2);
                    marker.pose.orientation.w = 1;
                    marker.pose.orientation.y = 0;
                    marker.pose.orientation.x = 0;
                    marker.pose.orientation.z = 0;
                    marker.color.a = 1.0;
                    marker.color.g = 1.0;
                    marker.header.stamp = msg->header.stamp;
                    marker.header.frame_id = "map";
                    marker.type = visualization_msgs::Marker::SPHERE;
                    marker.scale.x = 0.25;
                    marker.scale.y = 0.25;
                    marker.scale.z = 0.25;
                    markers_msg.markers.push_back(marker);
                    ++id;
                } else {
                    ROS_WARN("State: Something other than GAUSSIAN or MIXTURE wuht");
                }
            } else
            {
                ROS_WARN("State: Something other than position is being filtered wuht");
            }

        }

	}
    world_state_marker_pub_.publish(markers_msg);   
}

int main(int argc, char** argv)
{
ros::init(argc, argv, "visualizer");
ros::NodeHandle nh;
ros::Rate loop_rate(30);

ros::Subscriber health_sub = nh.subscribe("/health_status", 1, healthSubCB);

ros::Subscriber world_ev_sub = nh.subscribe("/world_evidence", 1, worldEvidenceCallback);
ros::Subscriber world_st_sub = nh.subscribe("/world_state", 1, worldStateCallback);

world_evidence_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/human_detections_vis/evidence", 10);
world_state_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/human_detections_vis/state", 10);

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
// vis_pub.publish( marker );
int count = 0;
while(ros::ok()) {
    vis_pub.publish(marker);
    ++count;
    std::stringstream ss;
    std::vector<std::string> heartbeatNames {"lidar", "seek", "realsense_rgb" "realsense_depth", 
                                    "transformed_imu", "localize", "detection", 
                                    "detection_filtering", "slam"};
    ss << "Health Status" << std::endl;
    if(status.size() <= heartbeatNames.size()) {
        for (auto ind = 0; ind < status.size(); ++ind)
        {
            ss << heartbeatNames.at(ind) << ":\t" << (status.at(ind) ? "<span style=\"color: green;\">OK</span>" :
                                        "<span style=\"color: red;\">BAD</span>" )
            << std::endl;
        }
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