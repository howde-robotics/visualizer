#include <sstream>
#include <vector>
#include <string>

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <jsk_rviz_plugins/OverlayText.h>
#include <tf2_ros/transform_listener.h>
#include <sound_play/sound_play.h>
#include <sound_play/SoundRequest.h>

#include "dragoon_messages/watchHeartbeat.h"
#include "dragoon_messages/watchStatus.h"

#include "wire_msgs/WorldEvidence.h"
#include "wire_msgs/WorldState.h"
#include "problib/conversions.h"

struct visualizer 
{
    ros::NodeHandle nh;

    ros::ServiceServer resetStateMarkerService;
    ros::Subscriber health_sub;
    ros::Subscriber world_ev_sub;
    ros::Subscriber world_st_sub;
    ros::Subscriber behavior_state_sub;

    ros::Publisher world_evidence_marker_pub_;
    ros::Publisher world_state_marker_pub_;
    ros::Publisher overlay_pub;
    ros::Publisher detection_overlay_pub;
    ros::Publisher vis_pub;

    jsk_rviz_plugins::OverlayText healthMonitorText;
    jsk_rviz_plugins::OverlayText detectedLocsText;
    visualization_msgs::Marker dragoonMarker;
    std::string behavior_state_text;

    ros::Timer timer_;

    sound_play::SoundClient soundPlayer;

    visualizer(double timerFreq) : nh(), soundPlayer(nh, "/robotsound")
    {
        
        timer_ = nh.createTimer(ros::Rate(timerFreq), &visualizer::timerCallback, this);

        resetStateMarkerService = nh.advertiseService("/visualizer/reset_state_markers", 
            &visualizer::resetWorldStateMarkerCB, this);
        
        health_sub = nh.subscribe("/health_status", 1, &visualizer::healthSubCB, this);
        world_ev_sub = nh.subscribe("/world_evidence", 1, &visualizer::worldEvidenceCallback, this);
        world_st_sub = nh.subscribe("/world_state", 1, &visualizer::worldStateCallback, this);
        behavior_state_sub = nh.subscribe("/behavior_state_text", 1, &visualizer::behaviorStateCallback, this);
        
        world_evidence_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/human_detections_vis/evidence", 1);
        world_state_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/human_detections_vis/state", 1);
        overlay_pub = nh.advertise<jsk_rviz_plugins::OverlayText>("guiText", 1);
        detection_overlay_pub = nh.advertise<jsk_rviz_plugins::OverlayText>("detected_locs_gui_text", 1);
        vis_pub = nh.advertise<visualization_msgs::Marker>( "dragoon_mesh", 0);
        
        dragoonMarker= makeDragoonMarker(
            "base_link",
            "package://visualizer/meshes/DRAGOON_SINGLE_bin.STL");
    }

    void behaviorStateCallback(const std_msgs::String &msg)
    {
        behavior_state_text = msg.data;
    }

    void timerCallback(const ros::TimerEvent& e)
    {
        
        vis_pub.publish(dragoonMarker);
    
        //health monitor information
        healthMonitorText.text = healthStatusStringStream.str();
        overlay_pub.publish(healthMonitorText);

        //detection locations information
        detectedLocsText.text = detectionsStringStream.str();
        detection_overlay_pub.publish(detectedLocsText);
    }

    std::vector<std::string> heartbeatNames {
        "Lidar", "Thermal", "RGB", "Depth", 
        "IMU", "Detection Localization", "Visual Detection", 
        "Detection Filtering", "SLAM"};

    std::stringstream healthStatusStringStream;
    void healthSubCB(const dragoon_messages::watchStatusConstPtr& msg)
    {
        healthStatusStringStream.clear();//clear error bit
        healthStatusStringStream.str("");//clear actual string
        healthStatusStringStream << "Health Status" << std::endl;
        if(msg->status_array.size() <= heartbeatNames.size()) {
            for (auto ind = 0; ind < msg->status_array.size(); ++ind)
            {
                healthStatusStringStream << heartbeatNames.at(ind) << ":\t" << (msg->status_array.at(ind) ? 
                    "<span style=\"color: green;\">OK</span>" :
                    "<span style=\"color: red;\">BAD</span>" )
                    << std::endl;
            }
        } else {
            healthStatusStringStream << "No Health Status to report" << std::endl;
        }
        healthStatusStringStream << "Behavior State: " << behavior_state_text << std::endl;
    }

    visualization_msgs::Marker makeSphereMarker(std::string ns, int32_t id, 
        double x, double y, double z,
        float alpha, float color_r, float color_g, float color_b, 
        ros::Time stamp, std::string frame_id, double diameter) {

        visualization_msgs::Marker marker;
        marker.ns = ns;
        marker.id = id;
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = z;
        marker.pose.orientation.w = 1;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.x = 0;
        marker.pose.orientation.z = 0;
        marker.color.a = alpha;
        marker.color.r = color_r;
        marker.color.g = color_g;
        marker.color.b = color_b;
        marker.header.stamp = stamp;
        marker.header.frame_id = frame_id;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.scale.x = diameter;
        marker.scale.y = diameter;
        marker.scale.z = diameter;
        return marker;
    }

    visualization_msgs::Marker makeDragoonMarker(std::string frame, std::string resourceLocation) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame;
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
        marker.color.a = 1.0; 
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.mesh_resource = resourceLocation;
        return marker;
    }

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
                        const pbl::Vector& mean = gauss->getMean();
                        visualization_msgs::Marker marker = makeSphereMarker("detections/evidence", id, 
                            mean(0), mean(1), mean(2),
                            0.5, 1.0, 0.0, 0.0,
                            msg->header.stamp, "map", 0.2);
                        markers_msg.markers.push_back(marker);
                        ++id;
                    }
                }
            }

        }
        // // Publish marker
        world_evidence_marker_pub_.publish(markers_msg);
    }

    int numHumansDetected = 0;
    std::string markerLocNamespace = "detections/stateLoc";
    std::string markerBoundNamespace = "detections/stateBounds";
    std::stringstream detectionsStringStream;
    /*
    * World state callback. Creates a marker for each detection.
    */
    void worldStateCallback(const wire_msgs::WorldState::ConstPtr& msg) {


        visualization_msgs::MarkerArray markers_msg;
        int id = 0;
        detectionsStringStream.clear();//clears error bit, not the string itself...dumb as fuck Bjarne
        detectionsStringStream.str("");//now the string is cleared
        detectionsStringStream << "DETECTED HUMANS:" << std::endl;
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
                        const pbl::Vector& mean = gauss->getMean();
                        visualization_msgs::Marker markerLoc = makeSphereMarker(markerLocNamespace, id, 
                            mean(0), mean(1), mean(2), 
                            1.0, 0.0, 1.0, 0.0, 
                            msg->header.stamp, "map", 0.25);
                        markers_msg.markers.push_back(markerLoc);

                        //Add +- 0.5 m sphere representing our target distance
                        visualization_msgs::Marker markerBound = makeSphereMarker(markerBoundNamespace, id, 
                            mean(0), mean(1), mean(2), 
                            0.2, 0.0, 0.0, 1.0, 
                            msg->header.stamp, "map", 1.0);
                        markers_msg.markers.push_back(markerBound);

                        detectionsStringStream << "Human " << id 
                            << ": (" << mean(0) << ", " << mean(1) << ", " << mean(2) << ")" << std::endl;

                        ++id;

                        //Check if we have detected a new human
                        if (id > numHumansDetected) 
                        {
                            soundPlayer.say("Fuck yeah, Human Detected");
                            //update
                            numHumansDetected = id;
                        }
                    }
                }
            }
        }
        world_state_marker_pub_.publish(markers_msg);   
    }

    bool resetWorldStateMarkerCB(std_srvs::Empty::Request &req,
                             std_srvs::Empty::Response &resp) 
    {
        //clear world state markers
        visualization_msgs::MarkerArray markers_msg;
        for (auto id = 0; id < numHumansDetected; ++id) {
            visualization_msgs::Marker markerLoc;
            markerLoc.ns = markerLocNamespace;
            markerLoc.id = id;
            markerLoc.action = visualization_msgs::Marker::DELETE;
            markers_msg.markers.push_back(markerLoc);
            
            visualization_msgs::Marker markerBound;
            markerBound.ns = markerBoundNamespace;
            markerBound.id = id;
            markerBound.action = visualization_msgs::Marker::DELETE;
            markers_msg.markers.push_back(markerBound);

        }
        world_state_marker_pub_.publish(markers_msg);
        numHumansDetected = 0;
        detectionsStringStream.clear();
        detectionsStringStream.str("");
        return true;
    }
};

int main(int argc, char** argv)
{
ros::init(argc, argv, "visualizer");
double timerFreq(10);
visualizer visNode(timerFreq);
// ros::NodeHandle nh;
// sound_play::SoundClient sc = sound_play::SoundClient(nh, "robotsound");
// ros::Rate loop_rate(0.3);
while(ros::ok()) {
    ros::spinOnce();
}
return 0;
}
