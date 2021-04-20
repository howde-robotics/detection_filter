#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cmath>

#include "dragoon_messages/Objects.h"
#include "wire_msgs/WorldEvidence.h"
#include "problib/pdfs/PDF.h"
#include "problib/conversions.h"

/**
 * Some things tunable to change human detection filtering behavior:
 * wire_core/models/world_object_models.xml
 * detection_filter::detectionMaxThresh_
 * detection_filter::detectionMinThresh_
 * detection_filter::obj_evidence::certainty
 * visualizer::mixtureWeightThresh
 * detection_filter::covariances
 * 
 */

class detection_filter {
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;
    ros::NodeHandle nh_, private_nh_;
    ros::Subscriber human_det_sub_;
    ros::Publisher evidence_pub_;
    double detectionMaxThresh_;
    double detectionMinThresh_;
    double probabilityMinThresh_;
    
public:

    detection_filter() : tfListener_(tfBuffer_), private_nh_("~")
    {
        private_nh_.param<double>("detectionMaxThresh_", detectionMaxThresh_, 15);  // m
        private_nh_.param<double>("detectionMinThresh_", detectionMinThresh_, 0.05); // m
        private_nh_.param<double>("probabilityMinThresh_", probabilityMinThresh_, 0.4); // percentage
        human_det_sub_ = nh_.subscribe("/ObjectPoses", 5, &detection_filter::objectPosesCB, this);
        evidence_pub_ = nh_.advertise<wire_msgs::WorldEvidence>("/world_evidence", 10, true);
    }

    void objectPosesCB(const dragoon_messages::ObjectsConstPtr& msg) 
    {
        for (auto obj : msg->objects_info) {
            if (obj.Class == "person" && obj.probability > probabilityMinThresh_) {
                //Preprocess distances
                double distSq = pow(obj.pose.x, 2.0) + pow(obj.pose.y, 2.0) + pow(obj.pose.z, 2.0);
                if (distSq > pow(detectionMaxThresh_, 2) || distSq < pow(detectionMinThresh_, 2)) {
                    continue;//ignore this detection
                }
                //make a transform
                geometry_msgs::TransformStamped transformStamped;
                transformStamped.header.stamp = ros::Time::now();
                transformStamped.header.frame_id = "camera_depth_optical_frame";
                transformStamped.child_frame_id = "__human";//Not being broadcast so wont ever show up in tf tree
                transformStamped.transform.translation.x = obj.pose.x;
                transformStamped.transform.translation.y = obj.pose.y;
                transformStamped.transform.translation.z = obj.pose.z;
                transformStamped.transform.rotation.x = 0;
                transformStamped.transform.rotation.y = 0;
                transformStamped.transform.rotation.z = 0;
                transformStamped.transform.rotation.w = 1;

                geometry_msgs::TransformStamped globalLoc;
                try {
                    globalLoc = tfBuffer_.transform<geometry_msgs::TransformStamped>(transformStamped, "map", ros::Duration(1));
                } 
                catch (tf2::TransformException &ex) 
                {
                  ROS_WARN("Failure %s\n", ex.what()); //Print exception which was caught
                }
                    
                
                //Publish to global kalman filter
                wire_msgs::WorldEvidence world_evidence;
                world_evidence.header.stamp = ros::Time::now();
	            world_evidence.header.frame_id = "/map";

                wire_msgs::ObjectEvidence obj_evidence;
                // obj_evidence.certainty = 0.01;
                wire_msgs::Property posProp;
	            posProp.attribute = "position";
                double x = globalLoc.transform.translation.x;
                double y = globalLoc.transform.translation.y;
                double z = globalLoc.transform.translation.z;
                pbl::PDFtoMsg(pbl::Gaussian(pbl::Vector3(x, y, z), pbl::Matrix3(0.5, 0.5, 0.5)), posProp.pdf);
                obj_evidence.properties.push_back(posProp);

                world_evidence.object_evidence.push_back(obj_evidence);
                evidence_pub_.publish(world_evidence);
            }
        }
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "detection_filter");
    
    detection_filter df;
    
    ros::Rate loop_rate(10);

    while(ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
}