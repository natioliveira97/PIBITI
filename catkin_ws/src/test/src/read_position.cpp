#include "ros/ros.h"
#include <fiducial_msgs/FiducialTransformArray.h>
#include <visualization_msgs/Marker.h>

ros::Publisher vis_pub;

void publisher(int id, float x, float y, float z, float ox, float oy, float oz, float ow){
    ROS_INFO("x = %lf, y=%lf, x=%lf", x, y, z);
    visualization_msgs::Marker marker;
    marker.header.frame_id = "camera";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = id;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation.x = ox;
    marker.pose.orientation.y = oy;
    marker.pose.orientation.z = oz;
    marker.pose.orientation.w = ow;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.001;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    vis_pub.publish( marker );
}


void readCallback(const fiducial_msgs::FiducialTransformArray::ConstPtr& fiducial_transform){

    for(int i=0; i<fiducial_transform->transforms.size(); ++i){

        for(auto marker_it = fiducial_transform->transforms.begin(); marker_it != fiducial_transform->transforms.end(); ++marker_it){
            int id = marker_it->fiducial_id;

            float x = marker_it->transform.translation.x;
            float y = marker_it->transform.translation.y;
            float z = marker_it->transform.translation.z;

            float ox = marker_it->transform.rotation.x;
            float oy = marker_it->transform.rotation.y;
            float oz = marker_it->transform.rotation.z;
            float ow = marker_it->transform.rotation.w;
            
            publisher(id,x,y,z,ox,oy,oz,ow);
        }
    }
}


int main(int argc, char **argv){

    ros::init(argc, argv, "listener");

    ros::NodeHandle nd;

    ros::Subscriber sub = nd.subscribe("/fiducial_transforms", 1000, readCallback);

    vis_pub = nd.advertise<visualization_msgs::Marker>("ponto3d", 0 );

    ros::spin();

    return 0;
}