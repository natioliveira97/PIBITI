#include "ros/ros.h"
#include <fiducial_msgs/FiducialTransformArray.h>
#include "fiducial_msgs/FiducialTransform.h"
#include <visualization_msgs/Marker.h>
#include <Eigen/Dense>

ros::Publisher vis_pub;

int n_states = 10;
int n_measurements = 7;


Eigen::VectorXd measurement = Eigen::VectorXd::Zero(n_measurements);
Eigen::VectorXd state_estimation = Eigen::VectorXd::Zero(n_states);
Eigen::MatrixXd cov_state = Eigen::MatrixXd::Identity(n_states,n_states);


Eigen::MatrixXd F = Eigen::MatrixXd::Identity(n_states,n_states);
Eigen::MatrixXd H = Eigen::MatrixXd::Zero(n_measurements,n_states);
Eigen::MatrixXd H2 = Eigen::MatrixXd::Zero(1,n_states);
Eigen::MatrixXd R = Eigen::MatrixXd::Identity(n_measurements,n_measurements);
Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(n_states,n_states);
Eigen::MatrixXd Qa = Eigen::MatrixXd::Identity(n_states,n_states);
Eigen::MatrixXd K = Eigen::MatrixXd::Zero(n_states,n_measurements);
Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n_states,n_states);
Eigen::MatrixXd R2 = Eigen::MatrixXd::Zero(1,1);
Eigen::MatrixXd q1, q2, last_q;


float qv = pow(0.01,2);
float qq = pow(0.001,2);
double rerror = pow(0.00001,2);

ros::Publisher * pose_pub; 
fiducial_msgs::FiducialTransform ft;


ros::Time old_time;
ros::Time new_time;
ros::Duration dt;

bool init = true;


void publisher(int id, float x, float y, float z, float ox, float oy, float oz, float ow, float r, float g){
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
    ROS_INFO("id: %d, qx= %lf, qy=%lf, qz=%lf, qw=%lf", id, ox, oy, oz, ow);
    marker.pose.orientation.x = ox;
    marker.pose.orientation.y = oy;
    marker.pose.orientation.z = oz;
    marker.pose.orientation.w = ow;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.001;
    marker.color.a = 1.0;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = 0.0;
    vis_pub.publish( marker );
}

void init_filter(){
    std::cout<<("init")<<std::endl;
    state_estimation(0)=measurement(0);
    state_estimation(1)=measurement(1);
    state_estimation(2)=measurement(2);
    state_estimation(3)=measurement(3);
    state_estimation(4)=measurement(4);
    state_estimation(5)=measurement(5);
    state_estimation(6)=measurement(6);


    old_time = ros::Time::now();

    H << 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, 1;

    
    R << 0.0001,    0.0000,   -0.0000,    0.0000,   -0.0000,   -0.0000,   -0.0000,
    0.0000,   0.0007,   -0.0000,    0.0000,    0.0000,   -0.000,   -0.0000,
   -0.0000,   -0.0000,    0.0089,   -0.0000,    0.0000,    0.000,    0.0000,
    0.0000,    0.0000,   -0.0000,    0.0002,    0.0000,   -0.0000,   -0.0000,
   -0.0000,    0.0000,    0.0000,    0.0000,    0.0071,   -0.0000,   -0.000,
   -0.0000,   -0.000,     0.000,     -0.0000,   -0.0000,    0.2378,    0.000,
   -0.0008,   -0.0000,    0.0000,   -0.0000,   -0.000,    0.000,    0.1265;

    R2 << 0.000000000001;

   R=R*0.0001;



    Qa << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, qv, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, qv, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, qv, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, qq, 0.0, 0.0, 0.0, 
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, qq, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, qq, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, qq;
    
}

void prediction(){
    F << 1.0, 0.0, 0.0, dt.toSec(), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 1.0, 0.0, 0.0, dt.toSec(), 0.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 1.0, 0.0, 0.0, dt.toSec(), 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 
         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

    Q = F*Qa*F.transpose();


    state_estimation = F*state_estimation;
    cov_state = F*cov_state*F.transpose()+Q;
}

void correction(){

    
    last_q = state_estimation.segment(6,4);
    q1 = measurement.segment(3,4);
    q2 = -q1;

    if((last_q-q2).norm()<(last_q-q1).norm()){
        ROS_INFO("qx= %lf, qy=%lf, qz=%lf, qw=%lf, norma = %lf", (last_q-q1)(0), (last_q-q1)(1), (last_q-q1)(2), (last_q-q1)(3), (last_q-q1).norm());
        ROS_INFO("qx= %lf, qy=%lf, qz=%lf, qw=%lf, norma = %lf", (last_q-q2)(0), (last_q-q2)(1), (last_q-q2)(2), (last_q-q2)(3), (last_q-q2).norm());

        
        measurement(3)=-measurement(3);
        measurement(4)=-measurement(4);
        measurement(5)=-measurement(5);
        measurement(6)=-measurement(6);    
    }





    K = cov_state*H.transpose()*(H*cov_state*H.transpose()+R).inverse();
    state_estimation = state_estimation+ K*(measurement-H*state_estimation);
    cov_state = (I-K*H)*cov_state*(I-K*H).transpose()+K*R*K.transpose();

    H2 << 0, 0, 0 ,0, 0, 0, 2*state_estimation(6), 2*state_estimation(7), 2*state_estimation(8), 2*state_estimation(9);
    K = cov_state*H2.transpose()*(H2*cov_state*H2.transpose()+R2).inverse();
    state_estimation = state_estimation+K*(1-pow(state_estimation(6),2)-pow(state_estimation(7),2)-pow(state_estimation(8),2)-pow(state_estimation(9),2));
    cov_state = (I-K*H2)*cov_state*(I-K*H2).transpose()+K*R2*K.transpose();


}


void measurementCallback(const fiducial_msgs::FiducialTransformArray::ConstPtr& fiducial_transform){

    for(int i=0; i<fiducial_transform->transforms.size(); ++i){

        for(auto marker_it = fiducial_transform->transforms.begin(); marker_it != fiducial_transform->transforms.end(); ++marker_it){
            
            int id = marker_it->fiducial_id;

            //publisher(id,marker_it->transform.translation.x,marker_it->transform.translation.y,marker_it->transform.translation.z,marker_it->transform.rotation.x,marker_it->transform.rotation.y,marker_it->transform.rotation.z,marker_it->transform.rotation.w, 1.0, 0.0);

            measurement(0) = marker_it->transform.translation.x;
            measurement(1) = marker_it->transform.translation.y;
            measurement(2) = marker_it->transform.translation.z;
            measurement(3) = marker_it->transform.rotation.x;
            measurement(4) = marker_it->transform.rotation.y;
            measurement(5) = marker_it->transform.rotation.z;
            measurement(6) = marker_it->transform.rotation.w;


            if(init){
                init_filter();
                init = false;
            }
            else{
                new_time = ros::Time::now();
                dt = new_time-old_time;
                old_time = new_time;

                prediction();
                correction();

                fiducial_msgs::FiducialTransformArray fta;
                fta.header.stamp = fiducial_transform->header.stamp;
                fta.header.frame_id = fiducial_transform->header.frame_id;
                fta.image_seq = fiducial_transform->header.seq;

                ft.transform.translation.x = state_estimation(0);
                ft.transform.translation.y = state_estimation(1);
                ft.transform.translation.z = state_estimation(2);
                ft.transform.rotation.x = state_estimation(6);
                ft.transform.rotation.y = state_estimation(7);
                ft.transform.rotation.z = state_estimation(8);
                ft.transform.rotation.w = state_estimation(9);

                fta.transforms.push_back(ft);
                pose_pub->publish(fta);

                publisher(id+100, state_estimation(0) + 0.2, state_estimation(1), state_estimation(2), state_estimation(6), state_estimation(7), state_estimation(8), state_estimation(9), 0.0, 1.0);

            }  

        }
    }
}



int main(int argc, char **argv){

    ros::init(argc, argv, "listener");

    ros::NodeHandle nd;

    ros::Subscriber sub = nd.subscribe("/fiducial_transforms", 1000, measurementCallback);

    vis_pub = nd.advertise<visualization_msgs::Marker>("ponto3d", 0 );
    pose_pub = new ros::Publisher(nd.advertise<fiducial_msgs::FiducialTransformArray>("fiducial_transforms_filtered", 1));

    ros::spin();

    return 0;
}