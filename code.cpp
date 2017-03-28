#include <ros/ros.h>
#include <mavros_msgs/mavlink_convert.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Mavlink.h>
#include <sensor_msgs/Imu.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

//#include <opencv2/opencv.hpp>
//#include <opencv2/cxcore.h>
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include <Eigen/Dense>

#include <iostream>
#include <fstream>
using namespace std;
using namespace Eigen;
mavros_msgs::State current_state;
mavros_msgs::Mavlink current_mav;

mavlink_message_t mmsg;
mavlink_attitude_t attitude;
mavlink_local_position_ned_t localpos;

geometry_msgs::PoseStamped pose;
geometry_msgs::PoseStamped current_position;
mavros_msgs::ActuatorControl current_control;
cv_bridge::CvImage img_msg;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void mav_cb(const mavros_msgs::Mavlink::ConstPtr& msg){
    current_mav = *msg;
    mavros_msgs::mavlink::convert(current_mav, mmsg);
    switch (mmsg.msgid){
        case MAVLINK_MSG_ID_ATTITUDE:
            mavlink_msg_attitude_decode(&mmsg,&attitude);
            roll1=attitude.roll;
            pitch1=attitude.pitch;
            yaw=attitude.yaw;
            yawspeed=attitude.yawspeed;
            // mavlink_euler_to_quaternion(attitude.roll,attitude.pitch,attitude.yaw,q);
            break;
        case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
            mavlink_msg_local_position_ned_decode(&mmsg,&localpos);
            z=-localpos.z;
            v=-localpos.vz;
            break;

    }
    
}

void control_cb(const mavros_msgs::ActuatorControl& msg){
    current_control = *msg;
}

void position_cb(const geometry_msgs::PoseStamped& msg){
    current_position = *msg;
    pose = *msg;
}

int main(int argc, char **argv)
{
    
    /***************************************
    *
    * FourP Variables 
    *
    ***************************************/

    /***************************************
    *
    * PID Variables 
    *
    ***************************************/
    float pE=0,iE=0,dE=0,ek=0,ek1=0,ek2=0;
    float kp=0,ki=0,kd=0;
    //Matrix and vectors
    //Matrix3f Rx4,Rx5,R;
    //Vector3f p[4],tmpv_arr[4],q,q1,tmpv,voe,dvoe,qe1,dqe1,qee,ve,q2,f;
    //RowVector3f tmph;
    /***************************************
    *
    * Flying log 
    *
    ***************************************/


    
    ifstream infile("para.txt");
    infile>>kp>>ki>>kd>>thr_sp>>r_err>>p_err;
    infile.close();
    /***************************************
    *
    *  ROS
    *
    ***************************************/
    ros::init(argc, argv, "dachuang");
    ros::NodeHandle nh, mav_sub_handle, cont_sub_handle, pose_sub_ha
    ros::NodeHandle cont_pub_handle, pose_pub_handle ;
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber mav_sub = mav_sub_handle.subscribe<mavros_msgs::Mavlink>
            ("mavlink/from",10,mav_cb);
    ros::Subscriber cont_sub = cont_sub_handle.subscribe<mavros_msgs::ActuatorControl>
            ("mavros/actuator_control",10,control_cb);
    ros::Subscriber pose_sub = pose_sub_handle.subscribe<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local",10,position_cb);

    ros::Publisher pose_pub = pose_pub_handle.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::Publisher cont_pub = cont_pub_handle.advertise<mavros_msgs::ActuatorControl>
            ("mavros/actuator_control",10);
    

    ros::Rate rate(30.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    /***************************************
    *
    *  OpenCV
    *
    ***************************************/
    IplImage* pFrame = NULL;
    CvCapture* pCapture = cvCreateCameraCapture(-1);    // -1 means to prompt a window of cameras for user to choose

    //Frame Width and Height
    cvSetCaptureProperty(pCapture, CV_CAP_PROP_FRAME_WIDTH, 480);
    cvSetCaptureProperty(pCapture, CV_CAP_PROP_FRAME_HEIGHT, 270);
    pFrame=cvQueryFrame( pCapture );    // fetch a frame from camera
    
    CvVideoWriter *writer=cvCreateVideoWriter("video.avi",CV_FOURCC('M','J','P','G'),5,cvGetSize(pFrame));
    //  ( const char* filename, int fourcc, double fps, CvSize frame_size, int is_color=1 )
    // CV_FOURCC('M','J','P','G') = motion-jpeg codec
    //fps 被创建视频流的帧率。
    //frame_size 视频流的大小。

    /***************************************
    *
    *  Calculating and Publishing
    *
    ***************************************/
    int piccnt = 0;
    bool isfirst = true;
    while(ros::ok()) {
        pFrame=cvQueryFrame( pCapture );
        if(!pFrame)break;
        
        

        ROS_INFO("%f",yaw);
        if (current_state.mode=="OFFBOARD"){
            if (piccnt == 4){
                cvWriteFrame(writer,pFrame);
                piccnt=0;
            }else {
                piccnt++;
            }

            pose_pub.publish(pose);
        }else{
            isfirst=true; 

            pose_pub.publish(pose);
        }

        ros::spinOnce();
        rate.sleep();
    }
    cvReleaseCapture(&pCapture);
    cvReleaseVideoWriter(&writer);
    //cvDestroyWindow("video");
    outfile.close();
    return 0;
}

