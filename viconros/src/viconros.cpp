//
// Created by uav on 2020/12/29.
//

#include <iostream>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>

// Motion Capture
#ifdef ENABLE_VICON
#include <libmotioncapture/vicon.h>
#endif


int main(int argc, char **argv)
{
    ros::init(argc, argv, "viconros");
    ros::NodeHandle nl("~");
    ros::Publisher    vicon_pub = nl.advertise<geometry_msgs::PoseStamped> ("mocap/pos", 1);
    ros::Publisher    vicon_pub1 = nl.advertise<geometry_msgs::TwistStamped> ("mocap/vel", 1);
    ros::Publisher    vicon_pub2 = nl.advertise<geometry_msgs::TwistStamped> ("mocap/acc", 1);

    geometry_msgs::PoseStamped msg;
    geometry_msgs::TwistStamped msg1;
    geometry_msgs::TwistStamped msg2;
    std::string hostName;
    std::string modelsegmentName;//model and segment must have same name

    std::string motionCaptureType;
    // Make a new client
    libmotioncapture::MotionCapture* mocap = nullptr;
    if (false)
    {
    }
#ifdef ENABLE_VICON
        else
  {
    nl.getParam("vicon_host_name", hostName);
    nl.getParam("modelsegment_name", modelsegmentName);
    std::cout<<"hostName:"<<hostName<<std::endl;
    std::cout<<"modelsegment_name:"<<modelsegmentName<<std::endl;
    mocap = new libmotioncapture::MotionCaptureVicon(hostName,
      /*enableObjects*/ true,
      /*enablePointcloud*/ true);
  }
#endif


//    pcl::PointCloud<pcl::PointXYZ>::Ptr markers(new pcl::PointCloud<pcl::PointXYZ>);
    // std::vector<libmotioncapture::Object> objects;
    libmotioncapture::Object object;
    while(ros::ok()){

        mocap->waitForNextFrame();
        if (mocap->supportsObjectTracking()) {
            // mocap->getObjects(objects);
            mocap->getObjectByName(modelsegmentName,object);
                //if object.occluded() is false, means  data available
                if (object.occluded() == false) {
                    Eigen::Vector3f position = object.position();
                    Eigen::Quaternionf rotation = object.rotation();
                    Eigen::MatrixXf velocityacceleration=object.velocityacceleration();
                    Eigen::Vector3f euler=object.euler();

                    msg.header.stamp=ros::Time::now();
                    msg.pose.position.x =position(0);
                    msg.pose.position.y =position(1);
                    msg.pose.position.z =position(2);

                    msg.pose.orientation.x = rotation.vec()(0);
                    msg.pose.orientation.y = rotation.vec()(1);
                    msg.pose.orientation.z = rotation.vec()(2);
                    msg.pose.orientation.w = rotation.w();

                    // velocity
                    msg1.header.stamp=ros::Time::now();
                    msg1.twist.linear.x =velocityacceleration(0,0);
                    msg1.twist.linear.y =velocityacceleration(0,1);
                    msg1.twist.linear.z =velocityacceleration(0,2);
                    msg1.twist.angular.x =velocityacceleration(2,0);
                    msg1.twist.angular.y =velocityacceleration(2,1);
                    msg1.twist.angular.z =velocityacceleration(2,2);

                    // acceleration
                    msg2.header.stamp=ros::Time::now();
                    msg2.twist.linear.x =velocityacceleration(1,0);
                    msg2.twist.linear.y =velocityacceleration(1,1);
                    msg2.twist.linear.z =velocityacceleration(1,2);
                    msg2.twist.angular.x =velocityacceleration(3,0);
                    msg2.twist.angular.y =velocityacceleration(3,1);
                    msg2.twist.angular.z =velocityacceleration(3,2);

                    std::cout<<object.name()+" current_pos"<<"   x:"<<msg.pose.position.x<<"   y:"<<msg.pose.position.y<<"   z:"<<msg.pose.position.z<<std::endl;
                    std::cout<<object.name()+" current_vel"<<"  vx:"<<msg1.twist.linear.x<<"   vy:"<<msg1.twist.linear.y<<"  vz:"<<msg1.twist.linear.z<<std::endl;
                    std::cout<<object.name()+" current_acc"<<"  ax:"<<msg2.twist.linear.x<<"   ay:"<<msg2.twist.linear.y<<"  az:"<<msg2.twist.linear.z<<std::endl;

                    std::cout<<object.name()+" current_Euler"<<"   roll:"<<euler(0)<<"   pitch:"<<euler(1)<<"   yaw:"<<euler(2)<<std::endl;
                    std::cout<<object.name()+" current_Quaternion"<<"  qx:"<<msg.pose.orientation.x<<"   qy:"<<msg.pose.orientation.y<<"   qz:"<<msg.pose.orientation.z<<"  qw"<<msg.pose.orientation.w<<std::endl;
                    std::cout<<object.name()+" current_agnlespeed"<<"  wx:"<<msg1.twist.angular.x<<"   wy:"<<msg1.twist.angular.y<<"  wz:"<<msg1.twist.angular.z<<std::endl;
                    std::cout<<object.name()+" current_angleacceleration"<<"  alphax:"<<msg2.twist.angular.x<<"   alphay:"<<msg2.twist.angular.y<<"  alphaz:"<<msg2.twist.angular.z<<std::endl;

                    vicon_pub.publish(msg);
                    vicon_pub1.publish(msg1);
                    vicon_pub2.publish(msg2);
                }
        }


        ros::spinOnce();
    }

    return 0;
}
