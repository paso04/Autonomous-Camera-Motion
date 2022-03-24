
#include <ros/ros.h>
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/PoseStamped.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include "rosserial_arduino/Adc.h"
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>


// ------------------------------
// Declation of GLOBAL VARIABLES
float PSM1x, PSM1y, PSM1z;
float PSM1ox, PSM1oy, PSM1oz, PSM1ow;

float PSM2x, PSM2y, PSM2z;
float PSM2ox, PSM2oy, PSM2oz, PSM2ow;

float ECMx, ECMy, ECMz;
float ECMox, ECMoy, ECMoz, ECMow;

int Clutch_pressed, Camera_pressed;

// bool Tool_in_FOV, Tool_in_Centre;
// float adc1, adc2, adc3, adc4;

ros::Time begin;
ros::Time now;
double time_sec;

//void ADCcallback(const rosserial_arduino::Adc::ConstPtr& msg1)
//{
//  adc1 = msg1->adc0;
//  adc2= msg1->adc1;
//  adc3 = msg1->adc2;
//  adc4 = msg1->adc3;
//}

// ------------------------------

void ToolCallbackPSM1(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // Position
    PSM1x = msg->pose.position.x;
    PSM1y = msg->pose.position.y;
    PSM1z = msg->pose.position.z;

    // Orientation
    PSM1ox = msg->pose.orientation.x;
    PSM1oy = msg->pose.orientation.y;
    PSM1oz = msg->pose.orientation.z;
    PSM1ow = msg->pose.orientation.w;

}

void ToolCallbackPSM2(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // Position
    PSM2x = msg->pose.position.x;
    PSM2y = msg->pose.position.y;
    PSM2z = msg->pose.position.z;

    // Orientation
    PSM2ox = msg->pose.orientation.x;
    PSM2oy = msg->pose.orientation.y;
    PSM2oz = msg->pose.orientation.z;
    PSM2ow = msg->pose.orientation.w;

}

void ToolCallbackECM(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // Position
    ECMx = msg->pose.position.x;
    ECMy = msg->pose.position.y;
    ECMz = msg->pose.position.z;

    // Orientation
    ECMox = msg->pose.orientation.x;
    ECMoy = msg->pose.orientation.y;
    ECMoz = msg->pose.orientation.z;
    ECMow = msg->pose.orientation.w;

}

void Clutch_Callback(const sensor_msgs::Joy::ConstPtr& msg)
{
    Clutch_pressed=msg->data;
}

void Camera_Callback(const sensor_msgs::Joy::ConstPtr& msg)
{
    Camera_pressed=msg->data;
}

// void Tool_in_FOV_Callback(const std_msgs::Bool::ConstPtr& msg)
// {
//     Tool_in_FOV=msg->data;
// }

// void Tool_in_Centre_Callback(const std_msgs::Bool::ConstPtr& msg)
// {
//     Tool_in_Centre=msg->data;
// }


int main(int argc, char **argv)
{
  // ------------------ NODE INITIALIZATION -------------------------------
    
  ros::init(argc, argv, "recording_node");
  ros::NodeHandle nh;

  ros::Subscriber sub_PSM1 = nh.subscribe("/PSM1/measured_cp", 1000, ToolCallbackPSM1);
  ros::Subscriber sub_PSM2 = nh.subscribe("/PSM2/measured_cp", 1000, ToolCallbackPSM2);
  ros::Subscriber sub_ECM = nh.subscribe("/ECM/measured_cp", 1000, ToolCallbackECM);

  ros::Subscriber sub_Clutch_pressed = nh.subscribe("/footpedals/clutch", 1000, Clutch_Callback);
  ros::Subscriber sub_Camera_pressed = nh.subscribe("/footpedals/camera", 1000, Camera_Callback);

  // ros::Subscriber sub_Tool_in_FOV = nh.subscribe("/Tool_in_FOV", 1000, Tool_in_FOV_Callback);
  // ros::Subscriber sub_Tool_in_Centre = nh.subscribe("/Tool_in_Centre", 1000, Tool_in_Centre_Callback);

  ros::Rate loop_rate(100);

  // ------------------ NODE INITIALIZATION -------------------------------
  

  // Get input from keyboard to set filename
  std::string filename;
  std::cout<<"Please, enter the filename: ";
  std::getline(std::cin, filename);
  //  std::cout<<"Your filename is: "<<filename << std::endl;

  std::string csv_filename;
  csv_filename = filename + ".csv";
//  std::cout<<"Your filename is: "<<csv_filename << std::endl;

  std::ofstream file_csv;
  file_csv.open(csv_filename.c_str());
  file_csv<<"Time"<<","
//          <<"Tool_in_FOV"<<","
//          <<"Tool_in_Centre"<<","
//          <<"ADC1"<<","
//          <<"ADC2"<<","
//          <<"ADC3"<<","
//          <<"ADC4"<<","
          <<"Clutch_pressed"<<","
          <<"Camera_pressed"<<","
          <<"PSM1_Position_x"<<","
          <<"PSM1_Position_y"<<","
          <<"PSM1_Position_z"<<","
          <<"PSM1_Orientation_x"<<","
          <<"PSM1_Orientation_y"<<","
          <<"PSM1_Orientation_z"<<","
          <<"PSM1_Orientation_w"<<","
          <<"PSM2_Position_x"<<","
          <<"PSM2_Position_y"<<","
          <<"PSM2_Position_z"<<","
          <<"PSM2_Orientation_x"<<","
          <<"PSM2_Orientation_y"<<","
          <<"PSM2_Orientation_z"<<","
          <<"PSM2_Orientation_w"<<","
          <<"ECM_Position_x"<<","
          <<"ECM_Position_y"<<","
          <<"ECM_Position_z"<<","
          <<"ECM_Orientation_x"<<","
          <<"ECM_Orientation_y"<<","
          <<"ECM_Orientation_z"<<","
          <<"ECM_Orientation_w"<<std::endl;


  begin = ros::Time::now();


  while (ros::ok())
    {


          ros::spinOnce();

          now = ros::Time::now();

          time_sec=(now-begin).toSec();

          // Write data to csv

          file_csv<<time_sec<<","
//                  <<Tool_in_FOV<<","
//                  <<Tool_in_Centre<<","
//                  <<adc1<<","
//                  <<adc2<<","
//                  <<adc3<<","
//                  <<adc4<<","
                  <<Clutch_pressed<<","
                  <<Camera_pressed<<","
                  <<PSM1x<<","
                  <<PSM1y<<","
                  <<PSM1z<<","
                  <<PSM1ox<<","
                  <<PSM1oy<<","
                  <<PSM1oz<<","
                  <<PSM1ow<<","
                  <<PSM2x<<","
                  <<PSM2y<<","
                  <<PSM2z<<","
                  <<PSM2ox<<","
                  <<PSM2oy<<","
                  <<PSM2oz<<","
                  <<PSM2ow<<","
                  <<ECMx<<","
                  <<ECMy<<","
                  <<ECMz<<","
                  <<ECMox<<","
                  <<ECMoy<<","
                  <<ECMoz<<","
                  <<ECMow<<std::endl;


          loop_rate.sleep();

      }

          file_csv.close();
          std::cout<<"Closing and saving the csv file!!!";


          ros::shutdown();


}
