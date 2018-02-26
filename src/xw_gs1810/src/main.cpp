#include <ros/ros.h> 
#include <serial/serial.h>  //ROS已经内置了的串口包 
#include <std_msgs/String.h> 
#include <std_msgs/Empty.h> 
#include <boost/algorithm/string/split.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <boost/algorithm/string/split.hpp>
#include <stdio.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <sstream>
#include <math.h>
#include "xw_gs1810/time_util.h"
using namespace std;
serial::Serial ser; //声明串口对象 
geometry_msgs::Point gyro_msgs_backup;
geometry_msgs::Point gyro_msgs;
sensor_msgs::Imu gyro_data,gyro_data_backup;
string header;
ros::Time start, end1, end2;
//vector<double> test;
double test[12]={0};
//回调函数 
//void write_callback(const std_msgs::String::ConstPtr& msg) 
//{ 
//    ROS_INFO_STREAM("xw_gs1810:Writing to serial port" <<msg->data); 
//   ser.write(msg->data);   //发送串口数据 
//} 

int main (int argc, char** argv) 
{ 
    //初始化节点 
    ros::init(argc, argv, "read_gyro_ros"); 
    //声明节点句柄 
    ros::NodeHandle nh; 
    
    //freopen("/home/administrator/tmp/xw_gs1810.txt", "w", stdout) ;
    //订阅主题，并配置回调函数 
    //ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback); 
    //发布主题 
    //ros::Publisher read_pub = nh.advertise<std_msgs::String>("read", 1000); 
    ros::Publisher gyro_pub = nh.advertise<geometry_msgs::Point>("costly_imu_raw",100);
    ros::Publisher gyro_data_pub = nh.advertise<sensor_msgs::Imu>("costly_imu",1);

    try 
    { 
    //设置串口属性，并打开串口 
        ser.setPort("/dev/costly_imu"); 
        ser.setBaudrate(115200); 
        serial::Timeout to = serial::Timeout::simpleTimeout(50); 
        ser.setTimeout(to); 
        ser.open(); 
    } 
    catch (serial::IOException& e) 
    { 
        ROS_ERROR_STREAM("xw_gs1810: Unable to open port "); 
        return -1; 
    } 

    //检测串口是否已经打开，并给出提示信息 
    if(ser.isOpen()) 
    { 
        ROS_INFO_STREAM("xw_gs1810: Serial Port initialized"); 
    } 
    else 
    { 
        return -1; 
    } 

    //指定循环的频率 
    ros::Rate loop_rate(20); 
    while(ros::ok()) 
    { 

	
        if(ser.available()){ 
	    start = ros::Time::now();
            //ROS_INFO_STREAM("Reading from serial port\n"); 
            std_msgs::String result; 
            result.data = ser.readline(2048,"\n"); 	
	    //cout<< "for test" << endl;
	    ser.flush();//清空缓存
            //ROS_INFO_STREAM("Read: " << result.data); 
	    char s[result.data.size()];
	    end1 = ros::Time::now();
	    for(int i = 0; i< result.data.size(); i++)
	    {
		s[i] = result.data[i];
	    }
	    for(int j=0; j< 5; j++)
	    {
		header.push_back(s[j]);
	    }
	    //cout << "header:" << header <<endl;
	    // check the HEADER
	    
	    char *p;
	    float *q;
	    const char *d=",";
            p = strtok(s,d);
	    int count =0;
	    while(p)
	    {
		//printf("%s\b\b",p);
		//printf("%f\n",atof(p));
		//test.push_back(atof(p));
		test[count] = atof(p);
		p=strtok(NULL,d);
		count ++;				
	    }
	    //cout << "test:" << test[0]<< test[1] <<" " <<test[2]<<" "<<test[3] <<" "<<test[4] <<" "<<test[5] <<" "<<test[6] <<" "<<test[7] <<" " <<test[8] <<" " <<test[9] <<" " <<test[10]<<" " << test[11]<< endl;
	    //cout << "test.size()" << test.size() << endl;
	    //cout << "result.data.size(): " << result.data.size() << endl;
	    //gyro_data.orientation.w = 1.0;
	    //gyro_data.orientation.x = 0.0;
	    //gyro_data.orientation.y = 0.0;
	    //gyro_data.orientation.z = 0.0;
	    
	    if(header == "$CAIN")
	    {
	    	//数据解释
		//test[1]:光纤陀螺 角速率; test[2]:光纤陀螺积分角度; test[3]:MEMS roll(度)； test[4]:MEMS pitch; test[5]: MEMS X轴角速度； test[6]:MEMS Y轴角速度(度/s)； test[7]:MEMS Z轴角速度； test[8]: MEMS X轴加速度；test[9]: MEMS 轴加速度(m/s2)；test[10]: MEMS Z轴加速度；
		tf::Quaternion q;
		//角度转弧度
		
		test[2]=fmod((test[2]/180*M_PI+M_PI),(2*M_PI))-M_PI;//yaw[rad]
		test[3]=test[3]/180*M_PI;//roll[rad]
		test[4]=test[4]/180*M_PI;//pitch[rad]
		
		//角速率转化
		test[1] = test[1]/180*M_PI; //vel_yaw[rad/s]
		test[5] = test[5]/180*M_PI; //vel_roll[rad/s]
		test[6] = test[6]/180*M_PI; //vel_picth[rad/s] 
		
		q = tf::createQuaternionFromRPY(test[3],test[4],test[2]);
		//gyro_data.orientation.w = cos(test[3]/2)*cos(test[4]/2)*cos(test[2]/2)+sin(test[3]/2)*sin(test[4]/2)*sin(test[2]/2);
		//gyro_data.orientation.x = sin(test[3]/2)*cos(test[4]/2)*cos(test[2]/2)-cos(test[3]/2)*sin(test[4]/2)*sin(test[2]/2);
		//gyro_data.orientation.y = cos(test[3]/2)*sin(test[4]/2)*cos(test[2]/2)+sin(test[3]/2)*cos(test[4]/2)*sin(test[2]/2);
		//gyro_data.orientation.z = cos(test[3]/2)*cos(test[4]/2)*sin(test[2]/2)-sin(test[3]/2)*sin(test[4]/2)*cos(test[2]/2);
		gyro_data.orientation.w = q.w();
		gyro_data.orientation.x = q.x();
		gyro_data.orientation.y = q.y();
		gyro_data.orientation.z = q.z();
		
		gyro_data.angular_velocity.x = test[3];
		gyro_data.angular_velocity.y = test[4];
		gyro_data.angular_velocity.z = test[2];
		
		gyro_data.linear_acceleration.x = test[8];
		gyro_data.linear_acceleration.y = test[9];
		gyro_data.linear_acceleration.z = test[10]-9.8;
		
		gyro_msgs.x = test[3];
		gyro_msgs.y = test[4];
		gyro_msgs.z = test[2];
		
		gyro_msgs_backup.x = test[3];
		gyro_msgs_backup.y = test[4];
		gyro_msgs_backup.z = test[2];
		
		gyro_data_backup.orientation.w = gyro_data.orientation.w;
		gyro_data_backup.orientation.x = gyro_data.orientation.x;
		gyro_data_backup.orientation.y = gyro_data.orientation.y;
		gyro_data_backup.orientation.z = gyro_data.orientation.z;
		
		gyro_data_backup.angular_velocity.x = gyro_data.angular_velocity.x;
		gyro_data_backup.angular_velocity.y = gyro_data.angular_velocity.y;
		gyro_data_backup.angular_velocity.z = gyro_data.angular_velocity.z;

		gyro_data_backup.linear_acceleration.x = gyro_data.linear_acceleration.x;
		gyro_data_backup.linear_acceleration.y = gyro_data.linear_acceleration.y;
		gyro_data_backup.linear_acceleration.z = gyro_data.linear_acceleration.z;
		
		
	    }
	    else{
		
		ROS_WARN("xw_gs1810: the imu data transform error, use the lastest data!");
		gyro_data.orientation.w = gyro_data_backup.orientation.w;
		gyro_data.orientation.x = gyro_data_backup.orientation.x;
		gyro_data.orientation.y = gyro_data_backup.orientation.y;
		gyro_data.orientation.z = gyro_data_backup.orientation.z;
		gyro_data.angular_velocity.x = gyro_data_backup.angular_velocity.x;
		gyro_data.angular_velocity.y = gyro_data_backup.angular_velocity.y;
		gyro_data.angular_velocity.z = gyro_data_backup.angular_velocity.z;

		gyro_data.linear_acceleration.x = gyro_data_backup.linear_acceleration.x;
		gyro_data.linear_acceleration.y = gyro_data_backup.linear_acceleration.y;
		gyro_data.linear_acceleration.z = gyro_data_backup.linear_acceleration.z;
		
		gyro_msgs.x = gyro_msgs_backup.x;
		gyro_msgs.y = gyro_msgs_backup.y;
		gyro_msgs.z = gyro_msgs_backup.z;
		
	    }
	    
	    gyro_data.header.frame_id = "base_link"; 
	    gyro_data.header.stamp = ros::Time::now();
	    gyro_data.orientation_covariance[0] = 1e-05;
	    gyro_data.orientation_covariance[1] = 0.0;
	    gyro_data.orientation_covariance[2] = 0.0;
	    
	    gyro_data.orientation_covariance[3] = 0.0;
	    gyro_data.orientation_covariance[4] = 1e-05;
	    gyro_data.orientation_covariance[5] = 0.0;
	    
	    gyro_data.orientation_covariance[6] = 0.0;
	    gyro_data.orientation_covariance[7] = 0.0;
	    gyro_data.orientation_covariance[8] = 1e-6;
	    
	    
	    gyro_data.angular_velocity_covariance[0] = 1e-05;
	    gyro_data.angular_velocity_covariance[1] = 0.0;
	    gyro_data.angular_velocity_covariance[2] = 0.0;
	    
	    gyro_data.angular_velocity_covariance[3] = 0.0;
	    gyro_data.angular_velocity_covariance[4] = 1e-05;
	    gyro_data.angular_velocity_covariance[5] = 0.0;
	    
	    gyro_data.angular_velocity_covariance[6] = 0.0;
	    gyro_data.angular_velocity_covariance[7] = 0.0;
	    gyro_data.angular_velocity_covariance[8] = 1e-6;
	    //test.clear();
	    header.clear();
	    gyro_data_pub.publish(gyro_data);
	    gyro_pub.publish(gyro_msgs);
            //read_pub.publish(result); 
	    end2 =ros::Time::now();
            double delta_end = (end2 - start).toSec();
            double delta_min = (end1- start).toSec();
	    //cout << "delta_time: " <<delta_end <<" " <<delta_min<< endl;
	    
        }
	
        //处理ROS的信息，比如订阅消息,并调用回调函数 
        ros::spinOnce(); 
        loop_rate.sleep(); 
	//ros::spin();
    } 
} 


















