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
vector<double> test;
using namespace std;
string header;
geometry_msgs::Twist time_test;
geometry_msgs::Point gyro_msgs_backup;
geometry_msgs::Point gyro_msgs;
sensor_msgs::Imu gyro_data,gyro_data_backup;
ros::Time time_backup;
int loop_rate;
int time_out;
int main(int argc, char **argv)
{
	ros::init(argc, argv, "Imu_talker_time_test");
	ros::NodeHandle nh("~");
	nh.param<int>("loop_rate",loop_rate, 30);
	nh.param<int>("time_out",time_out, 10 );
	ros::Publisher gyro_pub = nh.advertise<geometry_msgs::Point>("costly_imu_raw",100);
    	ros::Publisher gyro_data_pub = nh.advertise<sensor_msgs::Imu>("costly_imu",1);
	ros::Publisher time_test_pub = nh.advertise<geometry_msgs::Twist>("time_test",1);
	//rosbag::Bag Imu_bag("Imu_bag.bag", rosbag::bagmode::Write);
	ros::Rate loop_rate(loop_rate);
	std::string port = "/dev/costly_imu";
	uint32_t baudrate = 115200;
	serial::Serial mySerial;
	mySerial.setPort(port);
	mySerial.setBaudrate(baudrate);
	serial::Timeout to = serial::Timeout::simpleTimeout(time_out); 
        mySerial.setTimeout(to); 
	mySerial.open();
	int count = 0;
	while (ros::ok())
	{
	    //sensor_msgs::Imu gyro_data;
	    //gyro_data.header.stamp = ros::Time::now();
	    //gyro_data.header.frame_id = "/base_link_tmp";
	    std_msgs::String result, result_backup; 
            result.data = mySerial.readline(1024,"\n"); 
		
	    //cout<< "loop_rate" << loop_rate <<endl;
	    //ROS_INFO_STREAM("Read: " << result.data); 
	    mySerial.flush();//清空缓存
            //cout << "size: " << result.data.size() << endl; 
	    char s[result.data.size()];
	    //end1 = ros::Time::now();
	    for(int i = 0; i< result.data.size(); i++)
	    {
		s[i] = result.data[i];
	    }
	    for(int j=0; j< 5; j++)
	    {
		header.push_back(s[j]);
	    }
	    //cout << "header:" << header <<endl;
	    // check the HEADE
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
		test.push_back(atof(p));
		p=strtok(NULL,d);
		count ++;				
	    }
	    //cout << "test:"<< test[0]<< " " << test[1] <<" " <<test[2]<<" "<<test[3] <<" "<<test[4] <<" "<<test[5] <<" "<<test[6] <<" "<<test[7] <<" " <<test[8] <<" " <<test[9] <<" " <<test[10]<< " " << test[11]<<endl;
           //cout << "test.size(): " << test.size() << endl; 


	
		//std::cout << "test:" << test[0] << " " << test[1] << " "<< test[2] << " " << test[3] << " " << test[4] << " " << test[5] << " " << test[6] << " " << test[7] << " " << test[8] << " "<< test[9] << " " << test[10] << " " << test[11] << std::endl;
 		if(test.size() == 12 && header == "$CAIN")
	    {
	    	//数据解释
		//test[1]:光纤陀螺 角速率; test[2]:光纤陀螺积分角度; test[3]:MEMS roll(度)； test[4]:MEMS pitch; test[5]: MEMS X轴角速度； test[6]:MEMS Y轴角速度(度/s)； test[7]:MEMS Z轴角速度； test[8]: MEMS X轴加速度；test[9]: MEMS 轴加速度(m/s2)；test[10]: MEMS Z轴加速度；
		tf::Quaternion q;
		//角度转弧度
		gyro_msgs.x = test[3];
		gyro_msgs.y = test[4];
		gyro_msgs.z = test[2];
		
		gyro_msgs_backup.x = test[3];
		gyro_msgs_backup.y = test[4];
		gyro_msgs_backup.z = test[2];

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
		
		
		//gyro_data_backup.header.stamp = ros::Time::now();
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
		
		time_test.linear.y = 1.0;
	    }
	    else{
		
		//ROS_WARN("xw_gs1810: the imu data transform error, use the lastest data!");
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
		time_test.linear.y = -1.0;
		
	    }
	    gyro_data.header.frame_id = "base_link"; 
	    gyro_data.header.stamp = ros::Time::now();
	    time_test.linear.x  = (gyro_data.header.stamp - time_backup).toSec();
	    std::cout <<"time: "<< ros::Time::now() << "    " <<"dalta_time: " << time_test.linear.x << "         " << "flag "<< time_test.linear.y << std::endl;
	    time_backup= gyro_data.header.stamp;
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
	    test.clear();
	    header.clear();
	    time_test_pub.publish(time_test);
	    gyro_data_pub.publish(gyro_data);
	    gyro_pub.publish(gyro_msgs);
	    
	    ros::spinOnce();
	    loop_rate.sleep();
	}
	//ros::spin();
	//Imu_bag.close();
	return 0;
 }
