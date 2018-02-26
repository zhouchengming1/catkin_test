#include<ros/ros.h>
//#include<loam_continuous/Angle.h>
#include <iostream>
#include<sstream>
#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include<sys/types.h>
#include<sys/stat.h>
#include<fcntl.h>  //文件控制定义，主要完成串口通信中对文件的读写操作
#include<unistd.h>  //　linux标准函数定义
#include<termios.h>  // POSIX终端控制定义
#include <boost/graph/graph_concepts.hpp>
#include<string.h>
#include <sensor_msgs/Imu.h>
#include <boost/lexical_cast.hpp>
#include <tf/transform_datatypes.h>
#include <boost/algorithm/string.hpp>

//using namespace loam_continuous;
using std::vector;
using std::string;


int set_opt(int fd,int nSpeed, int nBits, char nEvent, int nStop)  //打开设备的文件描述符，波特率，位数，校验，停止位
{
    //newtio:新的端口设置 ,oldtio:之前的端口设置
    struct termios newtio,oldtio;
    if  ( tcgetattr( fd,&oldtio)  !=  0) {
        perror("SetupSerial 1");
        return -1;
    }
    bzero( &newtio, sizeof( newtio ) );	//将该结构体里的数据全部设置为０
    newtio.c_cflag  |=  CLOCAL | CREAD;
    newtio.c_cflag &= ~CSIZE;  //将以前的标记位清空

    switch( nBits )
    {
        case 7:
            newtio.c_cflag |= CS7;
            break;
        case 8:
            newtio.c_cflag |= CS8;
            break;
    }

    switch( nEvent )
    {
        case 'O':
            newtio.c_cflag |= PARENB;
            newtio.c_cflag |= PARODD;
            newtio.c_iflag |= (INPCK | ISTRIP);
            break;
        case 'E':
            newtio.c_iflag |= (INPCK | ISTRIP);
            newtio.c_cflag |= PARENB;
            newtio.c_cflag &= ~PARODD;
            break;
        case 'N':
            newtio.c_cflag &= ~PARENB;
            break;
    }

    switch( nSpeed )
    {
        case 2400:
            cfsetispeed(&newtio, B2400);
            cfsetospeed(&newtio, B2400);
            break;
        case 4800:
            cfsetispeed(&newtio, B4800);
            cfsetospeed(&newtio, B4800);
            break;
        case 9600:
            cfsetispeed(&newtio, B9600);
            cfsetospeed(&newtio, B9600);
            break;
        case 115200:
            cfsetispeed(&newtio, B115200);
            cfsetospeed(&newtio, B115200);
            break;
        case 460800:
            cfsetispeed(&newtio, B460800);
            cfsetospeed(&newtio, B460800);
            break;
        default:
            cfsetispeed(&newtio, B9600);
            cfsetospeed(&newtio, B9600);
            break;
    }

    if( nStop == 1 )
        newtio.c_cflag &=  ~CSTOPB;
    else if ( nStop == 2 )
        newtio.c_cflag |=  CSTOPB;

    /*当已读了指定量的数据后，或者已经过了给定的时间后，即通知系统返回。这种技术使用了termios结构中c_cc数组的两个变量：MIN和TIME。
    C_cc数据中的这两个元素的下标名为VMIN和VTIME。MIN说明一个read返回前的最小字节数。8TIME说明等待数据到达的分秒数。*/
    newtio.c_cc[VTIME]  = 0;
    newtio.c_cc[VMIN] = 5;

    tcflush(fd,TCIFLUSH); //对输入缓冲区进行清空

    if((tcsetattr(fd,TCSANOW,&newtio))!=0)  //设置新的串口属性生效
    {
        perror("com set error");
        return -1;
    }
    printf("set done!\n\r");
    return 0;
}

int main(int argc,char **argv)
{

    ros::init(argc,argv,"Imu_talker");
    ros::NodeHandle n;
    //ros::Publisher chatter_pub=n.advertise<Angle>("angle",2);
    ros::Rate loop_rate(100);
    int fd1,nset1,nread;
    char buf[1];
    char *pos = buf;
    int angle;
    float angle1;
    double delay;
    char sta = 's';
    char end = 'e';

    n.param<double>("delay", delay, 0);
    fd1 = open("/dev/costly_imu", O_RDWR);//打开串口
    if (fd1 == -1)
    {
        ROS_INFO("serial port open error !/n");
        exit(1);
    }
    else
        ROS_INFO("serial port open success!");

    nset1 = set_opt(fd1, 115200, 8, 'N', 1);//设置串口属性
    if (nset1 == -1)
    {
        ROS_INFO("serial port set error!/n");
        exit(1);
    }
    else
        ROS_INFO("serial port set success!");

    bool isFirstTime = true;

    int nCounter = 0;
    std::string currLine;
    ros::Time t;
    sensor_msgs::Imu gyro_data;
    ros::Publisher gyro_data_pub = n.advertise<sensor_msgs::Imu>("costly_imu",1);

    while(true)
    {

        nread = read(fd1, buf, 1);

        if(nread)
        {
            std::string strr = buf;
	
            if(strr == "$")
            {
                t = ros::Time::now();
                if(isFirstTime)
                {
                    currLine.clear();
                    isFirstTime = false;
                }else{

                    try{		    
	
                    std::string fullLine = currLine;	    

                    //publish imu information.
                    vector<string> elems;

                    boost::split(elems, fullLine, boost::is_any_of(","));

                    if (elems.size() < 11 || elems[0] != "$CAIN") continue;

                    tf::Quaternion q;

		    //try
		    //{
                    double imu_yaw = fmod((M_PI + (boost::lexical_cast<double>(elems[2]) * M_PI / 180.0)), (2 * M_PI)) -
                                     M_PI;//yaw[rad]
                    double imu_roll = boost::lexical_cast<double>(elems[3]) * M_PI / 180.0;//roll[rad]
                    double imu_pitch = boost::lexical_cast<double>(elems[4]) * M_PI / 180.0;//pitch[rad]

                    q = tf::createQuaternionFromRPY(imu_roll, imu_pitch, imu_yaw);
                    gyro_data.orientation.w = q.w();
                    gyro_data.orientation.x = q.x();
                    gyro_data.orientation.y = q.y();
                    gyro_data.orientation.z = q.z();
		    //}
		    //catch(boost::bad_lexical_cast & e)
		    //{
			//ROS_INFO_STREAM("yaw roll pitch input error: " << e.what());
			//currLine.clear();
			//continue;
		    //}
                    //cout<<"imu yaw = "<<imu_yaw<<endl;
		    

		    //try
	    	    //{
                    double yaw_velo = boost::lexical_cast<double>(elems[1]) * M_PI / 180.0; //vel_yaw[rad/s]
                    double roll_velo = boost::lexical_cast<double>(elems[5]) * M_PI / 180.0; //vel_roll[rad/s]
                    double pitch_velo = boost::lexical_cast<double>(elems[6]) * M_PI / 180.0; //vel_picth[rad/s]

		    gyro_data.angular_velocity.x = roll_velo;
                    gyro_data.angular_velocity.y = pitch_velo;
                    gyro_data.angular_velocity.z = yaw_velo;

		    //}
		    //catch(boost::bad_lexical_cast & e)
		    //{
			//ROS_INFO_STREAM("yaw roll pitch velo input error: " << e.what());
			//currLine.clear();
			//continue;
 		    //}	

		   // try
		    //{
                    gyro_data.linear_acceleration.x = boost::lexical_cast<double>(elems[8]);
                    gyro_data.linear_acceleration.y = boost::lexical_cast<double>(elems[9]);
                    gyro_data.linear_acceleration.z = boost::lexical_cast<double>(elems[10]);

                    gyro_data.header.frame_id = "base_link";
                    gyro_data.header.stamp = t;
                    gyro_data.orientation_covariance[0] = 4e-4;
                    gyro_data.orientation_covariance[1] = 0.0;
                    gyro_data.orientation_covariance[2] = 0.0;

                    gyro_data.orientation_covariance[3] = 0.0;
                    gyro_data.orientation_covariance[4] = 4e-4;
                    gyro_data.orientation_covariance[5] = 0.0;

                    gyro_data.orientation_covariance[6] = 0.0;
                    gyro_data.orientation_covariance[7] = 0.0;
                    gyro_data.orientation_covariance[8] = 1e-6;


                    gyro_data.angular_velocity_covariance[0] = 4e-4;
                    gyro_data.angular_velocity_covariance[1] = 0.0;
                    gyro_data.angular_velocity_covariance[2] = 0.0;

                    gyro_data.angular_velocity_covariance[3] = 0.0;
                    gyro_data.angular_velocity_covariance[4] = 4e-4;
                    gyro_data.angular_velocity_covariance[5] = 0.0;

                    gyro_data.angular_velocity_covariance[6] = 0.0;
                    gyro_data.angular_velocity_covariance[7] = 0.0;
                    gyro_data.angular_velocity_covariance[8] = 1e-6;
		    //}
		    //catch(boost::bad_lexical_cast & e)
		    //{
		    //	ROS_INFO_STREAM("acceleration input error: " << e.what());
			//currLine.clear();
			//continue;
                    //}

                    gyro_data_pub.publish(gyro_data);

			//std::cout<<"yaw angle = "<<imu_yaw<<std::endl;


                    currLine.clear();
                    } 
                    catch(...)
                    {
                         currLine.clear();
                    }
                }
            }
            currLine += strr;
        }

        //nCounter++;
    }

    close(fd1);
    return 0;
}
