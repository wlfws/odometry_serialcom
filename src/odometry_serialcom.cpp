/*********************************************************************
 * Software License Agreement (BSD License)
 * Copyright (c) 2013, Intelligent Robotics Lab, DLUT.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. and Intelligent Robotics 
 *        Lab, DLUT nor the names of its contributors may be used to endorse or
 *       promote products derived from this software without specific prior written 
 *       permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
#include "std_msgs/String.h"
#include "odometry_serialcom/odometry_serialcom.h"


#define DEFAULT_PORT 1
#define DEFAULT_BAUDRATE 9600
#define MAX_BUF 1024
typedef unsigned char BYTE;
typedef unsigned short int WORD;

#define CRC16_GEN_POL 0xa001
#define MKSHORT(a,b) ((unsigned short) (a) | ((unsigned short)(b) << 8))
#include <tf/transform_broadcaster.h>

#include <iostream>
#include <fstream>
unsigned char _read0[10],_read1[10];
bool b_is_received_params = false;
bool b_reset_odom = false;
double d_frequency;
double d_wheel_separation;
double d_height;
int i_gear_ratio;
int i_cpr;
double d_wheel_diameter;
int fd_;//the file discriptor of the serial port
using namespace std;
class SerialCom
{
public:
  SerialCom();
  ~SerialCom();
  int openPort(int fd, int comport);
  int setOpt(int fd,int nspeed,int nbits,char nevent,int nstop);
  int packSend(int fd,char ptr,char buff[]);
  int comInit();
  void SetBuf(char* buf,bool change/* =false */,bool addCRC/*=false*/);
  void sendcommand();
  ros::NodeHandle serial_node_handle_;

private:

  char _buf[MAX_BUF+3];//传输data，中间值

  char _crc[10];
  unsigned char CRC_H,CRC_L;		
  int _len;
  int port_,baudrate_;
  char command_buff_[30];
  ros::Time current_time_,last_time_;
  int nwrite;
  int nread;
  char hex[16];
public:	
  void SetBuf(char* buf,int len,bool change,bool addCRC);
  void Clear();
  char* GetBuf();
  int getLen();
  WORD CRC16(unsigned char *p,WORD len);
  void delay(unsigned int z);

 
  bool operator==(SerialCom& in);
};



SerialCom::SerialCom()
{
  char hex_[16]={'0','1','2','3','4','5','6','7','8','9','a','b','c','d','e','f'};
  memcpy(hex, hex_, sizeof(hex)); 
  serial_node_handle_.param("Port",port_,DEFAULT_PORT);
  serial_node_handle_.param("BaudRate",baudrate_,DEFAULT_BAUDRATE);
  comInit();
  
  
  
  
  //ROS_INFO("data_number:%d\n",nwrite);
  
}

SerialCom::~SerialCom()
{
  close(fd_);
}

//open the serial port
int SerialCom::openPort(int fd, int comport)
{
  if(comport == 1)//port 1
  {
    fd = open("/dev/ttyMXUSB0",O_RDWR|O_NOCTTY|O_NDELAY);
    
    if(-1  ==  fd)
    {
      ROS_INFO("failed to open /dev/ttyUSB0！");
      return(-1);
    }
  }
  else if(comport == 2)//port 2
  {
    fd = open("/dev/ttyMXUSB1",O_RDWR|O_NOCTTY|O_NDELAY);
    if(-1 == fd)
    {
      ROS_INFO("failed to open /dev/ttyUSB1！");
      return(-1);
    }
  }

  if(fcntl(fd,F_SETFL,0)<0)
    ROS_INFO("fcntl failed!");
  else
    ROS_INFO("fcntl=%d.",fcntl(fd,F_SETFL,0));

  if(isatty(STDIN_FILENO) == 0)
    ROS_INFO("standsrd input is not a terminal device.");
  else
    ROS_INFO("is a tty sucess.");

  ROS_INFO("the return value of the serial open function=%d，!=-1,indicates succeed to open the serial.",fd);

  return fd;
}

//setup the serial port
int SerialCom::setOpt(int fd,int nspeed,int nbits,char nevent,int nstop)
{
  struct termios newtio,oldtio;

  //check the parameter of the serial port to see whether there is error or not
  if(tcgetattr(fd,&oldtio) != 0)
  {
    ROS_INFO("failed to setup the serial，failed to save the serial value!");
    return -1;	
  }

  bzero(&newtio,sizeof(newtio));

  newtio.c_cflag |= CLOCAL | CREAD;
  newtio.c_cflag &= ~CSIZE;

  switch(nbits)
  {
    case 7:
      newtio.c_cflag |= CS7;
      break;
    case 8:
      newtio.c_cflag |= CS8;
      break;	
  }

  switch(nevent)
  {
    case 'O': 
      newtio.c_cflag |= PARENB;
      newtio.c_cflag |= PARODD;
      newtio.c_iflag |= (INPCK | ISTRIP);
      break;
    case 'E':
      newtio.c_iflag |= (INPCK | ISTRIP);
      newtio.c_cflag |= PARENB ;
      newtio.c_cflag &= ~ PARODD;
      break;
    case 'N':
      newtio.c_cflag &= ~ PARENB;
      break;		
  }

  //setup the baud rate
  switch(nspeed)
  {
    case 2400:
      cfsetispeed(&newtio,B2400);
      cfsetospeed(&newtio,B2400);
      break;		
    case 4800:
      cfsetispeed(&newtio,B4800);
      cfsetospeed(&newtio,B4800);
      break;	
    case 9600:
      cfsetispeed(&newtio,B9600);
      cfsetospeed(&newtio,B9600);
      break;	
    case 19200:
      cfsetispeed(&newtio,B19200);
      cfsetospeed(&newtio,B19200);
      break;	
    case 38400:
      cfsetispeed(&newtio,B38400);
      cfsetospeed(&newtio,B38400);
      break;
    case 57600:
      cfsetispeed(&newtio,B57600);
      cfsetospeed(&newtio,B57600);
      break;		
    case 115200:
      cfsetispeed(&newtio,B115200);
      cfsetospeed(&newtio,B115200);
      break;	
    case 460800:
      cfsetispeed(&newtio,B460800);
      cfsetospeed(&newtio,B460800);
      break;	
    default:
      cfsetispeed(&newtio,B9600);
      cfsetospeed(&newtio,B9600);
      break;		
  }

  //setup the stop bit
  if(nstop == 1)
    newtio.c_cflag &= ~ CSTOPB;
  else if(nstop == 2)
    newtio.c_cflag |= CSTOPB;

  //setup the waitting time and the minimum amount of the characters received 
  newtio.c_cc[VTIME] = 0;
  newtio.c_cc[VMIN] = 0;

  //deal with the characters which were not received.
  tcflush(fd,TCIFLUSH);

  //activate the new serial setup
  if((tcsetattr(fd,TCSANOW,&newtio))!=0)
  {
    ROS_INFO("failed to activate the new serial setup！");
    return -1;
  }

  ROS_INFO("serial setup success!\n");

  return 0;
}





int SerialCom::comInit()
{
  int i;

  if((fd_ = openPort(fd_,port_)) < 0)
  {
    ROS_INFO("failed to setup the serial！");
    return 0;
  }

  if((i = setOpt(fd_,baudrate_,8,'N',1)) < 0)
  {
    ROS_INFO("failed to setup the serial！");
    return 0;
  }

  ROS_INFO("the serial openned，setup the serial successed。the file operator=%d",fd_); 
  
  return 0;
}


///Packet转换数据
void SerialCom::SetBuf(char* buf,bool change/* =false */,bool addCRC/*=false*/)
{
	_buf[0]='\0';	
	if ( !change )
	{
		strcpy(_buf,buf);
		_len=(int)strlen(buf);
	}
	else
	{
		char c;
		_len=0;
		for(int i=0; *(buf+i)!='\0'&&i<MAX_BUF*2; i++)
		{
			c=*(buf+i);
			if ( c>='0' && c<='9' )
				c=(char)(c-'0');
			else if( c>='a' && c<='f' )
				c=(char)(c-'a'+10);
			else if( c>='A' && c<='F')
				c=(char)(c-'A'+10);
			else
				continue;
			if ( _len%2==0 )
				_buf[(int)(_len/2)]=c<<4;
			else
				_buf[(int)(_len/2)]=( _buf[(int)(_len/2)] | c );
			_len++;
		}
		//_buf[0]=1;
		_len=(_len+1)/2;
                
	}
	if( addCRC )
	{
		//_buf[0]=1;		
		WORD crc=CRC16((BYTE*)GetBuf(),getLen());;
		int high,low;
		//ROS_INFO("crc--%u",crc);
		high=crc/256;
		low=crc%256;
		_buf[_len]=char(high);
		_buf[_len+1]=char(low);
		_len+=2;
		int temp;
		temp=(crc/256)/16;
		_crc[0]=hex[temp];
		temp=(crc/256)%16;
		_crc[1]=hex[temp];
		temp=(crc%256)/16;
		_crc[2]=hex[temp];
		temp=(crc%256)%16;
		_crc[3]=hex[temp];
		_crc[4]='\0';
		//ROS_INFO("_buf0::%d",int(_buf[0]));
		//ROS_INFO("_buf1::%d",int(_buf[1]));
		//ROS_INFO("_buf2::%d",int(_buf[2]));
		//ROS_INFO("_buf3::%d",int(_buf[3]));
		//ROS_INFO("_buf4::%d",int(_buf[4]));
	}
	_buf[_len]='\0';
	
}


void SerialCom::Clear()
{
	_len=0;
	_buf[_len]='\0';
}


char* SerialCom::GetBuf()
{
	return _buf;
}


int SerialCom::getLen()
{
	return _len;
}



bool SerialCom::operator==(SerialCom& in)
{
	if ( _len==in.getLen() )
	{
		for(int i=0;i<_len;i++ )
		{
			if( _buf[i]!=*(in.GetBuf()+i) )
				return false;
		}
		return true;
	}
	return false;
}

WORD SerialCom::CRC16(unsigned char *p,WORD len)
{
	unsigned char i;
	WORD j;
	WORD uiCRC=0xffff;
		for(j=0;j<len;j++)
		{
		uiCRC^=(*p);
		p++;
			for(i=8;i!=0;i--)
			{
			if(uiCRC&1){uiCRC>>=1;uiCRC^=0xa001;}
			else
			uiCRC>>=1;
			}
		}
	return(uiCRC);
}
void SerialCom::sendcommand()
{
  command_buff_[0]='\0';
  strcpy(command_buff_,"01 04 00 40 00 06 71 DC");//generate the control command
  SetBuf(command_buff_,true,false);
  nwrite=write(fd_,_buf,8);
  delay(10);
  nread=read(fd_,_read0,17);
  ROS_INFO("_read0[3]=%d",_read0[3]);
  ROS_INFO("_read0[4]=%d",_read0[4]);
  ROS_INFO("_read0[5]=%d",_read0[5]);
  ROS_INFO("_read0[6]=%d",_read0[6]);
  ROS_INFO("_read0[11]=%d",_read0[11]);
  ROS_INFO("_read0[12]=%d",_read0[12]);
  ROS_INFO("_read0[13]=%d",_read0[13]);
  ROS_INFO("_read0[14]=%d",_read0[14]);
  delay(900);
}
void SerialCom::delay(unsigned int z)
{
    unsigned int i,j;
    for(i=z;i>0;i--)
        for(j=110;j>0;j--);  // 利用无实际意义的for循环来进行延时
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "odometry_serialcom");
	stringstream ss;
	SerialCom serial;
	ros::NodeHandle nh_private_;
	ros::NodeHandle n;
	if (!nh_private_.getParam ("d_wheel_separation", d_wheel_separation))
	  d_wheel_separation = 0.39;
	if (!nh_private_.getParam ("d_height", d_height))
	  d_height = 0;
	if (!nh_private_.getParam ("i_gear_ratio", i_gear_ratio))
	  i_gear_ratio = 30;
	if (!nh_private_.getParam ("i_cpr", i_cpr))
	  i_cpr = 500;
	if (!nh_private_.getParam ("d_frequency", d_frequency))
	  d_frequency = 10;
	if (!nh_private_.getParam ("d_wheel_diameter", d_wheel_diameter))
	  d_wheel_diameter = 0.138;
	realtime_tools::RealtimePublisher<nav_msgs::Odometry> * pose_pub = new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(n, "odom", 10);
	ros::Rate loop_rate(d_frequency);
	ros::Time read_time = ros::Time::now();
	ros::Duration dur_time;
	geometry_msgs::Pose2D odom_pose;
	geometry_msgs::Pose2D delta_odom_pose;
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	tf::Quaternion q;
	unsigned int f_left_read = 0, f_right_read = 0;
	unsigned int f_left_read_last = 0, f_right_read_last = 0; //4294967295.0
	float covariance[36] =	{0.01, 0, 0, 0, 0, 0,  // covariance on gps_x
										0, 0.01, 0, 0, 0, 0,  // covariance on gps_y
										0, 0, 99999, 0, 0, 0,  // covariance on gps_z
										0, 0, 0, 99999, 0, 0,  // large covariance on rot x
										0, 0, 0, 0, 99999, 0,  // large covariance on rot y
										0, 0, 0, 0, 0, 0.01};  // large covariance on rot z	
	for(int i = 0; i < 36; i++)
	{
		pose_pub->msg_.pose.covariance[i] = covariance[i];
	}		
	
	
	// Cleaning the encoder.
	write(fd_, "clean", 5);
	

        double roll = 0.0;
        double pitch = 0.0;
        double yaw = 0.0;
	int num=0;
	unsigned int test_num_l=0,test_num_r=0;
	while (ros::ok())
	{
		if(b_is_received_params)
		{
			ROS_DEBUG("EvarobotOdometry: Updating Odometry Params...");
			b_is_received_params = false;
		}
		
		if(b_reset_odom)
		{
			ROS_DEBUG("EvarobotOdometry: Resetting Odometry");
			
			write(fd_, "clean", 5);
			
			f_left_read = 0.0;
			f_right_read = 0.0;
			f_left_read_last = 0.0;
			f_right_read_last = 0.0;
			
			odom_pose.x = 0.0;
			odom_pose.y = 0.0;
			odom_pose.theta = 0.0;
			
			delta_odom_pose.x = 0.0;
			delta_odom_pose.y = 0.0;
			delta_odom_pose.theta = 0.0;
			
			b_reset_odom = false;
		}
		
		// Reading encoder.
		serial.sendcommand();
		f_left_read = ((_read0[3]*256+_read0[4])*256+_read0[5])*256+_read0[6];
		f_right_read =((_read0[11]*256+_read0[12])*256+_read0[13])*256+_read0[14];
		
		dur_time = ros::Time::now() - read_time;
		read_time = ros::Time::now();
		
		if(num<4)
		{
			f_left_read_last = f_left_read;	
			f_right_read_last = f_right_read;
			num++;
		}
		//ROS_INFO("f_left_read = %u f_left_read_last = %u",f_left_read,f_left_read_last);
		//ROS_INFO("f_right_read = %u f_right_read_last = %u",f_right_read,f_right_read_last);
		float f_delta_sr = 0.0, f_delta_sl = 0.0, f_delta_s = 0.0; 
		float f_rotvel_left = 0.0, f_rotvel_right = 0.0;
		float f_right_delta=0.0,f_left_delta=0.0;
		long long left_read,right_read,left_read_last,right_read_last;
		left_read=f_left_read;
		right_read=f_right_read;
		left_read_last=f_left_read_last;
		right_read_last=f_right_read_last;
		if(f_right_read_last > 4000000000.0 && f_right_read <= 10000)
		{
			f_right_delta = float(- 4294967296.0 + right_read_last - right_read);			
		}
		else if(f_right_read > 4000000000.0 && f_right_read_last <= 10000)
		{
			f_right_delta = float(right_read_last + 4294967296.0 - right_read);				
		}
		else if(f_right_read > 4000000000.0 && f_right_read_last == 0)
		{
			f_right_delta = float(4294967296.0 - right_read);				
		}
		else
		{
			f_right_delta = float(right_read_last - right_read);
		}

		if(f_left_read_last > 4000000000.0 && f_left_read <= 10000)
		{
			f_left_delta = float(4294967296.0 - left_read_last + left_read);
		}
		else if(f_left_read > 4000000000.0 && f_left_read_last <= 10000)
		{
			f_left_delta = float(- left_read_last - 4294967296.0 + left_read);
		}
		else if(f_left_read > 4000000000.0 && f_left_read_last == 0)
		{
			f_left_delta = float(- 4294967296.0 + left_read);		
		}
		else
		{
			f_left_delta = float(left_read - left_read_last);		
		}
		if(f_left_delta==4294967296.000000 || f_right_delta==4294967296.000000)
		{
			f_left_delta = 0.0;
			f_right_delta = 0.0;
		}
		//ROS_INFO("f_left_read = %f f_left_read_last = %f",f_left_read,f_left_read_last);
		//ROS_INFO("f_right_read = %f f_right_read_last = %f",f_right_read,f_right_read_last);
		test_num_l += f_left_delta;
		test_num_r +=f_right_delta;
		ROS_INFO("test_num_l = %u and test_num_r = %u ",test_num_l,test_num_r);
		ROS_INFO("f_right_delta = %f f_left_delta = %f",f_right_delta,f_left_delta);
		ROS_INFO("f_left_read = %u f_left_read_last = %u",f_left_read,f_left_read_last);
		ROS_INFO("f_right_read = %u f_right_read_last = %u",f_right_read,f_right_read_last);
		// Dönüş sayısı değişimi hesaplanıyor.
		f_delta_sr =  PI * d_wheel_diameter * f_right_delta / (4.11*i_gear_ratio * i_cpr);
		f_delta_sl =  PI * d_wheel_diameter * f_left_delta / (4.11*i_gear_ratio * i_cpr);

		f_rotvel_left = 2*PI * f_left_delta / (4.11*i_gear_ratio * i_cpr);
		f_rotvel_right = 2*PI * f_right_delta / (4.11*i_gear_ratio * i_cpr);

		// Oryantasyondaki değişim hesaplanıyor.
		delta_odom_pose.theta = (f_delta_sr - f_delta_sl) / d_wheel_separation;
		f_delta_s = (f_delta_sr + f_delta_sl) / 2;

		// x ve y eksenlerindeki yer değiştirme hesaplanıyor.
		delta_odom_pose.x = f_delta_s * cos(odom_pose.theta + delta_odom_pose.theta * 0.5);
		delta_odom_pose.y = f_delta_s * sin(odom_pose.theta + delta_odom_pose.theta * 0.5);

		// Calculate new positions.
		odom_pose.x += delta_odom_pose.x;
		odom_pose.y += delta_odom_pose.y;
		odom_pose.theta += delta_odom_pose.theta;

		// Yeni dönüş değerleri son okunan olarak atanıyor.
		f_right_read_last = f_right_read;
		f_left_read_last = f_left_read;

		//发布里程计，消息类型odometry	
		// Yayınlanacak Posizyon Verisi dolduruluyor.
		pose_pub->msg_.pose.pose.position.x = odom_pose.x;
		pose_pub->msg_.pose.pose.position.y = odom_pose.y;
		pose_pub->msg_.pose.pose.position.z = (float)d_height;


		roll = 0.0;
		pitch = 0.0;
		yaw = odom_pose.theta;
		
		
		pose_pub->msg_.pose.pose.orientation.x = sin(roll*0.5) * cos(pitch*0.5) * cos(yaw*0.5) - cos(roll*0.5) * sin(pitch*0.5) * sin(yaw*0.5);
		pose_pub->msg_.pose.pose.orientation.y = cos(roll*0.5) * sin(pitch*0.5) * cos(yaw*0.5) + sin(roll*0.5) * cos(pitch*0.5) * sin(yaw*0.5);
		pose_pub->msg_.pose.pose.orientation.z = cos(roll*0.5) * cos(pitch*0.5) * sin(yaw*0.5) - sin(roll*0.5) * sin(pitch*0.5) * cos(yaw*0.5);
		pose_pub->msg_.pose.pose.orientation.w = cos(roll*0.5) * cos(pitch*0.5) * cos(yaw*0.5) + sin(roll*0.5) * sin(pitch*0.5) * sin(yaw*0.5);
		//发送里程计tf
		transform.setOrigin( tf::Vector3(odom_pose.x, odom_pose.y, (float)d_height) );
		q.setRPY(0, 0, odom_pose.theta);
		transform.setRotation(q);
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));	

///////////////////////
		float f_lin_vel = 0, f_ang_vel = 0;
		float f_lin_vel_right = 0, f_lin_vel_left = 0;

		ROS_DEBUG_STREAM("EvarobotOdometry: dur_time: " << dur_time.toSec());

		if(dur_time.toSec() > 0)
		{		
			f_lin_vel_right = f_delta_sr / dur_time.toSec();
			f_lin_vel_left = f_delta_sl / dur_time.toSec();
			
			ROS_DEBUG_STREAM("EvarobotOdometry: VEl_LEFT: " << f_lin_vel_left << "  Vel_right: " << f_lin_vel_right << " dur: " << dur_time.toSec());

			f_rotvel_left /= dur_time.toSec();
	                f_rotvel_right /= dur_time.toSec();
			
			f_lin_vel = (f_lin_vel_right + f_lin_vel_left) / 2.0;
			f_ang_vel = (f_lin_vel_right - f_lin_vel_left) / d_wheel_separation;
		}
		else
		{
			ROS_INFO("dur_time.tosec() failed");
		}
		
		//ss.str("");
		//ss << n.resolveName(n.getNamespace(), true) << "/wheel_link";
		
		//vel_pub->msg_.header.frame_id = ss.str();
		//vel_pub->msg_.header.stamp = ros::Time::now();
		//vel_pub->msg_.left_vel = f_lin_vel_left;
		//vel_pub->msg_.right_vel = f_lin_vel_right;

		//rotation_pub->msg_.header.frame_id = ss.str();
		//rotation_pub->msg_.header.stamp = ros::Time::now();
                //rotation_pub->msg_.left_vel = f_rotvel_left;
                //rotation_pub->msg_.right_vel = f_rotvel_right;

		//if (vel_pub->trylock())
		//{
		//	vel_pub->unlockAndPublish();
		//}			
		
		//if (rotation_pub->trylock())
                //{
                //        rotation_pub->unlockAndPublish();
                //}

		// Yayınlacak Hız Verisi dolduruluyor.
		pose_pub->msg_.twist.twist.linear.x = f_lin_vel;
		pose_pub->msg_.twist.twist.linear.y = 0.0;
		pose_pub->msg_.twist.twist.linear.z = 0.0;

		pose_pub->msg_.twist.twist.angular.x = 0.0;
		pose_pub->msg_.twist.twist.angular.y = 0.0;
		pose_pub->msg_.twist.twist.angular.z = f_ang_vel;

		//uint32
		//msg.header.seq
		// ROS Zaman etiketi topiğe basılıyor. (time)
		pose_pub->msg_.header.stamp = ros::Time::now();

		// Odometry verisinin frame id'si yazılıyor. (string)
		ss.str("");
		ss << n.resolveName(n.getNamespace(), true) << "/odom";
		pose_pub->msg_.header.frame_id = ss.str();
		
		ss.str("");
		ss << n.resolveName(n.getNamespace(), true) << "/base_link";
		pose_pub->msg_.child_frame_id = ss.str();
		
		// Veri topikten basılıyor.
		if (pose_pub->trylock())
		{
			pose_pub->unlockAndPublish();
		}			
		//pub_freq.tick();

		ros::spinOnce();
		//updater.update();
		// Frekansı tutturmak için uyutuluyor.
		loop_rate.sleep();
	}

	// Enkoder sürücü dosyası kapatılıyor.
	//close(fd);
	return 0;
}

