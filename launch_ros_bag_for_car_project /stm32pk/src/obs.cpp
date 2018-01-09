
#include "ros/ros.h" 
#include <stm32pk/xtyh.h>  
#include <iostream>
//#include <geometry_msgs/Twist.h>
#include <signal.h>
void chatterCallback(const stm32pk::xtyh::ConstPtr &msg)
{
float _u1 = msg->huf[0];
float _u2 = msg->huf[1];
float _u3 = msg->huf[2];
float _u4 = msg->huf[3];
float _u5 = msg->huf[4];
float _u6 = msg->huf[5];
std::cout<<"-------------------"<<std::endl;
std::cout<<"6个传感器开始测距"<<std::endl;
std::cout<<"从左到右依次为distance1到distance6"<<std::endl;
std::cout<<"distance1:"<<_u1<<std::endl;
std::cout<<"distance2:"<<_u2<<std::endl;
std::cout<<"distance3:"<<_u3<<std::endl;
std::cout<<"distance4:"<<_u4<<std::endl;
std::cout<<"distance5:"<<_u5<<std::endl;
std::cout<<"distance6:"<<_u6<<std::endl;
std::cout<<"测距结束"<<std::endl;
std::cout<<"-------------------"<<std::endl;


}

int main(int argc, char *argv[])

{
	
ros::init(argc, argv, "sonar");
	
ros::NodeHandle n;
	
ros::Subscriber sub = n.subscribe("sonarn", 1000, chatterCallback);
//cmdVelPub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
ros::Rate loop_rate(10);
//geometry_msgs::Twist speed;
                     
while(ros::ok())
        
{
        
ros::spinOnce();
        
loop_rate.sleep();

        
}
        	
return 0;

}
