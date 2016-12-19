#include <ros/ros.h>
#include "std_msgs/String.h"
#include <sstream>
#include <string>
#include <kobuki_msgs/Led.h>

#define shapes_count 3

using namespace std;

class Chukwashape
{
private:
	ros::NodeHandle nh;
	ros::Subscriber sub_from_face;
	ros::Publisher shape_publisher;
	ros::Publisher led1_pub;
	ros::Publisher led2_pub;
	ros::Publisher party_pub;
	ros::Timer party_timer;
	kobuki_msgs::Led led1;
	kobuki_msgs::Led led2;
	bool startParty;

	struct ShapeStruct{
		string name;
		int est_time;
	} shapes[shapes_count];


	void callback_face(const std_msgs::String command_send)
	{
		string command_name;
		string command_rest;

		stringstream ss (command_send.data);
			ss >> command_name;
			ss >> command_rest;

		ROS_INFO("Recieved: \n name: %s \n data: %s", command_name.c_str(), command_rest.c_str());
		if(command_name == "MOVE")
		{
			moveChukwa(command_rest);
		}
		else if(command_name == "UPDATE")
		{
			publishShapes();
		}
		else if(command_name == "LED")
		{
			controlLEDs(command_rest);
		}
	}

	void moveChukwa(const string& shape)
	{
		// Run the shape
	}

	void controlLEDs(const string& led)
	{
		if(led == "1")
		{
			led1.value? led1.value = 0 : led1.value = 1;
			led1_pub.publish(led1);
		}
		else if(led == "2")
		{
			led2.value? led2.value = 0 : led2.value = 1;
			led2_pub.publish(led2);
		}
		else if(led == "PARTY")
		{
			// Change to opposit
			startParty = !startParty;
		}
	}

	

	void callParty(const ros::TimerEvent&)
	{
		if(startParty)
		{
			led1.value = rand() % 4;
			led1_pub.publish(led1);
					
			led2.value = rand() % 4;
			led2_pub.publish(led2);
		}
	}

	void publishShapes()
	{
		std_msgs::String all_shapes;
		stringstream ss;
		for (int i = 0; i < shapes_count; ++i)
		{
			ss << shapes[i].name << " ";
		}
		all_shapes.data = ss.str();

		shape_publisher.getNumSubscribers();

		while(! shape_publisher.getNumSubscribers() > 0){
    		ros::spinOnce();	
		}
		
		shape_publisher.publish(all_shapes);

		ROS_INFO("Shapes: %s", all_shapes.data.c_str());
	}
public:
	Chukwashape()
	{
		shape_publisher = nh.advertise<std_msgs::String>("Chukwa_shapes", 1);
		sub_from_face = nh.subscribe<std_msgs::String>("Chukwashape_trigger", 10, &Chukwashape::callback_face, this);

		// Party
		party_timer = nh.createTimer( ros::Duration(0.1), &Chukwashape::callParty, this);
		led1_pub = nh.advertise<kobuki_msgs::Led>("mobile_base/commands/led1", 1);
		led2_pub = nh.advertise<kobuki_msgs::Led>("mobile_base/commands/led2", 1);
		startParty = 0;

		// All the shapes
		shapes[0].name = "STAR";
		shapes[0].est_time = 5;

		shapes[1].name = "SQUARE";
		shapes[1].est_time = 4;


		publishShapes();
	};
	
};

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "Chukwashape");

	
	Chukwashape Chukwa_go;

	ros::spin();//loop until closed
	return 0;
}