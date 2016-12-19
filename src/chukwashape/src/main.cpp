#include <ros/ros.h>

// For the messages between nodes
#include "std_msgs/String.h"

// For splitting strings
#include <sstream>

// For using strings
#include <string>

// To publish to LED on kobuki
#include <kobuki_msgs/Led.h>

// Count of predefined shapes
#define shapes_count 2

using namespace std;

class Chukwashape
{
private:
	// Initialize variables
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

	// Struct with the shapes (Known size)
	struct ShapeStruct{
		string name;
		int est_time; // Not in use
	} shapes[shapes_count];

	// When a command is published by interface
	void callback_face(const std_msgs::String command_send)
	{
		string command_name; // First part of the command
		string command_rest; // All the rest (if any)

		// Split the command string by space. First word is name, rest goes to "command_rest"
		stringstream ss (command_send.data);
			ss >> command_name;
			ss >> command_rest;

		// Output recived command (for debugging)
		ROS_INFO("Recieved: \n name: %s \n data: %s", command_name.c_str(), command_rest.c_str());
		
		// Check the diffrent command names and call the function
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

	// Make the Chukwa move in a shape
	void moveChukwa(const string& shape)
	{
		// Run the shape
	}

	// Called to control the LEDs
	void controlLEDs(const string& led)
	{
		// If 1 then LED 1
		if(led == "1")
		{
			// If "led1.value" has a value then put 0 else put 1
			// (Short if-else statement)
			led1.value ? led1.value = 0 : led1.value = 1;
			led1_pub.publish(led1); // Publishes to the kobuki_base
		}
		// If 2 then LED 2
		else if(led == "2")
		{
			// (Short if-else statement)
			led2.value? led2.value = 0 : led2.value = 1;
			led2_pub.publish(led2); // Publishes to the kobuki_base
		}
		// Specil mode
		else if(led == "PARTY")
		{
			// Change to opposit
			startParty = !startParty; // Used for timer
		}
	}

	// This function gets called every 0.1 sec by ros
	void callParty(const ros::TimerEvent&)
	{
		// Only do something if startParty is "true"
		if(startParty)
		{
			/* 
			BLACK   = 0
			GREEN   = 1
			ORANGE  = 2
			RED     = 3 */
			led1.value = rand() % 3; // Change led1 to a random value between 0-3
			led1_pub.publish(led1);
					
			led2.value = rand() % 3; // Change led1 to a random value between 0-3
			led2_pub.publish(led2);
		}
	}

	// This function publishes all the possible shape's name as a string with spaces
	// Ex: "STAR SQUARE"
	void publishShapes()
	{
		std_msgs::String all_shapes;
		stringstream ss;

		// Go through the struct
		for (int i = 0; i < shapes_count; ++i)
		{
			ss << shapes[i].name << " ";
		}

		// Put the string into the message.data to publish
		all_shapes.data = ss.str();

		// Make sure Interface is subscribed
		shape_publisher.getNumSubscribers();

		// If no subscribers spin once and check again
		while(! shape_publisher.getNumSubscribers() > 0){
    		ros::spinOnce(); // Spin once to update
		}
		
		// publish the shape string to interface node
		shape_publisher.publish(all_shapes);
	}
public:
	// Constructor
	Chukwashape()
	{
		// to publish the shape string to interface
		shape_publisher = nh.advertise<std_msgs::String>("Chukwa_shapes", 1);

		// to get the commands from interface
		sub_from_face = nh.subscribe<std_msgs::String>("Chukwashape_trigger", 10, &Chukwashape::callback_face, this);

		// create party timer
		party_timer = nh.createTimer( ros::Duration(0.1), &Chukwashape::callParty, this);

		// To change the individual LEDs
		led1_pub = nh.advertise<kobuki_msgs::Led>("mobile_base/commands/led1", 1);
		led2_pub = nh.advertise<kobuki_msgs::Led>("mobile_base/commands/led2", 1);
		
		// make sure party is not started
		startParty = 0;

		// All the shapes
		shapes[0].name = "HEART";
		shapes[0].est_time = 5;

		shapes[1].name = "SQUARE";
		shapes[1].est_time = 4;

		// Publish shapes first time to make interface updated
		publishShapes();
	};
	
};

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "Chukwashape");

	// Construct the class
	Chukwashape Chukwa_go;

	ros::spin();//loop until closed
	return 0;
}