#include <ros/ros.h>

// For the messages between nodes
#include <std_msgs/String.h>

// For splitting strings
#include <sstream>

// For using strings
#include <string>

// Message used to move the mobile base the Kobuki
#include <geometry_msgs/Twist.h>

// To publish to LED on kobuki
#include <kobuki_msgs/Led.h>

// Message used to play the sound on the Kobuki
#include <kobuki_msgs/Sound.h>

// Count of predefined shapes
#define shapes_count 2
#define max_steps 11 //maximum number of step for shapes

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
	ros::Publisher chukwa_move_pub;
	ros::Publisher chukwa_sound_pub;
	ros::Timer chukwa_shape_timer;
	ros::Timer party_timer;

	std_msgs::String all_shapes;
	geometry_msgs::Twist chukwa_twist;
	kobuki_msgs::Sound chukwa_sound;
	kobuki_msgs::Led led1;
	kobuki_msgs::Led led2;

	bool startParty;
	int step, counter_for_timer;


	int pattern[max_steps][2],
		square[max_steps][2]={{100,0},{0,47},{100,0},{0,47},{100,0},{0,47},{100,0},{0,47}},
		diamond[max_steps][2]={{150,0},{0,43},{24,0},{0,20},{52,0},{0,20},{24,0},{0,43},{65,0},{0,43}};


	// Struct with the shapes (Known size)
	struct ShapeStruct
	{
		string name;
	}
	shapes[shapes_count];

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
		if(command_name == "SHAPE")
		{
			moveChukwa(command_rest);
		}
		else if(command_name ==  "LED")
		{
			controlLEDs(command_rest);
		}
		else if(command_name ==  "SOUND")
		{
			playSounds(command_rest);
		}
	}

	// Make the Chukwa move in a shape
	void moveChukwa(const string& shape)
	{
		if (shape == "1")
		{
			copy_array(square, pattern);
			chukwa_shape_timer.start();
		}
		else if (shape == "2")
		{
			copy_array(diamond, pattern);
			chukwa_shape_timer.start();
		}
		step = 0;
		counter_for_timer++;
	}

	void copy_array(int copyfrom[max_steps][2], int copyto[max_steps][2])
	{
		for (int i = 0; i < max_steps; ++i)
		{
			for (int j = 0; j < 2; ++j)
			{
				copyto[i][j]=copyfrom[i][j];
			}
		}
	}

	void publishing(const ros::TimerEvent&)
	{
		//set speed based on instructions from 2D array
		//forward
		if (pattern[step][0] != 0 && pattern[step][1] == 0)
		{
			chukwa_twist.linear.x = 0.125;
			chukwa_twist.angular.z = 0;
		}
		//turn
		else if (pattern[step][0] == 0 && pattern[step][1] != 0)
		{
			chukwa_twist.linear.x = 0;
			chukwa_twist.angular.z = 0.785;
		}
		else if (pattern[step][0] == 0 && pattern[step][1] == 0)
		//nothing
		{
			chukwa_twist.linear.x = 0;
			chukwa_twist.angular.z = 0;
		}


	//go to next step when needed
		if (pattern[step][0] == 0 && pattern[step][1] == 0)
		{
			next_step();
		}
		//forward
		else if (pattern[step][0] != 0 && pattern[step][1] == 0 && counter_for_timer > pattern[step][0])
		{
			next_step();
		}
		//turn
		else if (pattern[step][0] == 0 && pattern[step][1] != 0 && counter_for_timer > pattern[step][1])
		{
			next_step();
		}
		//publish speed
		chukwa_move_pub.publish(chukwa_twist);
		//count up
		counter_for_timer++;
	}

	void next_step()
	{
		//go to next step in 2D array
		step++;
		//reset timer
		counter_for_timer = 0;

		// choose Kobuki sound
		chukwa_sound.value = 6;
		//play sound from kobuki
		chukwa_sound_pub.publish(chukwa_sound);

		//if no further commands in array (both are 0s), stop the timer and reset values
		if (pattern[step][0] == 0 && pattern[step][1] == 0)
		{
			chukwa_twist.linear.x = 0;
			chukwa_twist.angular.z = 0;
			step = 0;
			chukwa_shape_timer.stop();
		}
		//wait for 1s
		ros::Duration(1).sleep();
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
			startParty = !startParty; // Used for timer //why dont you just stop/start timer?
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
			led1.value = rand() % 4; // Change led1 to a random value between 0-3
			led1_pub.publish(led1);
			led2.value = rand() % 4; // Change led1 to a random value between 0-3
			led2_pub.publish(led2);
		}
	}

	void playSounds(const string& shape)
	{
		// choose Kobuki sound
		chukwa_sound.value = atoi(shape.c_str()) - 1; //convert string into int (-1 is because of indexing from 0)

		chukwa_sound_pub.publish(chukwa_sound);
	}

	// This function publishes all the possible shape's name as a string with spaces
	// Ex: "SQUARE DIAMOND"
	void publishShapes()
	{
		stringstream ss;

		// Go through the struct
		for (int i = 0; i < shapes_count; ++i)
		{
			ss << shapes[i].name << " ";
		}

		// Put the string into the message.data to publish
		all_shapes.data = ss.str();

		// If no subscribers spin once and check again
		while(! shape_publisher.getNumSubscribers() > 0)
		{
			ros::spinOnce(); // Spin once to update
		}

		// publish the shape string to interface node
		shape_publisher.publish(all_shapes);
	}
public:
	// Constructor
	Chukwashape()
	{
		// to get the commands from interface
		sub_from_face = nh.subscribe<std_msgs::String>("Chukwashape_trigger", 10, &Chukwashape::callback_face, this);

		// to publish the shape string to interface
		shape_publisher = nh.advertise<std_msgs::String>("Chukwa_shapes", 1);

		// to send velocity commands
		chukwa_move_pub = nh.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 1);
		// to play sounds from Kobuki
		chukwa_sound_pub = nh.advertise<kobuki_msgs::Sound>("mobile_base/commands/sound", 1);
		// To change the individual LEDs
		led1_pub = nh.advertise<kobuki_msgs::Led>("mobile_base/commands/led1", 1);
		led2_pub = nh.advertise<kobuki_msgs::Led>("mobile_base/commands/led2", 1);

		// draw shapes
		chukwa_shape_timer = nh.createTimer(ros::Duration(0.05),  &Chukwashape::publishing, this);
		// create party timer
		party_timer = nh.createTimer( ros::Duration(0.1), &Chukwashape::callParty, this);

		// All the shapes
		shapes[0].name = "SQUARE";
		shapes[1].name = "DIAMOND";

		counter_for_timer = 0;
		// make sure party is not started
		startParty = 0;

		chukwa_shape_timer.stop();

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