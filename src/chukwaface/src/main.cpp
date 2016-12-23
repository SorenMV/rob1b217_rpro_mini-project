#include <ros/ros.h>

// For the messages between nodes
#include <std_msgs/String.h>

// For splitting strings
#include <sstream>

// For the shape vector (Unkown size)
#include <vector>


using namespace std;

class Chukwaface
{
private:
	// Initialize variables
	ros::NodeHandle nh;
	ros::Subscriber sub_from_shape;
	ros::Publisher command_pub;

	vector<string> shapes;
	std_msgs::String command;

	char choice;

	// This function gets called when topic is recieved (the shapes are updated) 
	// and puts them into a vector
	void callback_shape(const std_msgs::String shapes_str)
	{
		string str(shapes_str.data.c_str());
		string buf;
		stringstream ss(str);

		// Stringstream splits the string by spaces
		while(ss >> buf) 
		{
			shapes.push_back(buf); // Put the buffer into the vactor
		}
	}

	// This function gets called when you choose "Shapes" in menu
	void showShapes()
	{
		ros::spinOnce(); //to update available shapes

		// "Clear" and show shapes
		system("clear");
		cout << "*****************************\n";
		if(shapes.empty())
			{
				cout << "No shapes available!\n";
			}
		else // Use an iterrator to split vector in to a string
			{
				int count = 0;
				for (vector<string>::iterator i = shapes.begin(); i != shapes.end(); ++i)
				{
					cout << ++count << " - " << *i << "\n"; //show all shapes
				}
			}
		cout << "0 - Back\n"
			 << "*****************************\n";

		do
		{
			cin >> choice;

		switch(choice)
			{
				case('1'):
					command.data = "SHAPE 1";
					break;
				case('2'):
					command.data = "SHAPE 2";
					break;
				case('0'):
					start();
					break;
				default:
					ROS_WARN("'%c' is not a valid input!", choice);
			}

			// publish the SHAPE command
			command_pub.publish(command);

		}while(ros::ok());
	}

	// This function gets called when user wants to change the LEDs
	void showChangeLeds()
	{
		// "Clear" the screen and show choices
		system("clear");
		cout	<< "*************************\n"
				<< "1 - Toggle led1\n"
				<< "2 - Toggle led2\n"
				<< "3 - Toggle DISCO-MODE\n"
				<< "0 - Back\n"
				<< "*************************\n";

		do
		{
			cin >> choice;

			// Publish the choice to "Chukwashape" so it can change the LEDs
			switch(choice)
			{
				case('1'):
					command.data = "LED 1"; 
					break;
				case('2'):
					command.data = "LED 2";
					break;
				case('3'):
					command.data = "LED PARTY";
					break;
				case('0'):
					start();
					break;
				default:
					ROS_WARN("'%c' is not a valid input!", choice);
			}

			// publish the LED command
			command_pub.publish(command);

		} while(ros::ok());
			
	}

	// This function gets called when user wants to see "About"
	void showAbout()
	{
		// "Clear" and display the content
		system("clear");
		cout	<< "*************************\n"
				<< "This program is made by: B217\n"
				<< "Group members:\n"
				<< "Aleksandra Zasadni\n"
				<< "Andrej Orsula\n"
				<< "Asger Printz Madsen\n"
				<< "Christoffer Sand Andersen\n"
				<< "Jesper Frederik Hansen\n"
				<< "Lukas Wyon\n"
				<< "Soren Myhre Voss\n\n"
				<< "0 - Back\n"
				<< "*************************\n";

		do
		{
			cin >> choice;

			if (choice == '0')
			{
				start();
			}
			else
			{
				ROS_WARN("'%c' is not a valid input!", choice);
			}
		}while(ros::ok());
	}

	// This function gets called when user wants to see "Help" 
	void showHelp()
	{
		// "Clear" and show help
		system("clear");
		cout	<< "*************************\n"
				<< "No help available.\n"
				<< "0 - Back\n"
				<< "*************************\n";

		do
		{
			cin >> choice;

			if (choice == '0')
			{
				start();
			}
			else
			{
				ROS_WARN("'%c' is not a valid input!", choice);
			}
		}while(ros::ok());
	}

	// This function gets called to start the interface
	void start()
	{
		// Do this first then check for condition
		system("clear");
		cout	<< "*************************\n"
				<< "1 - Show Shapes\n"
				<< "2 - Change LEDs\n"
				<< "3 - About\n"
				<< "9 - Help\n"
				<< "0 - EXIT\n"
				<< "*************************\n";
		do
		{
			cin >> choice;

			// Check for choice and run the correct function
			switch(choice)
			{
				case('1'):
					showShapes();
					break;
				case('2'):
					showChangeLeds();
					break;
				case('3'):
					showAbout();
					break;
				case('9'):
					showHelp();
					break;
				case('0'):
					ros::shutdown();
					break;
				default:
					ROS_WARN("'%c' is not a valid input!", choice);
			}
		}while(ros::ok());
	}

public:
	// Constructor
	Chukwaface()
	{
		// Subscribe to get the possible shapes
		sub_from_shape = nh.subscribe<std_msgs::String>("Chukwa_shapes", 10, &Chukwaface::callback_shape, this);

		// Publish the commands to the "Shape" node
		command_pub = nh.advertise<std_msgs::String>("Chukwashape_trigger", 10);

		// Run the start function to show first interface
		start();
	};
};

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "Chukwaface");

	// Construct the class
	Chukwaface Chukwa_face;

	//loop until closed
	ros::spin();
	return 0;
}