#include <ros/ros.h>

// For the messages between nodes
#include "std_msgs/String.h"

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
		// For the choice later
		int choice;
		
		do
		{
			ros::spinOnce(); // Spin once to update the topic
			system("clear");
			cout << "*************************" << "\n";
			
			if(shapes.empty())
			{
				cout << "No shapes available." << "\n";
			}
			else
			{
				int count = 1;

				// Use an iterrator to split vector in to a string
				for (vector<string>::iterator i = shapes.begin(); i != shapes.end(); ++i)
				{
					cout << count++ << " - " << *i << "\n";
				}
			}
			cout << "0 - Back" << "\n";
			cout << "*************************" << "\n";
		  	
			cin >> choice;

			std_msgs::String command;
			switch(choice)
			{
				case(1):
					command.data = "SHAPE 1";
					break;
				case(2):
					command.data = "SHAPE 2";
					break;
			}

			// publish the SHAPE command
			command_pub.publish(command);
	
		}while(choice != 0 || ros::ok());
	}

	// This function gets called when user wants to change the LEDs
	void showChangeLeds()
	{
		int choice;
		do
		{
			// "Clear" the screen and show choices
			system("clear");
			cout << "*************************" << "\n";
			cout << "1 - Toggle led1" << "\n";
			cout << "2 - Toggle led2" << "\n";
			cout << "3 - Toggle DISCO-MODE" << "\n";
			
			cout << "0 - Back" << "\n";
			cout << "*************************" << "\n";
		  	
			cin >> choice;
			
			// Publish the choice to "Chukwashape" so it can change the LEDs
			std_msgs::String command;
			switch(choice)
			{
				case(1):
					command.data = "LED 1"; 
					break;
				case(2):
					command.data = "LED 2";
					break;
				case(3):
					command.data = "LED PARTY";
					break;
			}

			// publish the LED command
			command_pub.publish(command);
			
		} while(choice != 0 || ros::ok());
			
	}

	// This function gets called when user wants to see "About"
	void showAbout()
	{
		int choice;
	   	
		do
		{
			// "Clear" and display the content
			system("clear");
			cout << "*************************" << "\n"
				 << "This program is made by: B217" << "\n"
				 << "Group members:" << "\n"
				 << "Aleksandra Zasadni\n"
				 << "Andrej Orsula\n"
				 << "Asger Printz Madsen\n"
				 << "Christoffer Sand Andersen\n"
				 << "Jesper Frederik Hansen\n"
				 << "Lukas Wyon\n"
				 << "Soren Myhre Voss\n\n"
				 << "0 - Back" << "\n"
				 << "*************************" << "\n";
		  	
			cin >> choice;
			
		}while(choice != 0 || ros::ok());
	}

	// This function gets called when user wants to see "Help" 
	void showHelp()
	{
		int choice;
	   	
		do
		{
			system("clear");
			cout << "*************************" << "\n"
				 << "1 - Help" << "\n"
				 << "No help available." << "\n"
				 << "0 - Back" << "\n"
				 << "*************************" << "\n";
		  	
			cin >> choice;
			
		}while(choice != 0 || ros::ok());
	}

	// This function gets called to start the interface
	void start()
	{
		int choice;
	   	
	   	// Do this first then check for condition
		do
		{
			ros::spinOnce(); // Spin once to update
			system("clear");
			cout << "*************************" << "\n"
				 << "1 - Show Shapes" << "\n"
				 << "2 - Change LEDs" << "\n"
				 << "3 - About" << "\n"
				 << "9 - Help" << "\n"
				 << "0 - EXIT" << "\n"
				 << "*************************" << "\n";
		  	
			cin >> choice;

			// Check for choice and run the correct function
			switch(choice)
			{
			 	case(1):
			 		showShapes();
			 		break;
			 	case(2):
			 		showChangeLeds();
			 		break;
			 	case(3):
			 		showAbout();
			 		break;
			 	case(9):
			 		showHelp();
			 		break;
			 	case(0):
			 	 ros::shutdown();
			 	// signal(SIGINT, mySigintHandler);
			}
			
		}while(choice != 0 || ros::ok());
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