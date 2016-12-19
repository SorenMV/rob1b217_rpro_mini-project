#include <ros/ros.h>
#include "std_msgs/String.h"
#include <sstream>
#include <vector>



using namespace std;

class Chukwaface
{
private:
	ros::NodeHandle nh;
	ros::Subscriber sub_from_shape;
	ros::Publisher command_pub;
	vector<string> shapes;

	
	void callback_shape(const std_msgs::String shapes_str)
	{
		string str(shapes_str.data.c_str());
		string buf;
		stringstream ss(str);

		while(ss >> buf) 
		{
			shapes.push_back(buf);
		}
	}

	void showShapes()
	{
		int choice;
		do
		{
			ros::spinOnce();
			system("clear");
			cout << "*************************" << "\n";
			if(shapes.empty())
			{
				cout << "No shapes available." << "\n";
			}
			else
			{
				int count = 1;
				for (vector<string>::iterator i = shapes.begin(); i != shapes.end(); ++i)
				{
					cout << count++ << " - " << *i << "\n";
				}
			}
			cout << "0 - Back" << "\n";
			cout << "*************************" << "\n";
		  	
			cin >> choice;

			//
	
		}while(choice != 0 && ros::ok());
	}

	void showChangeLeds()
	{
		int choice;
		do
		{
			ros::spinOnce();
			system("clear");
			cout << "*************************" << "\n";
			cout << "1 - Toggle led1" << "\n";
			cout << "2 - Toggle led2" << "\n";
			cout << "3 - Toggle DISCO-MODE" << "\n";
			
			cout << "0 - Back" << "\n";
			cout << "*************************" << "\n";
		  	
			cin >> choice;
			
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

			command_pub.publish(command);
			
		} while(choice != 0 && ros::ok());
		

		
	}

	
	void showAbout()
	{
		int choice;
	   	
		do
		{
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
			
		}while(choice != 0 && ros::ok());
	}


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
			
		}while(choice != 0 && ros::ok());
	}

	void start()
	{
		int choice;
	   	
		do
		{
			ros::spinOnce();
			system("clear");
			cout << "*************************" << "\n"
				 << "1 - Show Shapes" << "\n"
				 << "2 - Change LEDs" << "\n"
				 << "3 - About" << "\n"
				 << "9 - Help" << "\n"
				 << "0 - EXIT" << "\n"
				 << "*************************" << "\n";
		  	
			cin >> choice;

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
			}
			
		}while(choice != 0 && ros::ok());
	}

public:
	Chukwaface()
	{
		sub_from_shape = nh.subscribe<std_msgs::String>("Chukwa_shapes", 10, &Chukwaface::callback_shape, this);
		command_pub = nh.advertise<std_msgs::String>("Chukwashape_trigger", 10);
		

		start();
	};
	
};

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "Chukwaface");

	
	Chukwaface Chukwa_face;

	ros::spin();//loop until closed
	return 0;
}