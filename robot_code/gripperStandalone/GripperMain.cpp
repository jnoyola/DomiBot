#include <QThread>

#include <chrono>
#include <thread>
#include <iostream>
#include "SchunkGripper.h"
using namespace std;

void OpenGripper();
void CloseGripper();

SchunkGripper *schunkGripper;

int main(int argc, char *argv[])
{
    schunkGripper = new SchunkGripper();
    schunkGripper->start(QThread::TimeCriticalPriority);
    std::cout << "Starting the SchunkGripper thread ..." << std::endl;

		string state;
		while(1)
		{
			std::cin >> state;

			if(state=="o")
			{
				OpenGripper();
			}
			else if(state=="c")
			{
				CloseGripper();
			}
			else
			{
				break;
			}
		}

		return 0;
}

void CloseGripper()
{
    schunkGripper->SetDesiredPosition(35, 0, 100);
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
}

void OpenGripper()
{
    schunkGripper->SetDesiredPosition(50, 0, 100);
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
}
