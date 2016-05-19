Network Card IP:
IP: 	 192.168.1.2
Netmask: 255.255.255.0
Gateway: 192.168.1.0


Package Installation on Ubuntu
To add the Robotics Library repository to your system, execute the following commands in your terminal.

sudo apt-add-repository ppa:roblib/ppa
sudo apt-get update

You can then install RL by a terminal command or by using your favorite package manager, e.g., the Ubuntu Software Center, Synaptic, or similar applications.

sudo apt-get install librl

If you want to develop software using the Robotics Library, you should install the development files.

sudo apt-get install librl-dev
