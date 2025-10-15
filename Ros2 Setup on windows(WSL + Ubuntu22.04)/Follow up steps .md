Setting up a ROS2 development environment on Windows using WSL is a very powerful and popular method. It gives you the best of both worlds: the full power of Linux for ROS and the convenience of Windows for everything else.

Let's walk through this step-by-step, just like for a beginner.

What We Are Doing
We will use the Windows Subsystem for Linux (WSL2) to install a full version of Ubuntu 22.04 inside your Windows machine. Then, inside that Ubuntu environment, we will install ROS2 Humble and Gazebo.

Prerequisite: You must have a modern version of Windows 10 (version 2004 and higher) or Windows 11.

Part 1: Install WSL and Ubuntu
This is the easiest step. Microsoft has made this a one-command process.

Open PowerShell as an Administrator.

Click your Start Menu.

Type "PowerShell".

Right-click on "Windows PowerShell" and select "Run as administrator".

Run the Install Command.

In the blue PowerShell window, copy and paste the following command and press Enter. This command tells Windows to enable all the necessary features, download the latest WSL kernel, and install Ubuntu 22.04.

PowerShell
```python
wsl --install -d Ubuntu-22.04
#Reboot Your Computer.
```
After the command finishes, it will likely ask you to restart your computer. Do it. This is a necessary step.

Set Up Ubuntu.

After rebooting, an Ubuntu terminal window will automatically open to finish the installation. If it doesn't, just find "Ubuntu 22.04" in your Start Menu and open it.

It will ask you to create a UNIX username and a password.

Important: This is for your Ubuntu environment only. It has nothing to do with your Windows login. Make sure to remember this password, as you will need it for sudo commands!

Part 2: Get Ubuntu Ready for ROS2
Now that you have a fresh Ubuntu system, we need to update it and install some essential tools before we get to ROS itself.

Update Your System.

This command updates the list of available software and then upgrades all your installed packages to their latest versions.

Bash
```
sudo apt update && sudo apt upgrade -y
#Install Essential Tools.
```
These are common development tools that ROS2 and Gazebo depend on for building code.

Bash
```
sudo apt install -y build-essential cmake git python3-pip
#Part 3: Install ROS2 Humble Hawksbill
#Now we'll follow the official process to install ROS2 Humble.
```
Add the ROS2 Repository.

These commands tell your Ubuntu system where to download ROS2 from. It's like adding a new store to your phone's app store list.

Bash
```
# First, authorize our GPG key with apt
sudo apt install -y software-properties-common
sudo add-apt-repository universe

# Add the ROS 2 GPG key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add the repository to your sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
Install ROS2 Desktop.
```
Now that your system knows where to find ROS2, update your package list again and then run the installation command. We install ros-humble-desktop, which includes everything you need: ROS, RViz2, examples, and more.

Bash
```
sudo apt update
sudo apt install -y ros-humble-desktop
Source the Environment.
```
For your terminal to know where to find the ROS2 commands, you need to "source" the setup file.

Bash
```
# This command loads ROS2 into your current terminal
source /opt/ros/humble/setup.bash

# To make this happen automatically in every new terminal, add it to your .bashrc file
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
Part 4: Install Gazebo and the ROS2 Bridge
Finally, let's install the simulator and the crucial package that lets it talk to ROS2.
```
Install Gazebo.

Bash
```
sudo apt install -y gazebo
#Install the ROS2-Gazebo Integration Packages.
```
This is the "bridge" that allows ROS2 to control Gazebo and get sensor data back.

Bash
```
sudo apt install -y ros-humble-gazebo-ros-pkgs
Part 5: The Magic Step - Verifying Graphics
Gazebo is a graphical application. Modern WSL (called WSLg) automatically handles displaying Linux GUI apps on your Windows desktop! We just need to make sure it's working.
```
Run a Test.

In your Ubuntu terminal, install a simple test application called xeyes.

Bash
```
sudo apt install -y x11-apps
#Launch the Test App.
```
Now, just type:

Bash
```
xeyes
```
A small window should pop up on your Windows desktop with a pair of eyes that follow your mouse cursor. If you see this, your graphics are working perfectly! You can close the xeyes window and the terminal you ran it from.

Part 6: Final Test - Launch Gazebo with ROS2
Let's confirm everything works together. You will need two Ubuntu terminals for this.

In Terminal 1:

Launch Gazebo using the ROS2 launch file. This starts the simulator and the ROS2 bridge.

Bash
```
ros2 launch gazebo_ros gazebo.launch.py
```
After a moment, the Gazebo window should appear on your screen, showing an empty world.

In Terminal 2:

Use a ROS2 command to see if it can communicate with the Gazebo instance you just launched.

Bash
```
ros2 topic list
```
You should see a list of topics, including /clock and /tf. Seeing these means ROS2 is successfully communicating with Gazebo!
