Think of it like this: one program will be the "game world" (the subscriber) where the turtle lives, and another program will be the "game controller" (the publisher) that tells the turtle how to move.

You will need two separate terminal windows for this.

## Step 1: Start the TurtleSim World (The Subscriber)
This command starts the simulator node, which waits for instructions.

Open your first terminal.

Make sure your ROS2 Humble environment is sourced (source /opt/ros/humble/setup.bash).

Run the following command:

Bash
```
ros2 run turtlesim turtlesim_node
```
A new window will pop up with a light blue background and a single turtle in the center. This window is the turtlesim node. It is now "subscribing" to (listening for) commands on a topic that controls the turtle's movement.

## Step 2: Start the Keyboard Controller (The Publisher)
Now, we'll start the node that sends the movement commands.

Open a new, separate terminal. Don't close the first one!

In this second terminal, source your ROS2 environment again.

Bash
```
source /opt/ros/humble/setup.bash
```
Run the command to start the keyboard controller:

Bash
```
ros2 run turtlesim turtle_teleop_key
```
This terminal will now become active. You'll see instructions like:

Reading from keyboard
---------------------------
Use arrow keys to move the turtle.
'q' to quit.
This node is now "publishing" your keystrokes as velocity commands.

## Step 3: Drive the Turtle!
Click on the second terminal (the one running turtle_teleop_key) to make sure it's the active window.

Use the arrow keys on your keyboard.

You will see the turtle in the blue window move around in response to your key presses! You have successfully used one node (the keyboard teleop) to publish commands to another node (the turtlesim world).

