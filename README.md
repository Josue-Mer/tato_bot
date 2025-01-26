## Autonomous Luggage Cart Robot (Terry)

This is a project for an autonomous luggage cart based on ROS. It uses the project articubot_one from [Josh Newans](https://github.com/joshnewans/articubot_one/tree/c75cf3360e4574ff40aa6438c07739389ad3f8ed) as a template and expands from it to create more nodes and adjust it for the desired functionality.

The system uses readings from UWB sensors, 2 as anchors in the robot, and 1 as a tag with the user. The two anchors are connected to an ESP32-S3 microcontroller, which gets distance readings from each anchor to the tag, then calculates an x,y position of the tag in respect to an anchor and sends the position through a topic called uwb_position with the use of micro-ROS. Then, it matches the UWB position readings to readings from the LiDAR and the depth camera (OAK-D Lite, programmed to detect a person) to locate the user and follow them using Nav2.

Developed by Josué Merán & Ithan Pacheco.
