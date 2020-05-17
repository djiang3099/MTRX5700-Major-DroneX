echo "alias dReset='rostopic pub -1 /ardrone/reset std_msgs/Empty "{}" '" >> ~/.bash_aliases
echo "alias dLand='rostopic pub -1 /ardrone/land std_msgs/Empty "{}" '" >> ~/.bash_aliases
echo "alias dTakeoff='rostopic pub -1 /ardrone/takeoff std_msgs/Empty "{}" '" >> ~/.bash_aliases

echo "alias dHover='rostopic pub -1 /cmd_vel geometry_msgs/Twist "linear:x: 0.0  y: 0.0  z: 0.0 angular: x: 0.0 y: 0.0 z: 0.0" '" >> ~/.bash_aliases

echo "alias dFlattrim='rosservice call /ardrone/flattrim '" >> ~/.bash_aliases

source ~/.bash_aliases




