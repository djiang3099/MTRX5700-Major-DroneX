echo "alias dReset='rostopic pub -1 /ardrone/reset std_msgs/Empty "{}" '" >> ~/.bash_aliases
echo "alias dLand='rostopic pub -1 /ardrone/land std_msgs/Empty "{}" '" >> ~/.bash_aliases
echo "alias dTakeoff='rosservice call /ardrone/flattrim  && rostopic pub -1 /ardrone/takeoff std_msgs/Empty "{}" '" >> ~/.bash_aliases

echo "alias dFlattrim='rosservice call /ardrone/flattrim '" >> ~/.bash_aliases

source ~/.bash_aliases




