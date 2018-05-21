#! /bin/bash


printf 'Running Vergence Tracking System'
printf '\n'
#x-terminal-emulator -e roscore 2>/dev/null &
sleep 1 &&
read -p 'Running marker tracking.. Please select the algorithm use in tracking: ' input_choice

x-terminal-emulator -e rosrun platform_vision trackingSystem.py $input_choice 2>/dev/null &

sleep 2 &&
printf '\n'
printf 'Running Slave Camera Controller..' 
cd /home/abdulla/dev/Active-stereo-Vision-Platform/ros

x-terminal-emulator -e rosrun platform_vision fastMatchingPyramid.py 2>/dev/null &

sleep 3 &&
printf '\n'
printf 'Running Graphic Output..\n' 
x-terminal-emulator -e rosrun platform_controller depthBaseVerge.py 2>/dev/null &

echo '  ' 
read -p 'Do you want to run multi-target saver code (y / n): ' input_choice

if [ “$input_choice” = “y” ]
then
x-terminal-emulator -e rosrun platform_controller multi-target-test.py 2>/dev/null &
else
printf 'Okay nothing will be run\n' 
fi
