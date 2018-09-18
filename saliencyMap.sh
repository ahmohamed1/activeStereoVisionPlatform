#! /bin/bash


printf 'Running Vergence Tracking System'
printf '\n'
#x-terminal-emulator -e roscore 2>/dev/null &
sleep 1 &&
x-terminal-emulator -e rosrun platform_vision rectify_image $input_choice 2>/dev/null &

sleep 2 &&
printf '\n'
printf 'Running Master Camera Controller..' 

x-terminal-emulator -e rosrun visual_attention_base saliencyMapCpp 2>/dev/null &

sleep 2 &&
printf '\n'
printf 'Running Slave Camera Controller..' 
x-terminal-emulator -e rosrun platform_vision fastTemplateMtching 200 70 2>/dev/null &
