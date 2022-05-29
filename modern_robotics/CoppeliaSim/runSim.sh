#!/bin/bash

echo "Running CoppeliaSim simulator (local downloaded version)"

cd InstallUbuntu/CoppeliaSim_Player_V4_2_0_Ubuntu16_04
# ls

# run the coppeliaSim executable (script)
# (try $1 = '../../Scenes/V-REP_scenes/Scene6_youBot_cube.ttt ') - from the 'current' path
./coppeliaSim.sh $1


# or uncomment and run this to show a preloaded pick-and-place with Kuka (Scene 6)
#./coppeliaSim.sh ../../Scenes/V-REP_scenes/Scene6_youBot_cube.ttt

# others:
# ./coppeliaSim.sh ../../Scenes/V-REP_scenes/Scene1_UR5.ttt
# ./coppeliaSim.sh ../../Scenes/V-REP_scenes/Scene2_UR5_csv.ttt
# ./coppeliaSim.sh ../../Scenes/V-REP_scenes/Scene3_youBot.ttt
# ./coppeliaSim.sh ../../Scenes/V-REP_scenes/Scene4_youBot_csv.ttt
# ./coppeliaSim.sh ../../Scenes/V-REP_scenes/Scene5_motion_planning.ttt
# ./coppeliaSim.sh ../../Scenes/V-REP_scenes/Scene6_youBot_cube.ttt
# ./coppeliaSim.sh ../../Scenes/V-REP_scenes/Scene7_MTB_csv.ttt
# ./coppeliaSim.sh ../../Scenes/V-REP_scenes/Scene8_gripper_csv.ttt


# (tested that 'run' works with all scenes (1-8) 
#./run

# -------------------------------------------------

# user-manual: (start reading from here always):
# https://coppeliarobotics.com/helpFiles/index.html

# other resources:
# https://coppeliarobotics.com/resources

# developing code:
# https://github.com/CoppeliaRobotics


# My Notes..
#To open some pre-installed scenes:
#1. File, open Scene..
#2. Pick up a .ttt file from github_robotics/modern_robotics/CoppeliaSim/Scenes/V-REP_scenes
#3. Run the "play" button to play the scene. Some scenes can offer direct manipulation of params, while most offer .csv files (example: Scene7_example.csv)
#4. This page has intros to some tutorials to play the 8 scenes : http://hades.mech.northwestern.edu/index.php/CoppeliaSim_Introduction

#-----

#Scene examples:
#1. Pick and place : load scene: ../Scenes/V-REP_scenes/Scene6_youBot_cube.ttt and play simulation with file ../Scenes/V-REP_scenes/Scene6_example.csv

# ----------------------------------------------------






