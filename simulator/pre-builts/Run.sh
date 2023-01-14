#!/bin/bash

# you can simulate xml files under the model dir using simulate, like so:

cd mujoco-2.3.1/bin
./simulate ../model/humanoid/humanoid.xml &

# note humanoid has 27DOF. 
# MJPC: humanoid has 21 tasks and the task(goal) is 'standing' (RL/planners: iLQG, GD, Predictive Sampling)

# other pre-built models are:

# ./balloons/balloons.xml
# ./flag/flag.xml
# ./mug/mug.xml
# ./plugin/coil.xml
# ./plugin/press.xml
# ./plugin/cable.xml
# ./plugin/floppy.xml
# ./plugin/scene.xml
# ./plugin/belt.xml
# ./plugin/jelly.xml
# ./humanoid100/humanoid_body.xml
# ./humanoid100/humanoid100.xml
# ./composite/particle_free.xml
# ./composite/scene.xml
# ./composite/rope.xml
# ./composite/loop.xml
# ./composite/grid2pin.xml
# ./composite/particle_free2d.xml
# ./composite/cloth.xml
# ./composite/softbox.xml
# ./composite/particle.xml
# ./adhesion/active_adhesion.xml
# ./slider_crank/slider_crank.xml
# ./humanoid/humanoid.xml
# ./humanoid/22_humanoids.xml
# ./hammock/humanoid_body.xml
# ./hammock/hammock.xml
# ./tendon_arm/arm26.xml

# additional models from menagerie are:
# (using $PATH_MENAGERIE with ./simulate)

# ./universal_robots_ur5e/ur5e.xml
# ./universal_robots_ur5e/scene.xml
# ./anybotics_anymal_b/scene.xml
# ./anybotics_anymal_b/anymal_b.xml
# ./robotiq_2f85/scene.xml
# ./robotiq_2f85/2f85.xml
# ./franka_emika_panda/scene.xml
# ./franka_emika_panda/panda.xml
# ./franka_emika_panda/hand.xml
# ./franka_emika_panda/panda_nohand.xml
# ./unitree_a1/a1.xml
# ./unitree_a1/scene.xml
# ./anybotics_anymal_c/scene.xml
# ./anybotics_anymal_c/anymal_c.xml
# ./agility_cassie/cassie.xml
# ./agility_cassie/scene.xml
# ./shadow_hand/left_hand.xml
# ./shadow_hand/scene_right.xml
# ./shadow_hand/scene_left.xml
# ./shadow_hand/right_hand.xml

# examples and notes:
# ./simulate ../../../mujoco_menagerie/anybotics_anymal_b/scene.xml
# use the controls menu to manipulate joint forces and watch the robot move/fall

#  ./simulate ../../../mujoco_menagerie/franka_emika_panda/scene.xml 
#  in this one I can control 7 actuators via the actuator menu (like the dentist robot model)

# ./simulate ../../../mujoco_menagerie/universal_robots_ur5e/scene.xml 
# similar to franka_emika, this also I can manipulate actuators for pick & place. Seems like on pause setting actuators can 
# simulate a run, and that joint values are modifiable only in the pause state. 

# ./simulate ../../../mujoco_menagerie/robotiq_2f85/scene.xml 
# this is a good one for upwards object grasping (set the control menu on right to grasp and ungrasp with fingers)

# ./simulate ../../../mujoco_menagerie/shadow_hand/scene_left.xml 
# this is used to grasp an object with fingers of a hand

# additional models from dm_control (with rl environments in mujoco):
# (using $PATH_DM_CONTROL with ./simulate)

# ./suite/quadruped.xml
# ./suite/fish.xml
# ./suite/common/materials.xml
# ./suite/common/skybox.xml
# ./suite/common/visual.xml
# ./suite/walker.xml
# ./suite/ball_in_cup.xml
# ./suite/finger.xml
# ./suite/pendulum.xml
# ./suite/hopper.xml
# ./suite/dog.xml
# ./suite/manipulator.xml
# ./suite/acrobot.xml
# ./suite/cheetah.xml
# ./suite/humanoid.xml
# ./suite/reacher.xml
# ./suite/stacker.xml
# ./suite/point_mass.xml
# ./suite/cartpole.xml
# ./suite/lqr.xml
# ./suite/humanoid_CMU.xml
# ./suite/swimmer.xml
# ./mujoco/testing/assets/model_with_ball_joints.xml
# ./mujoco/testing/assets/arm.xml
# ./mujoco/testing/assets/model_with_third_order_actuators.xml
# ./mujoco/testing/assets/cartpole_no_names.xml
# ./mujoco/testing/assets/humanoid.xml
# ./mujoco/testing/assets/model_with_assets.xml
# ./mujoco/testing/assets/cartpole.xml
# ./mujoco/testing/assets/sphere.xml
# ./entities/props/duplo/duplo2x4.xml
# ./composer/arena.xml
# ./mjcf/schema.xml
# ./mjcf/test_assets/lego_brick.xml
# ./mjcf/test_assets/included_with_invalid_filenames.xml
# ./mjcf/test_assets/arena.xml
# ./mjcf/test_assets/attribute_test_schema.xml
# ./mjcf/test_assets/model_with_include.xml
# ./mjcf/test_assets/included.xml
# ./mjcf/test_assets/test_model.xml
# ./mjcf/test_assets/model_with_assets.xml
# ./mjcf/test_assets/model_with_invalid_filenames.xml
# ./mjcf/test_assets/robot_arm.xml
# ./mjcf/test_assets/model_with_nameless_assets.xml
# ./third_party/kinova/jaco_arm.xml
# ./third_party/kinova/jaco_hand.xml
# ./third_party/kinova/common.xml
# ./third_party/ant/ant.xml
# ./locomotion/soccer/assets/boxhead/boxhead.xml
# ./locomotion/walkers/assets/humanoid_CMU_V2019.xml
# ./locomotion/walkers/assets/dog_v2/scene.xml
# ./locomotion/walkers/assets/dog_v2/dog.xml
# ./locomotion/walkers/assets/dog_v2/dog_base.xml
# ./locomotion/walkers/assets/humanoid_CMU_V2020.xml
# ./locomotion/walkers/assets/jumping_ball/jumping_ball_with_head.xml
# ./locomotion/walkers/assets/rodent.xml



