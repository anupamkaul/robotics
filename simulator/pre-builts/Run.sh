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

# ./simulate ../../../dm_control/dm_control/suite/quadruped.xml

#./dm_control/dm_control/suite/quadruped.xml
#./dm_control/dm_control/suite/fish.xml
#./dm_control/dm_control/suite/common/materials.xml
#./dm_control/dm_control/suite/common/skybox.xml
#./dm_control/dm_control/suite/common/visual.xml
#./dm_control/dm_control/suite/walker.xml
#./dm_control/dm_control/suite/ball_in_cup.xml
#./dm_control/dm_control/suite/finger.xml
#./dm_control/dm_control/suite/pendulum.xml
#./dm_control/dm_control/suite/hopper.xml
#./dm_control/dm_control/suite/dog.xml
#./dm_control/dm_control/suite/manipulator.xml
#./dm_control/dm_control/suite/acrobot.xml
#./dm_control/dm_control/suite/cheetah.xml
#./dm_control/dm_control/suite/humanoid.xml
#./dm_control/dm_control/suite/reacher.xml
#./dm_control/dm_control/suite/stacker.xml
#./dm_control/dm_control/suite/point_mass.xml
#./dm_control/dm_control/suite/cartpole.xml
#./dm_control/dm_control/suite/lqr.xml
#./dm_control/dm_control/suite/humanoid_CMU.xml
#./dm_control/dm_control/suite/swimmer.xml
#./dm_control/dm_control/mujoco/testing/assets/model_with_ball_joints.xml
#./dm_control/dm_control/mujoco/testing/assets/arm.xml
#./dm_control/dm_control/mujoco/testing/assets/model_with_third_order_actuators.xml
#./dm_control/dm_control/mujoco/testing/assets/cartpole_no_names.xml
#./dm_control/dm_control/mujoco/testing/assets/humanoid.xml
#./dm_control/dm_control/mujoco/testing/assets/model_with_assets.xml
#./dm_control/dm_control/mujoco/testing/assets/cartpole.xml
#./dm_control/dm_control/mujoco/testing/assets/sphere.xml
#./dm_control/dm_control/entities/props/duplo/duplo2x4.xml
#./dm_control/dm_control/composer/arena.xml
#./dm_control/dm_control/mjcf/schema.xml
#./dm_control/dm_control/mjcf/test_assets/lego_brick.xml
#./dm_control/dm_control/mjcf/test_assets/included_with_invalid_filenames.xml
#./dm_control/dm_control/mjcf/test_assets/arena.xml
#./dm_control/dm_control/mjcf/test_assets/attribute_test_schema.xml
#./dm_control/dm_control/mjcf/test_assets/model_with_include.xml
#./dm_control/dm_control/mjcf/test_assets/included.xml
#./dm_control/dm_control/mjcf/test_assets/test_model.xml
#./dm_control/dm_control/mjcf/test_assets/model_with_assets.xml
#./dm_control/dm_control/mjcf/test_assets/model_with_invalid_filenames.xml
#./dm_control/dm_control/mjcf/test_assets/robot_arm.xml
#./dm_control/dm_control/mjcf/test_assets/model_with_nameless_assets.xml
#./dm_control/dm_control/third_party/kinova/jaco_arm.xml
#./dm_control/dm_control/third_party/kinova/jaco_hand.xml
#./dm_control/dm_control/third_party/kinova/common.xml
#./dm_control/dm_control/third_party/ant/ant.xml
#./dm_control/dm_control/locomotion/soccer/assets/boxhead/boxhead.xml
#./dm_control/dm_control/locomotion/walkers/assets/humanoid_CMU_V2019.xml
#./dm_control/dm_control/locomotion/walkers/assets/dog_v2/scene.xml
#./dm_control/dm_control/locomotion/walkers/assets/dog_v2/dog.xml
#./dm_control/dm_control/locomotion/walkers/assets/dog_v2/dog_base.xml
#./dm_control/dm_control/locomotion/walkers/assets/humanoid_CMU_V2020.xml
#./dm_control/dm_control/locomotion/walkers/assets/jumping_ball/jumping_ball_with_head.xml
#./dm_control/dm_control/locomotion/walkers/assets/rodent.xml



