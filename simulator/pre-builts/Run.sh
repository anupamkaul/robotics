#!/bin/bash

# you can simulate xml files under the model dir using simulate, like so:

cd mujoco-2.3.1/bin
./simulate ../model/humanoid/humanoid.xml &

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

# example:
# ./simulate ../../../mujoco_menagerie/anybotics_anymal_b/scene.xml
# (use the controls menu to manipulate joint forces and watch the robot move/fall)




