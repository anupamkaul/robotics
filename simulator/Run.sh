#!/bin/bash

# 1. Install mujoco on my linux machine

echo -e "1. Installing mujoco (getting github source)...\n"

git clone https://github.com/anupamkaul/mujoco.git
    cd mujoco
    git checkout main
    git remote add upstream https://github.com/deepmind/mujoco.git
    git remote -v

    # to-do (if needed): git pull upstream to master (and git merge with mine for latest updates)
    # to-do: build instructions

# 2. Install mujoco_menagerie on my linux machine

echo -e "2. Installing mujoco_menagerie (getting github source)...\n"

git clone https://github.com/anupamkaul/mujoco_menagerie.git
    cd mujoco_menagerie
    git checkout main
    git remote add upstream https://github.com/deepmind/mujoco_menagerie.git
    git remote -v

    # to-do (if needed): git pull upstream to master (and git merge with mine for latest updates)
    # to-do: build instructions
    # see pre-builts/Run.sh on how to invoke xml models using simulate with menagerie

	https://github.com/deepmind/mujoco_mpc.git

# 3. Install mujoco_mpc on my linux machine

echo -e "3. Installing mujoco_mpc (getting github source)...\n"

git clone https://github.com/anupamkaul/mujoco_mpc.git
    cd mujoco_mpc
    git checkout main
    git remote add upstream https://github.com/deepmind/mujoco_mpc.git
    git remote -v

    # to-do (if needed): git pull upstream to master (and git merge with mine for latest updates)
    # to-do: build instructions
    # see pre-builts/Run.sh on how to invoke xml models using simulate with mpc









