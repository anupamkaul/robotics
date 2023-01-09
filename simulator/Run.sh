#!/bin/bash

# 1. Install mujoco on my linux machine

echo -e "Installing mujoco (getting github source)...\n"

git clone https://github.com/anupamkaul/mujoco.git
    cd mujoco
    git checkout main
    git remote add upstream https://github.com/deepmind/mujoco.git
    git remote -v
    # to-do (if needed): git pull upstream to master (and git merge with mine for latest updates)

# 2. Building: refer to these guides for building for source for ubuntu:







