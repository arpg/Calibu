#!/bin/bash
SCRIPTS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd $SCRIPTS_DIR/..;

REPO_URL=https://github.com/gwu-robotics/rpg-cmake.git
DEST_DIR=cmake_modules

git subtree pull --prefix $DEST_DIR  $REPO_URL  master --squash
