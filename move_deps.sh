#! /bin/bash

BUILD_YEAR=2021
BUILD_REPO_DIR=./build/repos
RELEASE_DIR=$BUILD_REPO_DIR/releases/com/thriftybot/frc
MOVE_DIR=C:/Users/Public/wpilib/$BUILD_YEAR/maven/com/thriftybot/frc
VENDOR_DIR=C:/Users/Public/wpilib/$BUILD_YEAR/vendordeps

echo "*** Moving maven directories ***"
cp -r $RELEASE_DIR/ThriftySwerve-cpp $MOVE_DIR
cp -r $RELEASE_DIR/ThriftySwerve-java $MOVE_DIR

echo "*** Moving vendor deps ***"
cp -r ./vendordeps/* $VENDOR_DIR
