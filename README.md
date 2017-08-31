# Robotics - Control of Mobile Robot
This repository contains my implementation of the programming asssignments of Control of Mobile Robots course delivered by Georgia Institute of Technology on Coursera

## 1. Mobile Robot Simulator
The programming exercises use a simulated version of a robot called the QuickBot. The simulated QuickBot is equipped with five **infrared (IR) range sensors**, of which three are located in the front and two are located on its sides. The simulated QuickBot has a two-wheel difierential drive system (two wheels, two motors) with a wheel encoder for each wheel. You can build the QuickBot yourself by following the hardware videos located under the Course Resources tab.

![quickbot](./images/quickbot.jpg)

## 2. Differential-Drive, Odometry, and IR
The purpose of this week’s programming assignment is to implement the functions for the robot to **move and sense**.

    1. Transform the outputs of our controllers to the control inputs of the mobile robot.
    2. Keep track of where the robot is located.
    3. Convert raw sensor values to distances.
