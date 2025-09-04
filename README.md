# Outdoor Autonomous Delivery Robot

**Quick Overview**

This repo hopefully walks you through the code I wrote for my graduation project—how it works, what its issues and limitations are, and how you can get it running on your own hardware.
I originally wanted to use *MATLAB/Simulink* to build all the models and generate code with *Simulink Coder*. Sure, building everything from scratch gives you a tighter grip over the system and more flexibility—but with the limited time I had, it just wasn’t practical. So I had to make a few compromises.

The first was using *ROS2* as my middleware. Writing your own can be fun, no doubt—but let’s be honest, it’s kind of obsolete. ROS2 is mature, well-supported, and saves a ton of time. Now, the elephant in the room was figuring out how to integrate the already developed and tested Simulink models into the ROS2 environment. Luckily, the *ROS Toolbox* came to the rescue—it bridges the gap and makes the whole process surprisingly smooth.

The second one is more of a design choice than a compromise. Trying to achieve real-time performance by deploying the entire codebase—Perception, Mapping, Path Planning, and Control—on a single *Raspberry Pi* is, frankly, a bit of a naive dream. Especially when the code isn’t mature yet. So, I introduced two additional microcontrollers to help distribute the load and keep things responsive—kinda like a mini distributed control system (DCS).

The *Arduino Mega* was chosen mainly because it has four serial communication ports, which fit my design requirements perfectly. Its main job was to control the motors based on incoming control commands and gather feedback, which it then passed on to the second microcontroller, the *ESP32*.
To be completely honest, I added the ESP32 because it supports the *micro-ROS* library, which made it possible to publish the odometry data coming from the Mega. Without that, integrating low-level feedback into the ROS2 ecosystem would’ve been a lot messier.
I also took advantage of the ESP32 by offloading some initial tasks to it—like performing sensor fusion with the IMU. It made sense to handle that locally before passing data into the ROS2 pipeline, and it helped reduce the load on the Raspberry Pi. (See the [figure below](Docs/figures/figure_1.png).)

The third—and thankfully final—compromise was using the [YDLiDAR ROS2 Driver Package](https://github.com/YDLIDAR/ydlidar_ros2_driver) instead of the models I was developing. The models weren’t fully tested yet, and I was running outta time. That said, you’ll still find the Simulink models included in the repo, in case you want to explore or build on it later.

<img width="2048" height="1137" alt="figure_1" src="Docs/figures/figure_1.png" />


The codebase is pretty diverse—it includes Simulink models, Arduino IDE code (C++), Python scripts, and a bunch of configuration files sprinkled throughout. Hope you enjoy digging into it 😉. 

GODA

**System Design**
