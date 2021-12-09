# Awesome Simulation [![Awesome](https://cdn.rawgit.com/sindresorhus/awesome/d7305f38d29fed78fa85652e3a63e154dd8e8829/media/badge.svg)](https://github.com/sindresorhus/awesome)

A curated list of awesome simulation frameworks and projects


- [Awesome Simulation](#awesome-simulation)
    - [PyBullet](#PyBullet)
    - [MuJoCo](#MuJoCo)
    - [Gazebo](#Gazebo)
    - [Brax](#Brax)
    - [NVIDIA-Omniverse](#NVIDIA-Omniverse)
    - [Unreal Engine](#Unreal-Engine)

- [Reinforcement Learning Toolkit](#Reinforcement-Learning-Platform)
    - [Open AI Gym](#Open-AI-Gym)
    - [MLAgent](#ml-agents)

- [Contributing](#contributing)

---

## [PyBullet](https://github.com/bulletphysics/bullet3)

[Github](https://github.com/bulletphysics/bullet3)

[Guide](https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#heading=h.2ye70wns7io3)

License: zlib license

Languages: python 3

Physics Engine: bullet

* [pybullet-gym](https://github.com/benelot/pybullet-gym) - Open-source implementation of OpenAI Gym MuJoCo environments for use with the OpenAI Gym Reinforcement Learning Research Platform
* [pybullet_robots](https://github.com/erwincoumans/pybullet_robots) - Prototyping robots for PyBullet
* [ravens](https://github.com/google-research/ravens) - Train robotic agents to learn pick and place with deep learning for vision-based manipulation in PyBullet. Transporter Nets, CoRL 2020.
* [gym-pybullet-drones](https://github.com/utiasDSL/gym-pybullet-drones) - PyBullet Gym environments for single and multi-agent reinforcement learning of quadcopter control
* [pybullet-planning](https://github.com/caelan/pybullet-planning) - PyBullet Planning
* [pybullet_planning](https://github.com/yijiangh/pybullet_planning) - A suite of utility functions to facilitate robotic planning related research on the pybullet physics simulation engine.
* [pybullet-robot-envs](https://github.com/robotology-playground/pybullet-robot-envs)
* [pybullet_rendering](https://github.com/ikalevatykh/pybullet_rendering) - External rendering for PyBullet
* [quadruped_ctrl](https://github.com/Derek-TH-Wang/quadruped_ctrl) - MIT mini cheetah quadruped robot simulated in pybullet environment using ros.
* [panda-gym](https://github.com/qgallouedec/panda-gym) - OpenaAI Gym Franka Emika Panda robot environment based on PyBullet.
* [pybullet_multigoal_gym](https://github.com/IanYangChina/pybullet_multigoal_gym) - Pybullet version of the multigoal robotics environment from OpenAI Gym

## [MuJoCo](https://mujoco.org)

[Github](https://github.com/deepmind/mujoco)

[Documents](https://mujoco.readthedocs.io/en/latest/overview.html)

License: Apache License 2.0

Languages: C

Physics Engine: MuJoCo

* [dm_control](https://github.com/deepmind/dm_control) - DeepMind's software stack for physics-based simulation
* [mujoco-py](https://github.com/openai/mujoco-py) - mujoco-py allows using MuJoCo from Python 3
* [mujoco-worldgen](https://github.com/openai/mujoco-worldgen) - Automatic object XML generation for Mujoco
* [mjrl](https://github.com/aravindr93/mjrl) - Reinforcement learning algorithms for MuJoCo tasks
* [multiagent_mujoco](https://github.com/schroederdewitt/multiagent_mujoco) - Benchmark for Continuous Multi Agent Robotic Control, based on OpenAI's Mujoco Gym environments
* [metaworld](https://github.com/rlworkgroup/metaworld) - An open source robotics benchmark for meta- and multi-task reinforcement learning
* [cassie-mujoco-sim](https://github.com/osudrl/cassie-mujoco-sim) - A simulation library for Agility Robotics' Cassie robot using MuJoCo
* [MuJoCo_RL_UR5](https://github.com/PaulDanielML/MuJoCo_RL_UR5) - A MuJoCo/Gym environment for robot control using Reinforcement Learning. The task of agents in this environment is pixel-wise prediction of grasp success chances.

## [Gazebo](http://gazebosim.org/)

[Github](https://github.com/osrf/gazebo)

License: Apache License 2.0

Support Languagues: C++, Python

Physics: ODE, Bullet, Simbody and DART

* [osrf/gazebo_tutorials](https://github.com/osrf/gazebo_tutorials) - Tutorials for gazebo
* [ros-simulation/gazebo_ros_pkgs](https://github.com/ros-simulation/gazebo_ros_pkgs) - Wrappers, tools and additional API's for using ROS with Gazebo
* [osrf/gazebo_models](https://github.com/osrf/gazebo_models) - Model database
* [PX4/PX4-SITL_gazebo](https://github.com/PX4/PX4-SITL_gazebo) - Set of plugins, models and worlds to use with OSRF Gazebo Simulator in SITL and HITL.
* [robin-shaun/XTDrone](https://github.com/robin-shaun/XTDrone) - UAV Simulation Platform based on PX4, ROS and Gazebo
* [ethz-asl/rotors_simulator](https://github.com/ethz-asl/rotors_simulator) - RotorS is a UAV gazebo simulator
* [lihuang3/ur5_ROS-Gazebo](https://github.com/lihuang3/ur5_ROS-Gazebo) - Universal Robot (UR5) Pick and Place Simulation in ROS-Gazebo with a USB Cam and Vacuum Grippers
* [ignitionrobotics/ign-gazebo](https://github.com/ignitionrobotics/ign-gazebo) - Open source robotics simulator. Through Ignition Gazebo users have access to high fidelity physics, rendering, and sensor models.
* [turtlebot/turtlebot_simulator](https://github.com/turtlebot/turtlebot_simulator) - Launcers for Gazebo simulation of the TurtleBot
* [mit-racecar/racecar_gazebo](https://github.com/mit-racecar/racecar_gazebo) - A gazebo-based simulator of the MIT Racecar.

## [Brax](https://arxiv.org/abs/2106.13281)

[Github](https://github.com/google/brax)

License: Apache 2.0

Support Languages: Python

## [NVIDIA-Omniverse](https://developer.nvidia.com/nvidia-omniverse-platform)

[IsaacSim](https://developer.nvidia.com/isaac-sim)
[IsaacGym](https://developer.nvidia.com/isaac-gym)
[IsaacGymEnvs](https://github.com/NVIDIA-Omniverse/IsaacGymEnvs)


## [Unreal Engine](https://www.unrealengine.com/en-US/)

[Github](https://github.com/EpicGames/UnrealEngine)

To access this repository, you have to get permission from epic games

[Documentation](https://docs.unrealengine.com/4.27/en-US/)

License: Unreal Engine

Support Languages: C++

Physics Engine: PhyX

* [UnrealEnginePython](https://github.com/20tab/UnrealEnginePython) - Embed Python in Unreal Engine 4
* [UnrealCV](https://github.com/unrealcv/unrealcv) - Connecting Computer Vision to Unreal Engine

# Reinforcement Learning Toolkit

## [Open AI Gym](https://gym.openai.com/)

[Github](https://github.com/openai/gym) - A toolkit for developing and comparing reinforcement learning algorithms.

License: MIT License

Language: Python

## [ml-agents](https://github.com/Unity-Technologies/ml-agents)

License: Apache 2.0

Language: C#, Python

# Contributing

Your contributions are always welcome!
