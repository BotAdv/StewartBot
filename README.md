# StewartBot

____

## Introduce

This project is a bachelor's degree graduation project, which aims to verify the feasibility of parallel configuration of large-load robots.
The instructor is [Xu Kailiang](https://zdh.xaut.edu.cn/info/1043/1330.htm),

____

## To Do List

- [ ] Framework Build  
- [ ] CanOpen Nimotion Motor Control  
- [ ] Muti Motor Control  
- [ ] Kinematic modeling  
- [ ] Structural design
- [ ] Simulation && Prototype verification  
- [ ] Binocular visual perception module  
- [ ] LiDAR mapping module
- [ ] Enclosed space SLAM

____

## Implementation

### Environment setup

| Items      | Option |  
| ----------- | ----------- |  
|*System* |**X86_64-Unbutu22.04** |  
|*Software systems* |**ROS2 humble**  |  
|*Hardware platform* |**NiMotion Motor-STM4248A** |  
|*Communication* |**CANopen** |  
|*Controller* |**BECKOFF**|  

### Get source code  

- Create workspace

  ```shell
  mkdir -p robot_ws/src
  ```  

  ```shell
  cd robot_ws
  ```  

- Initialize the workspace  

  ```shell
  colcon build
  ```  

  ```shell
  cd src
  ```  

- Fetch source code  

  ```shell
  git clone https://github.com/BotAdv/StewartBot.git
  ```

[StewartBot-dev](https://github.com/BotAdv/StewartBot.git)

____
