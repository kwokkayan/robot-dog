# 3D Model
## Information
1. 3D models are required for 3D printing and simulation in ros
2. for 3D printing: use sketchup to draw stl models.
3. for ros simulation: use [xacro](https://github.com/ros/xacro/wiki) or other tools to configure the model. Then, import the model to [champ assistant](https://github.com/chvmp/champ_setup_assistant) to create the packages. The ros plugin in VS code can be used to visualize the model in development.
## Tasks
1. Create URDF model for ROS simulation. Use the current stl models for now.
2. 3D printing: battery, microcontroller, and IMU holder inside main body. 
3. 3D printing: Modify lower leg joints to allow greater freedom of movement. 
4. 3D printing: Add ball bearings to upper leg joints for smooth movement.
5. 3D printing: Modify SBC holder on top to add damping for greater stability.