

# Bidirectional Informed RRT* for Mobile Manipulation

[//]: # ([![N|Solid](https://cldup.com/dTxpPi9lDf.thumb.png)](https://nodesource.com/products/nsolid) )

This repository contains an efficient sampling-based planning framework, called BI^(2)RRT*, that extends the [Informed RRT*] of Gammell et al. towards bidirectional search, informed sampling for omnidirectional mobile manipulator robotic platforms and satisfaction of arbitrary geometric endeffector task constraints. The associated paper of the framework presented at the IEEE/RSJ International Conference on Intelligent Robots and Systems can be found in [Burget et al.]. 

### Features
  - Planning can be performed for the mobile mase / manipulator only or for the entire mobile manipulator
  - Satisfaction of arbitrary geometric end-effector task constraints (position and orientation)
    - Constraints are defined w.r.t a task frame (specified by the user)  
    - A bounded interval defines a permitted negative/positive deflection of the end-effector w.r.t the task frame (specified by the user)
  - Edge cost weights can be chosen in order to set a motion preference for the planner (e.g. to prefer paths that move the robotic arm rather than the base)
  - Other RRT-based planning algorithms can be obtained by activating/deactivating the features: bidirectional search, tree optimization, informed sampling

### Repository Packages

 - `birrt_star_algorithm`: Implementation of the BI^(2)RRT* algorithm
 - `planner_data_structures`: Defines the structure of the planner elements, i.e. tree, nodes and edges
 - `planner_param_config`: YAML configuration files for the planner 
 - `planner_heuristics`: Heuristics used by the planner
 - `validity_checker`: Library used by the planner to perform collision checks
 - `planning_world_builder`: Set of predefined planning worlds (other worlds can be added)
 - `planning_scenarios`: Example Planning Tasks (uses the planning worlds defined in the package above
 - `planner_statistics`: Stores the statistical results of planning runs (runtime, solution path cost etc.)

### Requirements

When planning is performed for a robot including a mobile base, the kinematic model of the robot defined in an URDF is expected to represent the base motion as a chain of two prismatic joints (x,y motion) followed by a revolute joint (base rotation), e.g.your URDF should contain something similar to
```sh
<link name="base_link">
    <visual>
      <geometry>
        <sphere radius="0.0"/>
      </geometry>
      <material name="blue">
        <color rgba="0.0 0.0 0.8 1.0"/>
      </material>
    </visual>
  </link>
  
  
   <link name="base_x_link">
    <visual>
      <geometry>
        <sphere radius="0.0"/>
      </geometry>
      <material name="blue">
        <color rgba="0.0 0.0 0.8 1.0"/>
      </material>
    </visual>
  </link>
  
  
  <joint name="base_x_joint" type="prismatic">
    <parent link="base_link"/>
    <child link="base_x_link"/>
	<limit effort="1000.0" lower="-100.0" upper="100.0" velocity="0.5"/> 
	<axis xyz="1 0 0"/>
	<!-- <origin rpy="0 1.57075 0" xyz="0 0 0"/> -->
  </joint>
  
  
   <link name="base_y_link">
    <visual>
      <geometry>
        <sphere radius="0.0"/>
      </geometry>
      <material name="blue">
        <color rgba="0.0 0.0 0.8 1.0"/>
      </material>
    </visual>
  </link>
  
  
  <joint name="base_y_joint" type="prismatic">
    <parent link="base_x_link"/>
    <child link="base_y_link"/>
	<axis xyz="0 1 0"/>
	<limit effort="1000.0" lower="-100.0" upper="100.0" velocity="0.5"/> 
	<!-- <origin rpy="-1.57075 0 0" xyz="0 0 0"/> -->
  </joint>
  
  
   <link name="platform_link">
    <visual>
      <geometry>
        <sphere radius="0.0"/>
      </geometry>
      <material name="blue">
        <color rgba="0.0 0.0 0.8 1.0"/>
      </material>
    </visual>
  </link>
  
  
  <joint name="platform_joint" type="fixed">
    <parent link="base_y_link"/>
    <child link="platform_link"/>
    <limit effort="1000.0" lower="-10.0" upper="10.0" velocity="0.5"/>
    <origin rpy="0 0 0" xyz="0 0 0"/>
  </joint>
  
  
  <link name="base_theta_link">
    <visual>
      <geometry>
        <sphere radius="0.0"/>
      </geometry>
      <material name="blue">
        <color rgba="0.0 0.0 0.8 1.0"/>
      </material>
    </visual>
  </link>
  
  
  <joint name="base_theta_joint" type="continuous">
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <parent link="platform_link"/>
    <child link="base_theta_link"/>
    <axis xyz="0 0 1"/>
  </joint>
```

Furhermore, a SRDF defining arbitrary kinematic chains later subject to planning queries is required, e.g.
```sh
<group name="omnirob_lbr_sdh">
  <chain base_link="base_link" tip_link="lbr_flange_link"/>
</group>
```


### Setup

Start a first launch file to set up a couple of things for planning as follows
```sh
<launch>

	<!-- Some paths required by the planner-->
	<param name="package_path" value="$(find kuka_motion_control)" />
	<param name="planner_package_path" value="$(find planner_statistics)" />
	<!-- Path required when precomputed terminal configurations are used for planning -->
	<param name="terminal_configs_path" value="$(find planning_scenarios)" />
	
	<!-- Publish world frame to which the robot will be connected to (for virtual joint in the SRDF)-->
	<node pkg="tf" type="static_transform_publisher" name="world_frame_publisher" args="0 0 0 0 0 0 /world_frame /base_link 100"/>

    <!-- Load URDF and SRDF -->
	<param name="robot_description" textfile="$(find omnirob_description)/urdf/omnirob_lbr_sdh2_extended_wo_pole.urdf" />
	<param name="robot_description_semantic" textfile="$(find omnirob_description)/srdf/omnirob_lbr_sdh2_wo_pole.srdf" />

	<!-- Joint and Robot State Publisher for simulated robot -->
	<node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher" />
	<node name="robot_state_publisher" pkg="robot_state_publisher" type="state_publisher" />
	
	<!-- Load a planner parameter configuration -->
	<rosparam command="load" file="$(find planner_param_config)/planner_config/planner_parameters_omnirob.yaml"/>

</launch>
```
Then, start a second launch file containing only the following part of the "*move_group.launch*" file
```sh
<launch>
<!-- GDB Debug Option -->
	<arg name="debug" default="false" />
	<arg unless="$(arg debug)" name="launch_prefix" value="" />-->
	<arg if="$(arg debug)" name="launch_prefix" value="gdb -x $(find moveit_config_omnirob_lbr_sdh)/launch/gdb_settings.gdb --ex run --args" />
	
	<!-- Verbose Mode Option -->
	<arg name="info" default="$(arg debug)" />
	<arg unless="$(arg info)" name="command_args" value="" />
	<arg     if="$(arg info)" name="command_args" value="--debug" />

	<!-- move_group settings -->
	<arg name="publish_monitored_planning_scene" default="true"/>

	<!-- Start the actual move_group node/action server -->
	<node name="$(arg robot)_move_group" launch-prefix="$(arg launch_prefix)" pkg="moveit_ros_move_group" type="move_group" respawn="false" output="screen" args="$(arg command_args)">
	  <!-- Set the display variable, in case OpenGL code is used internally -->
	  <env name="DISPLAY" value="$(optenv DISPLAY :0)" />
	  
	  <!-- MoveGroup capabilities to load -->
	  <param name="capabilities" value="move_group/MoveGroupCartesianPathService
				      move_group/MoveGroupExecuteService
				      move_group/MoveGroupKinematicsService
				      move_group/MoveGroupMoveAction
				      move_group/MoveGroupPickPlaceAction
				      move_group/MoveGroupPlanService
				      move_group/MoveGroupQueryPlannersService
				      move_group/MoveGroupStateValidationService
				      move_group/MoveGroupGetPlanningSceneService
				      move_group/ClearOctomapService
				      " />

	  <!-- Publish the planning scene of the physical robot so that rviz plugin can know actual robot -->
	  <param name="planning_scene_monitor/publish_planning_scene" value="$(arg publish_monitored_planning_scene)" />
	  <param name="planning_scene_monitor/publish_geometry_updates" value="$(arg publish_monitored_planning_scene)" />
	  <param name="planning_scene_monitor/publish_state_updates" value="$(arg publish_monitored_planning_scene)" />
	  <param name="planning_scene_monitor/publish_transforms_updates" value="$(arg publish_monitored_planning_scene)" />
	</node>
</launch>
```
This launch file is contained in the "*moveit_config_your_robot_name*" package which is generated for you by running the [MoveIt Setup Assistant] for your robot.

Finally start RViz and set the fixed frame in RViz to "*base_link*" (here's an example RViz setup)
```sh
<launch>
	<node pkg="rviz" type="rviz" name="rviz" args="-d $(find kuka_motion_control)/motion_control_display.rviz" />
</launch>
```

### Planning Examples

Planning examples can be found in the package `planning_scenarios`, which in turn makes use of predefined planning worlds implemented in the package `planning_world_builder`. Planning results, i.e. joint and end-effector trajectories, are strored as .txt files in the folder "*robot_motion_planning/planner_statistics/data*". Code for motion execution can be found in the `motion trajectory_execution` package in the [robot_motion_execution] repository.  

In the terminal:
```sh
rosrun planning_scenarios run_block_planning PLANNER_NAME PLANNING_GROUP NUM_PLANNING_RUNS FLAG_ITERATIONS_OR_TIME MAX_ITERATIONS_TIME RVIZ_SHOW_TREE
```
with:
   - PLANNER_NAME: bi_informed_rrt_star,bi_rrt_star,bi_informed_rrt,bi_rrt_connect etc.
   - PLANNING_GROUP: A planning group from your srdf
   - NUM_PLANNING_RUNS: Number of desired planning runs 
   - FLAG_ITERATIONS_OR_TIME: Run planning for a specific number of iterations (set 0) or seconds (set 1)
   - MAX_ITERATIONS_TIME: Maximum iterations/time available for planning
   - RVIZ_SHOW_TREE: Publish tree nodes and edges and current solution path in Rviz (set 1) or 0 otherwise.

### Todos

 - The packages "planning_server" and "planner_msgs" are currently under construction and will soon allow to call the planner via a ros service call
 - Add Chaining for Task Space Regions to define constraints relative to multiple task frames

[//]: # ( ++++++++++++++++++++++++++++++++++++++++++++++ Web Links +++++++++++++++++++++++++++++++++++++++++ )

[//]: # (These are reference links used in the body of this note and get stripped out when the markdown processor does its job. There is no need to format nicely because it shouldn't be seen. Thanks SO - http://stackoverflow.com/questions/4823468/store-comments-in-markdown-syntax)


   [Informed RRT*]: <https://arxiv.org/pdf/1404.2334v3.pdf>
   [Burget et al.]: http://www2.informatik.uni-freiburg.de/~burgetf/pub/burget16iros.pdf
   [MoveIt Setup Assistant]: http://docs.ros.org/hydro/api/moveit_setup_assistant/html/doc/tutorial.html
   [robot_motion_execution]: http://docs.ros.org/hydro/api/moveit_setup_assistant/html/doc/tutorial.html
   


