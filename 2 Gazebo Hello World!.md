
# Gazebo: Hello, world!
In this section, you will start from scratch and develop your own package in ROS throughout this project!

You  [previously](https://classroom.udacity.com/nanodegrees/nd209/parts/c199593e-1e9a-4830-8e29-2c86f70f489e/modules/2919466f-aa2b-4424-b86a-98b0a53ce335/lessons/658c94f5-f806-4273-9001-9e2838e56856/concepts/a777bc7a-95d4-44ca-b4e3-119718e3a213)  added packages to your ROS workspace,  `catkin_ws`. You can follow the same instructions to create  `catkin_ws`  in the Udacity provided Project Workspace's  `/home/workspace/`  directory.

Let's start by navigating to the  `src`  directory and creating a new empty package.

```u
$ cd /home/workspace/catkin_ws/src/
$ catkin_create_pkg udacity_bot
```

Next, create folders,  `launch`  and  `worlds`, that will further define the structure of your package.

```bash
$ cd udacity_bot
$ mkdir launch
$ mkdir worlds
```

## Gazebo worlds

In  `worlds`, you will save each individual Gazebo world. A world is a collection of models such as your robot, and a specific environment such as the cafe from the Pick and Place project! You can also define several other physical properties specific to this world.

[](https://classroom.udacity.com/nanodegrees/nd209/parts/dad7b7cc-9cce-4be4-876e-30935216c8fa/modules/f5048868-4bd8-4e8d-8c6b-69bd559ed9db/lessons/3db51895-010e-4922-b322-4817cb76524e/concepts/65ec3b3d-6438-45bd-9930-d748c99258f4#)

![](https://s3.amazonaws.com/video.udacity-data.com/topher/2017/June/593ae492_gazebo-demo/gazebo-demo.gif)


Let's create a simple world, with no objects or models that will be launched later in Gazebo.

```bash
$ cd worlds
$ nano udacity.world
```

Add the following to  `udacity.world`

```xml
<?xml version="1.0" ?>

<sdf version="1.4">

  <world name="default">

    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Light source -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- World camera -->
    <gui fullscreen='0'>
      <camera name='world_camera'>
        <pose>4.927360 -4.376610 3.740080 0.000000 0.275643 2.356190</pose>
        <view_controller>orbit</view_controller>
      </camera>
    </gui>

  </world>
</sdf>
```

**Note:** Copying over to the Workspace Desktop GUI can be tricky. Instead, copy the content through the Workspace IDE as shown below. Just open (double-click) on the file in the IDE and paste the code there.

[](https://classroom.udacity.com/nanodegrees/nd209/parts/dad7b7cc-9cce-4be4-876e-30935216c8fa/modules/f5048868-4bd8-4e8d-8c6b-69bd559ed9db/lessons/3db51895-010e-4922-b322-4817cb76524e/concepts/65ec3b3d-6438-45bd-9930-d748c99258f4#)

![](https://s3.amazonaws.com/video.udacity-data.com/topher/2018/May/5b046b1c_workspace-paste-ex/workspace-paste-ex.png)

The  `.world`  file uses the XML file format to describe all the elements that are being defined with respect to the Gazebo environment. The simple world that you are creating above, has the following elements -

-   `<sdf>`: The base element which encapsulates the entire file structure and content.
-   `<world>`: The world element defines the world description and several properties pertaining to that world. In this example, you are adding a ground plane, a light source, and a camera to your world. Each model or property can have further elements that describe it better. For example, the  `camera`has a  `pose`  element which defines its position and orientation.
-   `<include>`: The include element, along with the  `<uri>`  element, provide a path to a particular model. In Gazebo there are several models that are included by default, and you can include them in creating your environment.

# Launch files

Next, we will create a  `launch`  file. Launch files in ROS allow us to execute more than one node simultaneously, which helps avoid a potentially tedious task of defining and launching several nodes in separate shells or terminals.

```
$ cd ..
$ cd launch
$ nano udacity_world.launch
```

Add the following to your launch file.

```xml
<?xml version="1.0" encoding="UTF-8"?>

<launch>

  <arg name="world" default="empty"/> 
  <arg name="paused" default="false"/>
  <arg name="use_sim_time" default="true"/>
  <arg name="gui" default="true"/>
  <arg name="headless" default="false"/>
  <arg name="debug" default="false"/>

  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find udacity_bot)/worlds/udacity.world"/>
    <arg name="paused" value="$(arg paused)"/>
    <arg name="use_sim_time" value="$(arg use_sim_time)"/>
    <arg name="gui" value="$(arg gui)"/>
    <arg name="headless" value="$(arg headless)"/>
    <arg name="debug" value="$(arg debug)"/>
  </include>

</launch>
```

As in the case of the  `.world`  file, the  `.launch`  files are also based on XML. The structure for the file above, is essentially divided into two parts -

-   First, you define certain arguments using the  `<arg>`  element. Each such element will have a  `name`attribute and a  `default`  value.
-   Then, you include the  `empty_world.launch`  file from the  `gazebo_ros`  package. The  [empty_world](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/kinetic-devel/gazebo_ros/launch/empty_world.launch)  file includes a set of important definitions that are inherited by the world that we create. Using the  `world_name`  argument and the path to your  `.world`  file passed as the  `value`  to that argument, you will be able to launch your world in Gazebo.

## Launch it!

You can now use the launch file to launch your Gazebo environment!

```
$ cd /home/workspace/catkin_ws/
$ catkin_make
$ source devel/setup.bash
$ roslaunch udacity_bot udacity_world.launch
```

[](https://classroom.udacity.com/nanodegrees/nd209/parts/dad7b7cc-9cce-4be4-876e-30935216c8fa/modules/f5048868-4bd8-4e8d-8c6b-69bd559ed9db/lessons/3db51895-010e-4922-b322-4817cb76524e/concepts/65ec3b3d-6438-45bd-9930-d748c99258f4#)
 
![](https://s3.amazonaws.com/video.udacity-data.com/topher/2018/January/5a67c1d3_gazebo-empty-world/gazebo-empty-world.png)

Empty Gazebo world with a sun shining from the top!

It does look a bit bland, but don't worry, there will soon be a different world for your robot to explore!
<!--stackedit_data:
eyJoaXN0b3J5IjpbLTE2Nzc2MTcxMDgsMTU2NDk4NTgwMF19
-->