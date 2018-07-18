# Unified Robot Description Format (URDF)

Unified Robot Description Format or urdf, is an XML format used in ROS for representing a robot model. We can use a urdf file to define a robot model, its kinodynamic properties, visual elements and even model sensors for the robot. URDF can only describe a robot with rigid links connected by joints in a chain or tree-like structure. It cannot describe a robot with flexible links or parallel linkage.

A simple robot with two links and a joint can be described using urdf as follows:
```xml
<?xml version="1.0"?>
<robot name="two_link_robot">
  <!--Links-->
  <link name="link_1">
    <visual>
      <geometry>
        <cylinder length="0.5" radius="0.2"/>
      </geometry>
    </visual>
  </link>
  <link name="link_2">
    <visual>
      <geometry>
        <box size="0.6 0.1 0.2"/>
      </geometry>
    </visual>
  </link>
  <!--Joints-->
  <joint name="joint_1" type="continuous">
    <parent link="link_1"/>
    <child link="link_2"/>
  </joint>
</robot>
```

Since we use urdf files to describe several robot and environmental properties, they tend to get long and tedious to read through. This is why we use Xacro (XML Macros) to divide our single urdf file into multiple xacro files. While the syntax remains the same, we can now divide our robot description into smaller subsystems. For this project, we have simply divided our urdf into two xacro files:

-   `kr210.urdf.xacro`  - this file contains all the robot specific information like links, joints, actuators, etc.
-   `kr210.gazebo.xacro`  - this file contains gazebo specific information like robot material, frictional constants, and plugins to control the robot in gazebo

As  `kr210.urdf.xacro`  contains robot specific information like link lengths and joint offsets, it is the only file you need to derive DH parameters and create transform matrices. Since urdf (and xacro) files are basically XML, they use tags to define robot geometry and properties. The most important and commonly used tags with their elements are described below:

## `<robot>`  `</robot>`

This is a top level tag that contains all the other tags related to a given robot.

## `<link>`  `</link>`

Each rigid link in a robot must have this tag associated with it.

### Attributes

**name**: Requires a unique link name attribute.

### Elements

### `<visual>`  `</visual>`
This element specifies the appearance of the object for visualization purposes.

|Name|Description|
|--|--|
|`origin`|The reference frame of the visual element with respect to the reference frame of the link.|
|`geometry`|The shape of the visual object.|
|`material`|The material of the visual element.|


### `<collision>`  `</collision>`
The collision properties of a link. Note that this can be different from the visual properties of a link, for example, simpler collision models are often used to reduce computation time.
|Name|Description|
|--|--|
|`origin`|  The reference frame of the collision element, relative to the reference frame of the link.|
|`geometry`| See the geometry description in the above visual element. |



### `<inertial>`  `</inertial>`
|Name|Description|
|--|--|
|`origin`|  This is the pose of the inertial reference frame, relative to the link reference frame. The origin of the inertial reference frame needs to be at the center of gravity.|
|`mass`| The mass of the link is represented by the value attribute of this element. |
|`inertia`| The 3x3 rotational inertia matrix, represented in the inertia frame. Because the rotational inertia matrix is symmetric, only 6 above-diagonal elements of this matrix are specified here, using the attributes ixx, ixy, ixz, iyy, iyz, izz. |


Example snippet for  **`<link>`** tag with important elements:

```xml
  <link name="link_1">
    <inertial>
      <origin xyz="0 0 0.4" rpy="0 0 0"/>
      <mass value="${mass1}"/>
      <inertia ixx="30" ixy="0" ixz="0" iyy="50" iyz="0" izz="50"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://kuka_arm/meshes/kr210l150/visual/link_1.dae"/>
      </geometry>
      <material name="">
        <color rgba="0.75294 0.75294 0.75294 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://kuka_arm/meshes/kr210l150/collision/link_1.stl"/>
      </geometry>
    </collision>
  </link>

```

The  **`<link>`** tag has many more optional elements that can be used to define other properties like color, material, texture, etc. Refer  [this link](http://wiki.ros.org/urdf/XML/link)  for details on those tags.

## `<joint>`  `</joint>`

This tag typically defines a single joint between two links in a robot. The type of joints you can define using this tag include:


|Name|Description|
|--|--|
|Fixed|Rigid joint with no degrees of freedom. Used to  **weld**  links together.|
|Revolute|A range-limited joint that rotates about an axis.|
|Prismatic|A range-limited joint that slides along an axis|
|Continuous|
Similar to  **Revolute**  joint but has no limits. It can rotate continuously about an axis.|
|Planar|A 2D  **Prismatic**  joint that allows motion in a plane perpendicular to an axis.|
|Floating|A joint with 6 degrees of freedom, generally used for Quadrotors and UAVs|

### Attributes
**name**:  Unique joint name
**type**:  Type of joint

### Elements
To define a joint, we need to declare the axis of rotation/translation and the relationship between the two links that form the joint.


|Name|Description|
|--|--|
|`<origin>`|This is the transform from the parent link to the child link. The joint is located at the origin of the child link.|
|`<parent>`|Name of the Parent link for the respective joint.|
|`<child>`|Name of the child link for the respective joint.|
|`<axis>`|Defines the axis of rotation for revolute joints, the axis of translation for prismatic joints, and the surface normal for planar joints. Fixed and floating joints do not use the axis field.|

Example snippet for  **`<joint>`** tag with important elements:

```xml
<joint name="joint_2" type="revolute">
  <origin xyz="0.35 0 0.42" rpy="0 0 0"/>
  <parent link="link_1"/>
  <child link="link_2"/>
  <axis xyz="0 1 0"/>
</joint>

```

Other optional elements under the  **`<joint>`** tag can be  [found here](http://wiki.ros.org/urdf/XML/joint).

There are many more optional tags and attributes that help to define various dynamic and kinematic properties of a robot along with sensors and actuators. Those will be covered in future lessons, for now you can refer  [this link](http://wiki.ros.org/urdf)  for more details on those.
<!--stackedit_data:
eyJoaXN0b3J5IjpbLTE2NTQyMzg1MThdfQ==
-->