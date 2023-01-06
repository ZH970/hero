# ***pluginlib***



**讲述插件实现流程**

Plugin: **effort_controllers_plugins** in controller_interface


## Base class
base class: 

```c++
#include <control_msgs/JointControllerState.h>
#include <control_toolbox/pid.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <memory>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <std_msgs/Float64.h>
```



## Create the namespace

Plugin joint_velocity_controller.h:

The head file of the plugin  (namespce)



## Resign this plugin

Cpp:

Include the base and head then:

<u>**function implementation & export**</u>

```c++
PLUGINLIB_EXPORT_CLASS( effort_controllers::JointVelocityController, controller_interface::ControllerBase)
```



CMakeLists.txt:

## <u>**Declaere a C++ library**</u>

```
include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/joint_effort_controller.cpp
  src/joint_group_effort_controller.cpp
  src/joint_group_position_controller.cpp
  src/joint_position_controller.cpp
  src/joint_velocity_controller.cpp
)
```



## Add plugin to ROS

effort_controllers_plugins.xml:

The description of this plugin.



package.xml:

```
<export>
  <controller_interface plugin="${prefix}/effort_controllers_plugins.xml"/>
</export>
```



Finally, catkin build 

## check:

`rospack plugins --attrib=plugin <plugin_name>`

or:

check the catkin_ws/devel/lib

check for **lib<plugin_name>.so** 