**multi_robot_layer**
==================

## Attaching Plugin

* First, copy the multi\_robot\_layer to the *catkin_ws/src*. Then, do *catkin_make*.

*  In order to attach *multi\_robot\_layer* plugin, insert

```
- {name: multi_robot_layer,       type: "costmap_2d::MultiRobotLayer"}
```

to array of plugins of local costmap and global costmap just below the *obstacle_layer*.

## Explanation

* *multi\_robot\_layer* detects all footprints that are published by robots whatever their names are.

* *multi\_robot\_layer* customizes footprints for every robot. For example, own footprint of a robot cannot be an obstacle on own costmap.

* *multi\_robot\_layer* fills polygons that are published by robots, with *LETHAL_OBSTACLE*.

* *multi\_robot\_layer* clears old footprint locations and creates obstacles new footprint areas.
