%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Exercise 1 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Robot: Any

Package: ics_gazebo

$ roslaunch ics_gazebo tiago.launch world_suffix:=tutorial1

$ roslaunch ics_gazebo hsrb.launch world_suffix:=tutorial1


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Exercise 2 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Robot: Any

Package: ics_gazebo

$ roslaunch ics_gazebo tiago.launch world_suffix:=tutorial1

$ roslaunch ics_gazebo hsrb.launch world_suffix:=tutorial1


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Exercise 3 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

#############

Robot: Tiago

Package: ics_gazebo, turtle_vis_tiago

$ roslaunch ics_gazebo tiago.launch

$ roslaunch turtle_vis_tiago tiago_base.launch

$ rosrun turtle_vis_tiago turtle_set_position_tiago_node

$ 2,2,0

#############

Robot: HSRB

Package: ics_gazebo, turtle_vis_hsrb

$ roslaunch ics_gazebo hsrb.launch

$ roslaunch turtle_vis_hsrb hsrb_base.launch

$ rosrun turtle_vis_hsrb turtle_set_position_hsrb_node

$ 2,2,0.5


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Exercise 4 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Robot: Tiago

Package: ics_gazebo, controllers_tutorials

$ roslaunch ics_gazebo tiago.launch

$ rosrun controller_manager controller_manager kill head_controller

$ roslaunch controllers_tutorials new_head_controller.launch

###############

Robot: HSRB

Package: ics_gazebo, controllers_tutorials

$ roslaunch ics_gazebo hsrb.launch

$ export ROS_NAMESPACE=/hsrb

$ rosrun controller_manager controller_manager kill head_controller

$ roslaunch controllers_tutorials new_head_controller.launch

