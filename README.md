# mobot_controller
This is an updated version from the previous midterm package (which can be found here: https://github.com/ngquan0304/ecse476_midterm/tree/midterm_final)

## Code Usage

Below are the command snippets to run the code in the simulation with mobot in side a pen navigating using map and odometry.

1. Under the directory where the simulation and urdf reside:
Start up gazebo and spawn mobot:

`roslaunch mobot_urdf mobot_in_pen.lauch`

Load the map of the starting pen:

`roscd maps/starting_pen`

`rosrun map_server map_server starting_pen_map.yaml`

2. Under the directory where the mobot_controller resides:

Using Odom from gazebo.

Run AMCL to localize the robot in the map.  AMCL updates are only about 1Hz, and thus
unsuitable for steering feedback.

`rosrun amcl amcl`

Start mobot_controller's nodes:

`rosrun mobot_controller current_state_publisher`

`rosrun mobot_controller des_state_publisher_service`

`rosrun mobot_controller linear_steering_wrt_amcl_and_odom`

## Running tests/demos

Run rviz:

`rviz`

In rviz, add a `map` that reads from topic `/map` and add an `axis` that reads the tf of `base_link`

Initiate the path planner:

`rosrun mobot_controller navigation_coordinator`  

This path planner was currently set to perform the below trajectory:
1. Predocking table 1 (reorienting vertex)
2. Docking table 1
3. Back up (0.5 m)
4. Predocking table 2 (reorienting vertex)
5. Docking table 2
6. Back up (0.5 m)                                    (haven't done yet)
7. Moving back and reorienting back to initial pose   (haven't done yet)
  
