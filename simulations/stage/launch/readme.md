These are the launch files for the stage simulation

* minibots_4_stage.launch
	* launches a Stage simulation of the field with four small robots
	* each small robot will also have a minibot_navigator node
	* includes the magic_odomFromStage_4 node
	* For minibot testing with 4 minibots

* minibots_6_stage.launch
	* launches a Stage simulation of the field with six small robots
	* each small robot will also have a minibot_navigator node
	* includes the magic_odomFromStage_6 node
	* For minibot testing with 6 minibots

* field_empty.launch
	* launches a Stage simulation of the field with no robots and no obstacles

* field_obstacles.launch
	* launches a Stage simulation of the field with no robots and few obstacles

* field_robots_start.launch
	* launches a Stage simulation of the field with robots and no obstacles. Robots are in starting configuration(clustered together)
	* If cannot launch, use "chmod +x" command on magic_odom.py
	* control by rostopic

* field_robots_deployed.launch
	* launches a Stage simulation of the field with robots and no obstacles. Robots are in deployed configuration(sensor tower and digger position)
	* If cannot launch, use "chmod +x" command on magic_odom.py
	* control by rostopic

* field_robots_start_obstacles.launch
	* launches a Stage simulation of the field with robots and obstacles. Robots are in starting configuration(clustered together)
	* If cannot launch, use "chmod +x" command on magic_odom.py
	* control by rostopic

* field_robots_deployed_obstacles.launch
	* launches a Stage simulation of the field with robots and obstacles. Robots are in deployed configuration(sensor tower and digger position)
	* If cannot launch, use "chmod +x" command on magic_odom.py
	* control by rostopic
