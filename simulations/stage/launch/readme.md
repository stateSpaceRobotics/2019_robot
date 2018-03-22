These are the launch files for the stage simulation

To use a simulation, include the desired simulation launch file in your main robot control launch file.

Example:
    <include file="$(find ssr_stage)/launch/field_empty.launch"/>

* field\_empty.launch
	* launches a Stage simulation of the field with no robots and no obstacles

* field\_obstacles.launch
	* launches a Stage simulation of the field with no robots and few obstacles

* field\_robots\_start.launch
	* launches a Stage simulation of the field with robots and no obstacles. Robots are in starting configuration(clustered together)
	* control by rostopics

* field\_robots\_deployed.launch
	* launches a Stage simulation of the field with robots and no obstacles. Robots are in deployed configuration(sensor tower and digger position)
	* control by rostopics

* field\_robots\_start\_obstacles.launch
	* launches a Stage simulation of the field with robots and obstacles. Robots are in starting configuration(clustered together)
	* control by rostopics

* field\_robots\_deployed\_obstacles.launch
	* launches a Stage simulation of the field with robots and obstacles. Robots are in deployed configuration(sensor tower and digger position)
	* control by rostopics

* single\_bot\_empty.launch
	* launches a Stage simulation of the field with only one robot and no obstacles. For pathfinding test
	* control by rostopics

* single\_bot\_obstacles.launch
	* lanuches a Stage simulation of the field with only one robot and obstacles. For pathfinding test
	* control by rostopics

* multi\_bot\_empty.launch
	* Exactly like field\_robots\_start.launch, but for pathfinding testing

* multi\_bot\_obstacles.launch
	* Exactly like field\_robots\_start\_obstacles.launch, but for pathfinding testing

