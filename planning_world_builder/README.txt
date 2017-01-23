
!!!!!!!!!!!!!!! NOTE !!!!!!!!!!!!!!!

Collision checking doe not work properly if a collision object intersects the axis of the global frame!!!


Examples for wrong obstacle placement:
  
				  Y
				  ^
				  |
				  |
				  |
				  |		obstacle
				  |		-------
				  |		|/////|
				  x - - - - - - - - > X
				  Z		|/////|
						-------
						
				  Y
				  ^
				  |
				  |
				  |
				  |obstacle
			   ---|----
			   |//|///|
			   |  x - | - - - - - - > X
			   |//Z///|
			   --------
			   
			   				  Y
				  ^
			   ---|---
			   |//|//| obstacle
			   |//|//|
			   ---|---
				  |
				  |
				  x - - - - - - - - > X
				  Z
				  
				  
				  
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Regarding the available Planning Worlds:
  
If you change one of the worlds (e.g. in "insertTwoRoomsOffice()") you need to apply the changes to "generateStartGoalConfigs.cpp"
and the corresponding planning_scenario node (here "run_two_rooms_office_scenario.cpp)