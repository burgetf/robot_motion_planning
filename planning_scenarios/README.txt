#++++++++++++++++ HowTo: Run a planning scenario +++++++++++

Syntax:
rosrun planning_scenarios run_SCENARIO_planning PLANNER NUM_RUNS FLAG_MAX_ITER_TIME MAX_ITER_TIME_VAL FLAG_RVIZ_SHOW_TREE

SCENARIO = {block / empty_world / cart_narrow_corridor / cart_three_gates / cart_tunnel / cart_two_rooms_office}
PLANNER = {bi_informed_rrt_star, bi_rrt_star,.....}
FLAG_MAX_ITER_TIME = {0 = use max iter, 1 = use max time}
MAX_ITER_TIME_VAL = {max_iter or max_time value}
FLAG_RVIZ_SHOW_TREE = {0 = don't show tree in RViz, 1 = show tree in RViz}

Examples:
rosrun planning_scenarios run_block_planning bi_informed_rrt_star 1 0 400 1 (run planner max. 400 iterations)
rosrun planning_scenarios run_block_planning bi_informed_rrt_star 1 1  60 1 (run planner max. 60 seconds)