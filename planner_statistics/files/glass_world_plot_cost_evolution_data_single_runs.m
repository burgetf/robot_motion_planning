% +++++Plotting the Cost Evolution of the Best Solution Path ++++++

%Set Scene name
scene_name = 'glass';

%% Load Cost Evolution Files

%Load Cost Evolution Data
file_path = strcat('../data/bi_informed_rrt_star/',scene_name,'_scene_cost_evolution_run_0.txt');
fileID = fopen(file_path);
H = textscan(fileID,'%s %s %s %s %s %s %s %s %s',1);
bi_informed_rrt_star_cost = textscan(fileID,'%s %u %f %f %f %f %f %f %f');
fclose(fileID);

%Load Cost Evolution Data
file_path = strcat('../data/uni_informed_rrt_star/',scene_name,'_scene_cost_evolution_run_0.txt');
fileID = fopen(file_path);
H = textscan(fileID,'%s %s %s %s %s %s %s %s %s',1);
uni_informed_rrt_star_cost = textscan(fileID,'%s %u %f %f %f %f %f %f %f');
fclose(fileID);

%Load Cost Evolution Data
file_path = strcat('../data/bi_rrt_star/',scene_name,'_scene_cost_evolution_run_0.txt');
fileID = fopen(file_path);
H = textscan(fileID,'%s %s %s %s %s %s %s %s %s',1);
bi_rrt_star_cost = textscan(fileID,'%s %u %f %f %f %f %f %f %f');
fclose(fileID);

%Load Cost Evolution Data
file_path = strcat('../data/uni_rrt_star/',scene_name,'_scene_cost_evolution_run_0.txt');
fileID = fopen(file_path);
H = textscan(fileID,'%s %s %s %s %s %s %s %s %s',1);
uni_rrt_star_cost = textscan(fileID,'%s %u %f %f %f %f %f %f %f');
fclose(fileID);


%planner_name iteration planning_time_elapsed cost_solution_total cost_solution_revolute cost_solution_prismatic theoretical_cost_total theoretical_cost_revolute theoretical_cost_prismatic


%% Get Total Cost Evolution
bi_informed_rrt_star_cost_total = bi_informed_rrt_star_cost{4};
uni_informed_rrt_star_cost_total = uni_informed_rrt_star_cost{4};
bi_rrt_star_cost_total = bi_rrt_star_cost{4};
uni_rrt_star_cost_total = uni_rrt_star_cost{4};

% Get Time Elapsed Data
bi_informed_rrt_star_time_elapsed = bi_informed_rrt_star_cost{3};
uni_informed_rrt_star_time_elapsed = uni_informed_rrt_star_cost{3};
bi_rrt_star_time_elapsed = bi_rrt_star_cost{3};
uni_rrt_star_time_elapsed = uni_rrt_star_cost{3};

%Find the highest first solution cost among the cost evolution files
first_solution_index1 = find(bi_informed_rrt_star_cost_total<10000,1);
first_solution_value1 = bi_informed_rrt_star_cost_total(first_solution_index1);
first_solution_time1 = bi_informed_rrt_star_time_elapsed(first_solution_index1);
final_solution_index1 = find(bi_informed_rrt_star_cost_total==min(bi_informed_rrt_star_cost_total),1);
final_solution_time1 = bi_informed_rrt_star_time_elapsed(final_solution_index1);

first_solution_index2 = find(uni_informed_rrt_star_cost_total<10000,1);
first_solution_value2 = uni_informed_rrt_star_cost_total(first_solution_index2);
first_solution_time2 = uni_informed_rrt_star_time_elapsed(first_solution_index2);
final_solution_index2 = find(uni_informed_rrt_star_cost_total==min(uni_informed_rrt_star_cost_total),1);
final_solution_time2 = uni_informed_rrt_star_time_elapsed(final_solution_index2);

first_solution_index3 = find(bi_rrt_star_cost_total<10000,1);
first_solution_value3 = bi_rrt_star_cost_total(first_solution_index3);
first_solution_time3 = bi_rrt_star_time_elapsed(first_solution_index3);
final_solution_index3 = find(bi_rrt_star_cost_total==min(bi_rrt_star_cost_total),1);
final_solution_time3 = bi_rrt_star_time_elapsed(final_solution_index3);

first_solution_index4 = find(uni_rrt_star_cost_total<10000,1);
first_solution_value4 = uni_rrt_star_cost_total(first_solution_index4);
first_solution_time4 = uni_rrt_star_time_elapsed(first_solution_index4);
final_solution_index4 = find(uni_rrt_star_cost_total==min(uni_rrt_star_cost_total),1);
final_solution_time4 = uni_rrt_star_time_elapsed(final_solution_index4);

%Cost of first solutions
first_solution_value = [first_solution_value1 ; first_solution_value2 ; first_solution_value3 ; first_solution_value4];
%Find the lowest final solution cost among the cost evolution files
final_solution_value = [min(bi_informed_rrt_star_cost_total) ; min(uni_informed_rrt_star_cost_total) ; min(bi_rrt_star_cost_total) ; min(uni_rrt_star_cost_total)];
%Time of first solutions
first_solution_time = [first_solution_time1 ; first_solution_time2 ; first_solution_time3 ; first_solution_time4];
%Time of last solutions
final_solution_time = [final_solution_time1 ; final_solution_time2 ; final_solution_time3 ; final_solution_time4];


%Use highest first solution cost to set the upper Y-Limit in the Plot
upper_cost_limit = max(first_solution_value) + 2;
%Use lowest final solution cost to set the lower Y-Limit in the Plot
lower_cost_limit = min(final_solution_value) - 2;

%Use lowest first solution time to set the lower X-Limit in the Plot
lower_time_limit = min(first_solution_time) - 5.0;
if lower_time_limit < 0.0
    lower_time_limit = 0.0;
end
%Use highest final solution time to set the upper X-Limit in the Plot
upper_time_limit = max(final_solution_time) + 5.0;


%% Plot results
num_vals = size(bi_informed_rrt_star_cost_total,1);
%Time Stamps Array
%time_elapsed = 0:1:num_poses;

%Set line width
line_width = 2;

%Set window title
win_title = strcat('Cost Evolution: ',scene_name, ' scenario');

%Plots
h = figure('Name',win_title,'NumberTitle','off');
plot(uni_rrt_star_time_elapsed, uni_rrt_star_cost_total,'k', ...
    bi_rrt_star_time_elapsed, bi_rrt_star_cost_total,'g', ...
    bi_informed_rrt_star_time_elapsed,bi_informed_rrt_star_cost_total,'b', ...
    uni_informed_rrt_star_time_elapsed,uni_informed_rrt_star_cost_total,'r','LineWidth',line_width);

legend('RRT*', 'BiRRT*', 'BiInformedRRT*','InformedRRT*')
%xlim([lower_time_limit upper_time_limit]);
%ylim([lower_cost_limit upper_cost_limit]);
xlabel('Computation Time [s]');
ylabel('Solution Cost (Total)');
title('Solution Cost vs. Computation Time');


