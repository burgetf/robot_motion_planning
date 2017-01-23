%% +++++Plotting the Average Cost Evolution of the Best Solution Path ++++++

%Set Scene name
scene_name = 'block';

%%+++++++++++++++++++++++++++ Setup ++++++++++++++++++++++++++++++++
%List of planner
planner_list = cell(1,8);
planner_list{1,1} = ['uni_rrt'];
planner_list{1,2} = ['uni_informed_rrt'];
planner_list{1,3} = ['uni_rrt_star'];
planner_list{1,4} = ['uni_informed_rrt_star'];

planner_list{1,5} = ['bi_rrt_connect'];
planner_list{1,6} = ['bi_informed_rrt'];
planner_list{1,7} = ['bi_rrt_star'];
planner_list{1,8} = ['bi_informed_rrt_star'];

%Planner names for plot legend
planner_legend_names = cell(1,8);
planner_legend_names{1,1} = ['RRT'];
planner_legend_names{1,2} = ['Informed RRT'];
planner_legend_names{1,3} = ['RRT*'];
planner_legend_names{1,4} = ['Informed RRT*'];

%planner_legend_names{1,5} = ['Hypothetical Optimal Cost'];

planner_legend_names{1,5} = ['RRT-Connect'];
planner_legend_names{1,6} = ['Bidirectional Informed RRT'];
planner_legend_names{1,7} = ['Bidirectional RRT*'];
planner_legend_names{1,8} = ['Bidirectional Informed RRT* (ours)'];

%Number of planners
num_planners = size(planner_list,2);

%Number of runs per planner
planners_runs = 100;
run_indices = planners_runs-1;


%Desired Solution Path Costs
cost_step_width = -1;
desired_solution_path_costs = 70 : cost_step_width : 0;

%Time Punishment for unreached costs
time_punishment_unreached_cost = 120;

%Accumulated Time required to reach desired solution costs
time_required_accumulated = zeros(num_planners,size(desired_solution_path_costs,2));

%Number of planner runs that reached a certain solution path cost
%num_solution_cost_reached =  zeros(num_planners,size(desired_solution_path_costs,2));

%Theoretical Best Solution Costs (linear interpolation between start and goal configs)
theoretical_best_solution_cost = 0;


%% +++++++++ Compute accumulated Best Solution Costs for each planner ++++++++++++++++
for i = 1:num_planners

    %Iterate through runs
    for j = 0:run_indices
        
        %Load Planner Statistics Data
        file_path = strcat('../data/', planner_list{i}, '/', scene_name,  '_scene_planner_statistics_run_',  int2str(j), '.txt');
        fileID = fopen(file_path);
        planner_statistics = textscan(fileID,'%s %s');
        fclose(fileID); 
        %Convert numbers from string to double
        for e = 3:size(planner_statistics{2},1)
            planner_statistics{2}{e} = str2double(planner_statistics{2}{e});
        end
        
        %Load Cost Evolution Data
        file_path = strcat('../data/', planner_list{i}, '/', scene_name,  '_scene_cost_evolution_run_',  int2str(j), '.txt');
        fileID = fopen(file_path);
        H = textscan(fileID,'%s %s %s %s %s %s %s %s %s',1);
        cost_evolution = textscan(fileID,'%s %u %f %f %f %f %f %f %f');
        fclose(fileID);

        %Get theoretical best solution cost
        theoretical_best_solution_cost = planner_statistics{2}{11};
        
        %If planner found a solution to the planning problem in run j
        if planner_statistics{2}{3} == 1        
            
            %Evolution of total cost
            total_cost_evolution = cost_evolution{4};
            
            %Time Evolution of total cost
            total_cost_time = cost_evolution{3};
            
            
            %Find Indices of element in the total cost vector where the total cost becomes lower than the desired cost 
            indices_cost_reached = [];
            for d = 1 : 1 : length(desired_solution_path_costs) 
                indices_cost_reached = [indices_cost_reached find(total_cost_evolution<=desired_solution_path_costs(d),1)];
            end
           
            %Collect times required to find specific costs
            for k = 1 : 1 : length(indices_cost_reached)
                time_required_accumulated(i,k) = time_required_accumulated(i,k) + total_cost_time(indices_cost_reached(k));
                %num_solution_cost_reached(i,k) = num_solution_cost_reached(i,k) + 1;
            end
            
            %Time Punishment of 120 seconds for unreached costs in planner runs
            time_required_accumulated(i,length(indices_cost_reached)+1:end) = time_required_accumulated(i,length(indices_cost_reached)+1:end) + time_punishment_unreached_cost;
            
        else
            disp('This planning run failed to find a path, continuing with next planner run statistics')
            
            %Time Punishment of 120 seconds for all costs when planning failed
            time_required_accumulated(i,:) = time_required_accumulated(i,:) + time_punishment_unreached_cost;
            
        end %if planner run successful
        
    end %run iterations
end %planner type iterations


%% +++++++++ Average Time required to reach the desired costs ++++++++++++++++
%Average Time required to reach desired solution costs
avg_time_required = zeros(num_planners,size(desired_solution_path_costs,2));

%Compute average time required to find solution of certain costs 
for i = 1:num_planners
    %avg_time_required(i,:) = time_required_accumulated(i,:)./num_solution_cost_reached(i,:);
    avg_time_required(i,:) = time_required_accumulated(i,:)./planners_runs;
end

%Tuncate trajectories when no solution for desired solution cost is available
avg_time_required_truncated = cell(num_planners,1);
%Used for x-axis in plots
desired_solution_costs_reached_array = cell(num_planners,1);
%Used for x-axis limit in plot
overall_lowest_cost_reached = 1000;
for i = 1:num_planners
    index_last_cost_reached = find(avg_time_required(i,:)==time_punishment_unreached_cost,1) ;
    avg_time_required_truncated{i} = avg_time_required(i,1:index_last_cost_reached);
    desired_solution_costs_reached_array{i} = desired_solution_path_costs(1) : cost_step_width : desired_solution_path_costs(index_last_cost_reached);
    
    if(overall_lowest_cost_reached > desired_solution_costs_reached_array{i}(end))
        overall_lowest_cost_reached = desired_solution_costs_reached_array{i}(end);
    end
end



%% +++++++++ Plot Average Best Solution Cost Evolution for each planner ++++++++++++++++

%Collect some line styles for the plot
line_style = {'--k' ; '--g' ; '--b' ; '--r' ; '-k' ; '-g' ; '-b' ; '-r'};

%Set line width
line_width = 2;

%Create Figure Handle
h = figure('Name','Success Rate vs. Desired Final Solution Cost','NumberTitle','off');

%Plot theoretical best solution cost thresholds
x=[theoretical_best_solution_cost,theoretical_best_solution_cost];
y=[0,time_punishment_unreached_cost];

%Plot results
hdl = plot(desired_solution_costs_reached_array{1},avg_time_required_truncated{1},line_style{1}, ...
           desired_solution_costs_reached_array{2},avg_time_required_truncated{2},line_style{2}, ...
           desired_solution_costs_reached_array{3},avg_time_required_truncated{3},line_style{3}, ...
           desired_solution_costs_reached_array{4},avg_time_required_truncated{4},line_style{4}, ...
           desired_solution_costs_reached_array{5},avg_time_required_truncated{5},line_style{5}, ...
           desired_solution_costs_reached_array{6},avg_time_required_truncated{6},line_style{6}, ...
           desired_solution_costs_reached_array{7},avg_time_required_truncated{7},line_style{7}, ...
           desired_solution_costs_reached_array{8},avg_time_required_truncated{8},line_style{8}, ...
           'LineWidth',line_width);
       
%Plot results
%for i = 1:num_planners
%    plot(desired_solution_costs_reached_array{i},avg_time_required_truncated{i},line_style{i},'LineWidth',line_width);
%    hold on;
%end


%plot(x,y,':m','LineWidth',line_width);

%Set some plot properties
%l = legend(planner_legend_names{1}, planner_legend_names{2}, planner_legend_names{3},planner_legend_names{4},planner_legend_names{5},planner_legend_names{6},planner_legend_names{7},planner_legend_names{8},'Hypothetical Optimal Cost','Location','northwest');
%set(l, 'Interpreter', 'none')
%hL = gridLegend(hdl,4,planner_legend_names,'Orientation','Horizontal','Fontsize',12);

% Get current position (which I used to keep overall size the same)
%currentLegendPosition = hL.Position;
% Define new position
%newLegendPosition = [currentLegendPosition(1) -0.01 currentLegendPosition([3 4])];
% Set new position
%hL.Position = newLegendPosition;


%set(gcf, 'PaperUnits', 'inches');
%set(gcf, 'PaperSize', [8.5 6.5]);
%set(gcf, 'PaperPositionMode', 'manual');
%set(gcf, 'PaperPosition', [0 0 8.5 6.5]);

%Set Plot size
set(gcf, 'Position', [100 10 800 400]);
%Set output size for EPS
set(gcf, 'PaperPosition', [0 0 800/100 400/100]);

set(gca,'XDir','reverse');
xlim([theoretical_best_solution_cost-2 desired_solution_path_costs(1)]);
ylim([0 time_punishment_unreached_cost+2]);
%xlabel('Desired Solution Cost','FontSize',17);
%ylabel('Avg. Solution Time [s]','FontSize',17);
xlabel('Desired Solution Cost $c_{SP}$','Interpreter','LaTex','FontSize',17)
ylabel('Avg. Solution Time [s]','Interpreter','LaTex','FontSize',17)
title('Desired Solution Cost vs. Avg. Solution Time','Interpreter','LaTex','FontSize',17)
%title('Desired Solution Cost vs. Avg. Solution Time','FontSize',17);

tick_size_x_axes = get(gca,'XTickLabel');
set(gca,'XTickLabel',tick_size_x_axes,'fontsize',15)
tick_size_y_axes = get(gca,'YTickLabel');
set(gca,'YTickLabel',tick_size_y_axes,'fontsize',15)

%Modify Y Axis Labelss
ax = gca;
ax.YTickLabel = {'0','20','40','60','80','100','N/A'};

%Save Plot as in PDF Format
file_path = ['../plots/' scene_name '_desired_solution_cost_vs_time_required'];
file_path2 = ['../figures/' scene_name '_desired_solution_cost_vs_time_required.fig'];
saveas(gcf,file_path,'epsc');
saveas(gcf,file_path2);

