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
planner_legend_names{1,6} = ['Informed BiRRT'];
planner_legend_names{1,7} = ['BiRRT*'];
planner_legend_names{1,8} = ['Informed BiRRT*'];




%Number of planners
num_planners = size(planner_list,2);

%Number of runs per planner
planners_runs = 100; %100
run_indices = planners_runs-1;

%Time considered after first solution
post_solution_time = 120.0; %in seconds

%Interpolation step width for cost evolution functions
interp_step_width = 0.1;

%Step width for standard deviation evaluation
std_step_width = 100;

%Successful runs counter
successful_run = zeros(1,num_planners);

%Accumulated best solution costs 
cost_evolution_accumulated = zeros(num_planners,(post_solution_time/interp_step_width)+1);

%Average best solution costs 
cost_evolution_average = zeros(num_planners,(post_solution_time/interp_step_width)+1);

%Standard deviation of solution costs
cost_evolution_std = zeros(num_planners,(post_solution_time/interp_step_width)+1);

%Theoretical Best Solution Costs (linear interpolation between start and goal configs)
theoretical_best_solution_cost = 0;

%% +++++++++ Compute accumulated Best Solution Costs for each planner ++++++++++++++++
for i = 1:num_planners

    cost_evolution_stacked = [];
    
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
            
            %Get iteration when first solution has been found
            first_solution_iter = planner_statistics{2}{6} + 1;
            
            %Get solution times after first solution has been found
            time_post_first_solution = cost_evolution{3}(first_solution_iter:end);
            
            %Express best cost solution times relative to the first solution time
            time_post_first_solution_relative = time_post_first_solution - time_post_first_solution(1);
            
            %Find last element within the 60 seconds recording bound
            last_solution_within_bound_index = find(time_post_first_solution_relative <= post_solution_time,1,'last');
            
            %Cost evolution after first solution has been found
            cost_evolution_post_first_solution = cost_evolution{4}(first_solution_iter:end);  
            
            %Time within bounds and their associated best solution costs
            time_within_bounds = time_post_first_solution_relative(1:last_solution_within_bound_index);
            cost_evolution_within_bounds = cost_evolution_post_first_solution(1:last_solution_within_bound_index);
            
            %Fill the array until a total time of "post_solution_time" has been reached
            last_solution_time = time_within_bounds(end);
            last_solution_val = cost_evolution_within_bounds(end);
            while (last_solution_time <= post_solution_time)
                last_solution_time = last_solution_time + 0.5;
                time_within_bounds = [time_within_bounds ; last_solution_time];
                cost_evolution_within_bounds = [cost_evolution_within_bounds ; last_solution_val];
            end
            
            %size(time_within_bounds)
            %size(cost_evolution_within_bounds)
            
            %Interpolate Cost Evolution Function
            % -> such that all cost evolution functions have the same number of values
            time_within_bounds_interp = 0 : interp_step_width : round(time_within_bounds(end));
            cost_evolution_within_bounds_interp = interp1(time_within_bounds,cost_evolution_within_bounds,time_within_bounds_interp);
            
            %size(time_within_bounds_interp)
            %size(cost_evolution_within_bounds_interp)
            %plot(time_within_bounds_interp,cost_evolution_within_bounds_interp,'o');
            
            %Sum up cost evolution values
            cost_evolution_accumulated(i,:) =  cost_evolution_accumulated(i,:) + cost_evolution_within_bounds_interp;
            
            %Stack the cost evolutions of all runs (later used to compute standard deviation)
            cost_evolution_stacked = [cost_evolution_stacked ; cost_evolution_within_bounds_interp];
           
            successful_run(i) = successful_run(i) + 1;
        else
            disp('This planning run failed to find a path, continuing with next planner run statistics')
        end %if planner run successful
    end %run iterations
    
    %Average best solution costs 
    cost_evolution_average(i,:) = cost_evolution_accumulated(i,:)./successful_run(i);
    
    %Compute standard deviation for cost evolutions of all planner runs
    std_dev_every_point = std(cost_evolution_stacked);
    %Use only every std_step_width-th stanard deviation for the plot
    indices = 1 : std_step_width : length(cost_evolution_stacked);
    cost_evolution_std(i,indices) = std_dev_every_point(indices);
   
    
end %planner type iterations


%% +++++++++ Average Cost Evolution Trajectries for each planner ++++++++++++++++
%Average best solution costs 
%cost_evolution_average = zeros(num_planners,(post_solution_time/interp_step_width)+1);
%for i = 1:num_planners
%    cost_evolution_average(i,:) = cost_evolution_accumulated(i,:)./successful_run(i);
%end


%% +++++++++ Plot Average Best Solution Cost Evolution for each planner ++++++++++++++++
%Interpolated time
time_interpolated = 0 : interp_step_width : post_solution_time;

%Collect some line styles for the plot
line_style = {'--k' ; '--g' ; '--b' ; '--r' ; '-k' ; '-g' ; '-b' ; '-r'};

%Set line width
line_width = 2;

%Create Figure Handle
h = figure('Name','Average Best Solution Path Cost','NumberTitle','off');
%get(gcf,'PaperPosition')
%set(gcf, 'Position', [680   678   560   700])

%Plot theoretical best solution cost thresholds
theoretical_best_solution_cost_vector = repmat(theoretical_best_solution_cost,1,length(time_interpolated));

%Plot results
hdl = plot(time_interpolated,cost_evolution_average(1,:),line_style{1}, ...
           time_interpolated,cost_evolution_average(2,:),line_style{2}, ...
           time_interpolated,cost_evolution_average(3,:),line_style{3}, ...
           time_interpolated,cost_evolution_average(4,:),line_style{4}, ...
           time_interpolated,cost_evolution_average(5,:),line_style{5}, ...
           time_interpolated,cost_evolution_average(6,:),line_style{6}, ...
           time_interpolated,cost_evolution_average(7,:),line_style{7}, ...
           time_interpolated,cost_evolution_average(8,:),line_style{8}, ...
           'LineWidth',line_width);
  
%time_interpolated,theoretical_best_solution_cost_vector(1,:),':m', ...
       
%errorbar(time_interpolated,cost_evolution_average(i,:),cost_evolution_std(i,:),line_col(i),'LineWidth',line_width);


%Set some plot properties
%l = legend(planner_legend_names{1}, planner_legend_names{2}, planner_legend_names{3},planner_legend_names{4},...
%    planner_legend_names{5}, planner_legend_names{6},planner_legend_names{7},planner_legend_names{8},...
%    'Orientation','horizontal','Location','southoutside');
%set(l, 'Interpreter', 'none')
%hL = gridLegend(hdl,4,planner_legend_names,'Orientation','Horizontal','Fontsize',12); %,'Box','off'

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
set(gcf, 'PaperPosition', [0 0 800/100 500/100]);

%xlim([lower_time_limit upper_time_limit]);
%ylim([lower_cost_limit upper_cost_limit]);
xlabel('Post Solution CPU Time [s]','FontSize',17);
ylabel('Avg. Solution Cost (Total)','FontSize',17);
title('Post Solution CPU Time vs. Avg. Solution Cost','FontSize',17);

tick_size_x_axes = get(gca,'XTickLabel');
set(gca,'XTickLabel',tick_size_x_axes,'fontsize',15)
tick_size_y_axes = get(gca,'YTickLabel');
set(gca,'YTickLabel',tick_size_y_axes,'fontsize',15)


file_path = ['../plots/' scene_name '_avg_cost_evolution_planner_comparison'];
file_path2 = ['../figures/' scene_name '_avg_cost_evolution_planner_comparison.fig'];
saveas(gcf,file_path,'epsc');
saveas(gcf,file_path2);

