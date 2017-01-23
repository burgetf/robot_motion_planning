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
planners_runs = 100;
run_indices = planners_runs-1;

%Desired Solution Path Costs
desired_solution_path_costs = 70 : -1 : 0;

%Success rates for planners and desired solution costs
success_rates = zeros(num_planners,size(desired_solution_path_costs,2));

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

        %Get theoretical best solution cost
        theoretical_best_solution_cost = planner_statistics{2}{11};
        
        %If planner found a solution to the planning problem in run j
        if planner_statistics{2}{3} == 1
            
            %Get final solution cost
            final_solution_cost = planner_statistics{2}{14};         
            
            %Find Index of element in the desired cost vector where the final_solution_cost is larger than the desired cost 
            index_smallest_reached_cost = find(final_solution_cost>=desired_solution_path_costs,1);
            index_smallest_reached_cost = index_smallest_reached_cost - 1;
            
            %Increment success rate for all desired costs larger than the actually reached cost
            success_rates(i,1:index_smallest_reached_cost) = success_rates(i,1:index_smallest_reached_cost) +1;
            
        else
            disp('This planning run failed to find a path, continuing with next planner run statistics')
        end %if planner run successful
        
    end %run iterations
end %planner type iterations


%% +++++++++ Desired Costs reached in [%] ++++++++++++++++
%Sucess rates in percent
success_rates_percent = zeros(num_planners,size(desired_solution_path_costs,2));
%Lowest reached cost of a planner (used for x-axis limit in plot)
overall_lowest_cost_reached = 1000;
for i = 1:num_planners
    success_rates_percent(i,:) = success_rates(i,:)./planners_runs * 100;
    
    index_lowest_cost_reached = find(success_rates_percent(i,:)==0,1) -1;
    if(desired_solution_path_costs(index_lowest_cost_reached) < overall_lowest_cost_reached)       
        overall_lowest_cost_reached = desired_solution_path_costs(index_lowest_cost_reached);
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
y=[0,100];

%Plot results
hdl = plot(desired_solution_path_costs,success_rates_percent(1,:),line_style{1}, ...
           desired_solution_path_costs,success_rates_percent(2,:),line_style{2}, ...
           desired_solution_path_costs,success_rates_percent(3,:),line_style{3}, ...
           desired_solution_path_costs,success_rates_percent(4,:),line_style{4}, ...
           desired_solution_path_costs,success_rates_percent(5,:),line_style{5}, ...
           desired_solution_path_costs,success_rates_percent(6,:),line_style{6}, ...
           desired_solution_path_costs,success_rates_percent(7,:),line_style{7}, ...
           desired_solution_path_costs,success_rates_percent(8,:),line_style{8}, ...
           'LineWidth',line_width);
       


%Set some plot properties
%l = legend(planner_legend_names{1}, planner_legend_names{2}, planner_legend_names{3},planner_legend_names{4},planner_legend_names{5},planner_legend_names{6},planner_legend_names{7},planner_legend_names{8},'Hypothetical Optimal Cost','Location','southwest');
%set(l, 'Interpreter', 'none')
%hL = gridLegend(hdl,4,planner_legend_names,'Orientation','Horizontal','Fontsize',8);

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
ylim([0 105]);
xlabel('Desired Solution Cost $c_{SP}$','Interpreter','LaTex','FontSize',17)
ylabel('Success Rate [\%]','Interpreter','LaTex','FontSize',17)

%ylabel('Success rate [%] $$','Interpreter','LaTex','FontSize',17)
%xlabel('Desired Solution Cost $c_{SP}$','FontSize',17);
%ylabel('Success rate [%]','FontSize',17);
%title('Desired Solution Cost vs. Success Rate','FontSize',17);
title('Desired Solution Cost vs. Success Rate','Interpreter','LaTex','FontSize',17)

tick_size_x_axes = get(gca,'XTickLabel');
set(gca,'XTickLabel',tick_size_x_axes,'fontsize',15)
tick_size_y_axes = get(gca,'YTickLabel');
set(gca,'YTickLabel',tick_size_y_axes,'fontsize',15)

%Save Plot as in PDF Format
file_path = ['../plots/' scene_name '_desired_solution_cost_vs_success_rate'];
file_path2 = ['../figures/' scene_name '_desired_solution_cost_vs_success_rate.fig'];
saveas(gcf,file_path,'epsc');
saveas(gcf,file_path2);


