%% +++++Plotting the Average Cost Evolution of the Best Solution Path ++++++


%Set Scene name
scene_name = 'glass';

%%+++++++++++++++++++++++++++ Setup ++++++++++++++++++++++++++++++++
%List of planner
planner_list = cell(1,6);
planner_list{1,1} = ['uni_rrt_star'];
planner_list{1,2} = ['uni_informed_rrt_star'];

planner_list{1,3} = ['bi_rrt_connect'];
planner_list{1,4} = ['bi_informed_rrt'];
planner_list{1,5} = ['bi_rrt_star'];
planner_list{1,6} = ['bi_informed_rrt_star'];


%Planner names for plot legend
planner_legend_names = cell(1,6);
planner_legend_names{1,1} = ['RRT*'];
planner_legend_names{1,2} = ['Informed RRT*'];

planner_legend_names{1,3} = ['RRT-Connect'];

%planner_legend_names{1,4} = ['Hypothetical Optimal Cost'];

planner_legend_names{1,4} = ['Informed BiRRT'];
planner_legend_names{1,5} = ['BiRRT*'];
planner_legend_names{1,6} = ['Informed BiRRT*'];


%Number of planners
num_planners = size(planner_list,2);

%Number of runs per planner
planners_runs = 100;
run_indices = planners_runs-1;

%Time after which a first solution is available
first_solution_available = 0 : 1 : 180;

%Number of runs for each planner that reached a first solution within 
time_solution_available = zeros(num_planners,size(first_solution_available,2));


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

        %If planner found a solution to the planning problem in run j
        if planner_statistics{2}{3} == 1
            
            %Get time of first solution
            first_solution_time = planner_statistics{2}{7};         
            
            %Find Index of element in the first_solution_available vector where the final_solution_cost is larger than the desired cost 
            index_smallest_reached_solution_time = find(first_solution_available>=first_solution_time,1);
            index_smallest_reached_solution_time = index_smallest_reached_solution_time - 1;
            
            %Increment success rate for all desired costs larger than the actually reached cost
            time_solution_available(i,index_smallest_reached_solution_time:end) = time_solution_available(i,index_smallest_reached_solution_time:end) +1;
            
        else
            disp('This planning run failed to find a path, continuing with next planner run statistics')
        end %if planner run successful
        
    end %run iterations
end %planner type iterations


%% +++++++++ Solution available within specific runtime in [%] ++++++++++++++++
%Solutions available in percent
time_solution_available_percent = zeros(num_planners,size(first_solution_available,2));
for i = 1:num_planners
    time_solution_available_percent(i,:) = time_solution_available(i,:)./planners_runs * 100
end


%% +++++++++ Plot Average Best Solution Cost Evolution for each planner ++++++++++++++++

%Collect some line styles for the plot
line_style = {'--b' ; '--r' ; '-k' ; '-g' ; '-b' ; '-r'};

%Set line width
line_width = 2;

%Create Figure Handle
h = figure('Name','Solution Available vs. Planning Runtime','NumberTitle','off');

size(first_solution_available)
size(time_solution_available_percent(1,:))
%Plot results
hdl = plot(first_solution_available,time_solution_available_percent(1,:),line_style{1}, ...
           first_solution_available,time_solution_available_percent(2,:),line_style{2}, ...
           first_solution_available,time_solution_available_percent(3,:),line_style{3}, ...
           first_solution_available,time_solution_available_percent(4,:),line_style{4}, ...
           first_solution_available,time_solution_available_percent(5,:),line_style{5}, ...
           first_solution_available,time_solution_available_percent(6,:),line_style{6}, ...
           'LineWidth',line_width);
       
%Plot results
%for i = 1:num_planners
%    plot(first_solution_available,time_solution_available_percent(i,:),line_style{i},'LineWidth',line_width);
%    hold on;
%end

%Set some plot properties
%l = legend(planner_legend_names{1}, planner_legend_names{2}, planner_legend_names{3},planner_legend_names{4}, planner_legend_names{5}, planner_legend_names{6},planner_legend_names{7},planner_legend_names{8},'Location','southeast');
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


%set(gca,'XDir','reverse');
%xlim([lower_time_limit upper_time_limit]);
ylim([0 105]);
%xlabel('Planning Runtime [s]','FontSize',17);
%ylabel('Solution Available [%]','FontSize',17);
%title('Planning Runtime vs. Solution Available','FontSize',17);
xlabel('Planning Runtime [s]','Interpreter','LaTex','FontSize',17)
ylabel('Solution Available [\%]','Interpreter','LaTex','FontSize',17)
title('Planning Runtime vs. Solution Available','Interpreter','LaTex','FontSize',17)

tick_size_x_axes = get(gca,'XTickLabel');
set(gca,'XTickLabel',tick_size_x_axes,'fontsize',13)
%set(gca,'xtick',[0, 45, 90, 135, 180])
tick_size_y_axes = get(gca,'YTickLabel');
set(gca,'YTickLabel',tick_size_y_axes,'fontsize',13)

%Save Plot as in PDF Format
file_path = ['../plots/' scene_name '_planning_runtime_vs_solution_available'];
file_path2 = ['../figures/' scene_name '_planning_runtime_vs_solution_available.pdf'];
saveas(gcf,file_path,'epsc');
saveas(gcf,file_path2);

