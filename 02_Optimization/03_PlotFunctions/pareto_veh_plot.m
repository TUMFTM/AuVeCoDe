function  pareto_veh_plot(varargin)

%% Description
% Function  plots first/middle/last concept of pareto front,
% saves the structs and compares the outputs in an excel table

%% Input:
% varargin:
% varargin{1}: pop
% varargin{2}: plot_settings
% varargin{3}: options


%% edited by Fabian
% read input variables
if nargin >= 1 % nargin == 1: only pop as input
    pop = varargin{1};
    if nargin >= 2 % nargin == 2: plot settings as additional input
        plot_settings = varargin{2};
    end
    if nargin >= 3 % nargin == 3: options for load Parameters_Pareto and save path for additional results 
        options = varargin{3};
    end
end

% Load Inputs
clear Parameters
% load Parameters Pareto for plotting
load(fullfile(options.AdditionalResultPath, "Parameters_Pareto.mat"));

%% Select vehicle concept parameters
switch plot_settings
    case "default"
        veh_green=pop(1);
        veh_red=pop(end);
        veh_blue=pop(ceil(end/2));
    case "best"
        veh_green=pop(1);
        veh_red=pop(2);
        veh_blue=pop(3);
end

%% Calculate vehicle concepts
[vehicle_green] = MAIN_AuVeCoDe(Parameters,veh_green.var);
[vehicle_blue] = MAIN_AuVeCoDe(Parameters,veh_blue.var);
[vehicle_red] = MAIN_AuVeCoDe(Parameters,veh_red.var);

%% Save vehicle concepts
path_veh_green= fullfile(options.AdditionalResultPath,'vehicle_result_green');
path_veh_blue= fullfile(options.AdditionalResultPath,'vehicle_result_blue');
path_veh_red= fullfile(options.AdditionalResultPath,'vehicle_result_red');

path_veh_green_input= fullfile(options.AdditionalResultPath,'vehicle_input_green');
path_veh_blue_input= fullfile(options.AdditionalResultPath,'vehicle_input_blue');
path_veh_red_input= fullfile(options.AdditionalResultPath,'vehicle_input_red');

recycle on
save(path_veh_green,'vehicle_green');
save(path_veh_blue,'vehicle_blue');
save(path_veh_red,'vehicle_red');

var_green=veh_green.var;
var_blue=veh_blue.var;
var_red=veh_red.var;
save(path_veh_green_input,'var_green');
save(path_veh_blue_input,'var_blue');
save(path_veh_red_input,'var_red');

%% Write and save result table
%% Write and save result table
WRITE_results(vehicle_green,vehicle_blue,vehicle_red)

%% Plot 3D vehicles
figure(3)
clf
ax1=subplot(1,3,1);
title('Concept 1: Green Square');
if vehicle_green.feasible==1 %only plot if feasible
    DISPLAY_vehicle(vehicle_green,Parameters,ax1);
end

ax2=subplot(1,3,2);
title('Concept 2: Blue Circle');
if vehicle_blue.feasible==1 %only plot if feasible
    DISPLAY_vehicle(vehicle_blue,Parameters,ax2);
end

ax3=subplot(1,3,3);
title('Concept 3: Red Cross');
if vehicle_red.feasible==1 %only plot if feasible
    DISPLAY_vehicle(vehicle_red,Parameters,ax3);
end

end

