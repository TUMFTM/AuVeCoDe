function [Variable_Names,Variable_Type,lb,ub,A,b] = Opt_Init(Parameters)
%% Description:
% Function initializes variables for optimization

% Author:   Daniel Telschow, Adrian König
% Date:     September 2020
% Update:   July 2021

%% Inputs:

%% Outputs:
% Variable names, types, lower bound and upper bound
% Linear constraint matrizes
% Objective Weightings

%% Implementation
% 1. Variable names
% 2. Variable types
% 3. Lower Bound
% 4. Upper Bound
% 5. Constraint matrizes
% 6. Objective Weightings

%% 1. Variable names
Variable_Names={'Motor Topology', 'Motor type',...
    'Gear type front','Gear type rear','Tire diameter','Cell type',...
    'Alpha Axle 1','Alpha Axle 2','Engine-Gear Angle Front','Engine-Gear Angle Rear',...
    'Axis Type','Gear Ratio 1 Front','Gear Ratio 1 Rear',...
    'Gearbox Opt Goal','Trunk Ratio',...
    'Interior Height', 'Small Overlap','Distance Wheelhouse to Bumper Front','Distance Wheelhouse to Bumper Rear',...
    'Steering Ratio','Torque Ratio','H30_front','H30_rear','Front Window Angle','Rear Window Angle',...
    'Front Trunk Type','Rear Trunk Type','Distance Interior to Front Wagon','Distance Interior to Rear Wagon'};

%% 2. Variable types (1 = float; 2 = integer)
Type = struct;
Type.motor_topology=2;
Type.motor_type = 2;
Type.gear_type_front = 2;
Type.gear_type_rear = 2;
Type.tire_diameter = 1;
Type.cell_type = 2;
Type.alpha_axle_1 = 1;
Type.alpha_axle_2 = 1;
Type.engine_gear_angle_front = 1;
Type.engine_gear_angle_rear = 1;
Type.axis_type = 2;
Type.gear_ratio_1_f=1;
Type.gear_ratio_1_r=1;
Type.gearbox_opt=2;
Type.trunk_ratio=1;
Type.int_height=1;
Type.small_overlap=2;
Type.wheelhouse_to_bumper_f=1;
Type.wheelhouse_to_bumper_r=1;
Type.steering_ratio=1;
Type.torque_ratio=1;
Type.H30_f=1;
Type.H30_r=1;
Type.window_angle_front=1;
Type.window_angle_rear=1;
Type.trunk_type_front=2;
Type.trunk_type_rear=2;
Type.interior_to_fwagon=1;
Type.interior_to_rwagon=1;

Variable_Type=cell2mat(struct2cell(Type)); % Variable type Matrix

%% 3. Lower Bound (Minimum Values)
Min = struct;
Min.motor_topology=1;
Min.motor_type = 1;
Min.gear_type_front = 1;
Min.gear_type_rear = 1;
Min.tire_diameter = Parameters.input.tire_diam_min;
Min.cell_type = 1;
Min.alpha_axle_1 = Parameters.input.axle_angle_1_min;
Min.alpha_axle_2 = Parameters.input.axle_angle_2_min;
Min.engine_gear_angle_front = Parameters.input.engine_angle_front_min;
Min.engine_gear_angle_rear = Parameters.input.engine_angle_rear_min;
Min.axis_type = 1;
Min.gear_ratio_1_f=Parameters.input.gear_ratio_1_front_min;
Min.gear_ratio_1_r=Parameters.input.gear_ratio_1_rear_min;
Min.gearbox_opt=1;
Min.trunk_ratio=Parameters.input.trunk_ratio_min;
Min.int_height=Parameters.input.int_height_min;
Min.small_overlap=1;
Min.wheelhouse_to_bumper_f=Parameters.input.dist_wheelhouse2bumper_f_min;
Min.wheelhouse_to_bumper_r=Parameters.input.dist_wheelhouse2bumper_r_min;
Min.steering_ratio=Parameters.input.steering_ratio_min;
Min.torque_ratio=Parameters.input.torque_ratio_min;
Min.H30_f=Parameters.input.H30_f_min;
Min.H30_r=Parameters.input.H30_r_min;
Min.window_angle_front=Parameters.input.window_angle_front_min;
Min.window_angle_rear=Parameters.input.window_angle_rear_min;
Min.trunk_type_front=1;
Min.trunk_type_rear=1;
Min.interior_to_fwagon=Parameters.input.dist_int2fw_min;
Min.interior_to_rwagon=Parameters.input.dist_int2rw_min;

lb=cell2mat(struct2cell(Min)); % Lower Bound Matrix

%% Upper Bound (Maximum Values)
Max = struct;
if isempty(Parameters.input.motor_topology)
    Max.motor_topology=8;
else
    Max.motor_topology=1;
end

if isempty(Parameters.input.engine_type)
    Max.motor_type = 2;
else
    Max.motor_type = 1;
end

if isempty(Parameters.input.gear_type_front)
    Max.gear_type_front = 3;
else
    Max.gear_type_front = 1;
end
    
if isempty(Parameters.input.gear_type_rear)
    Max.gear_type_rear = 3;
else
    Max.gear_type_rear = 1;
end

Max.tire_diameter = Parameters.input.tire_diam_max;

if isempty(Parameters.input.cell_type)
    Max.cell_type = 3;
else
    Max.cell_type = 1;
end

Max.alpha_axle_1 = Parameters.input.axle_angle_1_max;
Max.alpha_axle_2 = Parameters.input.axle_angle_2_max;
Max.engine_gear_angle_front = Parameters.input.engine_angle_front_max;
Max.engine_gear_angle_rear = Parameters.input.engine_angle_rear_max;

if isempty(Parameters.input.rear_axis_type)
    Max.axis_type = 4;
else
    Max.axis_type = 1;
end

Max.gear_ratio_1_f=Parameters.input.gear_ratio_1_front_max;

Max.gear_ratio_1_r=Parameters.input.gear_ratio_1_rear_max;


if isempty(Parameters.input.gearbox_opt)
    Max.gearbox_opt = 2;
else
    Max.gearbox_opt = 1;
end

Max.trunk_ratio=Parameters.input.trunk_ratio_max;
Max.int_height = Parameters.input.int_height_max;

Max.small_overlap = 2;
Max.wheelhouse_to_bumper_f=Parameters.input.dist_wheelhouse2bumper_f_max;
Max.wheelhouse_to_bumper_r=Parameters.input.dist_wheelhouse2bumper_r_max;
Max.steering_ratio=Parameters.input.steering_ratio_max;
Max.torque_ratio=Parameters.input.torque_ratio_max;
Max.H30_f=Parameters.input.H30_f_max;
Max.H30_r=Parameters.input.H30_r_max;
Max.window_angle_front=Parameters.input.window_angle_front_max;
Max.window_angle_rear=Parameters.input.window_angle_rear_max;

if isempty(Parameters.input.trunk_type_front)
    Max.trunk_type_front = 2;
else
    Max.trunk_type_front = 1;
end

if isempty(Parameters.input.trunk_type_rear)
    Max.trunk_type_rear = 2;
else
    Max.trunk_type_rear = 1;
end

Max.interior_to_fwagon=Parameters.input.dist_int2fw_max;
Max.interior_to_rwagon=Parameters.input.dist_int2rw_max;

ub=cell2mat(struct2cell(Max)); % Upper Bound Matrix

%% 5. Constraint matrizes
n=length(ub); % Number of independent variables
A = zeros(2,n); % First value = number of constraints
b = zeros(2,1);

end

