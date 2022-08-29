function WRITE_results(varargin)
%% Description:
% Designed by: Adrian König (FTM, Technical University of Munich), Daniel Telschow
%-------------
% Created on: 01.05.2021
% ------------
% Version: Matlab2020b
%-------------
% Description: Function that writes the vehicle results into an excel sheet
% ------------
% Sources:  [1] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - varagin: input can be one ore more vehicle structs
% ------------
% Output:   - Excel-Sheet located in 06_Results
% ------------

%% Implementation
% 1) Locate folder
% 2) Read vehicle
% 3) Write table and save

%% 1) Locate folder
[filepath] = fileparts(which('Run_AuVeCoDe')); % Main Folder
path_folder=fullfile(filepath,'06_Results'); % Result folder
formatOut='yymmdd'; %Right format
currentdate=datestr(now,formatOut); %current date in right format
path_table=fullfile(path_folder,(currentdate + "_Result.xlsx")); % Table will be saved as Excel file

%% 2) Read vehicle
%Prealocate Result Matrix
Date=datetime;
Result=cell(nargin+3,24);
Result{2,1}=Date;

%Write column names in Result matrix
Result{3,1}='Vehicle index';
Result{3,2}='Desired range set in km';
Result{3,3}='Achieved range in km';
Result{3,4}='Desired acceleration 0-100km/h in s';
Result{3,5}='Achieved acceleration 0-100km/h in s';
Result{3,6}='Desired max. speed in km/h';
Result{3,7}='Achieved max. speed in km/h';
Result{3,8}='Desired tire radius in mm';
Result{3,9}='Achieved tire radius in mm';
Result{3,10}='Tire width in mm';
Result{3,11}='Rim diameter in inch';
Result{3,12}='Gross battery capacity gross in kWh';
Result{3,13}='Net battery capacity gross in kWh';
Result{3,14}='Consumption in kWh/100km';
Result{3,15}='Torque front in Nm';
Result{3,16}='Power front in kW';
Result{3,17}='Torque rear in Nm';
Result{3,18}='Power rear in kW';
Result{3,19}='Length in mm';
Result{3,20}='Width in mm';
Result{3,21}='Height in mm';
Result{3,22}='Overhang front in mm';
Result{3,23}='Overhang rear in mm';
Result{3,24}='Wheelbase in mm';
Result{3,25}='EU empty mass in kg';
Result{3,26}='Cost in €';



for i=1:nargin   
    %Differentiate if only one or more vehicle are input
    j=i+3; %Row count for values starts from second row
    if nargin==1
        Result{j,1}='Vehicle';
    else
        Result{j,1}=sprintf('Vehicle %d',i);
    end
    if varargin{i}.feasible==1 %Only write values if vehicle is feasible
        Result{j,2}=round(varargin{i}.Input.range,2);
        Result{j,3}=round(varargin{i}.LDS.range,2);
        Result{j,4}=round(varargin{i}.Input.acceleration_time_req,2);
        Result{j,5}=round(varargin{i}.LDS.sim_acc.acc_time_is,2);
        Result{j,6}=round(varargin{i}.Input.max_speed,2);
        Result{j,7}=round(varargin{i}.LDS.sim_speed.max_speed_is,2);
        Result{j,8}=round(varargin{i}.Input.r_tire,2);
        Result{j,9}=round(varargin{i}.LDS.wheel.r_tire,2);
        Result{j,10}=round(varargin{i}.dimensions.CY.wheel_f_width,2);
        Result{j,11}=round(varargin{i}.dimensions.CX.rim_f_diameter,2);
        Result{j,12}=round(varargin{i}.battery.energy_is_gross_in_kWh,2);
        Result{j,13}=round(varargin{i}.battery.energy_is_net_in_kWh,2);
        Result{j,14}=round(varargin{i}.LDS.sim_cons.consumption100km,4);
        try
            Result{j,15}=round(varargin{i}.LDS.MOTOR{1, 1}.T_max,2);
            Result{j,16}=round(varargin{i}.LDS.MOTOR{1, 1}.P_max_mech,2);
        catch
            Result{j,15}='-';
            Result{j,16}='-';
        end
        try
            Result{j,17}=round(varargin{i}.LDS.MOTOR{1, 2}.T_max,2);
            Result{j,18}=round(varargin{i}.LDS.MOTOR{1, 2}.P_max_mech,2);
        catch
            Result{j,17}='-';
            Result{j,18}='-';
        end
        Result{j,19}=round(varargin{i}.dimensions.GX.vehicle_length,2);
        Result{j,20}=round(varargin{i}.dimensions.GY.vehicle_width,2);
        Result{j,21}=round(varargin{i}.dimensions.GZ.vehicle_height,2);
        Result{j,22}=round(varargin{i}.dimensions.GX.vehicle_overhang_f,0);
        Result{j,23}=round(varargin{i}.dimensions.GX.vehicle_overhang_r,0);
        Result{j,24}=round(varargin{i}.dimensions.GX.wheelbase,2);
        Result{j,25}=round(varargin{i}.masses.vehicle_empty_weight_EU,2);
        Result{j,26}=round(varargin{i}.Cost.Manufacturing_Cost,2);
    else
        Result{j,2}='Vehicle was not feasible';
    end
end

%% 3) Write table and save
try
    writecell(Result,path_table,'WriteMode','append');
catch
    error('Note: WriteMode not working with MATLAB 2019b or older!')
end
end

