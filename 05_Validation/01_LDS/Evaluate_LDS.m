%% Description:
%This script validates the LDS. For this scope, real vehicle are
%parametrized and simulated to derive their consumption. The data required
%to parametrize the vehicles are stored in an Excel table. Subsequently the
%calculated consumption is compared with the real vehicle measurement
%conducted at the Institute of Automotive Technology
%% Author: Lorenzo Nicoletti, 19.08.2021
%% Sources:
%Measurements real vehicle -> Ph.D. Thesis Alexander Koch
%Parametrized vehicle data -> Sources in the Excel Table
%% Implementation:
% 0) Choose a vehicle to simulate
% 1) Initialize the parametrized vehicle, and load the measured vehicle consumption
% 2) Calculate from the parametrized vehicle the consumption and compare it with the measured vehicle 
% 3) Subfunctions

%% 0) Choose a vehicle to simulate
%Select vehicle: 1=Tesla Model3; 2=VW ID.3
clear
close all

compare_Svenja=1;
vehicle_ID = 1; %1: Tesla Mode3 ; 2: VW ID.3

%% 1) Initialize the parametrized vehicle, and load the measured vehicle consumption

% 1A) Initialize Parametrized vehicle:
    %Get the path where the Excel Table with the vehicle data is saved
    folderpath=mfilename('fullpath');
    filename=mfilename;
    folderpath=folderpath(1:findstr(folderpath,filename)-1);

    %Read the data from the Tab vehicle
    vehicleexcel.settings.pathsimdata=[folderpath,'simulation_data_LDS_ev.xlsx'];
    [~,~,vehicleraw]=xlsread(vehicleexcel.settings.pathsimdata,'vehicle');
    vehicleraw=[vehicleraw(:,1),vehicleraw(:,vehicle_ID+1)];   %1st column always, it contains the var names. The 2nd component is the vehicle of interest.
    vehicleexcel=read_excel_data(vehicleraw,'vehicle',vehicleexcel);
    clear vehicleraw

    %Initialize the vehicle structure:
    [vehicle,Parameters]=function_initialize_LDS(vehicle_ID+1,vehicleexcel);
    %Parameters.LDS.correction_gearbox=1;
    vehicle = calc_vehicle_weight_no_battery(vehicle,Parameters);

% 1B) Load the measured data  
    name_real_data =strrep(vehicle.Input.driving_cycle,'measured','data');
    load([name_real_data,'.mat'])

%% 2) Simulate the longitudinal dynamic of the vehicle and plot results

%Simulate the longitudinal dynamics
[vehicle_correct] = calc_longitudinal_simulation(vehicle,Parameters);

%Compare the different in recuperated energy due to the fact that the ID.3
%stops recuperating at 4 km/h
if vehicle_ID==2
    
    E_bat_calc=vehicle_correct.LDS.sim_cons.E_bat_step; %Calculated vehicle enrgy in kWh
    v_calc=vehicle_correct.LDS.sim_cons.v*3.6;          %Calculated cycle speed in km/h
    E_bat_calc(find((E_bat_calc<0).*(v_calc<4)))=0;     %SEt to 0 the timestep where the vehicles recuperates with a speed below 4 km/h

    E_cum_base=vehicle_correct.LDS.sim_cons.E_bat_cum;
    E_cum_with_4kmh_limit=cumsum(E_bat_calc,'omitnan');
    delta=E_cum_with_4kmh_limit(end)-E_cum_base(end);
    disp(['The difference in energy recuperation due to speed limit is ',num2str(delta),' kWh'])
end

if exist('A0_Vars')
    Cycles{1}.A0_Vars=A0_Vars;
end

%Also add the efficiency map of Svenja if requested:
if compare_Svenja
    vehicle.Input.characteristic_forced{2}=NaN;
    [vehicle_Svenja] = calc_longitudinal_simulation(vehicle,Parameters);
    postprocessing_LDS_evaluation(Cycles,vehicle_correct.LDS,vehicle_Svenja.LDS)
else
    % Postprocessing
    postprocessing_LDS_evaluation(Cycles,vehicle_correct.LDS);
end

%% 3) Subfunctions
function [vehicle,Parameters]=function_initialize_LDS(id,vehicle)

    %Initialize the vehicle and Parameters struct:
    [vehicle,Parameters] = initialize_vehicle_struct(vehicle);

    %Load the further Calculation parameters as initialized in the excel-> Some Parameters of the MATLAB may be overweitten
    [~,~,parametersraw]=xlsread(vehicle.settings.pathsimdata,'Parameters');
    parametersraw=[parametersraw(:,1),parametersraw(:,id)];
    Parameters=read_excel_data(parametersraw,'Parameters',Parameters);
end