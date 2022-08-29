%% Analyse Vehicles
%% Description:
% Designed by: Adrian König (FTM, Technical University of Munich), Daniel Telschow
%-------------
% Created on: 17.02.2022
% ------------
% Version: Matlab2020b
%-------------
% Description: Function which helps to analyze created vehicles
% ------------
% Sources:  [1] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - Copy vehicle struct !including! the Parameter struct each in one folder of "Vehicle_for_Analyze_Vehicles"
% ------------
% Output:   - Plot and Table for analyzis
% ------------

%% Implementation


%% 1) Search and collect vehicles
clc; %Clear command window
close all %close all windows
folder = fileparts(which('Analyze_Vehicles')); %Determine where Startfile and thus MAIN_AuVeCoDe.m is
try
folder_veh= fullfile(folder,'Vehicles_for_Analyze_Vehicles');
catch
    error('Folder with vehicles is missing!')
end
%Search for vehicles in folders
FileList = dir(fullfile(folder_veh, '**', '*.mat'));
num_veh=length(FileList)/2; %number of vehicle/Parameter pairs
M_vehicles=cell(2,num_veh);

%Check if enough inputs are given (one vehicle also needs one Parameter struct)
if ~rem(num_veh*2,2)==0
    error('Parameter or vehicle is missing in one or more folders')
end

%% 2) Join vehicles and Paremter structs
for i=1:(length(FileList)/2)
    element1=load(fullfile(FileList(i*2-1).folder,FileList(i*2-1).name));
    element2=load(fullfile(FileList(i*2).folder,FileList(i*2).name));
    
    name1=fieldnames(element1);
    name2=fieldnames(element2);
    if prod(size(name1))==1 && prod(size(name2))==1 && ((strcmp(name1{1},'Parameters') && startsWith(name2{1},'vehicle')) || (strcmp(name2{1},'Parameters') && startsWith(name1{1},'vehicle')))
        if strcmp(name1{1},'Parameters')
            M_vehicles{1,i}=element1(1);
            M_vehicles{2,i}=element2(1);
        else            M_vehicles{1,i}=element2(1);
            M_vehicles{2,i}=element1(1);
        end
    else
        error('Error during loading vehicle and Parameter structs. Wrong format')
    end
    
    
end




%% 1) Read and write vehicle data
%Create Table
Result=cell(length(M_vehicles)+1,24);

%Write column names in Result matrix
Result{1,1}='Vehicle index';
Result{1,2}='Desired range set in km';
Result{1,3}='Achieved range in km';
Result{1,4}='Desired acceleration 0-100km/h in s';
Result{1,5}='Achieved acceleration 0-100km/h in s';
Result{1,6}='Desired max. speed in km/h';
Result{1,7}='Achieved max. speed in km/h';
Result{1,8}='Desired tire radius in mm';
Result{1,9}='Achieved tire radius in mm';
Result{1,10}='Tire width in mm';
Result{1,11}='Rim diameter in inch';
Result{1,12}='Gross battery capacity gross in kWh';
Result{1,13}='Net battery capacity gross in kWh';
Result{1,14}='Consumption in kWh/100km';
Result{1,15}='Torque front in Nm';
Result{1,16}='Power front in kW';
Result{1,17}='Torque rear in Nm';
Result{1,18}='Power rear in kW';
Result{1,19}='Length in mm';
Result{1,20}='Width in mm';
Result{1,21}='Height in mm';
Result{1,22}='Wheelbase in mm';
Result{1,23}='EU empty mass in kg';
Result{1,24}='Cost in €';



for i=1:length(M_vehicles)  
    %Differentiate if only one or more vehicle are input
    j=i+1; %Row count for values starts from second row
    Result{j,1}=sprintf('Vehicle %d',i);
    Name_veh=fieldnames(M_vehicles{2,i}); %Name of vehicle struct
    
    if M_vehicles{2, i}.(Name_veh{1}).feasible==1 %Only write values if vehicle is feasible
        Result{j,2}=round(M_vehicles{2, i}.(Name_veh{1}).Input.range,2);
        Result{j,3}=round(M_vehicles{2, i}.(Name_veh{1}).LDS.range,2);
        Result{j,4}=round(M_vehicles{2, i}.(Name_veh{1}).Input.acceleration_time_req,2);
        Result{j,5}=round(M_vehicles{2, i}.(Name_veh{1}).LDS.sim_acc.acc_time_is,2);
        Result{j,6}=round(M_vehicles{2, i}.(Name_veh{1}).Input.max_speed,2);
        Result{j,7}=round(M_vehicles{2, i}.(Name_veh{1}).LDS.sim_speed.max_speed_is,2);
        Result{j,8}=round(M_vehicles{2, i}.(Name_veh{1}).Input.r_tire,2);
        Result{j,9}=round(M_vehicles{2, i}.(Name_veh{1}).LDS.wheel.r_tire,2);
        Result{j,10}=round(M_vehicles{2, i}.(Name_veh{1}).dimensions.CY.wheel_f_width,2);
        Result{j,11}=round(M_vehicles{2, i}.(Name_veh{1}).dimensions.CX.rim_f_diameter,2);
        Result{j,12}=round(M_vehicles{2, i}.(Name_veh{1}).battery.energy_is_gross_in_kWh,2);
        Result{j,13}=round(M_vehicles{2, i}.(Name_veh{1}).battery.energy_is_net_in_kWh,2);
        Result{j,14}=round(M_vehicles{2, i}.(Name_veh{1}).LDS.sim_cons.consumption100km,4);
        try
            Result{j,15}=round(M_vehicles{2, i}.(Name_veh{1}).LDS.MOTOR{1, 1}.T_max,2);
            Result{j,16}=round(M_vehicles{2, i}.(Name_veh{1}).LDS.MOTOR{1, 1}.P_max_mech,2);
        catch
            Result{j,15}='-';
            Result{j,16}='-';
        end
        try
            Result{j,17}=round(M_vehicles{2, i}.(Name_veh{1}).LDS.MOTOR{1, 2}.T_max,2);
            Result{j,18}=round(M_vehicles{2, i}.(Name_veh{1}).LDS.MOTOR{1, 2}.P_max_mech,2);
        catch
            Result{j,17}='-';
            Result{j,18}='-';
        end
        Result{j,19}=round(M_vehicles{2, i}.(Name_veh{1}).dimensions.GX.vehicle_length,2);
        Result{j,20}=round(M_vehicles{2, i}.(Name_veh{1}).dimensions.GY.vehicle_width,2);
        Result{j,21}=round(M_vehicles{2, i}.(Name_veh{1}).dimensions.GZ.vehicle_height,2);
        Result{j,22}=round(M_vehicles{2, i}.(Name_veh{1}).dimensions.GX.wheelbase,2);
        Result{j,23}=round(M_vehicles{2, i}.(Name_veh{1}).masses.vehicle_empty_weight_EU,2);
        Result{j,24}=round(M_vehicles{2, i}.(Name_veh{1}).Cost.Manufacturing_Cost,2);
    else
        Result{j,2}='Vehicle was not feasible';
    end
end


%% 4) Plot vehicles in 1D
figure
clf
for i=1:num_veh
    if rem(num_veh,2)==0
        ax=subplot(2,num_veh/2,i);
    else
        ax=subplot(1,num_veh,i);
    end
    %Optional fixed axes, better zoom but not true to scale after zoom
    ax.Clipping = "off";
    Name_Par=fieldnames(M_vehicles{1,i});
    Name_veh=fieldnames(M_vehicles{2,i});
    DISPLAY_vehicle(M_vehicles{2,i}.(Name_veh{1}),M_vehicles{1,i}.(Name_Par{1}),ax)
    ax.XLim=[-1500, max(cell2mat(Result(2:end, 19)))];
    ax.YLim=[-500, max(cell2mat(Result(2:end, 20)))];
    ax.ZLim=[0, max(cell2mat(Result(2:end, 21)))];
end

%% 5) Plot properties

