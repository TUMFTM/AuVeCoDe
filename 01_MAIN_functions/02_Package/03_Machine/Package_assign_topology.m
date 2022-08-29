function [drivetrain_type,vehicle] = Package_assign_topology(vehicle)
%% Description:
% Designed by: Adrian König (FTM, Technical University of Munich)
%-------------
% Created on: 01.01.2021
% ------------
% Version: Matlab2020b
%-------------
% Description:  This function assigns the drivetraintyp (string to number)
% ------------
% Sources:  [1] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - vehicle: struct with includes the data of the vehicle
% ------------
% Output:   - vehicle: struct with includes the data of the vehicle
% ------------

%% Legend for drivetrain_type
% 0: no motor
% 1: motor with parallel gear
% 2: motor with coaxial gear (planetary)
% 3: two motors in coaxial position
% 4: motor with coaxial gear (layshaft)

drivetrain_type=[0 0];

%% One Front Motor
if (strcmp('GM_X',vehicle.Input.topology) || strcmp('GM_GM',vehicle.Input.topology) || strcmp('GM_2G2M',vehicle.Input.topology))
    if strcmp(vehicle.Input.gear_type_f,'Parallel')
        drivetrain_type(1)=1;
        vehicle.Input.gearbox_axles{1}='parallel';
        vehicle.Input.gearbox_type{1}='lay-shaft';
    elseif strcmp(vehicle.Input.gear_type_f,'Coaxial-layshaft')
        drivetrain_type(1)=4;
        vehicle.Input.gearbox_axles{1}='coaxial';
        vehicle.Input.gearbox_type{1}='lay-shaft';
    elseif strcmp(vehicle.Input.gear_type_f,'Coaxial')
        drivetrain_type(1)=2;
        vehicle.Input.gearbox_axles{1}='coaxial';
        vehicle.Input.gearbox_type{1}='planetary';
    end
    
end

%% One Rear Motor
if (strcmp('X_GM',vehicle.Input.topology) || strcmp('GM_GM',vehicle.Input.topology) || strcmp('2G2M_GM',vehicle.Input.topology)) 
    if strcmp(vehicle.Input.gear_type_r,'Parallel')
        drivetrain_type(2)=1;
        vehicle.Input.gearbox_axles{2}='parallel';
        vehicle.Input.gearbox_type{2}='lay-shaft';
    elseif strcmp(vehicle.Input.gear_type_r,'Coaxial-layshaft')
        drivetrain_type(2)=4;
        vehicle.Input.gearbox_axles{2}='coaxial';
        vehicle.Input.gearbox_type{2}='lay-shaft';
    elseif strcmp(vehicle.Input.gear_type_r,'Coaxial')
        drivetrain_type(2)=2;
        vehicle.Input.gearbox_axles{2}='coaxial';
        vehicle.Input.gearbox_type{2}='planetary';
    end
    
end

%% Two front motors
if (strcmp('2G2M_X',vehicle.Input.topology) || strcmp('2G2M_GM',vehicle.Input.topology) || strcmp('2G2M_2G2M',vehicle.Input.topology))
    if strcmp(vehicle.Input.gear_type_f,'Parallel')
        error('Two motors are not allowed with parallel gearbox')
    elseif strcmp(vehicle.Input.gear_type_f,'Coaxial-layshaft')
        error('Two motors are not allowed with coaxial layshaft gearbox')
    elseif strcmp(vehicle.Input.gear_type_f,'Coaxial')
        drivetrain_type(1)=3;
        vehicle.Input.gearbox_axles{1}='coaxial';
        vehicle.Input.gearbox_type{1}='planetary';
    end
    
end


%% Two rear motors
if (strcmp('X_2G2M',vehicle.Input.topology) || strcmp('GM_2G2M',vehicle.Input.topology) || strcmp('2G2M_2G2M',vehicle.Input.topology))
    if strcmp(vehicle.Input.gear_type_r,'Parallel')
        error('Two motors are not allowed with parallel gearbox')
    elseif strcmp(vehicle.Input.gear_type_r,'Coaxial-layshaft')
        error('Two motors are not allowed with coaxial layshaft gearbox')
    elseif strcmp(vehicle.Input.gear_type_r,'Coaxial')
        drivetrain_type(2)=3;
        vehicle.Input.gearbox_axles{2}='coaxial';
        vehicle.Input.gearbox_type{2}='planetary';
    end
    
end


end