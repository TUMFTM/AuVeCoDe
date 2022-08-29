function [v] = Package_Crossmember(v,Par)
%% Description:
% Designed by: Adrian König (FTM, Technical University of Munich)
%-------------
% Created on: 04.11.2021
% ------------
% Version: Matlab2020b
%-------------
% Description:  This function assigns the dimensions of the cross member along the X
%               direction (driving direction).
% ------------
% Sources:  [1] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - vehicle: struct with includes the data of the vehicle
% ------------
% Output:   - vehicle: struct with includes the data of the vehicle
% ------------

%% Implementation

%Assign the length of the cross memeber at the vehicle's front and rear end
v.dimensions.CX.crossmember_f   =   Par.bumper.dimensions.CX.crossmember_f; %The length of the cross memeber at the vehicle's front end in mm 
v.dimensions.CX.crossmember_r   =   Par.bumper.dimensions.CX.crossmember_r; %The length of the cross memeber at the vehicle's rear end in mm  

%Assign the width of the cross memeber at the vehicle's front and rear end
%Only for plot purposes. Assumed to be slightly wider than the distance comprised between the wheelhouses
v.dimensions.CY.crossmember_f   =   v.dimensions.GY.vehicle_width-v.dimensions.CY.wheelhouse_f_width;
v.dimensions.CY.crossmember_r   =   v.dimensions.GY.vehicle_width-v.dimensions.CY.wheelhouse_r_width*1.5;

%Assign the height of the cross memeber at the vehicle's front and rear end
v.dimensions.CZ.crossmember_f   =   Par.bumper.dimensions.CZ.crossmember_f; 
v.dimensions.CZ.crossmember_r   =   Par.bumper.dimensions.CZ.crossmember_r; 
end

