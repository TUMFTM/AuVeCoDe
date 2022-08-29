function [v] = Package_H30(v,Par)
%% Description:
% Designed by: Adrian König (FTM, Technical University of Munich)
%-------------
% Created on: 11.11.2021
% ------------
% Version: Matlab2020b
%-------------
% Description: Calculate the boundary height of the seat with the H30 Input
% ------------
% Sources:  [1] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - veh:  struct with the vehicle parameters
%           - Par:  struct with fixed parameters
% ------------
% Output:   - veh:    updated struct with the vehicle parameters
% ------------

%% Implementation:
%1) Read in parameters from vehicle struct and regression from Parameter
%   struct
%2) Calculate missing parameter
%3) Write result in vehicle struct

%% 1) Load needed Parameters
H30_cor_z   =   Par.dimensions.EZ.mannequin_z_tp;                       %Correction of cushion to H-Point 
H30_f       =   v.Input.H30_f;                                          %H30 measrurement of front row
H30_r       =   v.Input.H30_r;                                          %H30 measrurement of rear row
d_seat      =   v.Input.thickness_seat;                                 %Thickness of seat

%% 2) Calculate Measurements
v.interior.boundary_seat(1)    =  H30_f-H30_cor_z-d_seat(1);    %H30 measurment - correction of H-Point
v.interior.boundary_seat(2)    =  H30_r-H30_cor_z-d_seat(2);    %H30 measurment - correction of H-Point

%% 3) Overwrite Parameters not active/differently used in H30 Optimization mode
%No adjustment in z allowed for H30 Mode
v.Input.seat_adjustment_z(1) =   0;
v.Input.seat_adjustment_z(2) =   0;

%Seat height is H30 minus H30_cor_z
v.Input.seat_height(1)       =  H30_f-H30_cor_z;
v.Input.seat_height(2)       =  H30_r-H30_cor_z;
end

