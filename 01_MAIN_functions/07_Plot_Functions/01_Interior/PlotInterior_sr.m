function [a_pos,eu_1,eu_1_alpha,x_pos_seat,z_pos_seat,x_pos_br,z_pos_br,Lz_br] = PlotInterior_sr(vehicle,int_boundary_f_new,int_boundary_r_new,a_1,a_2)
%% Description:
% Designed by: Felix Fahn, Adrian König (FTM, Technical University of Munich)
%-------------
% Created on: 29.07.2021
% ------------
% Version: Matlab2020b
%-------------
% Description: %This function calculates the positions of the seats for single row
% ------------
% Sources:        [1] Felix Fahn, “Innenraummodellierung von autonomen Elektrofahrzeugen” Semester Thesis, Technical University of Munich, Institute of Automotive Technology, 2021
%                 [2] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - veh:  struct with the vehicle parameters (Number of Seats (per row, total); Interior type (vis-a-vis, conv., ...); Measurements of interior)
%           - Par:  struct with fixed parameters
%           - boundarys of interior
%           - a1 and a2 (safety distance for crash situations)
% ------------
% Output:   - Positions of the Seats
% ------------

%% Calculation of x- and z-Positions of seat and backrest
a_pos=[a_1 a_2];

eu_1 = [0; -2*pi/360*vehicle.Input.angle_br_max(1);0];
eu_1_alpha = [0; -2*pi/360*vehicle.Input.angle_br_min(1);0];

x1_seat = int_boundary_f_new(vehicle.Input.int_height + 1);
x1_br = int_boundary_f_new(vehicle.Input.int_height + 1) + vehicle.Input.seat_depth(1);

z1_seat = vehicle.Input.int_height + vehicle.interior.boundary_seat(1) + vehicle.Input.seat_adjustment_z(1);
z1_br = vehicle.Input.int_height + vehicle.interior.boundary_seat(1)+vehicle.Input.thickness_seat(1) + vehicle.Input.seat_adjustment_z(1);

x_pos_seat = [x1_seat-vehicle.Input.seat_adjustment_x(1) 0];
z_pos_seat = [z1_seat 0];
x_pos_br = [x1_br 0];
z_pos_br = [z1_br 0];

Lz_br = [-vehicle.Input.thickness_br(1)  -vehicle.Input.thickness_br(2)];

end