function [a_pos,eu_1,eu_1_alpha,eu_2,eu_2_alpha,x_pos_seat,z_pos_seat,x_pos_br,z_pos_br,Lz_br] = PlotInterior_con(vehicle,int_boundary_f_new,int_boundary_r_new,a_1,a_2)
%% Description:
% Designed by: Felix Fahn, Adrian König (FTM, Technical University of Munich)
%-------------
% Created on: 29.07.2021
% ------------
% Version: Matlab2020b
%-------------
% Description: %This function calculates the positions of the seats for conventional layout
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
eu_2 = [0; -2*pi/360*vehicle.Input.angle_br_max(2);0];
eu_2_alpha = [0; -2*pi/360*vehicle.Input.angle_br_min(2);0];

x1_seat = int_boundary_f_new(vehicle.Input.int_height + 1);
x2_seat = int_boundary_r_new(vehicle.Input.int_height + 1);
x1_br = int_boundary_f_new(vehicle.Input.int_height + 1) + vehicle.Input.seat_depth(1);
x2_br = int_boundary_r_new(vehicle.Input.int_height + 1)+vehicle.Input.seat_depth(2);

z1_seat = vehicle.Input.int_height + vehicle.interior.boundary_seat(1) + vehicle.Input.seat_adjustment_z(1);
z2_seat = vehicle.Input.int_height + vehicle.interior.boundary_seat(2) + vehicle.Input.seat_adjustment_z(2);
z1_br = vehicle.Input.int_height + vehicle.interior.boundary_seat(1)+vehicle.Input.thickness_seat(1) + vehicle.Input.seat_adjustment_z(1);
z2_br = vehicle.Input.int_height + vehicle.interior.boundary_seat(2)+vehicle.Input.thickness_seat(2) + vehicle.Input.seat_adjustment_z(2);

x_pos_seat = [x1_seat-vehicle.Input.seat_adjustment_x(1)  x2_seat-vehicle.Input.seat_adjustment_x(2)];
z_pos_seat = [z1_seat  z2_seat];
x_pos_br = [x1_br  x2_br];
z_pos_br = [z1_br  z2_br];

Lz_br = [-vehicle.Input.thickness_br(1)  -vehicle.Input.thickness_br(2)];


end