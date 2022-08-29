function []=Plot_Layer_Calculate(body,boundary,boundary_temp_1_front,boundary_temp_1_back,boundary_temp_2_back,boundary_temp_2_front,...
        boundary_temp_3_back,boundary_temp_3_front,boundary_temp_4_back,boundary_temp_4_front,...
        int_a2_x,int_a2_z,int_a3_f_x,int_a3_f_z,int_a3_r_x,int_a3_r_z,int_a4_x,int_a4_z)
%% Description:
% Designed by: Adrian König (FTM, Technical University of Munich)
%-------------
% Created on: 18.11.2021
% ------------
% Version: Matlab2020b
%-------------
% Description: OPTIONAL FUNCTION: Plots the layers needed for the boundary surfaces
% ------------
% Sources:  [1] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - Data from Package_Layer_Calculate, see Line 136
% ------------
% Output:   - Plotted Layers
% ------------

% Legend: 1=motor, 2=upper circle shape of wheelhouse, 3=rectangle shape of wheelhouse, 4=lower circle shape of wheelhouse
figure
hold on
plot(int_a3_r_x,int_a3_r_z,'.')
text(int_a3_r_x,int_a3_r_z,'rectangle_r','VerticalAlignment','bottom','HorizontalAlignment','right')
plot(int_a2_x,int_a2_z,'.')
text(int_a2_x,int_a2_z,'upper_circle','VerticalAlignment','bottom','HorizontalAlignment','right')
plot(int_a4_x,int_a4_z,'.')
text(int_a4_x,int_a4_z,'lower_circle','VerticalAlignment','bottom','HorizontalAlignment','right')
plot(int_a3_f_x,int_a3_f_z,'.')
text(int_a3_f_x,int_a3_f_z,'rectangle_f','VerticalAlignment','bottom','HorizontalAlignment','right')
plot(body(1,1),body(1,3),'.')
text(body(1,1),body(1,3),'motor','VerticalAlignment','bottom','HorizontalAlignment','right')

plot(boundary_temp_1_back(:,1),(1:1:length(boundary_temp_1_back)))
plot(boundary_temp_1_front(:,2),(1:1:length(boundary_temp_1_front)))
plot(boundary_temp_2_back(:,1),(1:1:length(boundary_temp_2_back)))
plot(boundary_temp_2_front(:,2),(1:1:length(boundary_temp_2_front)))
plot(boundary_temp_3_back(:,1),(1:1:length(boundary_temp_3_back)))
plot(boundary_temp_3_front(:,2),(1:1:length(boundary_temp_3_front)))
plot(boundary_temp_4_back(:,1),(1:1:length(boundary_temp_4_back)))
plot(boundary_temp_4_front(:,2),(1:1:length(boundary_temp_4_front)))
hold off
figure
hold on
plot(boundary(:,1),(1:1:length(boundary(:,1))))
plot(boundary(:,2),(1:1:length(boundary(:,2))))
hold off

end