function PlotWheelsDeflected(vehicle)
%% Description:
% Designed by: Adrian König (FTM, Technical University of Munich)
%-------------
% Created on: 01.04.2022
% ------------
% Version: Matlab2020b
%-------------
% Description: This function plots the deflected wheels
% ------------
% Sources:      [1] Lorenzo Nicoletti, "Parametric Modeling of Battery Electric Vehicles in the Early Development Phase", Technical University of Munich, Institute of Automotive Technology, 2022
%               [2] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - vehicle: struct with includes the data of the vehicle
% ------------
% Output:   - Plot of deflected wheels
% ------------


%% Implementation:
%1) Initialize the required parameter:
%2) Plot the front wheels
%3) Plot the rear wheels
%4) Assign the dataset to the vehicle struct.
%5) Subfunctions

%% 1) Initialize the required parameter:
t_r=vehicle.dimensions.GY.track_width_r;
w_r=vehicle.dimensions.CY.wheel_r_width;
d_r=vehicle.dimensions.CX.wheel_r_diameter;

y_displacement=vehicle.dimensions.EY.axis_displacement_max_rebound_r;
tire_inset=vehicle.dimensions.EY.rim_inset_r;
wb=vehicle.dimensions.GX.wheelbase;
width=vehicle.dimensions.GY.vehicle_width;

[x,z,y]=cylinder(d_r/2,1000);
y(1,:)=+(w_r/2-tire_inset);
y(2,:)=-(w_r/2+tire_inset);

camber_angle = vehicle.wheels.camber_angle_max_defl_r/180*pi;

[y_r,z_r]=rotate_in_X(camber_angle,x,y,z);

z=z+d_r/2;

z_r=z_r+vehicle.dimensions.EZ.max_deflection_r+d_r/2;
y_r=y_r+t_r/2+tire_inset-y_displacement;
x=x+wb;

%Change of y position
y_r_left=width/2-y_r;
y_r_right=width/2+y_r;


hold on

plot_wheel_3d(x,y_r_left,z_r,'k')

plot_wheel_3d(x,y_r_right,z_r,'k')

end

function [y_r,z_r]=rotate_in_X(rot_angle,x,y,z)
%% Description:
%This function rotates the wheel points on the Z-axis using a roration
%matrix. Only x_r and y_r are given as outputs, as the z-coordinate do not
%change with a rotation on the Z_axle.

%% Inputs:
%rot_angle: rotation angle;
%x,y,z: vectors with the point coordinates in x,y and z direction

Rotmatrix=[1,0,0; 0, cos(rot_angle), -sin(rot_angle); 0,sin(rot_angle),cos(rot_angle)];

A=Rotmatrix*[x(1,:);y(1,:);z(1,:)];
B=Rotmatrix*[x(2,:);y(2,:);z(2,:)];

x_r(1,:)=A(1,:);
y_r(1,:)=A(2,:);
z_r(1,:)=A(3,:);

x_r(2,:)=B(1,:);
y_r(2,:)=B(2,:);
z_r(2,:)=B(3,:);

end

function plot_wheel_3d(x,y,z,Color)
%% Description:
%This function simply plots the wheel

%Plot the wheels and close the surfaces of the non rotated wheel
surf(x,y,z,'FaceColor',Color,'EdgeColor','none','FaceAlpha',0.3)
fill3(x(1,:),y(1,:),z(1,:),Color,'FaceAlpha',0.3)
fill3(x(2,:),y(2,:),z(2,:),Color,'FaceAlpha',0.3)

end

