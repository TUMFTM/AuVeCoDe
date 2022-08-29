function DISPLAY_vehicle(vehicle,Parameters,axis)
%% Description:
% Designed by: Adrian König (FTM, Technical University of Munich)
%-------------
% Created on: 18.11.2021
% ------------
% Version: Matlab2020b
%-------------
% Description: This function displays the 3D components of the vehicle
% ------------
% Sources:  [1] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
%           [2] Object3d_Block(p,eu,Lx,Ly,Lz): https://www.sky-engin.jp/MATLABAnimation/chap06/chap06.html
%           [3] Object3d_Cylinder(p,eu,r,h,sc): https://www.sky-engin.jp/MATLABAnimation/chap08/chap08.html
% ------------
% Input:    - vehicle: struct with includes the data of the vehicle
%           - Parameters: struct with input and constant values
%           - axis: Figure where vehicle is plotted (Created in Startfile)
% ------------
% Output:   - Plot with vehicle package
% ------------

%% Implementation
% 1) Plot interior and powertrain boundaries
% 2) Plot manikins and seats
% 3) Plot wheelhouse and wheels
% 4) Plot powertrain
% 5) Plot battery 
% 6) Plot body and trunk
% 7) Plot miscellaneous parts
% 8) Plot Exterior
% 9) Plot settings

%% Initialize figure
cla(axis) % clear current plot
hold(axis); % hold to plot everything in the same diagram

%% 1) Plot interior and powertrain boundaries
PlotBoundaries(vehicle,axis)

%% Optional plot of points used for the cabin calculation needed for HVAC
%CALCULATE_Cabin(vehicle,Parameters)
%% 2) Plot manikins and seats
% 2a) Manikins
if Parameters.settings.plot_manikin % plot manikins if option is selected
    PlotHuman(vehicle,Parameters)
end

% 2b) Interior seats 
PlotInterior(vehicle,axis);

%% 3) Plot wheelhouse and wheels
%Only plot wheel envelope if selected
wheel_envelope=Parameters.settings.plot_wheel_env;
PlotWheelhouse(axis,vehicle,wheel_envelope)
if Parameters.settings.plot_wheel_env
    PlotWheelsDeflected(vehicle);
end
%Plot shock absorber rear
PlotShockAbsorberRear(vehicle)

%% 4) Plot powertrain
PlotPowertrain(axis,vehicle)

%% 5) Plot battery
PlotBattery_housing(axis,vehicle,Parameters)
% Optional detailed plot of battery cells, cover and cooling
if Parameters.settings.plot_battery==1    
    PlotBattery_space(vehicle);
    PlotBattery(vehicle);
end

%% 6) Plot body and trunk
PlotBody(axis,vehicle)
PlotTrunk(axis,vehicle)

%% 7) Plot miscellaneous parts (bumper, cooler, ...)
PlotMisc(axis,vehicle)

%% 8) Plot Exterior 
%!!!!!!!!!!!!!!!!TEMPORARY - UNDER DEVELOPMENT!!!!!!!!!!!!!!!!!!!
%Calculate exterior boundaries
overhang_f      =   vehicle.dimensions.GX.vehicle_overhang_f;
overhang_r      =   vehicle.dimensions.GX.vehicle_overhang_r;
wheelbase       =   vehicle.dimensions.GX.wheelbase;
middle          =   vehicle.dimensions.GY.vehicle_width/2;

pos_front       =   -overhang_f;                 %Most front point of car
pos_rear        =   wheelbase+overhang_r;    %Most rear point of car

plot3(pos_front, middle, 400,'-o','Color','b','MarkerSize',5,'MarkerFaceColor','#D9FFFF')
plot3(pos_rear, middle, 400,'-o','Color','b','MarkerSize',5,'MarkerFaceColor','#D9FFFF')

%% 9) Plot settings
xlabel(axis,'x');
ylabel(axis,'y');
zlabel(axis,'z');
axis.View=[-40, 20];
axis.XLim=[-1500, vehicle.dimensions.GX.vehicle_length];
axis.YLim=[-300, vehicle.dimensions.GY.vehicle_width+300];
axis.ZLim=[0, vehicle.dimensions.GZ.vehicle_height];
axis.XGrid='off';
axis.YGrid='off';
axis.ZGrid='off';
axis.XMinorGrid='off';
axis.YMinorGrid='off';
axis.ZMinorGrid='off';
axis.DataAspectRatio = [1 1 1];
hold(axis,'off');

end
