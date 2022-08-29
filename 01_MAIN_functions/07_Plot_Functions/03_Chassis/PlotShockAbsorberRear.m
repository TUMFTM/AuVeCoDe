function PlotShockAbsorberRear(v)
%% Description:
% Designed by: Adrian König (FTM, Technical University of Munich), Lorenzo Nicoletti (FTM, Technical University of Munich)
%-------------
% Created on: 01.04.2022
% ------------
% Version: Matlab2020b
%-------------
% Description: This function plots the shock absorber of the rear axle
% ------------
% Sources:      [1] Lorenzo Nicoletti, "Parametric Modeling of Battery Electric Vehicles in the Early Development Phase", Technical University of Munich, Institute of Automotive Technology, 2022
%               [2] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - vehicle: struct with includes the data of the vehicle
% ------------
% Output:   - Plot of shock absorber
% ------------

%% Implementation
%1) Read parameters
%2) Plot cylinders

%% 1) Read parameters
%Wheelbase of the vehicle in mm
wb=v.dimensions.GX.wheelbase;

%Assign the position of the lower and upper point of the shock absorber axle (in mm)
low_sa_x=v.dimensions.EX.lower_point_shock_absorber_front_axle;
upp_sa_x=v.dimensions.EX.upper_point_shock_absorber_front_axle;
low_sa_y=v.dimensions.EY.lower_point_shock_absorber_vehicle_center;
upp_sa_y=v.dimensions.EY.upper_point_shock_absorber_vehicle_center;
low_sa_z=v.dimensions.EZ.lower_point_shock_absorber_front_axle;
upp_sa_z=v.dimensions.EZ.upper_point_shock_absorber_front_axle;

%Inclination angle of the shock absorber (in GRAD)
angle_sa=v.wheels.angle_shock_absorber_r;

%Further dimensions of the shock absorber in mm
CZ_height_upper_sa=v.dimensions.CZ.height_upper_bearing_shock_absorber;     %Height of the upper part of the shock absorber
CX_diameter_upper_sa=v.dimensions.CX.upper_bearing_diameter_shock_absorber; %Diameter of the upper part of the shock absorber
CX_piston_diameter=v.dimensions.CX.piston_diameter;                         %Piston diameter


%% 2) Plot cylinders:
%Plot the upper part of the shock absorber, position it and rotate it for the inclination angle of the shock absorber
[x,y,z]=cylinder(CX_diameter_upper_sa/2,1000);
z=z*CZ_height_upper_sa+upp_sa_z-CZ_height_upper_sa;
y=y+upp_sa_y;
x=x+upp_sa_x;
width=v.dimensions.GY.vehicle_width; %Width needed for movement because of different y=0 in AuVeCoDe
cyl_upp=surface(x,y+0.5*width,z);
rotate(cyl_upp,[1,0,0],angle_sa,[wb,upp_sa_y+0.5*width,upp_sa_z]);

%Plot the lower part of the shock absorber, position it and rotate it for the inclination angle of the shock absorber
[x2,y2,z2]=cylinder(CX_piston_diameter/2,1000);
z2=z2*(upp_sa_z-low_sa_z)+low_sa_z;
y2=y2+low_sa_y;
x2=x2+low_sa_x;
cyl_lower=surface(x2,y2+0.5*width,z2);
rotate(cyl_lower,[1,0,0],angle_sa,[wb,low_sa_y+0.5*width,low_sa_z]);

%Plot the shock absorber on the other side (mirror the first shock absorber)
cyl_otherside_upper=surface(x,0.5*width-y,z);
cyl_otherside_lower=surface(x2,0.5*width-y2,z2);

rotate(cyl_otherside_upper,[1,0,0],-angle_sa,[wb,0.5*width-upp_sa_y,upp_sa_z]);
rotate(cyl_otherside_lower,[1,0,0],-angle_sa,[wb,0.5*width-low_sa_y,low_sa_z]);

end

