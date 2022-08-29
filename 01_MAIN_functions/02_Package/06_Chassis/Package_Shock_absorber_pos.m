function [vehicle] = Package_Shock_absorber_pos(vehicle)
%% Description:
% Designed by: Adrian König (FTM, Technical University of Munich), Lorenzo Nicoletti (FTM, Technical University of Munich)
%-------------
% Created on: 01.05.2022
% ------------
% Version: Matlab2020b
%-------------
% Description:  This function calculates the positioning of the shock absorber. The upper point of the shock absorber is placed as closed as possible to the
%               wheelhouse (in Y-direction).The lower point of the shock absorber is also placed as close as possible with the wheel. 

%               The reference system of this function has following characteristics:
%               X0: located at the front axle
%               Y0: located to the vehicle center
%               Z0: located at the height of the wheel center
% ------------
% Sources:  [1] M. Spreng, „Maßkettenanalyse am Hinterwagen zur Erstellung von Ersatzmodellen,“Bachelor thesis, Faculty of Mechanical Engineering, Ostbayerische Technische Hochschule Regensburg, Regensburg, 2020.
%           [2] Lorenzo Nicoletti, "Parametric Modeling of Battery Electric Vehicles in the Early Development Phase", Technical University of Munich, Institute of Automotive Technology, 2022
%           [3] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - Parameters (Par): struct with input and constant values
%           - vehicle(v): struct with data of the vehicle
% ------------
% Output:   - vehicle(v): struct with data of the vehicle
% ------------

%% Implementation
%1) Initialize the required variables
%2) Position upper part shock absorber
%3) Position lower part of the shock absorber
%4) Iterate to correct the position of the upper part
%5) Check if there is a contact between the shock absorber and the complete rebounded wheel
%6) Assign Outputs

%% 1) Initialize the required variables:

%topology of the vehicle, required to shift the shock absorber in X direction in case the rear axle has a machine
topology=vehicle.topology.drive;

%Wheel dimensions and wheelbase
tire_side_wall=vehicle.wheels.tire_side_wall;                       %in mm
tire_radius=vehicle.dimensions.CX.wheel_r_diameter/2;               %in mm
wheelbase=vehicle.dimensions.GX.wheelbase;                          %in mm

%Axis Type and ground clearance type
ground_clearance=vehicle.topology.ground_clearance;      
axis_type=vehicle.topology.axis_type_r;

%shock absorber dimensions in mm
upper_bearing_diameter_shock_absorber=vehicle.dimensions.CX.upper_bearing_diameter_shock_absorber;
height_upper_bearing_shock_absorber=vehicle.dimensions.CZ.height_upper_bearing_shock_absorber;
piston_diameter=vehicle.dimensions.CX.piston_diameter;

%Wheelhouse dimensions in mm
EY_wheelhouse_vehicle_center=vehicle.dimensions.EY.wheelhouse_r_vehicle_center;

%Wheelhouse height refered to the position of the rear axle
wheelhouse_height=vehicle.dimensions.CZ.wheelhouse_r_height-tire_radius;

%Points describing the position of the DIN0, max deflection and max rebound wheel
points_wheel_DIN0=vehicle.wheels.points_wheel_DIN0_yz;
points_wheel_max_deflection=vehicle.wheels.points_wheel_max_deflection_yz;
points_wheel_max_rebound=vehicle.wheels.points_wheel_max_rebound_yz;

%Values required to position the lower axis in mm
CY_wheelcarrier=vehicle.dimensions.EY.lower_axis_r_wheelcenter;
distance_z_wheelmiddle_bearing=vehicle.dimensions.EZ.lower_axis_r_wheelcenter; 

%Test the rest
track_width=vehicle.dimensions.GY.track_width_r;
inset=vehicle.dimensions.EY.rim_inset_r;

%% 2) Position upper part shock absorber:
%The angle of the shock absorber is not known so for the first positioning we suppose an angle of 0°

switch ground_clearance         % categorates the z-coordinate of the upper shock absorber bearing

 case 'highfloor'
    % Z-coordinate of the upper shock absorber point is at the wheel radius height but only by high-floor vehicles    
    upper_point_sa_z=tire_radius;                                                                   

 otherwise
    %Z-coordinate of the upper shock absorber point is at the height of the z-coordinate wheelhouse but only by low-floor vehicles 
    upper_point_sa_z=wheelhouse_height;        

end

upper_point_sa_y=EY_wheelhouse_vehicle_center-(upper_bearing_diameter_shock_absorber/2);  

%% 3) Position lower part of the shock absorber

switch axis_type

     case 'trapezoidal_link'
         EY_sa_wheelcarrier=72;       %[mm]Average after categorisation

     case 'sword_arm_link'
         EY_sa_wheelcarrier=89.24;    %[mm]Average after categorisation

     case 'five_link'
         EY_sa_wheelcarrier=78.22;    %[mm]Average after categorisation

end 

if strcmp(axis_type,'torsion_beam')
    
    %torsion_beam normally position the shock absorber vertically, i.e. with a 0° angle -> upper and lower point have the same Y-position!
    lower_point_sa_z=0;
    lower_point_sa_y=upper_point_sa_y;
else
    
    lower_point_sa_z=-distance_z_wheelmiddle_bearing;                               % [mm]calculates the z-coordinate of the lower shock absorber bearing(reference DIN0)
    lower_point_sa_y=(track_width/2)+inset-CY_wheelcarrier-EY_sa_wheelcarrier;      % [mm]calculates the y-coordinate of the lower shock absorger bearing(reference DIN0)
end

%The upper point cannot have an y value higher than the lower point: if that is the case set the shock absorber to an angle of 0°
upper_point_sa_y=min(upper_point_sa_y,lower_point_sa_y);

%% 4) Iterate to correct the position of the upper part
%Having positioned the lower part, the shock absorber angle can be derived. Therefore the upper point (which was initially positioned using an inclination angle of 0°)
%has to be shifted in order to avoid collision with the wheelhouse

delta=1;
i=1;
pos_y(1)=upper_point_sa_y;

% Iterate to find the correct position
if upper_point_sa_y==lower_point_sa_y
    
    angle_shock_absorber=0;
    
else
    
    while delta>0.01

        i=i+1;

        %Calculate the shock absorber angle
        angle_shock_absorber=atan(abs(lower_point_sa_y-upper_point_sa_y)/(upper_point_sa_z-lower_point_sa_z));               % [rad] Angle between the shock absorber and the XZ Level 

        %Reposition the shock upper point using the calculated angle
        pos_y(i)=EY_wheelhouse_vehicle_center-(upper_bearing_diameter_shock_absorber/2)*cos(angle_shock_absorber)-(height_upper_bearing_shock_absorber)*sin(angle_shock_absorber);

        %Test the delta y from the position of the previous loop
        delta=abs(pos_y(i)-pos_y(i-1));

    end

    % corrected y-coordinate of the upper_bearing_shockabsorber
    upper_point_sa_y=pos_y(end)-27.1;          

    %Angle needs to be recorrected too after the shifting!
    angle_shock_absorber=atan(abs(lower_point_sa_y-upper_point_sa_y)/(upper_point_sa_z-lower_point_sa_z));

end
%% 5) Check if there is a contact between the shock absorber and the complete rebounded wheel

%Load the reference point of the wheels in DIN0, max deflection and max rebound and reposition them according to the employed reference system:
points_wheel_DIN0(2,:)=points_wheel_DIN0(2,:)-tire_radius;
points_wheel_max_rebound(2,:)=points_wheel_max_rebound(2,:)-tire_radius;
points_wheel_max_deflection(2,:)=points_wheel_max_deflection(2,:)-tire_radius;

%Find the Y and Z coordinates of the critical point of the max rebounded wheel. This point shall not collide with the shock absorber!
critical_point_rebound_y=points_wheel_max_rebound(1,1);
critical_point_rebound_z=points_wheel_max_rebound(2,2)-tire_side_wall;

%Calculate the Y coordinate of the outer surface of the shock absorber at the height of the critical point of the max rebounded wheel
outer_lower_point_y=lower_point_sa_y+(piston_diameter/2)*cos(angle_shock_absorber);
outer_critc_point_y=lower_point_sa_y-critical_point_rebound_z*tan(angle_shock_absorber)+(piston_diameter/2)*cos(angle_shock_absorber);

%Check for collision
if outer_critc_point_y>critical_point_rebound_y
    
    %If there is collision shift the shock absorber in Y direction
    vehicle = errorlog(vehicle,'The given rim diameter is not sufficient to fit the brake mechanism, therefore it will be increased');
    offset=outer_critc_point_y-critical_point_rebound_y;
    upper_point_sa_y=upper_point_sa_y-offset;
    lower_point_sa_y=lower_point_sa_y-offset;
    
end

%% 6) Calculate the X position, depending on the topology
%Idea: If there is a machine on the rear axle, the X position of the shock
%absorber has to be changed, to avoid collision with the side shaft of the machine.
%The damper can be shifted behind or in front of the driveshaft, this can vary depending on the axle type and the manufacturer 

if ~strcmp(topology,'FWD') %There is a machine on the rear axle
    
    %Estimate a driveshaft radius of 30 mm, which should be above any possible riveshaft radius
    driveshaft_radius=30;

    %position the shock absorber 20 mm far away to the driveshaft -> This ensures there is no collision between driveshaft and shock absorber
    lower_point_sa_x=wheelbase-driveshaft_radius-piston_diameter/2-20;
    upper_point_sa_x=wheelbase-driveshaft_radius-piston_diameter/2-20;
    
else
    
    %Position the driveshaft at the very center of the rear axle:
    lower_point_sa_x=wheelbase;
    upper_point_sa_x=wheelbase;
end


%% 6) Assign the Outputs

%Position lower point (middle axis of the shock absorber) -> refer the z position to the ground
vehicle.dimensions.EX.lower_point_shock_absorber_front_axle=lower_point_sa_x;
vehicle.dimensions.EY.lower_point_shock_absorber_vehicle_center=lower_point_sa_y;
vehicle.dimensions.EZ.lower_point_shock_absorber_front_axle=lower_point_sa_z+tire_radius;

%Position upper point (middle axis of the shock absorber)-> refer the z position to the ground
vehicle.dimensions.EX.upper_point_shock_absorber_front_axle=upper_point_sa_x;
vehicle.dimensions.EY.upper_point_shock_absorber_vehicle_center=upper_point_sa_y;
vehicle.dimensions.EZ.upper_point_shock_absorber_front_axle=upper_point_sa_z+tire_radius;

%Inclination angle of the shock absorber (in DEG)
vehicle.wheels.angle_shock_absorber_r=rad2deg(angle_shock_absorber);

end

