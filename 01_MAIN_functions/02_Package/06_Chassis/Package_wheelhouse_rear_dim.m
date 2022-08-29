function [v]=Package_wheelhouse_rear_dim(v,Par)
%% Description:
% Designed by: Adrian König (FTM, Technical University of Munich), Lorenzo Nicoletti (FTM, Technical University of Munich)
%-------------
% Created on: 01.05.2022
% ------------
% Version: Matlab2020b
%-------------
% Description:
%This function calculates the the dimensions of the wheelhouse. To do so, it simulates the extreme wheele position. Starting from the normal wheel
%position (here defined as DIN0 position) the wheel with the maximum deflection is simulated to derive the required wheelhouse height and
%width. We also simulate the wheel with the max rebound, as this is needed later to define the position of the shock absorber. We suppose for the wheel
%in DIN0 position a camber angle of 1°, for the wheel in max deflection an angle of 4° and for the wheel in max rebound an
%angle of 0°.
%The first reference system used in this function lays in the XY-plane. This reference system is used to calculate the required space in Y direction in case of
%steering at the rear axle. We suppose that the wheel turns at its middle point (situated at tire_radius/2 and tire_width/2
%The grafic underneath shows an overview of the wheel with the employed the employed point numeration
% |                  ^X direction            1----------------4   
% |                  |                       |                |   
% |                  |                       |                |   
% |                  ------>Y direction      |                |   
% |vehicle center                            |                |   
% |                                          |                |
% |                                          |                |   
% |                                          |                |   
% |                                          |                |   
% |                                          |                |   
% |                                          2----------------3   

%The second system employed in this function lays at the point in the ZY plane, where the wheel can rotate. This point lays at tire_width/2+rim_inset.
%The grafic underneath shows an overview of the wheel with the employed reference siystem as well as the employed point numeration
%|                                                           2----------------3    
%|                                                           |                |    
%|                                                           |                | 
%|                                                           |         ^Z     |
%|                                      axis at wheel center |         |      |
%| vehicle center                        --------------------|         o -->Y |
%|                                                           |                |
%|                                                           |                |
%|                                       --------------------|                |
%|                                        lower axis         |                |
%|                                                           1----------------4
%
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
%2) Calculate X dimensions (wheelhouse length)
%3) Calculate Y dimensions (wheelhouse width)
%4) Calculate Z dimensions (wheelhouse height)
%5) Calculate lower axis position
%6) Plot the results if required
%7) Assign Outputs

%% 1) Initialize The required function variables

%Wheel dimensions in mm
tire_width=v.dimensions.CY.wheel_r_width;                   
rim_inset=v.dimensions.EY.rim_inset_r;                                             
tire_radius=v.dimensions.CX.wheel_r_diameter/2;

%camber angles are converted in RAD
camber_middle=(Par.rear_axle.middle_camber_angle/180)*pi;
camber_def=(Par.rear_axle.deflected_camber_angle/180)*pi;

%Other measures:
axistype=v.topology.axis_type_r;
track_width=v.dimensions.GY.track_width_r; %in mm

%Minimum Offsets between wheel and wheelhouse in mm
EX_wheel_wheelhouse=Par.dimensions.EX.wheel_wheelhouse;
EY_wheel_wheelhouse=Par.dimensions.EY.wheel_wheelhouse;
EZ_wheel_wheelhouse=Par.dimensions.EZ.wheel_wheelhouse;
corr_factor_X=Par.rear_axle.correction_factor_wheelhouse; 

%Max rebound and max deflection in Z-direction in mm:
max_def=v.dimensions.EZ.max_deflection_r;
max_reb=(3*max_def)/4; 

%Steering angle at the rear axle in GRAD
delta_r=v.wheels.steering_angle_r;

%% 2) Calculate X Dimensions (wheelhouse length)

%Length of the wheelhouse at the max deflected wheel: The max deflected wheel is the state of the wheel, which comes the closest to the wheelhouse.
%Therefore the length at the max deflected wheel center, has to have a minimum offset of 20 mm
wheelhouse_length=2*tire_radius + 2*EX_wheel_wheelhouse; %in mm

%Recorrect the length with the derived length factor at the position DIN0
wheelhouse_length=wheelhouse_length+2*corr_factor_X; %in mm

%% 3) Calculate Y Dimensions (wheelhouse width):

%According to the axis type, assign the typical axis lengh, as well as the position of the lower axis which is described by the variables
%CY_wheelcarrier and CZ_wheelcarrier. For torsion_beam bothnvariable are 0, as the axis is koaxial to the wheel center.


%A) Initialize the variables:
switch axistype
    
case 'trapezoidal_link'
        ax_len_lower_axis=Par.dimensions.CY.axis_length_rear.trapezoidal_link;          %[mm]Average after categorisation
        CY_wheelcarrier=Par.dimensions.CY.wheel_carrier.trapezoidal_link;               %[mm]Average of the y difference between the coilaxis bearing and the outside of the wheelcarrier after categoristion 
        ax_len_wheelaxis=ax_len_lower_axis+CY_wheelcarrier;                             %[mm] Axis length, which will be used for the wheelhouse calculation
        CZ_wheelcarrier=Par.dimensions.CZ.wheel_carrier.trapezoidal_link;               %[mm]Average after categorisation

case 'sword_arm_link'
        ax_len_lower_axis=Par.dimensions.CY.axis_length_rear.sword_arm_link;            %[mm]Average after categorisation
        CY_wheelcarrier=Par.dimensions.CY.wheel_carrier.sword_arm_link;                 %[mm]Average of the y difference between the coilaxis bearing and the outside of the wheelcarrier after categoristion 
        ax_len_wheelaxis=ax_len_lower_axis+CY_wheelcarrier;                             %[mm] Axis length, which will be used for the wheelhouse calculation
        CZ_wheelcarrier=Par.dimensions.CZ.wheel_carrier.sword_arm_link;                 %[mm]Average after categorisation

case 'five_link'
        ax_len_lower_axis=Par.dimensions.CY.axis_length_rear.five_link;                 %[mm]Average after categorisation 
        CY_wheelcarrier=Par.dimensions.CY.wheel_carrier.five_link;                      %[mm]Average of the y difference between the coilaxis bearing and the outside of the wheelcarrier after categoristion 
        ax_len_wheelaxis=ax_len_lower_axis+CY_wheelcarrier;                             %[mm] Axis length, which will be used for the wheelhouse calculation
        CZ_wheelcarrier=Par.dimensions.CZ.wheel_carrier.five_link;                      %[mm]Average after categorisation
 
otherwise %torsion_beam
        ax_len_lower_axis=Par.dimensions.CY.axis_length_rear.torsion_beam;              %[mm]Average after categorisation
        ax_len_wheelaxis=track_width/2;                                                 % The value of the torsion beam axis is fictional
        CZ_wheelcarrier=0;                                                              
        CY_wheelcarrier=0;
end


%B) Calculate the displacement caused by the camber effect:
%Position where the lower axis is flanged to the wheel with respect to the origin of the wheel (camber angle still 0 in this state)
pos_lower_axis(1)=-CY_wheelcarrier;                  %in mm
pos_lower_axis(2)=-CZ_wheelcarrier;  %in mm

%Initialize the four points of the wheel in the ZY plane, consider the rim_inset (camber angle still 0° in this state)
y(1)=-(tire_width/2+rim_inset);
y(2)=y(1);
y(3)=tire_width/2-rim_inset;
y(4)=y(3);
z(1)=-tire_radius;
z(2)=tire_radius;
z(3)=z(2);
z(4)=z(1);

%Calculate the repositioning of the points caused by the camber angle (1° for the DIN0 position)
rot_angle=camber_middle;                                                       %in rad
rot_matrix=[cos(rot_angle),-sin(rot_angle); sin(rot_angle),cos(rot_angle)];    
points_wheel_DIN0=rot_matrix*[y;z];                                              %in mm

%Shift the origin of the wheel at the DIN0 position in Z direction for the radius height
points_wheel_DIN0(2,:)=points_wheel_DIN0(2,:)+tire_radius;                            %in mm

%Reposition the lower axis considering the effect of the camber angle (1° for the DIN0 position)
pos_lower_axis_DIN0=rot_matrix*pos_lower_axis';                                %in mm
pos_lower_axis_DIN0(2)=pos_lower_axis_DIN0(2)+tire_radius;                        %in mm

%Calculate the repositioning of the points caused by the camber angle (4° for the max deflected wheel)
rot_angle=camber_def;                                                          %in rad
rot_matrix=[cos(rot_angle),-sin(rot_angle); sin(rot_angle),cos(rot_angle)];
points_wheel_max_defl=rot_matrix*[y;z];                                               %in mm

%Shift the points of the max deflected wheel in Z direction for the radius and the max deflection
points_wheel_max_defl(2,:)=points_wheel_max_defl(2,:)+tire_radius+max_def;                      %in mm

%Reposition the lower axis considering the effect of the camber angle (4° for the max deflected wheel)
pos_lower_axis_max_defl=rot_matrix*pos_lower_axis';                                %in mm
pos_lower_axis_max_defl(2)=pos_lower_axis_max_defl(2)+tire_radius+max_def;                 %in mm


%C) Calculate the displacement caused by the lower axis (the point where the lower axis is flanged has to move on a circle with a radius equal to the axis length).
if strcmp(axistype,'torsion_beam')
    
    %torsion beam axes are rigid and therefore we suppose that there is no displacement in Y direction
    Y_displacement_lower_axis=0;
    pos_lower_axis_max_defl=[0,tire_radius+max_def];
    
else

    %Z-displacement between the lower axis at DIN 0 and at the max deflection
    Z_displacement_lower_axis=abs(pos_lower_axis_max_defl(2)-(pos_lower_axis_DIN0(2))); %in mm

    %Calculate the angle of the lower axis
    angle_lower_axis=asin(Z_displacement_lower_axis/ax_len_lower_axis);                       %in rad

    %Calculate the displacement caused by the camber effect of 4°. Compare it with the wheel with 1° camber angle
    Y_displacement_lower_axis=ax_len_lower_axis-ax_len_lower_axis*cos(angle_lower_axis);      %in mm
    
end

%Shift the wheel with max deflection of an offset equal to the y displacement of the lower axis
points_wheel_max_defl(1,:)=points_wheel_max_defl(1,:)-Y_displacement_lower_axis; %in mm
points_wheel_max_defl(2,:)=points_wheel_max_defl(2,:);                           %in mm

%The extreme point in Y-direction for the max deflected wheel with consideration of the Y-displacement
y_displacement_def=abs(points_wheel_max_defl(1,2));                       %in mm

%Wheelhouse values: For this calculation we have to consider the whole wheel, which means we cannot use the reference system which is located at tire_width/2 + rim_insetet!
wheelhouse_width=y_displacement_def+tire_width/2-rim_inset+EY_wheel_wheelhouse;                                        %in mm

%Calculate the distance between wheelhouse and vehicle center
EY_wheelhouse_vehicle_center=track_width/2-y_displacement_def-EY_wheel_wheelhouse+rim_inset;

% D) Calculate the position for the wheel with the max rebound:
%Points of the wheel in max rebound
points_wheel_max_rebound(1,:)=y;
points_wheel_max_rebound(2,:)=z+tire_radius-max_reb;

%Position of the lower axis with max rebound
pos_lower_axis_max_rebound(1)=pos_lower_axis(1);
pos_lower_axis_max_rebound(2)=pos_lower_axis(2)-max_reb;

if strcmp(axistype,'torsion_beam')
    
    Y_displacement_lower_axis_max_rebound=0;
    
else

    %Z-displacement between the lower axis at DIN 0 and at the max deflection
    Z_displacement_lower_axis_max_rebound=abs(abs(pos_lower_axis_max_rebound(2))-abs((pos_lower_axis_DIN0(2)))); %in mm

    %Calculate the angle of the lower axis
    angle_lower_axis=asin(Z_displacement_lower_axis_max_rebound/ax_len_lower_axis);                       %n rad

    %Calculate the displacement caused by the camber effect of 4°. Compare it with the wheel with 1° camber angle
    Y_displacement_lower_axis_max_rebound=ax_len_lower_axis-ax_len_lower_axis*cos(angle_lower_axis);      %in mm
    
end

%Recalculate the position of the wheel while considering also the axle displacement 
points_wheel_max_rebound(1,:)=points_wheel_max_rebound(1,:)-Y_displacement_lower_axis_max_rebound;

%E) If a steering angle is given at the rear axle, the wheelhouse width needs to be recalculated
%Points describing the wheel in the XY plane (see numeration system underneath)
x(1)=tire_radius;
x(2)=-tire_radius;
x(3)=-tire_radius;
x(4)=tire_radius;
y(1)=-tire_width/2;
y(2)=-tire_width/2;
y(3)=tire_width/2;
y(4)=tire_width/2;
points_wheel_no_rotation=[y;x];    

if delta_r>0 %The vehicle has a steering angle at the rear axle

    %Calculate the repositioning of the points caused by the camber angle (1° for the DIN0 position)
    rot_angle=-deg2rad(delta_r);                                         %in rad
    rot_matrix=[cos(rot_angle),sin(rot_angle); -sin(rot_angle),cos(rot_angle)];  
    points_wheel_max_rotation=rot_matrix*[y;x];                                              %in mm

    %Refer the points to the vehicle center
    points_wheel_no_rotation(1,:)=points_wheel_no_rotation(1,:)+track_width/2;
    points_wheel_max_rotation(1,:)=points_wheel_max_rotation(1,:)+track_width/2;
    
    %As the wheel has a steering angle, the width of the wheelhouse needs to be corrected
    EY_wheelhouse_vehicle_center=abs(points_wheel_max_rotation(1,1))-EY_wheel_wheelhouse;    
    wheelhouse_width_rear_angle_steering=v.dimensions.GY.vehicle_width/2-EY_wheelhouse_vehicle_center;
    
    %Choose for the wheelhouse width the case (between max wheel displacement and max wheel steering) that requires more space
    wheelhouse_width=max(wheelhouse_width,wheelhouse_width_rear_angle_steering);
       
    
else
    points_wheel_max_rotation=[];
%     points_wheel_no_rotation(1,:)=points_wheel_no_rotation(1,:)+track_width/2;
%     EY_wheelhouse_vehicle_center=abs(points_wheel_no_rotation(1,1))-EY_wheel_wheelhouse;
%     wheelhouse_width=v.dimensions.GY.vehicle_width/2-EY_wheelhouse_vehicle_center;
end

%% 4) Calculate Z Dimensions (wheelhouse height):

%The extreme point in Z-direction for the max deflected wheel
Z_max=points_wheel_max_defl(2,3);

%Add at the highest point, the minimum offsset for the skid_chain
wheelhouse_height=Z_max+EZ_wheel_wheelhouse;

%% 5) Calculate Lower axis position
%Calculate back to find the relative position of the axle with respect to the motor, thus deriving the motor space in Y
coilaxis_inner_bearing_y_coordinate=(track_width/2+rim_inset)-ax_len_wheelaxis;
coilaxis_inner_bearing_z_coordinate=-CZ_wheelcarrier;     

%% 6) Plot the results

%Refer the position of the Points to the vehicle center
points_wheel_DIN0(1,:)=points_wheel_DIN0(1,:)+track_width/2+rim_inset;
points_wheel_max_defl(1,:)=points_wheel_max_defl(1,:)+track_width/2+rim_inset;
points_wheel_max_rebound(1,:)=points_wheel_max_rebound(1,:)+track_width/2+rim_inset;

%% 7) Assign the required Outputs:

v.dimensions.CX.wheelhouse_r_length=wheelhouse_length;
v.dimensions.CY.wheelhouse_r_width=wheelhouse_width;
v.dimensions.CZ.wheelhouse_r_height=wheelhouse_height;
v.dimensions.EY.wheelhouse_r_vehicle_center=EY_wheelhouse_vehicle_center;
v.dimensions.EY.axis_vehicle_center=coilaxis_inner_bearing_y_coordinate;
v.dimensions.EZ.axis_wheel_center=coilaxis_inner_bearing_z_coordinate;
v.dimensions.CY.axis_length_r=ax_len_lower_axis;

v.dimensions.EZ.lower_axis_r_wheelcenter=CZ_wheelcarrier; %Needed for positioning the shock absorber
v.dimensions.EY.lower_axis_r_wheelcenter=CY_wheelcarrier;

%Assign the wheel points
v.wheels.points_wheel_max_rebound_yz=points_wheel_max_rebound;
v.wheels.points_wheel_DIN0_yz=points_wheel_DIN0;
v.wheels.points_wheel_max_deflection_yz=points_wheel_max_defl;
v.wheels.points_wheel_r_no_rotation=points_wheel_no_rotation;
v.wheels.points_wheel_r_max_rotation=points_wheel_max_rotation;

%Assign the Y_displacement required later for the plor:
v.dimensions.EY.axis_displacement_max_rebound_r=Y_displacement_lower_axis;

%Angle of the wheel in the maximum deflected position (required for the plotting)
v.wheels.camber_angle_max_defl_r=camber_def;
end