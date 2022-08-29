function [veh] = Package_wheelhouse_front_dim(veh, Par)
%% Description:
% Designed by: Adrian König (FTM, Technical University of Munich), Lorenzo Nicoletti (FTM, Technical University of Munich)
%-------------
% Created on: 01.05.2022
% ------------
% Version: Matlab2020b
%-------------
% Description:  This function calculates the axle distribution for different cases.
%               The function calculates the load according to the 88% and 100% rule
%               The calculated loads will be used in the following function to dimension
%               the wheels
%               The grafic underneath shows an overview of the wheel with the employed the employed point numeration
%
%               |                  ^X direction            1----------------4   
%               |                  |                       |                |   
%               |                  |                       |                |   
%               |                  ------>Y direction      |                |   
%               |vehicle center                            |                |   
%               |                                          |                |
%               |                                          |                |   
%               |                                          |                |   
%               |                                          |                |   
%               |                                          |                |   
%               |                                          2----------------3   
% ------------
% Sources:  [1] Lorenzo Nicoletti, "Parametric Modeling of Battery Electric Vehicles in the Early Development Phase", Technical University of Munich, Institute of Automotive Technology, 2022
%           [2] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - Parameters (Par): struct with input and constant values
%           - vehicle(v): struct with data of the vehicle
% ------------
% Output:   - vehicle(v): struct with data of the vehicle

%% Implementation
%1) Local input declaration
%2) Calculate steering front and rear
%3) Calculate wheelhouse dimensions
%4) Calculate the wheelhouse as point cloud -> Calculate wheelhouse shape
%5) Assign Outputs

%% 1) Local input declaration

%Inputs for the calculation of the steering angle/angles
wb=veh.dimensions.GX.wheelbase;
t_f=veh.dimensions.GY.track_width_f;
o_f=veh.dimensions.GX.vehicle_overhang_f;

%Inputs for the calculation of the wheel volume
d_f=veh.dimensions.CX.wheel_f_diameter;
w_f=veh.dimensions.CY.wheel_f_width;

%Wheelhouse position
EX_wheel_wheelhouse=Par.dimensions.EX.wheel_wheelhouse;         %Min offset between wheel and wheelhouse in mm (measured at the max deflected position of the wheelhose)
corr_factor_X=Par.rear_axle.correction_factor_wheelhouse;       %Correction factor to recualculate the wheelouse lenght at the non deflected height of the wheel in mm
EY_wheel_wheelhouse=Par.dimensions.EY.wheel_wheelhouse;         %Min offset between wheel and wheelhouse in mm (measured at the max deflected position of the wheelhose)
CY_wheelhouse_thickness=Par.dimensions.CY.wheelhouse_thickness; %Wheelhouse thickness in mm  

%Read turning diameter
R_tc = veh.Input.steering_radius*1000;  %mm
delta_f_i_deg_max = 36; %max. possible steering angle for one axle


%% 2) Calculate steering angles
   delta_f_x_deg=delta_f_i_deg_max;
   delta_r_x_deg=ceil(delta_f_x_deg*(1-veh.Input.steering_ratio)/veh.Input.steering_ratio);

   delta_f=linspace(0,delta_f_x_deg);
   delta_r=linspace(0,delta_r_x_deg);
   for i=1:numel(delta_r)
       
       %Auxiliary parameters to make the following equation easier to write:
       c1=wb*tand(delta_f(i))/(tand(delta_f(i))+tand(delta_r(i)));
       R1=c1/tand(delta_f(i));
       
       %New resulting turning radius (with the given i-nte rear steering angle)
       R_tc_new(i)=sqrt(R1^2+(c1+o_f)^2);
       
   end
   
   %Find the steering angle, which allows a turning radius as close as possible to the initial turning radius
   [~,k]=min(abs(R_tc-R_tc_new));
    
   %Assign the value of steering angle (in deg)
   delta_r_i_deg = delta_r(k);
   delta_f_i_deg = delta_f(k);
   

%% 4) Calculate wheelhouse dimensions-> THIS PART HAS TO BE KEPT BEFORE THE UPDATE OF ANDREA

%Length of the wheelhouse at the max deflected wheel: The max deflected wheel is the state of the wheel, which comes the closest to the wheelhouse.
%Therefore the length at the max deflected wheel center, has to have a minimum offset of 20 mm
CX_wheelhouse_f=d_f+ 2*EX_wheel_wheelhouse; %in mm

%Recorrect the length with the derived length factor at the position DIN0
CX_wheelhouse_f=CX_wheelhouse_f+2*corr_factor_X; %in mm

%Wheelhouse dimensions in Y direction, depending from the maximum steering angle (front and rear)
CY_wheelhouse_f = (d_f/2)*sind(delta_f_i_deg)+(w_f/2)*cosd(delta_f_i_deg)+w_f/2+EY_wheel_wheelhouse+CY_wheelhouse_thickness;

%Refer position of the wheelhouse to the center of the vehicle
EY_wheelhouse_vehicle_center=veh.dimensions.GY.vehicle_width/2-CY_wheelhouse_f;

%% 5) Calculate the wheelhouse as point cloud -> Calculate wheelhouse shape
%Calculate the repositioning of the points due to the steering angle
%Points describing the wheel in the XY plane (see numeration system above)
x(1)=d_f/2;
x(2)=-d_f/2;
x(3)=-d_f/2;
x(4)=d_f/2;
y(1)=-w_f/2;
y(2)=-w_f/2;
y(3)=w_f/2;
y(4)=w_f/2;

%The point 2 is the closest to the battery, therefore the critical point.
%For the Y direction also consider the wheelhouse thickness
%For the X direction the thickness is already contained in CX_wheelhouse_f
basis_point(2)=x(2)+10; %Add 10 mm minimum distance between wheel and wheelhouse
basis_point(1)=y(2)-10; %Add 10 mm minimum distance between wheel and wheelhouse

%Define a discretized vector from 0 to the max rotation angle
rot_angles=linspace(0,deg2rad(delta_f_i_deg));

%Calculate the travel path of the point for a rotation until delta_i_max
for i=1:numel(rot_angles)
    
    rot_matrix=[cos(rot_angles(i)),sin(rot_angles(i)); -sin(rot_angles(i)),cos(rot_angles(i))];
    point_path(:,i)=rot_matrix*basis_point';
end

%Shift the travel path in the correct coordinates in X
delta_x=-CX_wheelhouse_f*0.5-point_path(2,1);
point_path(2,:)=point_path(2,:)+delta_x;

%Shift the travel path in the correct coordinates in Y
point_path(1,:)=point_path(1,:)+t_f/2;
delta_y=EY_wheelhouse_vehicle_center-min(point_path(1,:));
point_path(1,:)=point_path(1,:)+delta_y;

%Calculate the other two segements of the wheelhouse
y_max=max(point_path(1,:));
segment_1(1,:)=linspace(t_f/2,y_max);
segment_1(2,:)=-ones(1,100)*CX_wheelhouse_f*0.5;

y_min=min(point_path(1,:)); 
segment_2(1,:)=ones(1,100)*y_min;
segment_2(2,:)=linspace(max(point_path(2,:)),0);

%Unite the segments and add the Z coordinate
wheelhouse_f(1,:)=-[segment_1(2,:),point_path(2,:),segment_2(2,:)];
wheelhouse_f(2,:)=[segment_1(1,:),point_path(1,:),segment_2(1,:)];
wheelhouse_f(3,:)=ones(1,size([segment_1,point_path,segment_2],2))*d_f/2;

%% 5) Assign Outputs

veh.wheels.steering_angle_f=delta_f_i_deg;
veh.wheels.steering_angle_r=delta_r_i_deg;

%assign dimensions of the wheelhouse (The heigth will be taken from the wheelhouse rear)
veh.dimensions.CX.wheelhouse_f_length=CX_wheelhouse_f;
veh.dimensions.CY.wheelhouse_f_width=CY_wheelhouse_f;
veh.dimensions.CY_wheelhouse_thickness=CY_wheelhouse_thickness;

%Position the wheelhose with respect to the front axle
veh.dimensions.EY.wheelhouse_f_vehicle_center=EY_wheelhouse_vehicle_center-CY_wheelhouse_thickness;
veh.dimensions.EX.wheelhouse_f_front_axle=0;

%Shape of the front wheelhouse (calculated at point 5) -> Expressed as pointcloud
veh.wheels.wheelhouse_f=wheelhouse_f;

end

