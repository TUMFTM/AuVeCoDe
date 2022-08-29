function [veh] = CALCULATE_Cabin(veh,Par)
%% Description:
% Designed by: Adrian König (FTM, Technical University of Munich)
%-------------
% Created on: 14.02.2022
% ------------
% Version: Matlab2020b
%-------------
% Description: Calculates needed cabin measurement (angle, surface area, volume) for HVAC
% calculation
% ------------
% Sources:  [1] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - vehicle: struct with vehicle measurements
% ------------
% Output:   - vehicle: struct with measurements of cabin
% ------------

%% Explanation
%By example of left side
%       ws_lt x--------------------------x   rw_lt
%            /                            \
%           /                              \
%          /                                \
%   ws_lb x----x pl_tf          pl_tr x------x rw_lb      <= Driving direction
%               \                    /
%                \                  /
%                 \                /
%           pl_bf  x--------------x  pl_br               

%% Implementation
% 1) Read in needed points
% 2) Calculate the surfaces
%   a) Lower surface
%   b) Upper surface
%   c) Windshield
%   d) Rear Window area
%   e) Roof
%   f) Ground
% 3) Caclculate total surface area of cabin
% 4) Write parameters into vehicle struct

%% 1) Read in needed points
%ws=windshield rw=rear window / _ / l=left,r=right / b=bottom,t=top /f=front,r=rear
%Read points from windshield
ws_lb=veh.body.p_pane_f(1,:);
ws_rb=veh.body.p_pane_f(4,:);
ws_lt=veh.body.p_pane_f(2,:);
ws_rt=veh.body.p_pane_f(3,:);

%Read points of rear window
rw_lb=veh.body.p_pane_r(1,:);
rw_rb=veh.body.p_pane_r(4,:);
rw_lt=veh.body.p_pane_r(2,:);
rw_rt=veh.body.p_pane_r(3,:);

%Read points of platform
%p=platform / l=left,r=right / _ / b=bottom,t=top / f=front,r=rear
%lower points
pl_bf=[veh.body.A_ori(2,1)+sin(veh.body.A_ang(2,1))*veh.body.A_dim(2,3),veh.body.A_ori(2,2),veh.body.A_ori(2,3)+cos(veh.body.A_ang(2,1))*veh.body.A_dim(2,3)];
pl_br=[veh.body.A_ori_re(2,1)+sin(veh.body.A_ang_re(2,1))*veh.body.A_dim_re(2,3),veh.body.A_ori_re(2,2),veh.body.A_ori_re(2,3)+cos(veh.body.A_ang_re(2,1))*veh.body.A_dim_re(2,3)];
pr_bf=[veh.body.A_ori_R(2,1)+sin(veh.body.A_ang_R(2,1))*veh.body.A_dim_R(2,3),veh.body.A_ori_R(2,2),veh.body.A_ori_R(2,3)+cos(veh.body.A_ang_R(2,1))*veh.body.A_dim_R(2,3)];
pr_br=[veh.body.A_ori_R_re(2,1)+sin(veh.body.A_ang_R_re(2,1))*veh.body.A_dim_R_re(2,3),veh.body.A_ori_R_re(2,2),veh.body.A_ori_R_re(2,3)+cos(veh.body.A_ang_R_re(2,1))*veh.body.A_dim_R_re(2,3)];

%upper points
pl_tf=[veh.body.A_ori(2,1),veh.body.A_ori(2,2),veh.body.A_ori(2,3)];
pl_tr=[veh.body.A_ori_re(2,1),veh.body.A_ori_re(2,2),veh.body.A_ori_re(2,3)];
pr_tf=[veh.body.A_ori_R(2,1),veh.body.A_ori_R(2,2),veh.body.A_ori_R(2,3)];
pr_tr=[veh.body.A_ori_R_re(2,1),veh.body.A_ori_R_re(2,2),veh.body.A_ori_R_re(2,3)];

%% 2) Calculate the surfaces
%% a) Lower surface
x=[pl_tf(1),pl_tr(1),pl_br(1),pl_bf(1)];
y=[pl_tf(2),pl_tr(2),pl_br(2),pl_bf(2)];
z=[pl_tf(3),pl_tr(3),pl_br(3),pl_bf(3)];
if max(y)>0
    diplay('Warning! Side area not calculated properly')
end
A_side_low=polyarea(x,z); %Calculates surface area of lower side part

%% b) Upper surface
%Reset variables
x=0;
y=0; 
z=0; 

%Read in x-values
x(1)=ws_lt(1);
x(2)=ws_lb(1);
x(3)=rw_lt(1);
x(4)=rw_lb(1);

%Calculate difference of y-values due to side window angle
y(1)=ws_lt(2);
y(2)=ws_lb(2);
y_d_f=abs(y(1)-y(2));

y(3)=rw_lt(2);
y(4)=rw_lb(2);
y_d_r=abs(y(3)-y(4));

%Calculate difference of z-values due to side window angle and calculate new z-values (if windows
%would be rotated to a vertical position)
z(1)=ws_lt(3);
z(2)=ws_lb(3);
z_d_f=abs(z(1)-z(2));
d_tot_f=sqrt(z_d_f^2+y_d_f^2);
z(1)=z(1)+(d_tot_f-z_d_f);

z(3)=rw_lt(3);
z(4)=rw_lb(3);
z_d_r=abs(z(3)-z(4));
d_tot_r=sqrt(z_d_r^2+y_d_r^2);
z(3)=z(3)+(d_tot_r-z_d_r);

%Calculate the side window angles
angle_f=asind(y_d_f/d_tot_f);
angle_r=asind(y_d_r/d_tot_r);
angle_side=mean([angle_f,angle_r]);

%Calculate surface
A_side_high=polyarea(x,z); %assumption that side surface was turned to a flat surface

%% c) Windshield
%Reset variables
x=0;
y=0; 
z=0; 

%Read in y-values
y(1)=ws_lb(2);
y(2)=ws_lt(2);
y(3)=ws_rt(2);
y(4)=ws_rb(2);

%Calculate difference of x-values due to windshield angle (only left side due to symmetrie)
x(1)=ws_lb(1);
x(2)=ws_lt(1);
x_d=abs(x(1)-x(2));
%Read in right side
x(3)=ws_rt(1);
x(4)=ws_rb(1);


%Calculate difference of z-values due to side window angle and calculate new z-values (if windows
%would be rotated to a vertical position)
%only left side due to symmetrie
z(1)=ws_lb(3);
z(2)=ws_lt(3);
z_d=abs(z(1)-z(2));
d_tot=sqrt(z_d^2+x_d^2);
z(2)=z(2)+(d_tot-z_d);

z(3)=ws_rt(3);
z(4)=ws_rb(3);
z(3)=z(3)+(d_tot-z_d);

%Calculate the side window angles
angle_ws=asind(x_d/d_tot);

%Calculate surface
A_windshield=polyarea(y,z); %assumption that side surface was turned to a flat surface


%% d) Rear Window area
%Reset variables
x=0;
y=0; 
z=0; 

%Read in y-values
y(1)=rw_lb(2);
y(2)=rw_lt(2);
y(3)=rw_rt(2);
y(4)=rw_rb(2);

%Calculate difference of x-values due to windshield angle (only left side due to symmetrie)
x(1)=rw_lb(1);
x(2)=rw_lt(1);
x_d=abs(x(1)-x(2));
%Read in right side
x(3)=rw_rt(1);
x(4)=rw_rb(1);

%Calculate difference of z-values due to side window angle and calculate new z-values (if windows
%would be rotated to a vertical position)
%only left side due to symmetrie
z(1)=rw_lb(3);
z(2)=rw_lt(3);
z_d=abs(z(1)-z(2));
d_tot=sqrt(z_d^2+x_d^2);
z(2)=z(2)+(d_tot-z_d);

z(3)=rw_rt(3);
z(4)=rw_rb(3);
z(3)=z(3)+(d_tot-z_d);

%Calculate the side window angles
angle_rw=asind(x_d/d_tot);

%Calculate surface
A_rearwindow=polyarea(y,z); %assumption that side surface was turned to a flat surface

%% e) Roof
%Reset variables
x=0;
y=0; 
z=0; 
%Read in values
x=[ws_lt(1),ws_rt(1),rw_rt(1),rw_lt(1)];
y=[ws_lt(2),ws_rt(2),rw_rt(2),rw_lt(2)];
z=[ws_lt(3),ws_rt(3),rw_rt(3),rw_lt(3)];
if ~(max(z)==min(z))
    diplay('Warning! Side area not calculated properly')
end
A_roof=polyarea(x,y); %Calculates surface area of roof

%% f) Ground
%Reset variables
x=0;
y=0; 
z=0; 
%Read in values
x=[pl_bf(1),pr_bf(1),pr_br(1),pl_br(1)];
y=[pl_bf(2),pr_bf(2),pr_br(2),pl_br(2)];
z=[pl_bf(3),pr_bf(3),pr_br(3),pl_br(3)];
if 0.1<abs(max(z)-min(z))
    diplay('Warning! Side area not calculated properly')
end
A_floor=polyarea(x,y); %Calculates surface area of floor

%% 3) Caclculate total surface area of cabin
%% a) Load regressions and read in parameters
regr_windshield=Par.regr.exterior.windshield.eq;
regr_sidewindow=Par.regr.exterior.sidewindow.eq;
regr_rearwindow=Par.regr.exterior.rearwindow.eq;

wheelbase=veh.dimensions.GX.wheelbase; %vehicle wheelbase in mm
height=veh.dimensions.GZ.vehicle_height; %vehicle height in mm
width=veh.dimensions.GY.vehicle_width; %vehicle width in mm

%% b) Calculate surface areas
%Calculate window surface areas
A_windshield_reg = regr_windshield(width,height,angle_ws);
A_sidewindow_reg = regr_sidewindow(wheelbase,height,angle_ws,angle_rw);
A_rearwindow_reg = regr_rearwindow(width,height,angle_rw);

%Calculate cabin surface without windows
A_tot=(A_floor+2*A_side_low+A_roof+A_windshield+A_rearwindow+2*A_side_high)/(100^3);
A_tot_nowin=A_tot-(A_windshield_reg+A_sidewindow_reg+A_rearwindow_reg);


%% 4) Write parameters into vehicle struct
veh.aux.exterior_measurements.Alpha=90-angle_ws;    %90- due to angle between horizontal and window
veh.aux.exterior_measurements.Betta=90-angle_side;  %90- due to angle between horizontal and window
veh.aux.exterior_measurements.Gamma=90-angle_rw;    %90- due to angle between horizontal and window
veh.aux.exterior_measurements.Area_Front=A_windshield_reg;
veh.aux.exterior_measurements.Area_Side=A_sidewindow_reg;
veh.aux.exterior_measurements.Area_Back=A_rearwindow_reg;
veh.aux.exterior_measurements.Area_Cabin=A_tot_nowin;

%% Optional
% Call CALCULATE_Cabin from DISPLAY_vehicle to see the points
if veh.feasible==1
    plot3(ws_lb(1),ws_lb(2),ws_lb(3),'o','Color','b','MarkerSize',5,'MarkerFaceColor','#D9FFFF')
    plot3(ws_rb(1),ws_rb(2),ws_rb(3),'o','Color','b','MarkerSize',5,'MarkerFaceColor','#D9FFFF')
    plot3(ws_lt(1),ws_lt(2),ws_lt(3),'o','Color','b','MarkerSize',5,'MarkerFaceColor','#D9FFFF')
    plot3(ws_rt(1),ws_rt(2),ws_rt(3),'o','Color','b','MarkerSize',5,'MarkerFaceColor','#D9FFFF')
    plot3(rw_lb(1),rw_lb(2),rw_lb(3),'o','Color','b','MarkerSize',5,'MarkerFaceColor','#D9FFFF')
    plot3(rw_rb(1),rw_rb(2),rw_rb(3),'o','Color','b','MarkerSize',5,'MarkerFaceColor','#D9FFFF')
    plot3(rw_lt(1),rw_lt(2),rw_lt(3),'o','Color','b','MarkerSize',5,'MarkerFaceColor','#D9FFFF')
    plot3(rw_rt(1),rw_rt(2),rw_rt(3),'o','Color','b','MarkerSize',5,'MarkerFaceColor','#D9FFFF')
    plot3(pl_bf(1),pl_bf(2),pl_bf(3),'o','Color','b','MarkerSize',5,'MarkerFaceColor','#D9FFFF')
    plot3(pl_br(1),pl_br(2),pl_br(3),'o','Color','b','MarkerSize',5,'MarkerFaceColor','#D9FFFF')
    plot3(pr_bf(1),pr_bf(2),pr_bf(3),'o','Color','b','MarkerSize',5,'MarkerFaceColor','#D9FFFF')
    plot3(pr_br(1),pr_br(2),pr_br(3),'o','Color','b','MarkerSize',5,'MarkerFaceColor','#D9FFFF')
    plot3(pl_tf(1),pl_tf(2),pl_tf(3),'o','Color','b','MarkerSize',5,'MarkerFaceColor','#D9FFFF')
    plot3(pl_tr(1),pl_tr(2),pl_tr(3),'o','Color','b','MarkerSize',5,'MarkerFaceColor','#D9FFFF')
    plot3(pr_tf(1),pr_tf(2),pr_tf(3),'o','Color','b','MarkerSize',5,'MarkerFaceColor','#D9FFFF')
    plot3(pr_tr(1),pr_tr(2),pr_tr(3),'o','Color','b','MarkerSize',5,'MarkerFaceColor','#D9FFFF')
end

end