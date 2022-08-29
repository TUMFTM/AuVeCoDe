function[vehicle]=Package_Structure(vehicle,Parameters)
%% Description:
% Designed by: Michael Mast, Adrian König (FTM, Technical University of Munich)
%-------------
% Created on: 01.06.2021
% ------------
% Version: Matlab2020b
%-------------
% Description:  This function calculates the positions and dimensions of the structural
%               elements at the front of the vehicle
% ------------
% Sources:  [1] Michael Mast, “Packageplanung von autonomen Fahrzeugkonzepten im Vorder- und Hinterwagen,” Semester Thesis, Technical University of Munich, Institute of Automotive Technology, 2021
%           [2] Michael Mast, “Karosseriemodellierung autonomer Elektrofahrzeuge,” Master Thesis, Technical University of Munich, Institute of Automotive Technology, 2022
%           [3] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% 

%% Input:
% vehicle structure
% fixed parameters

%% Output
% updated vehicle struct

%% Implementation
%1) Calculating lugitudinal members
%2) Calculating angled members
%3) Mirroring members to right side
%4) Calculating Crosswise members
%5) Returning measurements

%% list of elements
% 1.1 side member           SM
% 1.2 front frame           FF
% 1.3 A-pillar strut        APS
% 2.1 front frame extension FFE
% 2.2 A-pillar              AS
% 2.3 2nd A-pillar          AS2
% 2.4 front frame strut     FFS
% 4.1 lower crossmember     CML
% 4.2 upper crossmember     CMU
% 4.3 window carrier        WC

% Key
% L     =   Logitudinal Members
% A     =   Angled Members
% Q     =   Crosswise Members
% _dim  =   Dimentions along Edges
% _ori  =   Coordinates of the Reference Point
% _ang  =   Angles of Rotation 
% _R    =   Right side of the Vehicle

C=zeros(9,3,2);   %Chassis matrix
%dim 1          =   ['SM';'FF';'APS';'FFE';'AS';'AS2';'FFS';'CML';'CMU';'WC']
%dim 2          =   [x,y,z]
%dim 3          =   [min, max]
%Orientation    =   ['l';'l';'l';'yz';'y';'v';'c';'y';'c'] l=longitudinal, y/z=angled in y/z, v=vertical, c=crosswise

L_dim=zeros(3,3);%3 dim lenght
L_ori=zeros(3,3);%3 dim origin
A_dim=zeros(4,3);%3 dim lenght without rotation
A_ang=zeros(4,2);%y angle, z angle
A_ori=zeros(4,3);%3 dim origin after rotation
Q_dim=zeros(3,3);%3 dim lenght
Q_ang=zeros(3,1);%y angle
Q_ori=zeros(3,3);%3 dim origin

%% 1) Longitudinal elements
% 1.1 Side member
C(1,1,1)=vehicle.dimensions.p_wheelhouse_left{1}(1,1)+(vehicle.dimensions.p_wheelhouse_left{1}(2,1)); %behind front wheelhouse
C(1,1,2)=vehicle.dimensions.GX.wheelbase-(vehicle.dimensions.p_wheelhouse_left{2}(2,1)); %infront of rear wheelhouse
C(1,2,1)=vehicle.dimensions.p_wheelhouse_left{1}(1,2)-(vehicle.dimensions.p_wheelhouse_left{1}(2,2)/2); %left Wheelhouse edge
C(1,2,2)=min(C(1,2,1)+Parameters.body.L_Frame_w,(vehicle.dimensions.GY.vehicle_width-vehicle.battery.installationspace.CY_batt_underfloor)/2); %inside to battery or standard width
C(1,3,1)=vehicle.dimensions.GZ.H156+1; %on underfloor level
C(1,3,2)=C(1,3,1)+Parameters.body.L_Frame_h; %up to standard height

% 1.2 Front frame
C(2,1,1)=vehicle.dimensions.p_bumper{1}(1,1)+(vehicle.dimensions.p_bumper{1}(2,1)/2); %behind front bumper
C(2,2,1)=vehicle.dimensions.p_wheelhouse_left{1}(1,2)+(vehicle.dimensions.p_wheelhouse_left{1}(2,2)/2); %inside of wheelhouse
C(2,2,2)=C(2,2,1)+Parameters.body.F_Frame_w; %up to standard width
if vehicle.dimensions.p_drivetrain{1}(2,3)==0 %without drivetrain
    C(2,3,1)=vehicle.dimensions.p_axle_height; %above axle
elseif C(2,2,2)>((vehicle.dimensions.GY.vehicle_width-vehicle.dimensions.p_drivetrain{1}(2,2))/2) %drivetrain out to frames
    C(2,3,1)=vehicle.dimensions.p_drivetrain{1}(1,3)+vehicle.dimensions.p_axle_height-vehicle.dimensions.p_powerel{1}(2,3);%above drivetrain but not electronics
else %drivetrain between frames
    C(2,3,1)=vehicle.dimensions.p_axle_height+vehicle.gearbox{1}.shafts.d_sh_3;%above drive shaft
end
C(2,3,2)=C(2,3,1)+Parameters.body.F_Frame_h;%up to standard height
if isnan(vehicle.interior.int_boundary_f2_new(ceil(C(2,3,2))))%below interior
    C(2,1,2)=vehicle.battery.installationspace.EX_batt_underfloor;%infornt of underfloor battery
elseif isnan(vehicle.interior.int_boundary_f2_new(floor(C(2,3,1)))) %on interior edge
    if vehicle.battery.installationspace.CX_batt_second_level_1==0 || vehicle.battery.installationspace.CY_batt_second_level_1<(vehicle.dimensions.GY.vehicle_width-(2*C(2,2,2)))%no 2nd level or between frame
       C(2,1,2)=min([vehicle.interior.int_boundary_f2_new(ceil(C(2,3,2))),vehicle.battery.installationspace.EX_batt_underfloor]);%infornt of underfloor and interior
    else %with 2nd level
       C(2,1,2)=min([vehicle.interior.int_boundary_f2_new(ceil(C(2,3,2))),vehicle.battery.installationspace.EX_batt_second_level_1,vehicle.battery.installationspace.EX_batt_underfloor]);%infornt of underfloor and interior and 2nd level
    end
else %on interior
    if vehicle.battery.installationspace.CX_batt_second_level_1==0 || vehicle.battery.installationspace.CY_batt_second_level_1<(vehicle.dimensions.GY.vehicle_width-(2*C(2,2,2)))%no 2nd level of between frame
        C(2,1,2)=vehicle.interior.int_boundary_f2_new(ceil(C(2,3,2)));%infornt of interior
    else %with 2nd level
        C(2,1,2)=min(vehicle.interior.int_boundary_f2_new(ceil(C(2,3,2))),vehicle.battery.installationspace.EX_batt_second_level_1);  %infornt of 2nd level and interior
    end
end

% 1.3 A-pillar strut
C(3,1,1)    =   0;%behind front axle
C(3,1,2)    =   C(1,1,1); %infornt of sidemember
C(3,2,1)    =   C(1,2,1); %outside wheelhouse edge
C(3,2,2)    =   (vehicle.dimensions.GY.vehicle_width - vehicle.interior.int_y_tot)/2; %up to interior width
C(3,3,2)    =   max(vehicle.body.Hood_height_F,vehicle.dimensions.CZ.wheelhouse_r_height+Parameters.body.A_Strut_h); %under hood, above wheelhouse
C(3,3,1)    =   C(3,3,2)-Parameters.body.A_Strut_h; %up to standard height
L_ori       =   C(1:3,:,1);%origin on min Koorinates
L_dim       =   diff(C(1:3,:,:),1,3); %lenght between koordinates

%% 2) Angled elements
% 2.1 front frame extension
C(4,1,1)=C(1,1,1); %behind wheelhouse
if vehicle.battery.installationspace.CZ_batt_underfloor ==0 %without underfloor battery
    if vehicle.battery.installationspace.CY_batt_second_level_1<(vehicle.dimensions.GY.vehicle_width-(2*C(2,2,2)))%2nd level between frame
        C(4,1,2)=vehicle.interior.int_boundary_f2_new(ceil(vehicle.Input.int_height+1)); %infront of 2nd level
    else %with 2nd level
        C(4,1,2)=min(vehicle.interior.int_boundary_f2_new(ceil(vehicle.Input.int_height+1)),vehicle.battery.installationspace.EX_batt_second_level_1); %infront of 2nd level and interior
    end
elseif (vehicle.battery.installationspace.EZ_batt_underfloor+vehicle.battery.installationspace.CZ_batt_underfloor) >= C(1,3,2) %at underfloor
    C(4,1,2)=vehicle.battery.installationspace.EX_batt_underfloor; %infront of underfloor
else %above underfloor
    C(4,1,2)=min(vehicle.interior.int_boundary_f2_new(ceil(vehicle.Input.int_height+1)),vehicle.battery.installationspace.EX_batt_underfloor); %infront of interior and underfloor
end

if C(4,1,1)<C(4,1,2) %space between wheelhouse and komponents
C(4,2,1)    =   C(1,2,2); %Y min SM max
C(4,2,2)    =   C(2,2,2); %Y max FF max
C(4,3,1)    =   C(1,3,2); %Z min SM max
C(4,3,2)    =   C(2,3,2); %Z max FF max
A_ang(1,2)=-atan(diff(C(4,2,:))/diff(C(4,1,:))); %z ange from xy-koordinates
if A_ang(1,2)<-0.7 %Angle above 45°
    C(4,2,1)=   C(2,2,1); %Y min FF min
   A_ang(1,2)=0; %straight down to Crossmember 
   C(4,1,1) =   vehicle.dimensions.p_drivetrain{1}(1,1);%behind drivetrain
end

A_dim(1,1)  =   sqrt(sum((diff(C(4,:,:),1,3)).^2)); %lenght acording to pythagoras
A_dim(1,2)  =   -min(diff(C(1,2,:)),diff(C(2,2,:))); %width like narrow FF/SM
A_dim(1,3)  =   -min(diff(C(1,3,:)),diff(C(2,3,:))); %height like narrow FF/SM
A_ori(1,:)  =   [C(4,1,1),C(4,2,2),C(4,3,2)];        %origin on upper, inner, front corner
A_ang(1,1)  =   atan(diff(C(4,3,:))/diff(C(4,1,:))); %y ange from xz-koordinates
else 
    C(4,2:3,:)=0;   %no extention pissoble
end

% 2.2 A-pillar
if ~strcmp(vehicle.Input.int_type,'vav') %front row forward
C(5,1,2)    =   min(max(C(4,1,2),C(1,1,1)+Parameters.body.A_Pillar_w),C(1,1,1)+Parameters.body.A_Pillar_dist); %behind wheelhouse, infront of komponentes, up to max. distance
else %front row backwards
C(5,1,2)    =   max(C(4,1,2),C(1,1,1)+Parameters.body.A_Pillar_w); %behind wheelhouse, infornt of komponents
end
% angle tangential to wheelhouse arch 
A_ang(2,1)=(pi/2)-(atan((vehicle.dimensions.CZ.wheelhouse_r_height-C(1,3,2)-(vehicle.dimensions.p_wheelhouse_left{1}(2,1)))/C(5,1,2))...
    +asin(((vehicle.dimensions.p_wheelhouse_left{1}(2,1))...
    +Parameters.body.A_Pillar_w)/sqrt(((vehicle.dimensions.CZ.wheelhouse_r_height-C(1,3,2)-(vehicle.dimensions.p_wheelhouse_left{1}(2,1)))^2)+(C(5,1,2)^2))));
if ((A_ang(2,1)*180/pi)<(90-vehicle.Input.angle_br_min(1)))&&strcmp(vehicle.Input.int_type,'vav') %angle steeper than backrest
    A_ang(2,1)=(90-vehicle.Input.angle_br_min(1))*(pi/180);%angle backrest
    C(5,1,2)=cos(A_ang(2,1))*((vehicle.dimensions.p_wheelhouse_left{1}(2,1))+Parameters.body.A_Pillar_w)...%moving lower point backward
        +tan(A_ang(2,1))*((vehicle.dimensions.CZ.wheelhouse_r_height-(vehicle.dimensions.p_wheelhouse_left{1}(2,1))-C(1,3,2))...
        +sin(A_ang(2,1))*((vehicle.dimensions.p_wheelhouse_left{1}(2,1))+Parameters.body.A_Pillar_w));
end

C(5,3,2)    =   C(3,3,2);%up to A-pillar strut
C(5,3,1)    =   C(1,3,2);%up above Sidemember
C(5,1,1)    =   C(5,1,2)-(tan(A_ang(2,1))*diff(C(5,3,:)));%front determined by angle and A-Pillar strut
C(5,2,1)    =   C(1,2,1);%outside Sidemember edge
C(5,2,2)    =   C(1,2,2);%inside Sidemember edge

if (C(5,1,1)<vehicle.body.Win_lower_F+Parameters.body.Win_Strut_w)%front infront of Window carrier
    C(5,1,1)=vehicle.body.Win_lower_F+Parameters.body.Win_Strut_w;%front at window carrier
    if C(5,1,1)>C(5,1,2)%front behind back
        C(5,1,2)=C(5,1,1);%verticaly down
    end
    A_ang(2,1)=(pi/2)-atan((diff(C(5,3,:)))/(diff(C(5,1,:))));%determine new angle
end

A_dim(2,1)  =   -Parameters.body.A_Pillar_w; %thickness according to parameters
A_dim(2,2)  =   diff(C(5,2,:),1,3);          %height between koordinates
A_dim(2,3)  =   -diff(C(5,3,:),1,3)/cos(A_ang(2,1)); %Lenght along angle
A_ori(2,1)  =   C(5,1,1); %origin at front left upper corner
A_ori(2,2)  =   C(5,2,1);
A_ori(2,3)  =   C(5,3,2);
A_ang(2,1)  =   -A_ang(2,1); %reersing angle for rotation matrix

% 2.3 2nd A-pillar
if ceil(C(5,3,2))<=find(~isnan(vehicle.interior.int_boundary_f2_new),1,'last')
    if (vehicle.interior.int_boundary_f2_new(ceil(C(5,3,2)))>=C(5,1,2)) &&(A_ang(2,1)>0.35) %when outside interior and angle above 20°
        C(6,1,1)=C(4,1,2)-Parameters.body.A_Pillar_w; %vertically up from of lower a pillar point to A-pillar strut
        C(6,1,2)=C(4,1,2);
        C(6,2,1)=C(1,2,1);
        C(6,2,2)=C(1,2,2);
        C(6,3,1)=C(1,3,2);
        C(6,3,2)=C(3,3,2);
        
        C(3,1,2)=C(6,1,1);%adjusting A-pillar strut
    else
        C(3,1,2)=C(5,1,1);%adjusting A-pillar strut
    end
else
    C(3,1,2)=C(5,1,1);%adjusting A-pillar strut
end

%update longitudinal
L_ori=C(1:3,:,1);
L_dim=diff(C(1:3,:,:),1,3);
%update angled
A_ori(3,:)  =   C(6,:,1);
A_dim(3,:)  =   diff(C(6,:,:),1,3);

% 2.4 front frame strut
C(10,1,1)=vehicle.dimensions.p_bumper{1}(1,1)-(vehicle.dimensions.p_bumper{1}(2,1)/2);%behind front bumper
% angle tangential to wheelhouse arch 
A_ang(4,1)=(pi/2)-(...
    -atan((vehicle.dimensions.CZ.wheelhouse_r_height-C(2,3,2)-(vehicle.dimensions.p_wheelhouse_left{1}(2,1)))/C(10,1,1))...
    +asin(((vehicle.dimensions.p_wheelhouse_left{1}(2,1))...
    +Parameters.body.F_Strut_w)/sqrt(((vehicle.dimensions.CZ.wheelhouse_r_height-C(2,3,2)-(vehicle.dimensions.p_wheelhouse_left{1}(2,1)))^2)+(C(10,1,1)^2)))...
    );
if ~isreal(A_ang(4,1))%when conection not possible
    A_ang(4,1)=0;
    C(10,1,1)=-(vehicle.dimensions.p_wheelhouse_left{1}(2,1))-Parameters.body.F_Strut_w;%infornt of wheelhouse
end

C(10,3,2)   =   C(3,3,2);%up to A-pillar strut
C(10,3,1)   =   C(2,3,2);%from Front Frame
C(10,1,2)   =   C(10,1,1)+(tan(A_ang(4,1))*diff(C(10,3,:)));%rear according to angle
C(10,2,2)   =   C(3,2,2); %inside on a-pillar strut inside
C(10,2,1)   =   (vehicle.dimensions.GY.vehicle_width - vehicle.dimensions.p_bumper{1}(2,2))/2;%outide on bumper end

if (C(10,1,2)>vehicle.body.Win_lower_F)%rear behind window carrier
    C(10,1,2)=vehicle.body.Win_lower_F;%under window carrier
    if (C(10,1,1)>vehicle.body.Win_lower_F)%complete behind window carrier
        C(10,1,1)=vehicle.body.Win_lower_F;%under window carrier
    end
    A_ang(4,1)=(pi/2)-atan((diff(C(10,3,:)))/(diff(C(10,1,:))));%determine new angle
end
if A_ang(4,1)==0 %vertically down
    C(3,1,1)=C(10,1,2)+Parameters.body.F_Strut_w; %adjusting A-pillar strut
else
    C(3,1,1)=C(10,1,2);%adjusting A-pillar strut
end
%update longitudinal
L_ori=C(1:3,:,1);
L_dim=diff(C(1:3,:,:),1,3);

A_dim(4,1)  =   Parameters.body.F_Strut_w;%thickness according to parameters
A_dim(4,2)  =   diff(C(10,2,:),1,3); %height between koordinates
A_dim(4,3)  =   -diff(C(10,3,:),1,3)/cos(A_ang(4,1)); %Lenght along angle
A_ori(4,1)  =   C(10,1,2); %origin at rear left upper corner
A_ori(4,2)  =   C(10,2,1);
A_ori(4,3)  =   C(10,3,2);

%% 3) Mirror left to right
%Y-origins negated and shifted to other side of vehicle
%Y-dimensions negated
%Y-angles negated
L_ori_R     =   L_ori;
L_ori_R(:,2)=   -L_ori(:,2)+vehicle.dimensions.GY.vehicle_width;
L_dim_R     =   L_dim;
L_dim_R(:,2)=   -L_dim(:,2);

A_ori_R     =   A_ori;
A_ori_R(:,2)=   -A_ori(:,2)+vehicle.dimensions.GY.vehicle_width;
A_dim_R     =   A_dim;
A_dim_R(:,2)=   -A_dim(:,2);
A_ang_R     =   A_ang;
A_ang_R(:,2)=   -A_ang(:,2);

%% 4) Crossmembers
C(7:9,2,1)=0; %from left vehicle edge
C(7:9,2,2)=vehicle.dimensions.GY.vehicle_width; %to right vehicle edge

% 4.1 lower crossmember
C(7,3,1)=C(1,3,1); %above underfloor
C(7,3,2)=C(7,3,1)+Parameters.body.C_Frame_h; %up to standard height
if vehicle.battery.installationspace.CZ_batt_underfloor ==0 %without underfloor battery
    C(7,1,2)=min(vehicle.interior.int_boundary_f2_new(ceil(vehicle.Input.int_height+1)),vehicle.battery.installationspace.EX_batt_second_level_1); %infront of interior and 2nd level
elseif (vehicle.battery.installationspace.EZ_batt_underfloor+vehicle.battery.installationspace.CZ_batt_underfloor) >= C(7,3,2)%2nd level between frames
    C(7,1,2)=vehicle.battery.installationspace.EX_batt_underfloor; %infront of underfloor
else %with 2nd level
    C(7,1,2)=min(vehicle.interior.int_boundary_f2_new(ceil(vehicle.Input.int_height+1)),vehicle.battery.installationspace.EX_batt_underfloor); %infront of interior and underfloor
end
C(7,1,1)=C(7,1,2)-Parameters.body.C_Frame_w; %Standard width
if (C(7,1,2)>C(1,1,1)+Parameters.body.C_Frame_w) %width out to sidemembers
    C(7,2,1)=C(1,2,2); 
    C(7,2,2)=C(7,2,2)-C(1,2,2); 
else %width between wheelhouses
    C(7,2,1)=vehicle.dimensions.CY.wheelhouse_f_width;
    C(7,2,2)=C(7,2,2)-C(7,2,1);
end

% 4.2 upper crossmember
C(8,3,1)=C(2,3,2); %above underfloor
C(8,3,2)=C(8,3,1)+Parameters.body.C_Frame_h;%up to standard height
if isnan(vehicle.interior.int_boundary_f2_new(ceil(C(8,3,2)))) %below interior
    C(8,1,2)=vehicle.battery.installationspace.EX_batt_underfloor; %infornt of underfloor
elseif isnan(vehicle.interior.int_boundary_f2_new(floor(C(8,3,1)))) %at interior edge
    if vehicle.battery.installationspace.CX_batt_second_level_1==0 %without 2nd level
        C(8,1,2)=min([vehicle.interior.int_boundary_f2_new(ceil(C(8,3,2))),vehicle.battery.installationspace.EX_batt_underfloor]);%infront of interior and underfloor
    else %with 2nd level
        C(8,1,2)=min([vehicle.interior.int_boundary_f2_new(ceil(C(8,3,2))),vehicle.battery.installationspace.EX_batt_second_level_1,vehicle.battery.installationspace.EX_batt_underfloor]);%infornt of interior 2nd level and underfloor
    end
else %above underfloor
    if vehicle.battery.installationspace.CX_batt_second_level_1==0 %without 2nd level
        C(8,1,2)=vehicle.interior.int_boundary_f2_new(ceil(C(8,3,2)));%infornt of interior
    else%with 2nd level
        C(8,1,2)=min(vehicle.interior.int_boundary_f2_new(ceil(C(8,3,2))),vehicle.battery.installationspace.EX_batt_second_level_1); %infront of interior and 2nd level
    end
end

for s=[1:size(vehicle.dimensions.p_boundaries_f,2)] %along all boundary parts
    b(s)=vehicle.dimensions.p_boundaries_f{3,s}(floor(C(8,3,1)));%at the lower front edge
end
C(8,1,1)=max(b);%behind boundary
if C(8,1,2)<C(8,1,1) %no space between boundary and interior
    C(8,:,:)=0;%no crossmember
else
    if diff(C(8,1,:))>Parameters.body.C_Frame_w %enough space for standard width
        C(8,1,2)=C(8,1,1)+Parameters.body.C_Frame_w;
    elseif (C(8,3,1)>vehicle.battery.installationspace.EZ_batt_second_level_1+vehicle.battery.installationspace.CZ_batt_second_level_1)||(C(8,1,2)<vehicle.battery.installationspace.EZ_batt_second_level_1)%engough space for angled segment
        if strcmp(vehicle.Input.int_type,'vav') %front row backwards
            Q_ang(2)=-(90-vehicle.Input.angle_br_min(1))*(pi/180);%backrest angle
        else %front row forward
            Q_ang(2)=A_ang(2,1); %a pillar angle
        end
        upper_edge=(floor(C(8,3,1)+(cos(Q_ang(2))*Parameters.body.C_Frame_h))); %upper front edge
        for s=[1:size(vehicle.dimensions.p_boundaries_f,2)] %along all boundary parts
            b(s)=vehicle.dimensions.p_boundaries_f{3,s}(upper_edge);%at the upper front edge
        end
        C(8,1,1)=max(b);%behind boundary
        C(8,1,2)=C(8,1,1)+(cos(atan(Parameters.body.C_Frame_h/diff(C(8,1,:)))+Q_ang(2))*sqrt(diff(C(8,1,:))^2+Parameters.body.C_Frame_h^2));%diagonally in free space
        C(8,1,1)=C(8,1,1)+(-sin(Q_ang(2))*Parameters.body.C_Frame_h);%adjusting lower front edge
    end
    C(8,2,1)=C(5,2,2);%inside A-pillar
    C(8,2,2)=C(8,2,2)-C(8,2,1);
end

% 4.3 window carrier
C(9,1,1)    =   vehicle.body.Win_lower_F;%at the lower window edge
C(9,1,2)    =   C(9,1,1)+Parameters.body.Win_Strut_w;%standard width
C(9,3,2)    =   C(3,3,2);%under window edge
C(9,3,1)    =   C(9,3,2)-Parameters.body.A_Strut_h;%standard height
C(9,2,1)    =   C(3,2,2);%width between A-pillar struts
C(9,2,2)    =   C(9,2,2)-C(9,2,1);

Q_ori       =   C(7:9,:,1);%origin on min Koorinates
Q_dim       =   diff(C(7:9,:,:),1,3);%lenght between koordinates

%% 5) Returning measurements
vehicle.body.L_ori  =   L_ori;
vehicle.body.L_dim  =   L_dim;
vehicle.body.L_ori_R=   L_ori_R;
vehicle.body.L_dim_R=   L_dim_R;
vehicle.body.A_ang  =   A_ang;
vehicle.body.A_ori  =   A_ori;
vehicle.body.A_dim  =   A_dim;
vehicle.body.A_ang_R=   A_ang_R;
vehicle.body.A_ori_R=   A_ori_R;
vehicle.body.A_dim_R=   A_dim_R;
vehicle.body.Q_ori  =   Q_ori;
vehicle.body.Q_dim  =   Q_dim;
vehicle.body.Q_ang  =   Q_ang;

vehicle.dimensions.p_bumper{1}(1,3)=(C(2,3,2)+C(2,3,1))/2;%adjusting bumper height
if vehicle.body.p_hood_f(2,3)<C(2,3,2)%adjusting hood and grill height to bumper height
    vehicle.body.p_hood_f(2,3)  =   C(2,3,2);
    vehicle.body.p_hood_f(3,3)  =   C(2,3,2);
    vehicle.body.p_grill_f(1,3) =   C(2,3,2);
    vehicle.body.p_grill_f(4,3) =   C(2,3,2);
end
end