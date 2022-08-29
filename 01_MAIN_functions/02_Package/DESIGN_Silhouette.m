function [vehicle]=DESIGN_Silhouette(vehicle,Parameters)
%% Description:
% Designed by: Michael Mast, Adrian König (FTM, Technical University of Munich)
%-------------
% Created on: 01.06.2021
% ------------
% Version: Matlab2020b
%-------------
% Description:  This function calculates the cornerpoints of the vehicle silhouette
%               and checks the visual angles of the driver
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
%1) Parameter
%2) Hood evaluation
%3) Front window evaluation
%4) Rear window evaluation
%5) Final Silhouette Parameters
%6) Sight evaluation

%Key
%Windows:
%Win_Lx=Window Length in X
%Win_Lower=Lower Window Edge X-Koordinate
%Win_Upper=Upper Window Edge X-Koordinate
%_F=Front of the Vehicle
%_R=Rear of the Vehicle
%
%Silhouette
%pane=Window Corners
%hood=Hood Corners
%grill=Front Fascia Corners
%_re=Rear of the Vehicle
%Nr.1/2 Left of the Vehicle 3/4 Right of the Vehicle
%View form the Left:
%                      pane 2/3          pane_re 2/3
%                            O------------O
%                 pane 1/4  /              \   pane_re 1/4
%    hood 2/3     hood 1/4 /                \  hood_re 1/4   hood_re 2/3
%   grill 1/4 O-----------O                  O------------O grill_re 1/4
%            /                                             \
% grill 2/3 O-----------------------------------------------O grill_re 2/3
%
%Sight:
%dash=Rear of the Front Hood
%hood=Front of the Front Hood
%down=total View over Hood
%roof=Front of Roof
%_a=viewangle to X-Axis
%
%
%% Parameters
Integrated_trunk_F  =   strcmp(vehicle.Input.trunk_type_front,'integrated');%switch to lower (false is upper)
Integrated_trunk_R  =   strcmp(vehicle.Input.trunk_type_rear,'integrated');%switch to lower (false is upper)
window_angle_f      =   vehicle.Input.window_angle_front *(pi/180);%angle from vertical to window front
window_angle_r      =   vehicle.Input.window_angle_rear *(pi/180);%angle from vertical to window rear


%% Hood evaluation

%if Parameters.input.trunk_volume>0 %with trunk
    if vehicle.body.Trunk_height_F<vehicle.dimensions.CZ.wheelhouse_r_height %front trunk lower than wheelhouse
        vehicle.body.Hood_height_F=vehicle.dimensions.CZ.wheelhouse_r_height+Parameters.body.A_Strut_h;
    elseif Integrated_trunk_F %front trunk higher than wheelhouse and integrated
        vehicle.body.Hood_height_F=min(vehicle.body.Trunk_height_F+Parameters.body.A_Strut_h,vehicle.body.Win_edge_F);
    else %front trunk hooded higher than wheelhouse
        vehicle.body.Hood_height_F=vehicle.body.Trunk_height_F+Parameters.body.A_Strut_h;
    end
    if vehicle.body.Trunk_height_R<vehicle.dimensions.CZ.wheelhouse_r_height %rear trunk lower than wheelhouse
        vehicle.body.Hood_height_R=vehicle.dimensions.CZ.wheelhouse_r_height+Parameters.body.C_Strut_h;
    elseif Integrated_trunk_R%rear trunk higher than wheelhouse and integrated
        vehicle.body.Hood_height_R=min(vehicle.body.Trunk_height_R+Parameters.body.C_Strut_h,vehicle.body.Win_edge_R);
    else %front trunk hooded higher than wheelhouse
        vehicle.body.Hood_height_R=vehicle.body.Trunk_height_R+Parameters.body.C_Strut_h;
    end
    vehicle.body.Trunk_edge_F=min(vehicle.body.Trunk_edge_F,vehicle.dimensions.p_cooler_ctr(1,1)-(vehicle.dimensions.p_cooler_ctr(2,1)/2));
    Win_lower_F=vehicle.body.Trunk_edge_F;
    Win_lower_R=vehicle.body.Trunk_edge_R;
% else %without trunk
%     vehicle.body.Trunk_edge_F=vehicle.dimensions.p_cooler_ctr(1,1)-(vehicle.dimensions.p_cooler_ctr(2,1)/2);
%     vehicle.body.Hood_height_F=vehicle.dimensions.CZ.wheelhouse_r_height+Parameters.body.A_Strut_h;
%     vehicle.body.Hood_height_R=vehicle.dimensions.CZ.wheelhouse_r_height+Parameters.body.C_Strut_h;
%     Win_lower_F=vehicle.dimensions.p_bumper{1}(1,1)+(vehicle.dimensions.p_bumper{1}(2,1)/2);
%     Win_lower_R=vehicle.dimensions.GX.wheelbase-vehicle.dimensions.p_bumper{2}(1,1)-(vehicle.dimensions.p_bumper{2}(2,1)/2);
%     vehicle.body.Trunk_edge_F=vehicle.dimensions.p_bumper{1}(1,1)-(vehicle.dimensions.p_bumper{1}(2,1)/2);
%     vehicle.body.Trunk_edge_R=vehicle.dimensions.GX.wheelbase-vehicle.dimensions.p_bumper{2}(1,1)+(vehicle.dimensions.p_bumper{2}(2,1)/2);
% end

Win_Lx_F=tan(window_angle_f)*(vehicle.dimensions.GZ.vehicle_height-vehicle.body.Hood_height_F);
Win_Lx_R=tan(window_angle_r)*(vehicle.dimensions.GZ.vehicle_height-vehicle.body.Hood_height_R);


%% Front Window Evaluation
if strcmp(vehicle.Input.int_type,'vav')
    Win_upper_F=min(vehicle.interior.int_boundary_f2_new);
else
    Win_upper_F=min(vehicle.interior.int_boundary_f2_new)+Parameters.body.F_Win_l;
end

if Integrated_trunk_F %pane attached on lower
   if (Win_lower_F+Win_Lx_F)>Win_upper_F
        vehicle.Error=13;
        return
   end
   Win_upper_F=(Win_lower_F+Win_Lx_F);
   
else %pane attached on upper
    if (Win_upper_F-Win_Lx_F)<Win_lower_F
        vehicle.Error=13;
        return
    end
   if (Win_upper_F-Win_Lx_F)>(vehicle.dimensions.p_wheelhouse_left{1}(1,1)+(vehicle.dimensions.p_wheelhouse_left{1}(2,1)))
       Win_lower_F=(vehicle.dimensions.p_wheelhouse_left{1}(1,1)+(vehicle.dimensions.p_wheelhouse_left{1}(2,1)));
       Win_upper_F=(Win_lower_F+Win_Lx_F);
   else
    Win_lower_F=(Win_upper_F-Win_Lx_F);
   end
end

    pane(1,1)=Win_lower_F;
    pane(2,1)=Win_upper_F;
    pane(3,1)=Win_upper_F;
    pane(4,1)=Win_lower_F;

 pane([1,4],3)= vehicle.body.Hood_height_F;
 pane([2,3],3)= vehicle.dimensions.GZ.vehicle_height;
 pane(1,2)=0;
 pane(4,2)=vehicle.dimensions.GY.vehicle_width;
 pane(2,2)=(vehicle.dimensions.GY.vehicle_width-vehicle.interior.int_y_tot)/2;
 pane(3,2)=(vehicle.dimensions.GY.vehicle_width+vehicle.interior.int_y_tot)/2;
 
 hood([1,4],1)=pane(1,1);
 hood([1,4],3)=pane(1,3);
 hood([2,3],1)=(vehicle.dimensions.p_bumper{1}(1,1)-(vehicle.dimensions.p_bumper{1}(2,1)/2));
 hood([1,2],2)=0;
 hood([3,4],2)=vehicle.dimensions.GY.vehicle_width;
 %Calculate height where hood intersects the trunk volume to define the height of front hood point
 if pane(1,1)-vehicle.body.Trunk_edge_F<0.1 %Case for integrated trunk (no hood)
     height_temp=0;
 else
     height_temp=pane(1,3)-((Parameters.body.A_Strut_h*(-(vehicle.dimensions.p_bumper{1}(1,1)-(vehicle.dimensions.p_bumper{1}(2,1)/2))+pane(1,1)))/(pane(1,1)-vehicle.body.Trunk_edge_F));
 end
 %Calculate which case causes higher trunk
 hood([2,3],3)=max(...
 [pane(2,3)-(((pane(2,3)-pane(1,3)+Parameters.body.A_Strut_h)*(-(vehicle.dimensions.p_bumper{1}(1,1)-(vehicle.dimensions.p_bumper{1}(2,1)/2))+pane(2,1)))/(pane(2,1)-vehicle.body.Trunk_edge_F)),...
 height_temp,...
 vehicle.dimensions.p_bumper{1}(1,3)+(vehicle.dimensions.p_bumper{1}(2,3)/2)]);
 
 grill([1,4],1)=hood(2,1);
 grill([1,4],3)=hood(2,3);
 grill([2,3],1)=-vehicle.dimensions.GX.vehicle_overhang_f;
 grill([1,2],2)=hood(2,2);
 grill([3,4],2)=hood(3,2);
%  grill(2,2)=(vehicle.dimensions.GY.vehicle_width-vehicle.dimensions.p_bumper{1}(2,2))/2;
%  grill(3,2)=(vehicle.dimensions.GY.vehicle_width+vehicle.dimensions.p_bumper{1}(2,2))/2;
 grill([2,3],3)=vehicle.dimensions.GZ.H156;
 

%% Rear Window Evaluation
if strcmp(vehicle.Input.int_type,'btb')
    Win_upper_R=max(vehicle.interior.int_boundary_r2_new)-Parameters.body.R_Win_l;
else
    Win_upper_R=max(vehicle.interior.int_boundary_r2_new);
end

if Integrated_trunk_R %pane attached on lower
    if (Win_lower_R-Win_Lx_R)<Win_upper_R
        vehicle.Error=13;
        return
    end
    Win_upper_R=(Win_lower_R-Win_Lx_R);
else %pane attached on upper
    if (Win_upper_R+Win_Lx_R)>Win_lower_R
        vehicle.Error=13;
        return
    end
    Win_lower_R=(Win_upper_R+Win_Lx_R);
end
pane_re(1,1)=Win_lower_R;
pane_re(2,1)=Win_upper_R;
pane_re(3,1)=Win_upper_R;
pane_re(4,1)=Win_lower_R;

pane_re([1,4],3)= vehicle.body.Hood_height_R;
pane_re([2,3],3)= vehicle.dimensions.GZ.vehicle_height;
pane_re(1,2)=0;
pane_re(4,2)=vehicle.dimensions.GY.vehicle_width;
pane_re(2,2)=(vehicle.dimensions.GY.vehicle_width-vehicle.interior.int_y_tot)/2;
pane_re(3,2)=(vehicle.dimensions.GY.vehicle_width+vehicle.interior.int_y_tot)/2;

hood_re([1,4],1)=pane_re(1,1);
hood_re([1,4],3)=pane_re(1,3);
hood_re([2,3],1)=(vehicle.dimensions.GX.wheelbase-vehicle.dimensions.p_bumper{2}(1,1)+(vehicle.dimensions.p_bumper{2}(2,1)/2));
hood_re([1,2],2)=0;
hood_re([3,4],2)=pane_re(4,2);
%Calculate height where rear hood intersects the trunk volume to define the height of rear hood point
if -pane_re(1,1)+vehicle.body.Trunk_edge_R<0.1 %Case for integrated trunk (no hood)
    height_temp_r=0;
else
    height_temp_r=pane_re(1,3)-((Parameters.body.C_Strut_h*((vehicle.dimensions.GX.wheelbase-vehicle.dimensions.p_bumper{2}(1,1)+(vehicle.dimensions.p_bumper{2}(2,1)/2))-pane_re(1,1)))/(-pane_re(1,1)+vehicle.body.Trunk_edge_R));
end
hood_re([2,3],3)=max(...
    [pane_re(2,3)-(((pane_re(2,3)-pane_re(1,3)+Parameters.body.C_Strut_h)*((vehicle.dimensions.GX.wheelbase-vehicle.dimensions.p_bumper{2}(1,1)+(vehicle.dimensions.p_bumper{2}(2,1)/2))-pane_re(2,1)))/(-pane_re(2,1)+vehicle.body.Trunk_edge_R)),...
    height_temp_r,...
    vehicle.dimensions.p_bumper{2}(1,3)+(vehicle.dimensions.p_bumper{2}(2,3)/2)]);

grill_re([1,4],1)=hood_re(2,1);
grill_re([1,4],3)=hood_re(2,3);
grill_re([2,3],1)=vehicle.dimensions.GX.wheelbase+vehicle.dimensions.GX.vehicle_overhang_r;
grill_re([1,2],2)=hood_re(2,2);
grill_re([3,4],2)=hood_re(3,2);
%  grill_re(2,2)=(vehicle.dimensions.GY.vehicle_width-vehicle.dimensions.p_bumper{2}(2,2))/2;
%  grill_re(3,2)=(vehicle.dimensions.GY.vehicle_width+vehicle.dimensions.p_bumper{2}(2,2))/2;
grill_re([2,3],3)=vehicle.dimensions.GZ.H156;



%% Final Silhouette Parameters

vehicle.body.p_pane_f=pane;
vehicle.body.p_hood_f=hood;
vehicle.body.p_grill_f=grill;
vehicle.body.p_pane_r=pane_re;
vehicle.body.p_hood_r=hood_re;
vehicle.body.p_grill_r=grill_re;
vehicle.body.Win_lower_F=Win_lower_F;
vehicle.body.Win_lower_R=Win_lower_R;
vehicle.body.Win_Lx_F=Win_Lx_F;
vehicle.body.Win_Lx_R=Win_Lx_R;
vehicle.body.Win_upper_F=Win_upper_F;
vehicle.body.Win_upper_R=Win_upper_R;

%% Sight angles
% according to UNECE Nr 125
% between 5° and 40° backrest angle of a foreward facing driver
if (vehicle.Input.angle_br_max(1)<=85)&&(vehicle.Input.angle_br_max(1)>=50)&&(strcmp(vehicle.Input.int_type,'vav')==0)
%% H-Point calculation
load('human.mat','legstop')
int_boundary_f_new=vehicle.interior.int_boundary_f_new; % initialize interior front boundary
int_boundary_r_new=vehicle.interior.int_boundary_r_new; % initialize interior rear boundary
a_pos=[vehicle.interior.a_1 vehicle.interior.a_2];% Side distance to interior front and rear
width_seat = vehicle.Input.backrest_width;
n_seat=vehicle.Input.n_seat;
%Y-position in the middle of the seat
if vehicle.Input.single_ar_outside(1) ==1
   y_position = vehicle.Input.wallgap(1) + vehicle.Input.armrest_width_single(1) + 0.5*width_seat(1) + a_pos(1);
else
   y_position = vehicle.Input.wallgap(1) + 0.5*width_seat(1) + a_pos(1);
end
%X-position offset infront of backrest
switch vehicle.Input.int_type %(vav=vis-a-vis, btb=back-to-back, con=conventional, sr=singlerow)
    case 'vav'
        [~,~,~,~,~,x_pos_seat,~,~,z_pos_br,~] = PlotInterior_vav(vehicle,int_boundary_f_new,int_boundary_r_new,a_pos(1),a_pos(2));
        offset_x = [100 vehicle.Input.seat_depth(2)-100+vehicle.Input.seat_adjustment_x(2)];
    case 'btb'
        [~,~,~,~,~,x_pos_seat,~,~,z_pos_br,~] = PlotInterior_btb(vehicle,int_boundary_f_new,int_boundary_r_new,a_pos(1),a_pos(2));
        offset_x = [vehicle.Input.seat_depth(1)-100+vehicle.Input.seat_adjustment_x(1) 100];
    case 'con'
        [~,~,~,~,~,x_pos_seat,~,~,z_pos_br,~] = PlotInterior_con(vehicle,int_boundary_f_new,int_boundary_r_new,a_pos(1),a_pos(2));
        offset_x = [vehicle.Input.seat_depth(1)-100+vehicle.Input.seat_adjustment_x(1) vehicle.Input.seat_depth(2)-100+vehicle.Input.seat_adjustment_x(2)];
    case 'sr'
        [~,~,~,x_pos_seat,~,~,z_pos_br,~]     = PlotInterior_sr(vehicle,int_boundary_f_new,int_boundary_r_new,a_pos(1),a_pos(2));
        offset_x = [vehicle.Input.seat_depth(1)-100+vehicle.Input.seat_adjustment_x(1) 0];
end
%Z-position over seat surface
z_legstop_lb    =   min(legstop.points(:,3));   %Lower boundary of thigh
offset_z        =   abs(z_legstop_lb); %Offset of coordination system in Catia and outer measurement and overlap due to sinking in
% Final Values Seat Reference
vehicle.manikin.SgRP(1)=x_pos_seat(1)+offset_x(1);
vehicle.manikin.SgRP(2)=y_position;
vehicle.manikin.SgRP(3)=z_pos_br(1)+offset_z;
%standard V points for 25° backrest
offset_V=[68,-5,665;68,-5,589];
%backrest correction, start 5° !!UNECE - DO NOT CHANGE!!
V_Cor=[-186,28;-177,27;-167,27;-157,27;-147,26;-137,25;-128,24;-118,23;-109,22;-99,21;-90,20;-81,18;-72,17;-62,15;-53,13;-44,11;-35,9;-26,7;...
    -18,5;-9,3;0,0;9,-3;17,-5;26,-8;34,-11;43,-14;51,-18;59,-21;67,-24;76,-28;84,-32;92,-35;100,-39;108,-43;115,-48;123,-52];
V_Cor=[V_Cor(:,1),zeros(length(V_Cor),1),V_Cor(:,2)];
offset_V=offset_V+V_Cor((86-vehicle.Input.angle_br_max(1)),:);
% V1 and V2 Coordinates
vehicle.manikin.V_points=offset_V+vehicle.manikin.SgRP;

%% sight angle calculation
dash_a = atand((vehicle.manikin.V_points(2,3)-pane(1,3))/(vehicle.manikin.V_points(2,1)-pane(1,1)));%angle on rear hood edge
hood_a = atand((vehicle.manikin.V_points(2,3)-hood(2,3))/(vehicle.manikin.V_points(2,1)-hood(2,1)));%angle on front hood edge
down_a = min(dash_a,hood_a); %smaller angle is further up
roof_a = atand((pane(2,3)-vehicle.manikin.V_points(1,3))/(vehicle.manikin.V_points(1,1)-pane(2,1)));%angle to roof edge
vehicle.manikin.AS_point=pane(1,:) + (pane(2,:)-pane(1,:)) * ((vehicle.manikin.V_points(1,3)-pane(1,3))/(pane(2,3)-pane(1,3)));%on A-pillar, at Eye hight
left_a = atand((vehicle.manikin.V_points(1,2)-vehicle.manikin.AS_point(2))/(vehicle.manikin.V_points(1,1)-vehicle.manikin.AS_point(1)));%angle to A-pillar

vehicle.body.down=vehicle.manikin.V_points(2,:)-[10000,0,10000*tand(down_a)];
vehicle.body.left=vehicle.manikin.V_points(1,:)-[10000,10000*tand(left_a),0];
vehicle.body.roof=vehicle.manikin.V_points(1,:)-[10000,0,-10000*tand(roof_a)];

if (down_a<5)||(roof_a<7)||(left_a<17)%angles over min Regulations
    vehicle=errorlog(vehicle,'Sight to narrow');
    vehicle.body.sight=0;
else %angles under min Regulation
    vehicle=errorlog(vehicle,'Driver can see');
    vehicle.body.sight=1;
end
else %not correct Interior layout
    vehicle=errorlog(vehicle,'Sight evaluation not possible');
    vehicle.body.sight=0;
end
end