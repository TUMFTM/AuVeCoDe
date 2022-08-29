function [Volume_DIN,Orientation,Volume_Euc,vehicle,input] = Package_Trunk(input,vehicle,Parameters)
%% Description:
% Designed by: Michael Mast, Adrian König (FTM, Technical University of Munich)
%-------------
% Created on: 01.06.2021
% ------------
% Version: Matlab2020b
%-------------
% Description:  This function calculates the trunk volume and compartments according to the desired
%               tunk total volume and distribution between frunk and trunk
% ------------
% Sources:  [1] Michael Mast, “Packageplanung von autonomen Fahrzeugkonzepten im Vorder- und Hinterwagen,” Semester Thesis, Technical University of Munich, Institute of Automotive Technology, 2021
%           [2] Michael Mast, “Karosseriemodellierung autonomer Elektrofahrzeuge,” Master Thesis, Technical University of Munich, Institute of Automotive Technology, 2022
%           [3] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% The Boundary surfaces are determined and stored in an Array
% TFront(   4,         3,    2)
%   ^       ^          ^     ^
% Trunk  Compartment  Axis  min/max
% -----------
% Information:
%  - Space for components that are placed later in development is allocated
%  - Not usable sections are evaluated and minimised
%  - The prozess is repeated for the Rear Trunk
%  - The fullfillment of the Target Volume is ensured
%  - Reference Volume is determined according to ISO 3832
%  - Euclidic and ISO Volume (with Orientations) are retuned
%  - The final result is ploted into the car (DISPLAY3D)
%
%% Implementation
% 1) Reading in Parameters
% a Front Trunk
% 2) Determining Boundary Surfaces
%   2.1) FA "infornt of Axle"
%   2.2) BA "behind Axle"
%   2.3) OA "over  Axle"
%   2.4) UC "Upper Compartment"
% 3) Eliminating negative Space
% 4) Komponentspace Allocation
%   4.1) Komponent Distribution 
%   4.2) Raising Upper Boundaries
%   4.3) Widening above Sidemember
% 5) Usability Analysis
% b Rear Trunk
%   repeat 2-5
% 6) Ajustment for Target Volume
% 7) Reference Volume Determination ISO 3832
% 8) Final Values and Plot


%% 1) Read in and initialize parameters
% Seats
UNECE_offset=[68 -5 665;68 -5 589];

SgRP_X=abs(max(vehicle.interior.int_boundary_f))-vehicle.interior.int_x_br(1)-vehicle.Input.int_x_overlap; %Seat reference Point X Pos
SgRP_Z=vehicle.Input.int_z_leg(1)+Parameters.manikin.z_leg_gap+ vehicle.Input.int_height;%Seat reference Point Z Pos
L104=vehicle.dimensions.GX.vehicle_overhang_f;
driver_to_back = vehicle.dimensions.GX.vehicle_overhang_r +vehicle.dimensions.GX.wheelbase -(SgRP_X + UNECE_offset(2,1)); %distrance driver Eye point to rear bumper

SgRP_X_r= abs(max(vehicle.interior.int_boundary_r))+vehicle.interior.int_x_br(2)+vehicle.Input.int_x_overlap; %Seat reference Point 2nd Row X Pos
SgRP_Z_r= vehicle.Input.int_z_leg(2)+Parameters.manikin.z_leg_gap+ vehicle.Input.int_height; %Seat reference Point 2nd Row Z Pos
L105=     vehicle.dimensions.GX.vehicle_overhang_r;
rear_to_front= vehicle.dimensions.GX.vehicle_overhang_f +(SgRP_X_r - UNECE_offset(2,1)); %distrance 2nd Row Eye Point to front bumper
   
sight_foreward=input.hood.sight_foreward;%sight to steet mm beyond vehicle
sight_backward=input.hood.sight_backward;
height_forward_above_street=input.hood.height_forward_above_street;

Hood_height_F=input.hood.Hood_height_F;%sight height override if 0 => sight level
Hood_height_R=input.hood.Hood_height_R;

% Spaces
Req_KS=input.space.Req_KS;    %estimated  Front Komponent space in Liters
Req_KSR=input.space.Req_KSR;    %estimated  Rear Komponent space in Liters
Targ_Vol_F=input.space.Targ_Vol_F;   %Target Front Trunk Volume in Liters
Targ_Vol_R=input.space.Targ_Vol_R;   %Target Rear Trunk Volume in Liters

% Boxes for Reference Volume
permmat(1:2,:)=[1,2,3;2,1,3]; %all posible orientations
permmat(3:4,:)=[3,2,1;2,3,1]; 
permmat(5:6,:)=[1,3,2;3,1,2]; 

if input.norm %1=DIN 0=SAE
% DIN Box
x=200;
y=100;
z=50;
box=[x,y,z]; %DIN box V=1Liter
dim=box(permmat)'; %Dimension in all Orientations (x-z,Or)
else
% SAE H box
x=325;
y=152;
z=114;
box=[x,y,z]; %SAE Box V=5,6Liter
dim=box(permmat)'; %Dimension in all Orientations 
end
% minimal Luggage measurment
pack_min=input.space.pack_min; %minimal measurement for usable Compartments in mm

Volume_DIN=0;
Orientation=0;
Volume_Euc=0;

%% 2.1.a) FA Dimensions Front
    TFront(1,1,1) = vehicle.dimensions.p_cooler_ctr(1,1)+0.5*vehicle.dimensions.p_cooler_ctr(2,1) + vehicle.dimensions.p_cooling_distance; % Cooler center-pos + half length Cooler + Coolerdistance
    if vehicle.dimensions.p_drivetrain{1}(2,1)==0 % without drivetrain
        TFront(1,1,2) = vehicle.dimensions.p_drivetrain{1}(1,1) - (vehicle.dimensions.CX.upper_bearing_diameter_shock_absorber/2); % allocating damper
    else
        TFront(1,1,2) = vehicle.dimensions.p_drivetrain{1}(1,1) - vehicle.dimensions.p_drivetrain{1}(2,1); %Component with lowest x boundary inc. Axle
    end    
    TFront(1,2,2) = vehicle.dimensions.GY.vehicle_width-vehicle.dimensions.CY.wheelhouse_f_width-Parameters.body.F_Frame_w; % right = total width - wheelhouse - sidemember
    TFront(1,2,1) = vehicle.dimensions.CY.wheelhouse_f_width+Parameters.body.F_Frame_w; %left = wheelhouse + sidemember
    TFront(1,3,1) = vehicle.dimensions.GZ.H156;      % underbody height (lowest possible position)
    TFront(1,3,2) = max(vehicle.dimensions.CZ.wheelhouse_r_height,vehicle.Input.int_height); % up to Wheelhouse height

   
%% 2.2.a) BA Dimensions Front
    TFront(2,1,1) = vehicle.dimensions.p_drivetrain{1}(1,1); % behind drivetrain / axle
    if strcmp(vehicle.Input.int_type,'vav') % backwards seating
        TFront(2,1,2) = min(vehicle.interior.int_boundary_f_new(find(~isnan(vehicle.interior.int_boundary_f_new),1))-vehicle.battery.installationspace.CX_batt_second_level_1(1),vehicle.battery.installationspace.EX_batt_underfloor) ; %rear = seat - 2nd battery
    else %forward seating
        TFront(2,1,2) = vehicle.interior.int_boundary_f2_new(find(~isnan(vehicle.interior.int_boundary_f2_new),1));% rear = footspace
    end
    if TFront(2,1,2)>= (vehicle.dimensions.GX.wheelbase-vehicle.battery.installationspace.CX_batt_underfloor)/2 %intruding underfloor battery
        TFront(2,1,2)= (vehicle.dimensions.GX.wheelbase-vehicle.battery.installationspace.CX_batt_underfloor)/2; %front of underfloor battery
    end
    
    if (vehicle.dimensions.CZ.wheelhouse_r_height<=vehicle.Input.int_height) %interior higher than wheelhouse
        TFront(2,1,2)= (vehicle.dimensions.GX.wheelbase-vehicle.battery.installationspace.CX_batt_underfloor)/2; %front of underfloor battery
        TFront(2,3,1)=  vehicle.dimensions.GZ.H156; % underbody height (lowest possible position)
        TFront(2,3,2)= vehicle.Input.int_height; % floor height
    elseif (TFront(2,1,2) <= (vehicle.dimensions.GX.wheelbase-vehicle.battery.installationspace.CX_batt_underfloor)/2) %infront of underfloor battery
        TFront(2,3,1)=  vehicle.dimensions.GZ.H156; % underbody height (lowest possible position)
        TFront(2,3,2) = max(vehicle.dimensions.CZ.wheelhouse_r_height,vehicle.Input.int_height); %up to Compartment section
    else % above underfloor battery
    TFront(2,3,2) = max(vehicle.dimensions.CZ.wheelhouse_r_height,vehicle.Input.int_height); % wheelhouse or interior higher
    TFront(2,3,1) = min(vehicle.dimensions.CZ.wheelhouse_r_height,vehicle.Input.int_height); % wheelhouse or interior lower
    end   
    TFront(2,1,2) = min(vehicle.interior.int_boundary_f2_new(ceil(TFront(2,3,2))),TFront(2,1,2)); %shift infront of seats          
    TFront(2,2,2) = vehicle.dimensions.GY.vehicle_width-vehicle.dimensions.CY.wheelhouse_f_width- Parameters.body.F_Frame_w; % right = total width - wheelhouse - sidemember
    TFront(2,2,1) = vehicle.dimensions.CY.wheelhouse_f_width+Parameters.body.F_Frame_w; %left = wheelhouse + sidemember


%% 2.3.a) OA Dimensions Front
    TFront(3,3,2) = max(vehicle.dimensions.CZ.wheelhouse_r_height,vehicle.Input.int_height); % up to Wheelhouse height or interior              
    TFront(3,1,1) = TFront(1,1,2); %front=drivetrain front
    TFront(3,1,2) = min(TFront(2,1,1),vehicle.interior.int_boundary_f2_new(ceil(TFront(3,3,2)))); %rear= drivetrain rear
    if vehicle.dimensions.p_drivetrain{1}(2,3)==0 %without drivetrain
       TFront(3,3,1) = vehicle.dimensions.GZ.H156;
    else
    TFront(3,3,1) = vehicle.dimensions.p_drivetrain{1}(1,3)+vehicle.dimensions.p_axle_height;  % Over drivetrain
    end
    TFront(3,2,2) = vehicle.dimensions.GY.vehicle_width-vehicle.dimensions.CY.wheelhouse_f_width-Parameters.body.F_Frame_w; % right = total width - wheelhouse -shock/sidemember
    TFront(3,2,1) = vehicle.dimensions.CY.wheelhouse_f_width + Parameters.body.F_Frame_w; %left = wheelhouse + shock/sidemember

%% 2.4.a) UC Dimensions  Front 
TFront(4,3,1) = max(vehicle.dimensions.CZ.wheelhouse_r_height,vehicle.Input.int_height); %above Wheelhouse height(height same as rear) or interior
if Hood_height_F==0 %no overwrite UNECE sight field
    if strcmp(vehicle.Input.int_type,'vav') %revered seating
        TFront(4,3,2)=max(((sight_backward-rear_to_front)*(SgRP_Z_r + UNECE_offset(2,3)))/(sight_backward), TFront(4,3,1) ); %Rear sight to ground 60m infront of vehicle
    else
        TFront(4,3,2)=max(((sight_foreward*(SgRP_Z + UNECE_offset(2,3) - height_forward_above_street))/(sight_foreward+(SgRP_X + UNECE_offset(2,1)+L104)))+height_forward_above_street, TFront(4,3,1) ); %sight to ground 2m infront of vehicle
    end
else %overwrite
    TFront(4,3,2)=Hood_height_F;
end
if (vehicle.dimensions.GZ.H156+vehicle.dimensions.p_cooler_ctr(2,3)> TFront(4,3,1)) %if cooler up beyond lower compartments
    TFront(4,1,1) = TFront(1,1,1); %compartment behind cooler
else
    TFront(4,1,1) = vehicle.dimensions.p_bumper{1}(1,1) + vehicle.dimensions.p_bumper{1}(2,1)*0.5; % compartment directly behind bumper
end
if ceil(TFront(4,3,2))>find(~isnan(vehicle.interior.int_boundary_f2_new),1,'last') %above seats
    TFront(4,1,2) = min(vehicle.interior.int_boundary_f2_new); %headrest
elseif TFront(4,3,2)<(vehicle.Input.int_height + vehicle.battery.installationspace.CZ_batt_second_level_1(1)) %under seat
    TFront(4,1,2) = vehicle.interior.int_boundary_f2_new(ceil(TFront(4,3,2)))-vehicle.battery.installationspace.CX_batt_second_level_1(1);%rear boundary at 2nd lvl Bat front
else
    TFront(4,1,2) = vehicle.interior.int_boundary_f2_new(ceil(TFront(4,3,2)));%rear boundary at backrest/interior front
end
TFront(4,2,1) = (vehicle.dimensions.GY.vehicle_width - vehicle.interior.int_y_tot)/2; %side boundarys same as interior
TFront(4,2,2) = (vehicle.dimensions.GY.vehicle_width + vehicle.interior.int_y_tot)/2;

%% 3.a) Negative Space Front
inv=find(TFront(:,:,2) < TFront(:,:,1)); %min above max
TFront(inv+12)=TFront(inv);%max=min

%% 4.1.a) Komponentspace distribution Front

Length=TFront(2,1,2)-TFront(1,1,1);
Width=TFront(1,2,2)-TFront(1,2,1);
Height=TFront(1,3,2)-TFront(1,3,1);
Trunk_Dim_F=diff(TFront,1,3); %3 dimentions compartments
Trunk_Vol=prod(Trunk_Dim_F,2); %volume induvidual compartments
Low_Vol=sum(Trunk_Vol(1:3)); %volume total lower compartments
Avl_Tot=(Length*Width*Height)-(vehicle.dimensions.p_drivetrain{1}(2,1)*vehicle.dimensions.p_drivetrain{1}(2,2)*vehicle.dimensions.p_drivetrain{1}(2,3)); %space below wheelhouse height
Avl_KS=Avl_Tot-Low_Vol;%komponent space unchanged

if Req_KS
    if  Avl_Tot<Req_KS %more komponent space that possible under wheelhouse height
        Dif_KS=Req_KS-Avl_Tot; %overfill above wheelhouse
        TFront(4,3,1)= TFront(4,3,1)+(Dif_KS/((TFront(4,1,2)- TFront(4,1,1))*(TFront(4,2,2)-TFront(4,2,1))));
        if TFront(4,3,2)<TFront(4,3,1) %pushing up the upper trunk
            TFront(4,3,2)=TFront(4,3,1);
        end
        TFront(1,3,2)= TFront(4,3,1); %colampsing BA,FA and OA
        TFront(1,3,1)= TFront(4,3,1);
        TFront(2,3,2)= TFront(4,3,1);
        TFront(2,3,1)= TFront(4,3,1);
        TFront(3,3,2)= TFront(4,3,1);
        TFront(3,3,1)= TFront(4,3,1);
        
        TFront(1,1,1)= TFront(1,1,2); %Shifting unused bounaries of BA,FA and OA
        TFront(2,1,2)= TFront(2,1,1);
        TFront(3,1,1)= TFront(2,1,1);
        TFront(3,1,2)= TFront(2,1,1);
        
        if ceil((vehicle.dimensions.GZ.vehicle_height*Parameters.body.Win_Factor)-Parameters.dimensions.CZ.roof_thickness)<TFront(4,3,1) %trunk higher than window edge limit (without roof thickness)
            TFront(4,3,1)=floor(vehicle.dimensions.GZ.vehicle_height*Parameters.body.Win_Factor);
            TFront(4,3,2)=TFront(4,3,1);
            input.space.Act_KS=Avl_Tot;
            %fprintf('Required Front Komponent Space to High for Vehicle \n');
            vehicle.Error=11;
            return
        end
        input.space.Act_KS=Req_KS; %retuning aquired komponent space
        
        
    elseif Avl_KS<Req_KS %more komp. Space
        Dif_KS=Req_KS-Avl_KS; %additional reqired komponent space
        input.space.Act_KS=Req_KS;
        if Dif_KS<=Trunk_Vol(2) %HVAC space behind drivetrain
            TFront(2,1,2)=TFront(2,1,2)-(Trunk_Dim_F(2,1)*(Dif_KS/Trunk_Vol(2))); %convetion BA along neg X
        elseif Dif_KS<=(Trunk_Vol(2)+Trunk_Vol(3)) %HVAC space over drivetrain
            TFront(2,3,1)= TFront(2,3,2);
            TFront(3,1,2)=TFront(3,1,2)-(Trunk_Dim_F(3,1)*((Dif_KS-Trunk_Vol(2))/Trunk_Vol(3)));%convetion OA along neg X
            TFront(2,1,1)= TFront(3,1,2); %colapsing BA
            TFront(2,1,2)= TFront(3,1,2);
        else %HVAC space over drivetrain + lifting FA lower
            TFront(2,3,1)= TFront(2,3,2); %colamping BA and OA
            TFront(3,3,1)= TFront(3,3,2);
            TFront(2,1,1)= TFront(1,1,2);
            TFront(3,1,1)= TFront(1,1,2);
            TFront(2,1,2)= TFront(1,1,2);
            TFront(3,1,2)= TFront(1,1,2);
            TFront(1,3,1)=TFront(1,3,1)+(Trunk_Dim_F(1,3)*((Dif_KS-Trunk_Vol(2)-Trunk_Vol(3))/Trunk_Vol(1)));%convetion FA along pos Z
        end
    else
        input.space.Act_KS=Avl_KS;
    end
end
    
%% 4.2.a) Rasing Lower Compartments if under seat Front
if (TFront(4,1,2) <= TFront(2,1,1))&&~(TFront(2,1,2)>vehicle.interior.int_boundary_f2_new(ceil(TFront(2,3,2)))) % BA under seat
    TFront(2,3,2) = vehicle.Input.int_height + vehicle.battery.installationspace.CZ_batt_second_level_1; %upper =seat
end
if (TFront(4,1,2) <= TFront(3,1,1))&&~(TFront(3,1,2)>vehicle.interior.int_boundary_f2_new(ceil(TFront(3,3,2)))) % OA under seat
    TFront(3,3,2) = vehicle.Input.int_height + vehicle.battery.installationspace.CZ_batt_second_level_1; %upper = seat
end  
%% 4.3.a) Widening lower Compartments if above Sidemember Front
 if TFront(1,3,1)>= (vehicle.dimensions.p_bumper{1}(1,3)+(vehicle.dimensions.p_bumper{1}(2,3)/2))  && TFront(1,3,1)~=TFront(1,3,2) %FA above sidemember
    TFront(1,2,2)= vehicle.dimensions.GY.vehicle_width-vehicle.dimensions.CY.wheelhouse_f_width; %right wheelhouse
    TFront(1,2,1)= vehicle.dimensions.CY.wheelhouse_f_width; %left wheelhouse
 end
 if TFront(2,3,1)>= (vehicle.dimensions.p_bumper{1}(1,3)+(vehicle.dimensions.p_bumper{1}(2,3)/2)) && TFront(2,3,1)~=TFront(2,3,2) %BA above sidemember
    TFront(2,2,2)= vehicle.dimensions.GY.vehicle_width-vehicle.dimensions.CY.wheelhouse_f_width; %right wheelhouse
    TFront(2,2,1)= vehicle.dimensions.CY.wheelhouse_f_width; %left wheelhouse
 end
%% 5.a) Useablilty Front

Trunk_Dim_F=diff(TFront,1,3); %updating dimensions
T_old=TFront;%cloning Array
viable=~~floor(Trunk_Dim_F(:,:)./pack_min); %comp, axis, smaller pack_min
change_comp =~prod(viable,2); %at least one axis not viable
change_ax=(viable.*(change_comp*ones(1,3)).*((sum(viable,2)-1)*ones(1,3)))+(~max((sum(viable,2)-1),0)*ones(1,3)); %only other two axis
wall(:,:,1)=(change_ax).*[1,1,1;0,1,1;0,1,1;1,1,0];%only outer boundaries
wall(:,:,2)=(change_ax).*[0,1,0;1,1,0;0,1,0;1,1,1];
new_ax=[T_old(:,:,1);T_old(:,:,2)];%(all,ax)
other=new_ax.*ones(1,1,4);%(all,ax,comp)
other=reshape(other(~~(([permute(~diag(ones(1,4)),[1 3 2]);permute(~diag(ones(1,4)),[1 3 2])]).*ones(1,3,1))),6,3,4);%only measurments of other compartments(other,ax,comp)
L_old=(permute(T_old(:,:,1),[3,2,1])); %ajusting structure (min,ax,comp)
U_old=(permute(T_old(:,:,2),[3,2,1])); %ajusting structure (max,ax,comp)
change_other_l=(new_ax<=L_old).*[permute(~diag(ones(1,4)),[1 3 2]);permute(~diag(ones(1,4)),[1 3 2])]; %lower value check
change_bound_l=~sum(change_other_l,1).*(permute(wall(:,:,1),[3,2,1])); %check if value needs to be raised 
change_other_u=(new_ax>=U_old).*[permute(~diag(ones(1,4)),[1 3 2]);permute(~diag(ones(1,4)),[1 3 2])]; %higher value check
change_bound_u=~sum(change_other_u,1).*(permute(wall(:,:,2),[3,2,1])); %check if value needs to be lowered
other=sort(other,'descend');
new_bound_l=zeros(1,3,4);
for i=1:3       %loop for new lower boundaries
    for j=1:4
        if change_bound_l(1,i,j) %needs to be changed
        new_bound_l(1,i,j)= other(sum(other(:,i,j)>=L_old(:,i,j)),i,j); %next higher
        end
    end
end
other=sort(other,'ascend');
new_bound_u=zeros(1,3,4);
for i=1:3       %loop for new upper boundaries
    for j=1:4
        if change_bound_u(1,i,j) %needs to be changed
        new_bound_u(1,i,j)= other(sum(other(:,i,j)<=U_old(:,i,j)),i,j); %next lower
        end
    end
end
T_new(1,:,:)=(new_bound_l.*change_bound_l)+(L_old.*~change_bound_l);%assigning new boudary
T_new(2,:,:)=(new_bound_u.*change_bound_u)+(U_old.*~change_bound_u);
TFront=(permute(T_new,[3,2,1]));%transfer Array



%% 2.1.b) FA Dimensions Rear
    TRear(1,1,2) = vehicle.dimensions.GX.wheelbase - vehicle.dimensions.p_bumper{2}(1,1) - vehicle.dimensions.p_bumper{2}(2,1)*0.5; % bumper center-pos + half length Bumper
    if (vehicle.dimensions.p_drivetrain{2}(1,1)==0) %without drivetrain
        TRear(1,1,1) = vehicle.dimensions.GX.wheelbase +(vehicle.dimensions.CX.upper_bearing_diameter_shock_absorber/2); % allocating damper
    else
        TRear(1,1,1) = vehicle.dimensions.GX.wheelbase - vehicle.dimensions.p_drivetrain{2}(1,1)+ vehicle.dimensions.p_drivetrain{2}(2,1); %Component with highest x boundary inc. Axle
    end      
    TRear(1,2,2) = vehicle.dimensions.GY.vehicle_width-vehicle.dimensions.CY.wheelhouse_r_width- Parameters.body.R_Frame_w-vehicle.dimensions.CX.upper_bearing_diameter_shock_absorber; % right = total width - wheelhouse
    TRear(1,2,1) = vehicle.dimensions.CY.wheelhouse_r_width+ Parameters.body.R_Frame_w+vehicle.dimensions.CX.upper_bearing_diameter_shock_absorber; %left = wheelhouse
    TRear(1,3,1) = max(vehicle.dimensions.GZ.H156,((TRear(1,1,2)-vehicle.dimensions.GX.wheelbase)*tand(20)));      % underbody height (lowest components)
    TRear(1,3,2) = max(vehicle.dimensions.CZ.wheelhouse_r_height,vehicle.Input.int_height); % up to Wheelhouse height

%% 2.2.b) BA Dimensions Rear
if (vehicle.dimensions.p_drivetrain{2}(1,1)==0) %without drivetrain
    TRear(2,1,2) = vehicle.dimensions.GX.wheelbase -(vehicle.dimensions.CX.upper_bearing_diameter_shock_absorber/2); % allocating Damper
else
    TRear(2,1,2) = vehicle.dimensions.GX.wheelbase - vehicle.dimensions.p_drivetrain{2}(1,1); % infront drivetrain / axle
end
if strcmp(vehicle.Input.int_type,'btb') % foreward seating 
    TRear(2,1,1) = vehicle.interior.int_boundary_r2_new(find(~isnan(vehicle.interior.int_boundary_r2_new),1));% rear = footspace
elseif strcmp(vehicle.Input.int_type,'sr') % foreward seating
    TRear(2,1,1) = max(vehicle.interior.int_boundary_r_new(find(~isnan(vehicle.interior.int_boundary_r_new),1))+vehicle.battery.installationspace.CX_batt_second_level_1,vehicle.battery.installationspace.EX_batt_underfloor +vehicle.battery.installationspace.CX_batt_underfloor) ;% rear = seat - 1st battery upper
else % foreward seating 
     TRear(2,1,1) = max(vehicle.interior.int_boundary_r_new(find(~isnan(vehicle.interior.int_boundary_r_new),1))+vehicle.battery.installationspace.CX_batt_second_level_2,vehicle.battery.installationspace.EX_batt_underfloor +vehicle.battery.installationspace.CX_batt_underfloor) ;% rear = seat - 2nd battery upper
end
if (vehicle.dimensions.CZ.wheelhouse_r_height<=vehicle.Input.int_height)%interior higher than wheelhouse
    TRear(2,1,1)= (vehicle.dimensions.GX.wheelbase+vehicle.battery.installationspace.CX_batt_underfloor)/2;%front of underfloor battery
    TRear(2,3,1)=  vehicle.dimensions.GZ.H156;% underbody height (lowest possible position)
    TRear(2,3,2)= vehicle.Input.int_height;%floor height
elseif (TRear(2,1,1) >= (vehicle.dimensions.GX.wheelbase+vehicle.battery.installationspace.CX_batt_underfloor)/2)%infront of underfloor battery
    TRear(2,3,1)=  vehicle.dimensions.GZ.H156;% underbody height (lowest possible position)
    TRear(2,3,2) = max(vehicle.dimensions.CZ.wheelhouse_r_height,vehicle.Input.int_height);%up to Compartment section
else
TRear(2,3,2) = max(vehicle.dimensions.CZ.wheelhouse_r_height,vehicle.Input.int_height); % wheelhouse or interior higher
TRear(2,3,1) = min(vehicle.dimensions.CZ.wheelhouse_r_height,vehicle.Input.int_height); % wheelhouse or interior lower
end
TRear(2,1,1) = max([vehicle.interior.int_boundary_r2_new(ceil(TRear(2,3,2))),TRear(2,1,1),]); %shift infront of seats   
TRear(2,2,2) = vehicle.dimensions.GY.vehicle_width-vehicle.dimensions.CY.wheelhouse_r_width- Parameters.body.R_Frame_w-vehicle.dimensions.CX.upper_bearing_diameter_shock_absorber; % right = total width - wheelhouse-sidemember
TRear(2,2,1) = vehicle.dimensions.CY.wheelhouse_r_width+ Parameters.body.R_Frame_w+vehicle.dimensions.CX.upper_bearing_diameter_shock_absorber; %left = wheelhouse + sidemember


%% 2.3.b) OA Dimensions Rear
    TRear(3,3,2) = max(vehicle.dimensions.CZ.wheelhouse_r_height,vehicle.Input.int_height); % up to Wheelhouse height or interior             
    TRear(3,1,2) = TRear(1,1,1); %front=drivetrain front
    TRear(3,1,1) = max(TRear(2,1,2),vehicle.interior.int_boundary_r2_new(ceil(TRear(3,3,2)))); %rear= drivetrain rear
    if vehicle.dimensions.p_drivetrain{2}(2,3)==0 %without drivetrain
       TRear(3,3,1) = vehicle.dimensions.GZ.H156 ; 
    else
    TRear(3,3,1) = vehicle.dimensions.p_drivetrain{2}(1,3)+vehicle.dimensions.p_axle_height;  % Over drivetrain (measured from axle height)
    end
    TRear(3,2,2) = vehicle.dimensions.GY.vehicle_width-vehicle.dimensions.CY.wheelhouse_r_width- (vehicle.dimensions.CX.upper_bearing_diameter_shock_absorber+Parameters.body.F_Frame_w); % right = total width - wheelhouse -shock/sidemember
    TRear(3,2,1) = vehicle.dimensions.CY.wheelhouse_r_width+ (vehicle.dimensions.CX.upper_bearing_diameter_shock_absorber+Parameters.body.F_Frame_w); %left = wheelhouse +shock/sidemember

%% 2.4.b) UC Dimensions  Rear  
    TRear(4,3,1) = max(vehicle.dimensions.CZ.wheelhouse_r_height,vehicle.Input.int_height); %above Wheelhouse height or interior
if Hood_height_R==0 %no overwrite UNECE sight field
    if strcmp(vehicle.Input.int_type,'btb')
        TRear(4,3,2)=max((((sight_foreward*(SgRP_Z_r + UNECE_offset(2,3) -height_forward_above_street))/(sight_foreward+(vehicle.dimensions.GX.wheelbase+L105-SgRP_X_r - UNECE_offset(2,1))))+height_forward_above_street),TRear(4,3,1)); %rear sight to ground 2m behind  vehicle
    else 
        TRear(4,3,2)=max(((sight_backward-driver_to_back)*(SgRP_Z + UNECE_offset(2,3)))/(sight_backward),TRear(4,3,1)); %driver sight to ground 60m behind vehicle
    end
else %overwrite
    TRear(4,3,2)=Hood_height_R;
end
TRear(4,1,2) = vehicle.dimensions.GX.wheelbase - vehicle.dimensions.p_bumper{2}(1,1) - vehicle.dimensions.p_bumper{2}(2,1)*0.5;  % compartment directly behind bumper
if ceil(TRear(4,3,2))>find(~isnan(vehicle.interior.int_boundary_r2_new),1,'last') %above seats
TRear(4,1,1) = max(vehicle.interior.int_boundary_r2_new); %headrest
elseif TRear(4,3,2)<vehicle.Input.int_height + vehicle.battery.installationspace.CZ_batt_second_level_2(1) %under seat
TRear(4,1,1) = vehicle.interior.int_boundary_r2_new(ceil(TRear(4,3,2)))+vehicle.battery.installationspace.CX_batt_second_level_2(1);%rear boundary at 2nd lvl Bat front
else
TRear(4,1,1) = vehicle.interior.int_boundary_r2_new(ceil(TRear(4,3,2)));%front boundary at backrest
end
TRear(4,2,1) = (vehicle.dimensions.GY.vehicle_width - vehicle.interior.int_y_tot)/2; %side boundarys same as interior
TRear(4,2,2) = (vehicle.dimensions.GY.vehicle_width + vehicle.interior.int_y_tot)/2;
%% 3.b) Negative space rear
inv=find(TRear(:,:,2) < TRear(:,:,1)); %min above max
TRear(inv+12)=TRear(inv); %max=min

%% 4.1.b) Componentspace distribution rear

LengthR=TRear(1,1,2)-TRear(2,1,1);
WidthR=TRear(1,2,2)-TRear(1,2,1);
HeightR=TRear(1,3,2)-TRear(1,3,1);
Trunk_Dim_R=diff(TRear,1,3); %3 dimentions compartments
Trunk_Vol_R=prod(Trunk_Dim_R,2); %volume induvidual compartments
Low_Vol_R=sum(Trunk_Vol_R(1:3)); %volume total lower compartments
Avl_Tot_R=(LengthR*WidthR*HeightR)-(vehicle.dimensions.p_drivetrain{2}(2,1)*vehicle.dimensions.p_drivetrain{2}(2,2)*vehicle.dimensions.p_drivetrain{2}(2,3));%space below wheelhouse height
Avl_KSR=Avl_Tot_R-Low_Vol_R;%komponent space unchanged
if Targ_Vol_R==0
    Req_KSR=Avl_KSR;
end

if Req_KSR
    if Avl_Tot_R<Req_KSR %more space required than available in vehicle
        Req_KSR=Avl_Tot_R;
        input.space.Act_KSR=Req_KSR;
        %fprintf('Required Rear Komponent Space to High for Vehicle \n');
        vehicle.Error=11;
        
    end
    if Avl_KSR<Req_KSR %more komp. Space
        Dif_KSR=Req_KSR-Avl_KSR; %additional reqired komponent space
        input.space.Act_KSR=Req_KSR;
        if TRear(3,3,1)>TRear(2,3,1) %OA higher BA
            if Dif_KSR<=(Trunk_Dim_R(2,1)*Trunk_Dim_R(2,2)*(TRear(3,3,1)-TRear(2,3,1))) %space behind drivetrain
                TRear(2,3,1)=TRear(2,3,1)+(Trunk_Dim_R(2,3)*(Dif_KSR/Trunk_Vol_R(2))); %convertion BA along posZ
            elseif Dif_KSR<=((Trunk_Dim_R(2,1)*Trunk_Dim_R(2,2)*(TRear(3,3,1)-TRear(2,3,1)))+(Trunk_Dim_R(1,1)*Trunk_Dim_R(1,2)*(TRear(3,3,1)-TRear(1,3,1)))) %space infornt of drivetrain
                TRear(1,3,1)=TRear(1,3,1)+(Trunk_Dim_R(1,3)*((Dif_KSR-(Trunk_Dim_R(2,1)*Trunk_Dim_R(2,2)*(TRear(3,3,1)-TRear(2,3,1))))/Trunk_Vol_R(1)));%convertion FA along posZ
                TRear(2,3,1)=TRear(3,3,1); %Ba convertion
            else %space over drivetrain + FA +BA
                TRear(3,3,1)=TRear(3,3,1)+((Dif_KSR-(Trunk_Dim_R(2,1)*Trunk_Dim_R(2,2)*(TRear(3,3,1)-TRear(2,3,1)))-(Trunk_Dim_R(1,1)*Trunk_Dim_R(1,2)*(TRear(3,3,1)-TRear(1,3,1))))/(WidthR*LengthR));%convertion OA+FA+BA along pos Z
                TRear(2,3,1)=TRear(3,3,1);
                TRear(1,3,1)=TRear(3,3,1);
            end
        else
            if Dif_KSR<=(Trunk_Dim_R(1,1)*Trunk_Dim_R(1,2)*(TRear(3,3,1)-TRear(1,3,1))) %space infront of drivetrain
                TRear(1,3,1)=TRear(1,3,1)+(Trunk_Dim_R(1,3)*(Dif_KSR/Trunk_Vol_R(1))); %convertion FA along posZ
            elseif Dif_KSR<=((Trunk_Dim_R(1,1)*Trunk_Dim_R(1,2)*(TRear(2,3,1)-TRear(1,3,1)))+(Trunk_Dim_R(3,1)*Trunk_Dim_R(3,2)*(TRear(2,3,1)-TRear(3,3,1)))) %space over drivetrain
                TRear(3,3,1)=TRear(3,3,1)+((Dif_KSR-(Trunk_Dim_R(1,1)*Trunk_Dim_R(1,2)*(TRear(3,3,1)-TRear(1,3,1))))/(Trunk_Dim_R(3,2)*(Trunk_Dim_R(1,1)+Trunk_Dim_R(3,1))));%convertion OA+FA along pos Z
                TRear(1,3,1)=TRear(3,3,1);
            else %space over drivetrain + FA +BA
                TRear(3,3,1)=TRear(3,3,1)+((Dif_KSR-(Trunk_Dim_R(3,1)*Trunk_Dim_R(3,2)*(TRear(2,3,1)-TRear(3,3,1)))-(Trunk_Dim_R(1,1)*Trunk_Dim_R(1,2)*(TRear(2,3,1)-TRear(1,3,1))))/(WidthR*LengthR));%convertion OA+FA+BA along pos Z
                TRear(2,3,1)=TRear(3,3,1);
                TRear(1,3,1)=TRear(3,3,1);
            end
        end
    else
        input.space.Act_KSR=Avl_KSR;
    end
end
 
%% 4.2.b) Rasing Lower Compartments if unter seat Rear
if (TRear(4,1,1) >= TRear(2,1,2))&&(TRear(2,1,1)<vehicle.interior.int_boundary_r2_new(ceil(TRear(2,3,2)))) % ba under seat
    TRear(2,3,2) = vehicle.Input.int_height + vehicle.battery.installationspace.CZ_batt_second_level_2; %upper =seat
    
end
if (TRear(4,1,1) >= TRear(3,1,2))&&(TRear(3,1,1)<vehicle.interior.int_boundary_r2_new(ceil(TRear(3,3,2)))) % oa under seat
    TRear(3,3,2) = vehicle.Input.int_height + vehicle.battery.installationspace.CZ_batt_second_level_2; %upper = seat
    
end
%% 4.3.b) Widening lower Compartments if above Sidemember Rear
     if TRear(1,3,1)>= (vehicle.dimensions.p_bumper{2}(1,3)+(vehicle.dimensions.p_bumper{2}(2,3)/2)) && TRear(1,3,1)~=TRear(1,3,2) %FA above sidemember
        TRear(1,2,2)= vehicle.dimensions.GY.vehicle_width-vehicle.dimensions.CY.wheelhouse_r_width; %right wheelhouse
        TRear(1,2,1)= vehicle.dimensions.CY.wheelhouse_r_width; %left Wheelhouse
     end
      if TRear(2,3,1)>= (vehicle.dimensions.p_bumper{2}(1,3)+(vehicle.dimensions.p_bumper{2}(2,3)/2)) && TRear(2,3,1)~=TRear(2,3,2) %BA above sidemember
        TRear(2,2,2)= vehicle.dimensions.GY.vehicle_width-vehicle.dimensions.CY.wheelhouse_r_width; %right wheelhouse
        TRear(2,2,1)= vehicle.dimensions.CY.wheelhouse_r_width; %left Wheelhouse
     end
     
%% 5.b) Useablilty Rear
Trunk_Dim_R=diff(TRear,1,3);
T_old=TRear;
viable=~~floor(Trunk_Dim_R(:,:)./pack_min); %comp, axis, smaller pack_min
change_comp =~prod(viable,2); %at leats one axis not viable
change_ax=(viable.*(change_comp*ones(1,3)).*((sum(viable,2)-1)*ones(1,3)))+(~max((sum(viable,2)-1),0)*ones(1,3)); %only other two axis
wall(:,:,1)=(change_ax).*[0,1,1;1,1,1;0,1,1;1,1,0];%only outer boundaries
wall(:,:,2)=(change_ax).*[1,1,0;0,1,0;0,1,0;1,1,1];
new_ax=[T_old(:,:,1);T_old(:,:,2)];%(all,ax)
other=new_ax.*ones(1,1,4);%(all,ax,comp)
other=reshape(other(~~(([permute(~diag(ones(1,4)),[1 3 2]);permute(~diag(ones(1,4)),[1 3 2])]).*ones(1,3,1))),6,3,4);%(other,ax,comp) 
L_old=(permute(T_old(:,:,1),[3,2,1])); %ajusting structure (min,ax,comp)
U_old=(permute(T_old(:,:,2),[3,2,1])); %ajusting structure (max,ax,comp)
change_other_l=(new_ax<=L_old).*[permute(~diag(ones(1,4)),[1 3 2]);permute(~diag(ones(1,4)),[1 3 2])]; %lower value check
change_bound_l=~sum(change_other_l,1).*(permute(wall(:,:,1),[3,2,1]));%check if value needs to be raised 
change_other_u=(new_ax>=U_old).*[permute(~diag(ones(1,4)),[1 3 2]);permute(~diag(ones(1,4)),[1 3 2])]; %higher value check
change_bound_u=~sum(change_other_u,1).*(permute(wall(:,:,2),[3,2,1]));%check if value needs to be lowered 
other=sort(other,'descend');
for i=1:3       %loop for new lower boundaries
    for j=1:4
        if change_bound_l(1,i,j) %needs to be changed
        new_bound_l(1,i,j)= other(sum(other(:,i,j)>=L_old(:,i,j)),i,j); %next higher
        end
    end
end
other=sort(other,'ascend');
for i=1:3       %loop for new lower boundaries
    for j=1:4
        if change_bound_u(1,i,j) %needs to be changed
        new_bound_u(1,i,j)= other(sum(other(:,i,j)<=U_old(:,i,j)),i,j); %next lower
        end
    end
end
T_new(1,:,:)=(new_bound_l.*change_bound_l)+(L_old.*~change_bound_l); %assigning new boundary
T_new(2,:,:)=(new_bound_u.*change_bound_u)+(U_old.*~change_bound_u);
TRear=(permute(T_new,[3,2,1])); %transfering Array

%% 6) Target Volume
Trunk_Dim_F=diff(TFront,1,3); %3 dimentions compartments front
Trunk_Vol=prod(Trunk_Dim_F,2); %volume induvidual compartments front
Trunk_Dim_R=diff(TRear,1,3); %3 dimentions compartments rear
Trunk_Vol_R=prod(Trunk_Dim_R,2); %volume induvidual compartments rear
Tot_Vol=sum([Trunk_Vol;Trunk_Vol_R]); %volume of all compartments

Tot_Vol_F=sum(Trunk_Vol);
Tot_Vol_R=sum(Trunk_Vol_R);
Dif_TS=Tot_Vol_F-Targ_Vol_F;
Dif_TSR=Tot_Vol_R-Targ_Vol_R;
t_win_f=max(ceil(vehicle.dimensions.GZ.vehicle_height*Parameters.body.Win_Factor),ceil(TFront(4,3,1))); %height of window for 'hatchfront' restriction
t_win_r=max(ceil(vehicle.dimensions.GZ.vehicle_height*Parameters.body.Win_Factor),ceil(TRear(4,3,1)));%height of window for hatchback restriction

if (Dif_TS>Trunk_Vol(4)) %Error Komponent and Luggage space (missing elseif for execution)
    TFront(4,3,2)=TFront(4,3,1); %eliminating Front UC
    Dif_KS=Dif_TS-Trunk_Vol(4);
    %fprintf('Target Front Trunk Volume to Low adjusting Komponent Space \n');
    input.space.Act_KS=input.space.Act_KS+Dif_KS;
    
    
    Length=TFront(2,1,2)-TFront(1,1,1);
    Width=TFront(1,2,2)-TFront(1,2,1);
    Height=TFront(1,3,2)-TFront(1,3,1);
    if Dif_KS<=Trunk_Vol(2) %HVAC space behind drivetrain
        TFront(2,1,2)=TFront(2,1,2)-(Trunk_Dim_F(2,1)*(Dif_KS/Trunk_Vol(2))); %convetion BA along neg X
    elseif Dif_KS<=(Trunk_Vol(2)+Trunk_Vol(3)) %HVAC space over drivetrain
        TFront(2,3,1)= TFront(2,3,2);
        TFront(3,1,2)=TFront(3,1,2)-(Trunk_Dim_F(3,1)*((Dif_KS-Trunk_Vol(2))/Trunk_Vol(3)));%convetion OA along neg X
        TFront(2,1,1)= TFront(3,1,2); %colapsing BA
        TFront(2,1,2)= TFront(3,1,2);
    else %HVAC space over drivetrain + lifting FA lower
        TFront(2,3,1)= TFront(2,3,2); %colamping BA and OA
        TFront(3,3,1)= TFront(3,3,2);
        TFront(2,1,1)= TFront(1,1,2);
        TFront(3,1,1)= TFront(1,1,2);
        TFront(2,1,2)= TFront(1,1,2);
        TFront(3,1,2)= TFront(1,1,2);
        TFront(1,3,1)=TFront(1,3,1)+(Trunk_Dim_F(1,3)*((Dif_KS-Trunk_Vol(2)-Trunk_Vol(3))/Trunk_Vol(1)));%convetion FA along pos Z
    end
else
    
    %Silhouette
    t_height_f=(ceil(TFront(4,3,1)):length(vehicle.interior.int_boundary_f2_new))'; %z-height matrix to seat
    t_back_f=vehicle.interior.int_boundary_f2_new(t_height_f); %rear Trunkspace edge at seatheight
    t_height_f=(ceil(TFront(4,3,1)):ceil(vehicle.dimensions.GZ.vehicle_height))';%z-height matrix to roof
    t_back_f=[t_back_f;t_back_f(end).*ones((ceil(vehicle.dimensions.GZ.vehicle_height)-length(vehicle.interior.int_boundary_f2_new)),1)];%extending rear edge from seats up to roof in z-direction
    if vehicle.Input.window_angle_front==0
        t_front_f=TFront(4,1,1).*ones((ceil(vehicle.dimensions.GZ.vehicle_height)-floor(TFront(4,3,1))),1);%vertikal front trunk edge along window upward
    else
        t_front_f=[TFront(4,1,1)*ones((t_win_f-ceil(TFront(4,3,1))),1);... %angled front trunk edge along window upward
            ((TFront(4,1,1)):...
            (((ceil(vehicle.dimensions.GZ.vehicle_height)-t_win_f)*sin(vehicle.Input.window_angle_front *(pi/180)))/(ceil(vehicle.dimensions.GZ.vehicle_height)-t_win_f)):...
            ((ceil(vehicle.dimensions.GZ.vehicle_height)-t_win_f)*sin(vehicle.Input.window_angle_front *(pi/180)))+(TFront(4,1,1)))'];
    end
    
    %              figure
    %              hold on
    %              plot([1:1:length(t_back_f)],t_back_f)
    %              plot(t_front_f)
    %              hold off
    
    h_pos_f=find(((t_height_f-TFront(4,3,1)).*(t_back_f-t_front_f))>=((Trunk_Vol(4)-Dif_TS)/Trunk_Dim_F(4,2)),1);%required volume divided by width (is req. side area) equal to lenght times height (is aval. side Area)
    
    if h_pos_f~=0
        TFront(4,3,2)=t_height_f(h_pos_f);
        TFront(4,1,2)=t_back_f(h_pos_f);
        TFront(4,1,1)=t_front_f(h_pos_f);
    else %front trunk to big, not feasible
        vehicle.Error=12;
        
    end
    
end

if (Dif_TSR>Trunk_Vol_R(4)) %Error Komponent and Luggage space (missing elseif for execution)
    TRear(4,3,2)=TRear(4,3,1); %eliminating Front UC
    Dif_KSR=Dif_TSR-Trunk_Vol_R(4);
    %fprintf('Target Rear Trunk Volume to Low adjusting Komponent Space \n');
    input.space.Act_KSR=input.space.Act_KSR+Dif_KSR;
    
    
    LengthR=TRear(1,1,2)-TRear(2,1,1);
    WidthR=TRear(1,2,2)-TRear(1,2,1);
    HeightR=TRear(1,3,2)-TRear(1,3,1);
    if TRear(3,3,1)>TRear(2,3,1) %OA higher BA
        if Dif_KSR<=(Trunk_Dim_R(2,1)*Trunk_Dim_R(2,2)*(TRear(3,3,1)-TRear(2,3,1))) %space behind drivetrain
            TRear(2,3,1)=TRear(2,3,1)+(Trunk_Dim_R(2,3)*(Dif_KSR/Trunk_Vol_R(2))); %convertion BA along posZ
        elseif Dif_KSR<=((Trunk_Dim_R(2,1)*Trunk_Dim_R(2,2)*(TRear(3,3,1)-TRear(2,3,1)))+(Trunk_Dim_R(1,1)*Trunk_Dim_R(1,2)*(TRear(3,3,1)-TRear(1,3,1)))) %space infornt of drivetrain
            TRear(1,3,1)=TRear(1,3,1)+(Trunk_Dim_R(1,3)*((Dif_KSR-(Trunk_Dim_R(2,1)*Trunk_Dim_R(2,2)*(TRear(3,3,1)-TRear(2,3,1))))/Trunk_Vol_R(1)));%convertion FA along posZ
            TRear(2,3,1)=TRear(3,3,1); %Ba convertion
        else %space over drivetrain + FA +BA
            TRear(3,3,1)=TRear(3,3,1)+((Dif_KSR-(Trunk_Dim_R(2,1)*Trunk_Dim_R(2,2)*(TRear(3,3,1)-TRear(2,3,1)))-(Trunk_Dim_R(1,1)*Trunk_Dim_R(1,2)*(TRear(3,3,1)-TRear(1,3,1))))/(WidthR*LengthR));%convertion OA+FA+BA along pos Z
            TRear(2,3,1)=TRear(3,3,1);
            TRear(1,3,1)=TRear(3,3,1);
        end
    else
        if Dif_KSR<=(Trunk_Dim_R(1,1)*Trunk_Dim_R(1,2)*(TRear(3,3,1)-TRear(1,3,1))) %space infront of drivetrain
            TRear(1,3,1)=TRear(1,3,1)+(Trunk_Dim_R(1,3)*(Dif_KSR/Trunk_Vol_R(1))); %convertion FA along posZ
        elseif Dif_KSR<=((Trunk_Dim_R(1,1)*Trunk_Dim_R(1,2)*(TRear(2,3,1)-TRear(1,3,1)))+(Trunk_Dim_R(3,1)*Trunk_Dim_R(3,2)*(TRear(2,3,1)-TRear(3,3,1)))) %space over drivetrain
            TRear(3,3,1)=TRear(3,3,1)+((Dif_KSR-(Trunk_Dim_R(1,1)*Trunk_Dim_R(1,2)*(TRear(3,3,1)-TRear(1,3,1))))/(Trunk_Dim_R(3,2)*(Trunk_Dim_R(1,1)+Trunk_Dim_R(3,1))));%convertion OA+FA along pos Z
            TRear(1,3,1)=TRear(3,3,1);
        else %space over drivetrain + FA +BA
            TRear(3,3,1)=TRear(3,3,1)+((Dif_KSR-(Trunk_Dim_R(3,1)*Trunk_Dim_R(3,2)*(TRear(2,3,1)-TRear(3,3,1)))-(Trunk_Dim_R(1,1)*Trunk_Dim_R(1,2)*(TRear(2,3,1)-TRear(1,3,1))))/(WidthR*LengthR));%convertion OA+FA+BA along pos Z
            TRear(2,3,1)=TRear(3,3,1);
            TRear(1,3,1)=TRear(3,3,1);
        end
    end
else
    
    %Silhouette
    t_height_r=(ceil(TRear(4,3,1)):length(vehicle.interior.int_boundary_r2_new))';%height matrix to seat
    t_front_r=vehicle.interior.int_boundary_r2_new(t_height_r);%front Trunkspace edge at seatheight
    t_height_r=(ceil(TRear(4,3,1)):ceil(vehicle.dimensions.GZ.vehicle_height))';%height matrix ro roof
    t_front_r=[t_front_r;t_front_r(end).*ones((ceil(vehicle.dimensions.GZ.vehicle_height)-length(vehicle.interior.int_boundary_r2_new)),1)];%extrending front edge from seats up to roof
    if vehicle.Input.window_angle_rear==0
        t_back_r=TRear(4,1,2).*ones((ceil(vehicle.dimensions.GZ.vehicle_height)-floor(TRear(4,3,1))),1);%vertical rear trunk edge along window upward
    else
        t_back_r=[TRear(4,1,2)*ones((t_win_r-ceil(TRear(4,3,1))),1);... %angled rear trunk edge along window upward
            ((TRear(4,1,2)):...
            ((-(ceil(vehicle.dimensions.GZ.vehicle_height)-t_win_r)*sin(vehicle.Input.window_angle_rear *(pi/180)))/(ceil(vehicle.dimensions.GZ.vehicle_height)-t_win_r)):...
            (-(ceil(vehicle.dimensions.GZ.vehicle_height)-t_win_r)*sin(vehicle.Input.window_angle_rear *(pi/180)))+(TRear(4,1,2)))'];
    end
    h_pos_r=find(((t_height_r-TRear(4,3,1)).*(t_back_r-t_front_r))>=((Trunk_Vol_R(4)-Dif_TSR)/Trunk_Dim_R(4,2)),1);%required volume divided by width (is req. side area) equal to lenght times height (is aval. side Area)
    if h_pos_r~=0
        TRear(4,3,2)=t_height_r(h_pos_r);
        TRear(4,1,1)=t_front_r(h_pos_r);
        TRear(4,1,2)=t_back_r(h_pos_r);
    else %Rear trunk to big, not feasible
        vehicle.Error=12;
        
    end
end


if (TRear(4,3,2)>(vehicle.dimensions.GZ.vehicle_height-Parameters.dimensions.CZ.roof_thickness))||(TFront(4,3,2)>(vehicle.dimensions.GZ.vehicle_height-Parameters.dimensions.CZ.roof_thickness)) %higher than vehicle front or rear
    
    vehicle.dimensions.GZ.vehicle_height=max(TRear(4,3,2),TFront(4,3,2))+Parameters.dimensions.CZ.roof_thickness;
    vehicle.Error=12;
end




%% 7.a) ISO 3832 Reference Volume Front
Trunk_Dim_F=diff(TFront,1,3);
[Volume_DIN(1),U(1),F(1),B(1),O(1)]=Package_DIN_Volume(Trunk_Dim_F,dim,box);

%% 7.b) ISO 3832 Reference Volume Rear
Trunk_Dim_R=diff(TRear,1,3);
[Volume_DIN(2),U(2),F(2),B(2),O(2)]=Package_DIN_Volume(Trunk_Dim_R,dim,box);
Orientation=[U',F',B',O'];


%% Euclidic Volume
Trunk_Dim_F=diff(TFront,1,3); %3 dimentions compartments front
Trunk_Vol=prod(Trunk_Dim_F,2); %volume induvidual compartments front
Trunk_Dim_R=diff(TRear,1,3); %3 dimentions compartments rear
Trunk_Vol_R=prod(Trunk_Dim_R,2); %volume induvidual compartments rear
Volume_Euc=sum([Trunk_Vol,Trunk_Vol_R],1)/(10^6); %volume of all compartments inLiters


%% Return values
input.hood.Trunk_height_F=max(TFront(4,3,2),vehicle.dimensions.p_cooler_ctr(1,3)+(vehicle.dimensions.p_cooler_ctr(2,3)/2));
input.hood.Trunk_height_R=TRear(4,3,2);
if Trunk_Vol(4)==0
    input.hood.Trunk_edge_F=TFront(1,1,1);
else
input.hood.Trunk_edge_F=min(TFront(4,1,1),TFront(1,1,1));
end
if Trunk_Vol_R(4)==0
    input.hood.Trunk_edge_R=TRear(1,1,2);
else
input.hood.Trunk_edge_R=max(TRear(4,1,2),TRear(1,1,2));
end
input.hood.Win_edge_F=t_win_f;
input.hood.Win_edge_R=t_win_r;
input.space.Targ_Vol_F=Targ_Vol_F;   %Target Front Trunk Volume in Liters
input.space.Targ_Vol_R=Targ_Vol_R;  %Target Rear Trunk Volume in Liters

vehicle.dimensions.p_trunk_front{1}(2,:)=Trunk_Dim_F(1,:);
vehicle.dimensions.p_trunk_front{2}(2,:)=Trunk_Dim_F(2,:);
vehicle.dimensions.p_trunk_front{3}(2,:)=Trunk_Dim_F(3,:);
vehicle.dimensions.p_trunk_front{4}(2,:)=Trunk_Dim_F(4,:);

vehicle.dimensions.p_trunk_front{1}(1,:)=TFront(1,:,1)+(Trunk_Dim_F(1,:)/2);
vehicle.dimensions.p_trunk_front{2}(1,:)=TFront(2,:,1)+(Trunk_Dim_F(2,:)/2);
vehicle.dimensions.p_trunk_front{3}(1,:)=TFront(3,:,1)+(Trunk_Dim_F(3,:)/2);
vehicle.dimensions.p_trunk_front{4}(1,:)=TFront(4,:,1)+(Trunk_Dim_F(4,:)/2);

vehicle.dimensions.p_trunk_rear{1}(2,:)=Trunk_Dim_R(1,:);
vehicle.dimensions.p_trunk_rear{2}(2,:)=Trunk_Dim_R(2,:);
vehicle.dimensions.p_trunk_rear{3}(2,:)=Trunk_Dim_R(3,:);
vehicle.dimensions.p_trunk_rear{4}(2,:)=Trunk_Dim_R(4,:);

vehicle.dimensions.p_trunk_rear{1}(1,:)=TRear(1,:,1)+(Trunk_Dim_R(1,:)/2);
vehicle.dimensions.p_trunk_rear{2}(1,:)=TRear(2,:,1)+(Trunk_Dim_R(2,:)/2);
vehicle.dimensions.p_trunk_rear{3}(1,:)=TRear(3,:,1)+(Trunk_Dim_R(3,:)/2);
vehicle.dimensions.p_trunk_rear{4}(1,:)=TRear(4,:,1)+(Trunk_Dim_R(4,:)/2);

plot_switch=0;
if plot_switch==1    
    %% 8) Plot
    clf; %clearing last window
    disp = axes; %creating koordinates
    DISPLAY_vehicle(vehicle,Parameters,disp); %displaying vehicle
    hold (disp);
    eu_t=[0;0;0];
    
    %Plot Front Compartments
    [F_t,V_t]=Object3d_Block(TFront(1,:,1)',eu_t,Trunk_Dim_F(1,1),Trunk_Dim_F(1,2),Trunk_Dim_F(1,3));
    patch(disp,'Faces', F_t, 'Vertices', V_t, 'FaceColor', 'red');
    [F_t,V_t]=Object3d_Block(TFront(2,:,1)',eu_t,Trunk_Dim_F(2,1),Trunk_Dim_F(2,2),Trunk_Dim_F(2,3));
    patch(disp,'Faces', F_t, 'Vertices', V_t, 'FaceColor', 'red');
    [F_t,V_t]=Object3d_Block(TFront(3,:,1)',eu_t,Trunk_Dim_F(3,1),Trunk_Dim_F(3,2),Trunk_Dim_F(3,3));
    patch(disp,'Faces', F_t, 'Vertices', V_t, 'FaceColor', 'red');
    [F_t,V_t]=Object3d_Block(TFront(4,:,1)',eu_t,Trunk_Dim_F(4,1),Trunk_Dim_F(4,2),Trunk_Dim_F(4,3));
    patch(disp,'Faces', F_t, 'Vertices', V_t, 'FaceColor', 'red');
    %Plot Rear Compartments
    [F_t,V_t]=Object3d_Block(TRear(1,:,1)',eu_t,Trunk_Dim_R(1,1),Trunk_Dim_R(1,2),Trunk_Dim_R(1,3));
    patch(disp,'Faces', F_t, 'Vertices', V_t, 'FaceColor', 'green');
    [F_t,V_t]=Object3d_Block(TRear(2,:,1)',eu_t,Trunk_Dim_R(2,1),Trunk_Dim_R(2,2),Trunk_Dim_R(2,3));
    patch(disp,'Faces', F_t, 'Vertices', V_t, 'FaceColor', 'green');
    [F_t,V_t]=Object3d_Block(TRear(3,:,1)',eu_t,Trunk_Dim_R(3,1),Trunk_Dim_R(3,2),Trunk_Dim_R(3,3));
    patch(disp,'Faces', F_t, 'Vertices', V_t, 'FaceColor', 'green');
    [F_t,V_t]=Object3d_Block(TRear(4,:,1)',eu_t,Trunk_Dim_R(4,1),Trunk_Dim_R(4,2),Trunk_Dim_R(4,3));
    patch(disp,'Faces', F_t, 'Vertices', V_t, 'FaceColor', 'green');
    
    hold (disp,'off');
end
end

