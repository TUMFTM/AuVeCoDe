function [vehicle] = DESIGN_trunk_space(vehicle,Parameters)
%% Description:
% Designed by: Michael Mast, Adrian König (FTM, Technical University of Munich)
%-------------
% Created on: 01.06.2021
% ------------
% Version: Matlab2020b
%-------------
% Description:  This main function calls the calculation function for the trunk volume and
%               iterativly increases the vehicle wagon height if needed
% ------------
% Sources:  [1] Michael Mast, “Packageplanung von autonomen Fahrzeugkonzepten im Vorder- und Hinterwagen,” Semester Thesis, Technical University of Munich, Institute of Automotive Technology, 2021
%           [2] Michael Mast, “Karosseriemodellierung autonomer Elektrofahrzeuge,” Master Thesis, Technical University of Munich, Institute of Automotive Technology, 2022
%           [3] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------

%% Input:
% vehicle structure
% fixed parameters

%% Output
% updated vehicle struct

%% Implementation
%1) Reading in input Variables
%2) Executing Funktion 
%3) adjusting input parameters
%4) output parameters


%% Input variables
%sight variables
input.hood.sight_foreward=2000; %sight to steet mm infornt of vehicle
input.hood.sight_backward=60000;%sight to steet mm behind vehicle
input.hood.height_forward_above_street=300; %hight above street

%sight height override  if 0 => sight level
input.hood.Hood_height_F=vehicle.dimensions.CZ.wheelhouse_r_height*1.5;
input.hood.Hood_height_R=vehicle.dimensions.CZ.wheelhouse_r_height*1.5;

%Spaces
Req_KS=350;    %estimated  Front Komponent space in Liters
Req_KSR=0;    %estimated  Rear Komponent space in Liters
input.space.Req_KS=1000000*Req_KS;
input.space.Req_KSR=1000000*Req_KSR;
input.space.Targ_Vol_F=vehicle.dimensions.trunk_vol_front;   %Target Total Trunk Volume in Liters
input.space.Targ_Vol_R=vehicle.dimensions.trunk_vol_rear;   %Target Total Trunk Volume in Liters
input.space.pack_min=200; %minimal measurement for usable Compartments in mm
input.space.Act_KS=0;
input.space.Act_KSR=0;

%Norm
input.norm=1;%1=DIN 0=SAE

%loop variables
vehicle.dimensions.trunk_vol_front_temp=0;
vehicle.dimensions.trunk_vol_rear_temp=0;
vehicle.dimensions.komp_vol_front_temp=0;
vehicle.dimensions.komp_vol_rear_temp=0;
iteration=1;
check_f=1;
check_r=1;
size_factor=0.9;

while(iteration<15&&(check_f||check_r))
%% Funktion execution
[Volume_DIN,Orientation,Volume_Euc,vehicle,input] = Package_Trunk(input,vehicle,Parameters);
 if vehicle.Error==11 %abort calculation in case of Error
     return
 end
% %% Result Display
% 
% %Output
%  fprintf('%4.0f Liters DIN Volume\n',Volume_DIN(1)+Volume_DIN(2));
%  fprintf('%4.1f Liter Eucledian Volume\n',Volume_Euc(1)+Volume_Euc(2));

%% Adjusting Inputs
Dif_F=vehicle.dimensions.trunk_vol_front-(Volume_DIN(1)*10^6);
Dif_R=vehicle.dimensions.trunk_vol_rear-(Volume_DIN(2)*10^6);
Dif_F_temp=vehicle.dimensions.trunk_vol_front-vehicle.dimensions.trunk_vol_front_temp;
Dif_R_temp=vehicle.dimensions.trunk_vol_rear-vehicle.dimensions.trunk_vol_rear_temp;

if iteration==1 %fist iteration values
    vehicle.dimensions.komp_vol_front_temp=input.space.Act_KS;   %temporary komponent space
    vehicle.dimensions.trunk_vol_front_temp=(Volume_DIN(1)*10^6);%temporary trunk volume
    vehicle.dimensions.komp_vol_rear_temp=input.space.Act_KSR;   %temporary komponent space
    vehicle.dimensions.trunk_vol_rear_temp=(Volume_DIN(2)*10^6); %temporary trunk volume
end

if (Dif_F_temp>=Dif_F)&&(Dif_F~=0) %front trunk still improving
    input.space.Targ_Vol_F=input.space.Targ_Vol_F+(size_factor*Dif_F);   %Target Total Trunk Volume in Liters
    input.space.Req_KS=size_factor*input.space.Act_KS;                   %Target Komponent space
    vehicle.dimensions.komp_vol_front_temp=input.space.Act_KS;   %temporary komponent space
    vehicle.dimensions.trunk_vol_front_temp=(Volume_DIN(1)*10^6);%temporary trunk volume
else %front trunk optimal
    check_f=0;
end

if (Dif_R_temp>=Dif_R)&&(Dif_R~=0)  %rear trunk still improving
    input.space.Targ_Vol_R=input.space.Targ_Vol_R+(size_factor*Dif_R);   %Target Total Trunk Volume in Liters
    input.space.Req_KSR=size_factor*input.space.Act_KSR;                 %Target Komponent space
    vehicle.dimensions.komp_vol_rear_temp=input.space.Act_KSR;   %temporary komponent space
    vehicle.dimensions.trunk_vol_rear_temp=(Volume_DIN(2)*10^6); %temporary trunk volume
else %rear trunk optimal
    check_r=0;
end

if vehicle.dimensions.trunk_vol_front==0 %deleting unwanted front trunk
    input.space.Targ_Vol_F=0;
end

if vehicle.dimensions.trunk_vol_rear==0  %deleting unwanted rear trunk
    input.space.Targ_Vol_R=0;
end

if Req_KS~=0                           %deleting unwanted front komponent space
    input.space.Req_KS=1000000*Req_KS;
end

if Req_KSR~=0                          %deleting unwanted rear komponent space
    input.space.Req_KSR=1000000*Req_KSR;
end
iteration=iteration+1;%next loop iteration
size_factor=size_factor-0.05; %reduce size_factor
end

%% Output Parameters
vehicle.dimensions.trunk_vol_front_DIN=vehicle.dimensions.trunk_vol_front_temp;
vehicle.dimensions.trunk_vol_rear_DIN=vehicle.dimensions.trunk_vol_rear_temp;
vehicle.dimensions.komp_vol_front=vehicle.dimensions.komp_vol_front_temp;
vehicle.dimensions.komp_vol_rear=vehicle.dimensions.komp_vol_rear_temp;
vehicle.body.Trunk_height_F=input.hood.Trunk_height_F;
vehicle.body.Trunk_height_R=input.hood.Trunk_height_R;
vehicle.body.Trunk_edge_F=input.hood.Trunk_edge_F;
vehicle.body.Trunk_edge_R=input.hood.Trunk_edge_R;
vehicle.body.Win_edge_F=input.hood.Win_edge_F;
vehicle.body.Win_edge_R=input.hood.Win_edge_R;
end