function PlotBattery_space(v)
%% Description:
% Designed by: Adrian König, Lorenzo Nicoletti (FTM, Technical University of Munich)
%-------------
% Created on: 01.04.2022
% ------------
% Version: Matlab2020b
%-------------
% Description: This function plots the battery space 
% ------------
% Sources:      [1] Lorenzo Nicoletti, "Parametric Modeling of Battery Electric Vehicles in the Early Development Phase", Technical University of Munich, Institute of Automotive Technology, 2022
%               [2] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - vehicle: struct with includes the data of the vehicle
% ------------
% Output:   - Plot of battery space
% ------------

%% Implementation:
%1) Initialize the required variables
%2) Plot bottom cover
%3) Plot battery cooling
%4) Plot small overlap space

%% 1) Initialize the required variables

%Polt settings can be changed here
lwidth=0.05;
ST=v.battery.spacetable;
FT=v.battery.filltable;
Color_cover='black';
Color_cooling='blue';
Alpha_cover=0.1;
Alpha_cooling=0.1;

%Battery topology
batt_topology=v.topology.battery_topology;  

%Size and position of the tunnel area (in mm)
CX_tunnel=table2array(ST('tunnel','CX'));
CY_tunnel=table2array(ST('tunnel','CY'));
CZ_tunnel=table2array(ST('tunnel','CZ'));
EX_tunnel=table2array(ST('tunnel','EX'));
EZ_tunnel=table2array(ST('tunnel','EZ'));

%Size and position of the second level area (in mm)
CX_second_level=table2array(ST('second level','CX'));
CY_second_level=table2array(ST('second level','CY'));
CZ_second_level=table2array(ST('second level','CZ'));
EX_second_level=table2array(ST('second level','EX'));
EZ_second_level=table2array(ST('second level','EZ'));

%Size and position of the underfloor area (in mm)
CX_underfloor=table2array(ST('underfloor','CX'));                 
CY_underfloor=table2array(ST('underfloor','CY'));                
CZ_underfloor=table2array(ST('underfloor','CZ'));                
EX_underfloor=table2array(ST('underfloor','EX'));
EZ_underfloor=table2array(ST('underfloor','EZ'));  %EZ underfloor is the distance between the top of lower battery cover and the ground

%Thickness of the top cover of the second level
CZ_batt_top_cover=v.dimensions.CZ.batt_top_cover;
CZ_batt_bottom_cover=v.dimensions.CZ.batt_bottom_cover;
CZ_cooling=v.dimensions.CZ.batt_cooling;
EZ_free_space_mod2cover=v.dimensions.EZ.free_space_module_to_top_cover;         %Distance between module top and top conver bottom along tzhe vertical (Z) diorection in mm

%AuVeCoDe-Change:
%Read y-Offset
y_pos(1,1)=v.dimensions.GY.vehicle_width/2-v.battery.installationspace.CY_batt_underfloor/2;
y_pos(2,1)=0;
y_pos(3,1)=v.dimensions.GY.vehicle_width/2-v.battery.installationspace.CY_batt_second_level_1/2;
y_pos(4,1)=v.dimensions.GY.vehicle_width/2-v.battery.installationspace.CY_batt_second_level_2/2;

y_pos(1,2)=v.dimensions.GY.vehicle_width/2+v.battery.installationspace.CY_batt_underfloor/2;
y_pos(2,2)=0;
y_pos(3,2)=v.dimensions.GY.vehicle_width/2+v.battery.installationspace.CY_batt_second_level_1/2;
y_pos(4,2)=v.dimensions.GY.vehicle_width/2+v.battery.installationspace.CY_batt_second_level_2/2;
%% 2) Plot bottom cover

if  table2array(FT('underfloor','NumcellZ'))==0
    if table2array(FT('tunnel','NumcellZ'))>0 %plot only if cells are inside
    %Bottom cover TUNNEL
    Object3d_Cube([EX_tunnel,EZ_tunnel-CZ_batt_bottom_cover],[CX_tunnel,CY_tunnel,CZ_batt_bottom_cover],Color_cover,Alpha_cover,y_pos(4,:));
    end
    
    if table2array(FT('second level','NumcellZ'))>0 %plot only if cells are inside
    %Bottom cover SECOND LEVEL
    Object3d_Cube([EX_second_level,EZ_second_level-CZ_batt_bottom_cover],[CX_second_level,CY_second_level,CZ_batt_bottom_cover],Color_cover,Alpha_cover,y_pos(3,:));
    end
elseif   table2array(FT('underfloor','NumcellZ'))>0
    
    %Bottom cover underfloor
    Object3d_Cube([EX_underfloor,EZ_underfloor-CZ_batt_bottom_cover],[CX_underfloor,CY_underfloor,CZ_batt_bottom_cover],Color_cover,Alpha_cover,y_pos(1,:));
end

%% 3) Plot battery cooling

%For each level of cells in the Underfloor, plot a cooling plate
for i=1:table2array(FT('underfloor','NumcellZ'))
   
   shift=(i-1)*table2array(FT('underfloor','PackdimZ'));
   Object3d_Cube([EX_underfloor,EZ_underfloor+shift],[CX_underfloor,CY_underfloor,CZ_cooling],Color_cooling,Alpha_cooling,y_pos(1,:));
end

%For each level of cells in the Second level, plot a cooling plate
for i=1:table2array(FT('second level','NumcellZ'))
   
   shift=(i-1)*table2array(FT('second level','PackdimZ'));
   Object3d_Cube([EX_second_level,EZ_second_level+shift],[CX_second_level,CY_second_level,CZ_cooling],Color_cooling,Alpha_cooling,y_pos(3,:));
end

%For each level of cells in the Tunnel, plot a cooling plate
for i=1:table2array(FT('tunnel','NumcellZ'))
   
   shift=(i-1)*table2array(FT('tunnel','PackdimZ'));
   Object3d_Cube([EX_tunnel,EZ_tunnel+shift],[CX_tunnel,CY_tunnel,CZ_cooling],Color_cooling,Alpha_cooling,y_pos(4,:));
end

%% 3) Plot battery top cover

if table2array(FT('underfloor','NumcellZ'))==0
    if table2array(FT('tunnel','NumcellZ'))>0 %plot only if cells are inside
    %Top cover tunnel
    Object3d_Cube([EX_tunnel,EZ_tunnel+CZ_tunnel+EZ_free_space_mod2cover],[CX_tunnel,CY_tunnel,CZ_batt_top_cover],Color_cover,Alpha_cover,y_pos(4,:));
    end
    
    if table2array(FT('second level','NumcellZ'))>0 %plot only if cells are inside
    %Top cover second level
    Object3d_Cube([EX_second_level,EZ_second_level+CZ_second_level+EZ_free_space_mod2cover],[CX_second_level,CY_second_level,CZ_batt_top_cover],Color_cover,Alpha_cover,y_pos(3,:));
    end
else   
    
    if v.settings.fill_second_level==1 && prod([CX_second_level,CY_second_level,CZ_second_level])>0
        %Correction factor to shorten the battery top cover, in case a second level is built
        delta=table2array(ST('second level','CX'));    
        Object3d_Cube([EX_second_level,EZ_second_level+CZ_second_level+EZ_free_space_mod2cover],[CX_second_level,CY_second_level,CZ_batt_top_cover],Color_cover,Alpha_cover,y_pos(3,:));
    else
        delta=0;
    end
    
    
    %Top cover underfloor
    Object3d_Cube([EX_underfloor,EZ_underfloor+CZ_underfloor+EZ_free_space_mod2cover],[CX_underfloor-delta,CY_underfloor,CZ_batt_top_cover],Color_cover,Alpha_cover,y_pos(1,:));
    
    if table2array(FT('tunnel','NumcellZ'))>0
    %Top cover tunnel
    Object3d_Cube([EX_tunnel,EZ_tunnel+CZ_tunnel+EZ_free_space_mod2cover],[CX_tunnel,CY_tunnel,CZ_batt_top_cover],Color_cover,Alpha_cover,y_pos(4,:));
    end
    
    if table2array(FT('second level','NumcellZ'))>0
    %Top cover second level
    Object3d_Cube([EX_second_level,EZ_second_level+CZ_second_level+EZ_free_space_mod2cover],[CX_second_level,CY_second_level,CZ_batt_top_cover],Color_cover,Alpha_cover,y_pos(3,:));
    end
end


%% 4) Plot small overlap space
if v.settings.fill_smalloverlapspace==1
    
    EX_batt_small_overlap=v.battery.installationspace.EX_batt_small_overlap_discretized;
    EX_battery_front_axle_min=v.dimensions.EX.battery_front_axle_min;
    
    %Size of the small overlap space (in mm) in Y direction, discretized
    CY_small_overlap=v.battery.installationspace.CY_batt_small_overlap./2;
    
    small_overlap_right=[EX_batt_small_overlap,CY_small_overlap];
    rear_segment =[max(EX_batt_small_overlap),CY_small_overlap(end);max(EX_batt_small_overlap),-CY_small_overlap(end)];
    small_overlap_left=[flip(EX_batt_small_overlap),-flip(CY_small_overlap)];
    front_segment=[min(min(EX_batt_small_overlap),EX_battery_front_axle_min),-CY_small_overlap(1);min(min(EX_batt_small_overlap),EX_battery_front_axle_min),+CY_small_overlap(1)];
    
    low_part_small_overlap=[small_overlap_right;rear_segment;small_overlap_left;front_segment];
    low_part_small_overlap(:,3)=ones(numel(low_part_small_overlap(:,1)),1)*(EZ_underfloor-CZ_batt_bottom_cover);

    %Plot small overlap space
    fill3(low_part_small_overlap(:,1),low_part_small_overlap(:,2),low_part_small_overlap(:,3),Color_cover,'FaceAlpha',Alpha_cover,'LineWidth',lwidth);
    fill3(low_part_small_overlap(:,1),low_part_small_overlap(:,2),low_part_small_overlap(:,3)+CZ_batt_bottom_cover,Color_cover','FaceAlpha',Alpha_cover,'LineWidth',lwidth);
    
    %Plot the cooling and top cover in the small overlap area
    low_part_small_overlap(:,3)=ones(numel(low_part_small_overlap(:,1)),1)*EZ_underfloor;
    fill3(low_part_small_overlap(:,1),low_part_small_overlap(:,2),low_part_small_overlap(:,3),Color_cooling,'FaceAlpha',Alpha_cooling,'LineWidth',lwidth);
    fill3(low_part_small_overlap(:,1),low_part_small_overlap(:,2),low_part_small_overlap(:,3)+CZ_cooling,Color_cooling','FaceAlpha',Alpha_cooling,'LineWidth',lwidth);

    low_part_small_overlap(:,3)=ones(numel(low_part_small_overlap(:,1)),1)*(EZ_underfloor+CZ_underfloor);
    fill3(low_part_small_overlap(:,1),low_part_small_overlap(:,2),low_part_small_overlap(:,3),Color_cover,'FaceAlpha',Alpha_cover,'LineWidth',lwidth);
    fill3(low_part_small_overlap(:,1),low_part_small_overlap(:,2),low_part_small_overlap(:,3)+CZ_batt_top_cover,Color_cover,'FaceAlpha',Alpha_cover,'LineWidth',lwidth);
    
end