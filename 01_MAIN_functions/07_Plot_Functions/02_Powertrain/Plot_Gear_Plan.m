function Plot_Gear_Plan(axis,vehicle,components)
%% Description:
% Designed by: Korbinian Moller, Adrian König (FTM, Technical University of Munich)
%-------------
% Created on: 01.10.2021
% ------------
% Version: Matlab2020b
%-------------
% Description: This function plots gear wheels
% ------------
% Sources:      [1] Korbinian Moller, “Antriebsstrangmodellierung zur Optimierung autonomer Elektrofahrzeuge,” Semester Thesis, Technical University of Munich, Institute of Automotive Technology, 2022
%               [2] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - vehicle: struct with includes the data of the vehicle
% ------------
% Output:   - Plot of gearbox
% ------------


%% Definition of needed Variables

%Colors
C1 = [0, 101, 189]./255;            % Wheels of the first stage (blue)
C2 = [100, 160, 200]./255;          % Wheels of the second stage (light blue)
C3 = [255, 100, 100]./255;          %temporary
C4 = [100, 255, 100]./255;          %temporary
CD = [204,204,204]./255;            % Differential cage (light grey)
CB = [10, 10, 10]./255;             % Bearings (black)
CS = [51,51,51]./255;               % Shafts (dark grey)

color_palette = [C1;C2;C3;C4;CD;CB;CS];

%Rotation Matrix for Euler Angles
eu = [pi/2;0; 0]; 

%Sidecount
sc = 80;

%Wheel Radius
r_w = vehicle.dimensions.CX.wheel_f_diameter/2; %Radius wheel

%% Implementation
for i = 1:size(components,1)
    
    h = components(i,6);
    p = components(i,1:3)' + [0;0.5 * h;r_w];
    color = color_palette(components(i,8),:);
    
    
    if components(i,7) == 0 % filled cylinder
        
        r = components(i,4);
        [F_g,F_g_b,V_g] = Object3d_Cylinder(p,eu,r,h,sc); %create face and vertices of gearbox
        
    elseif components(i,7) == 1 % hollow cylinder
        
        r_outer = components(i,4);
        r_inner = components(i,5);
        [F_g,F_g_b,V_g] = Object3d_Cylinder_hollow(p,eu,r_outer,r_inner,h,sc); %create face and vertices of gearbox
        
    else
        disp('Error while plotting planetary gear components!')
        return
    end
    
    %Plot
    patch(axis,'Faces', F_g, 'Vertices', V_g, 'FaceColor', color, 'FaceAlpha', 1); %use patch to plot cylinder surface (round surface)
    patch(axis,'Faces', F_g_b, 'Vertices', V_g, 'FaceColor', color, 'FaceAlpha', 1); %use patch to plot cylinder surface (flat surface)
end

end

