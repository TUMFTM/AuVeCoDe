function PlotTrunk(axis,v)
%% Description:
% Designed by: Michael Mast, Adrian König (FTM, Technical University of Munich)
%-------------
% Created on: 18.11.2021
% ------------
% Version: Matlab2020b
%-------------
% Description: %This function plots the front and rear trunk
% ------------
% Sources:        [1] Michael Mast, “Karosseriemodellierung autonomer Elektrofahrzeuge,” Master Thesis, Technical University of Munich, Institute of Automotive Technology, 2022
%                 [2] Michael Mast, “Packageplanung von autonomen Fahrzeugkonzepten im Vorder- und Hinterwagen,” Semester Thesis, Technical University of Munich, Institute of Automotive Technology, 2021
%                 [2] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - veh:  struct with the vehicle parameters
%           - axis: figure where vehicle is plotted
% ------------
% Output:   - Plotted body structure and silhouette

%% Implementation
% 1) Calculation of size and position of front and rear trunk
% 2) Plot front trunk
% 3) Plot rear trunk

%% 1) Calculation of size and position of front and rear trunk
if v.dimensions.trunk_vol_front>0
    % Front Trunk
    %Sizing of Trunk
    Lx_tr_f1=v.dimensions.p_trunk_front{1}(2,1); %Length of lower box front trunk (see DESIGN_trunk_space)
    Ly_tr_f1=v.dimensions.p_trunk_front{1}(2,2); %Width of lower box front trunk (see DESIGN_trunk_space)
    Lz_tr_f1=v.dimensions.p_trunk_front{1}(2,3); %Height of lower box front trunk (see DESIGN_trunk_space)
    
    Lx_tr_f2=v.dimensions.p_trunk_front{2}(2,1); %Length of upper box front trunk (see DESIGN_trunk_space)
    Ly_tr_f2=v.dimensions.p_trunk_front{2}(2,2); %Width of upper box front trunk (see DESIGN_trunk_space)
    Lz_tr_f2=v.dimensions.p_trunk_front{2}(2,3); %Height of upper box front trunk (see DESIGN_trunk_space)
    
    Lx_tr_f3=v.dimensions.p_trunk_front{3}(2,1); %Length of lower box front trunk (see DESIGN_trunk_space)
    Ly_tr_f3=v.dimensions.p_trunk_front{3}(2,2); %Width of lower box front trunk (see DESIGN_trunk_space)
    Lz_tr_f3=v.dimensions.p_trunk_front{3}(2,3); %Height of lower box front trunk (see DESIGN_trunk_space)
    
    Lx_tr_f4=v.dimensions.p_trunk_front{4}(2,1); %Length of lower box front trunk (see DESIGN_trunk_space)
    Ly_tr_f4=v.dimensions.p_trunk_front{4}(2,2); %Width of lower box front trunk (see DESIGN_trunk_space)
    Lz_tr_f4=v.dimensions.p_trunk_front{4}(2,3); %Height of lower box front trunk (see DESIGN_trunk_space)
    
    
    % Positioning of front trunk (lower and upper box)
    % adjustment of position due to plot functions (see sources)
    p_tr_f1=[v.dimensions.p_trunk_front{1}(1,1)-0.5*Lx_tr_f1; v.dimensions.p_trunk_front{1}(1,2)-0.5*Ly_tr_f1;v.dimensions.p_trunk_front{1}(1,3)-0.5*Lz_tr_f1];
    p_tr_f2=[v.dimensions.p_trunk_front{2}(1,1)-0.5*Lx_tr_f2; v.dimensions.p_trunk_front{2}(1,2)-0.5*Ly_tr_f2;v.dimensions.p_trunk_front{2}(1,3)-0.5*Lz_tr_f2];
    p_tr_f3=[v.dimensions.p_trunk_front{3}(1,1)-0.5*Lx_tr_f3; v.dimensions.p_trunk_front{3}(1,2)-0.5*Ly_tr_f3;v.dimensions.p_trunk_front{3}(1,3)-0.5*Lz_tr_f3];
    p_tr_f4=[v.dimensions.p_trunk_front{4}(1,1)-0.5*Lx_tr_f4; v.dimensions.p_trunk_front{4}(1,2)-0.5*Ly_tr_f4;v.dimensions.p_trunk_front{4}(1,3)-0.5*Lz_tr_f4];
    eu_tr_f=[0;0;0]; %Rotation Matrix for Euler Angles
end
if v.dimensions.trunk_vol_rear>0
    % Rear Trunk
    %Sizing of Trunk
    Lx_tr_r1=v.dimensions.p_trunk_rear{1}(2,1); %Length of lower box rear trunk (see DESIGN_trunk_space)
    Ly_tr_r1=v.dimensions.p_trunk_rear{1}(2,2); %Width of lower box rear trunk (see DESIGN_trunk_space)
    Lz_tr_r1=v.dimensions.p_trunk_rear{1}(2,3); %Height of lower box rear trunk (see DESIGN_trunk_space)
    
    Lx_tr_r2=v.dimensions.p_trunk_rear{2}(2,1); %Length of upper box rear trunk (see DESIGN_trunk_space)
    Ly_tr_r2=v.dimensions.p_trunk_rear{2}(2,2); %Width of upper box rear trunk (see DESIGN_trunk_space)
    Lz_tr_r2=v.dimensions.p_trunk_rear{2}(2,3); %Height of upper box rear trunk (see DESIGN_trunk_space)
    
    Lx_tr_r3=v.dimensions.p_trunk_rear{3}(2,1); %Length of lower box rear trunk (see DESIGN_trunk_space)
    Ly_tr_r3=v.dimensions.p_trunk_rear{3}(2,2); %Width of lower box rear trunk (see DESIGN_trunk_space)
    Lz_tr_r3=v.dimensions.p_trunk_rear{3}(2,3); %Height of lower box rear trunk (see DESIGN_trunk_space)
    
    Lx_tr_r4=v.dimensions.p_trunk_rear{4}(2,1); %Length of lower box rear trunk (see DESIGN_trunk_space)
    Ly_tr_r4=v.dimensions.p_trunk_rear{4}(2,2); %Width of lower box rear trunk (see DESIGN_trunk_space)
    Lz_tr_r4=v.dimensions.p_trunk_rear{4}(2,3); %Height of lower box rear trunk (see DESIGN_trunk_space)
    
    
    % Positioning of rear trunk (lower and upper box)
    % adjustment of position due to plot functions (see sources)
    p_tr_r1=[v.dimensions.p_trunk_rear{1}(1,1)-0.5*Lx_tr_r1; v.dimensions.p_trunk_rear{1}(1,2)-0.5*Ly_tr_r1;v.dimensions.p_trunk_rear{1}(1,3)-0.5*Lz_tr_r1];
    p_tr_r2=[v.dimensions.p_trunk_rear{2}(1,1)-0.5*Lx_tr_r2; v.dimensions.p_trunk_rear{2}(1,2)-0.5*Ly_tr_r2;v.dimensions.p_trunk_rear{2}(1,3)-0.5*Lz_tr_r2];
    p_tr_r3=[v.dimensions.p_trunk_rear{3}(1,1)-0.5*Lx_tr_r3; v.dimensions.p_trunk_rear{3}(1,2)-0.5*Ly_tr_r3;v.dimensions.p_trunk_rear{3}(1,3)-0.5*Lz_tr_r3];
    p_tr_r4=[v.dimensions.p_trunk_rear{4}(1,1)-0.5*Lx_tr_r4; v.dimensions.p_trunk_rear{4}(1,2)-0.5*Ly_tr_r4;v.dimensions.p_trunk_rear{4}(1,3)-0.5*Lz_tr_r4];
    eu_tr_r=[0;0;0]; %Rotation Matrix for Euler Angles
end

%% 2) Plot front trunk
if v.dimensions.trunk_vol_front>0
    % Plot front trunk FA
    if (Lx_tr_f1>0 && Ly_tr_f1>0 && Lz_tr_f1>0)
        [F_trf1,V_trf1]=Object3d_Block(p_tr_f1,eu_tr_f,Lx_tr_f1,Ly_tr_f1,Lz_tr_f1);
        patch(axis,'Faces', F_trf1, 'Vertices', V_trf1, 'FaceColor', [152 198 234]./255); %use patch to plot trunk
    end
    
    % Plot front trunk BA
    if (Lx_tr_f2>0 && Ly_tr_f2>0 && Lz_tr_f2>0)
        [F_trf2,V_trf2]=Object3d_Block(p_tr_f2,eu_tr_f,Lx_tr_f2,Ly_tr_f2,Lz_tr_f2);
        patch(axis,'Faces', F_trf2, 'Vertices', V_trf2, 'FaceColor', [152 198 234]./255); %use patch to plot trunk
    end
    
    % Plot front trunk OA
    if (Lx_tr_f3>0 && Ly_tr_f3>0 && Lz_tr_f3>0)
        [F_trf3,V_trf3]=Object3d_Block(p_tr_f3,eu_tr_f,Lx_tr_f3,Ly_tr_f3,Lz_tr_f3);
        patch(axis,'Faces', F_trf3, 'Vertices', V_trf3, 'FaceColor', [152 198 234]./255); %use patch to plot trunk
    end
    
    % Plot front trunk UC
    if (Lx_tr_f4>0 && Ly_tr_f4>0 && Lz_tr_f4>0)
        [F_trf4,V_trf4]=Object3d_Block(p_tr_f4,eu_tr_f,Lx_tr_f4,Ly_tr_f4,Lz_tr_f4);
        patch(axis,'Faces', F_trf4, 'Vertices', V_trf4, 'FaceColor', [152 198 234]./255); %use patch to plot trunk
    end
end    

%% 3) Plot rear trunk
if v.dimensions.trunk_vol_rear>0
    % Plot rear trunk FA
    if (Lx_tr_r1>0 && Ly_tr_r1>0 && Lz_tr_r1>0)
        [F_trr1,V_trr1]=Object3d_Block(p_tr_r1,eu_tr_r,Lx_tr_r1,Ly_tr_r1,Lz_tr_r1);
        patch(axis,'Faces', F_trr1, 'Vertices', V_trr1, 'FaceColor', [100 160 200]./255); %use patch to plot trunk
    end
    
    % Plot rear trunk BA
    if (Lx_tr_r2>0 && Ly_tr_r2>0 && Lz_tr_r2>0)
        [F_trr2,V_trr2]=Object3d_Block(p_tr_r2,eu_tr_r,Lx_tr_r2,Ly_tr_r2,Lz_tr_r2);
        patch(axis,'Faces', F_trr2, 'Vertices', V_trr2, 'FaceColor', [100 160 200]./255); %use patch to plot trunk
    end
    
    % Plot rear trunk OA
    if (Lx_tr_r3>0 && Ly_tr_r3>0 && Lz_tr_r3>0)
        [F_trr3,V_trr3]=Object3d_Block(p_tr_r3,eu_tr_r,Lx_tr_r3,Ly_tr_r3,Lz_tr_r3);
        patch(axis,'Faces', F_trr3, 'Vertices', V_trr3, 'FaceColor', [100 160 200]./255); %use patch to plot trunk
    end
    
    % Plot rear trunk UC
    if (Lx_tr_r4>0 && Ly_tr_r4>0 && Lz_tr_r4>0)
        [F_trr4,V_trr4]=Object3d_Block(p_tr_r4,eu_tr_r,Lx_tr_r4,Ly_tr_r4,Lz_tr_r4);
        patch(axis,'Faces', F_trr4, 'Vertices', V_trr4, 'FaceColor', [100 160 200]./255); %use patch to plot trunk
    end
end   



end
