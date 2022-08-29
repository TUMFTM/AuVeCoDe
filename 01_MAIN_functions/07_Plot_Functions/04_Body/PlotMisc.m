function PlotMisc(axis,v)
%% Description:
% Designed by: Adrian König (FTM, Technical University of Munich)
%-------------
% Created on: 18.11.2021
% ------------
% Version: Matlab2020b
%-------------
% Description: %This function plots remaining parts of the v e.g. the bumper or the cooler
% ------------
% Sources:        [1] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - veh:  struct with the vehicle parameters
%           - axis: figure where vehicle is plotted
% ------------
% Output:   - Plotted bumpers and cooler
% ------------

%% Implementation
% 1) Sizing and Positioning of components
% 2) Plot of components

%% 1) Sizing and Positioning of components
% Read in parameters
r_w = v.dimensions.CX.wheel_f_diameter/2; %Radius wheel

% a) Sizing of Cooler
Lx_co=v.dimensions.p_cooler_ctr(2,1); %Length of Cooler (see Package_CCSystem)
Ly_co=v.dimensions.p_cooler_ctr(2,2); %Width of Cooler (see Package_CCSystem)
Lz_co=v.dimensions.p_cooler_ctr(2,3); %Height of Cooler (see Package_CCSystem)

% b) Positioning of cooler
% Adjustment of position due to plot functions (Object3d_Block) and wheelstart at z=0
p_co=[v.dimensions.p_cooler_ctr(1,1)-0.5*Lx_co; 0.5*(v.dimensions.GY.vehicle_width-Ly_co);v.dimensions.p_cooler_ctr(1,3)+r_w-0.5*Lz_co];
eu_co=[0;0;0]; %Rotation Matrix for Euler Angles

% c) Sizing of bumpers
%Front
Lx_bu_f=v.dimensions.p_bumper{1}(2,1); %Length of front bumper (see Package_CCSystem)
Ly_bu_f=v.dimensions.p_bumper{1}(2,2); %Width of front bumper (see Package_CCSystem)
Lz_bu_f=v.dimensions.p_bumper{1}(2,3); %Height of front bumper (see Package_CCSystem)
%Rear
Lx_bu_r=v.dimensions.p_bumper{2}(2,1); %Length of rear bumper (see Package_CCSystem)
Ly_bu_r=v.dimensions.p_bumper{2}(2,2); %Width of rear bumper (see Package_CCSystem)
Lz_bu_r=v.dimensions.p_bumper{2}(2,3); %Height of rear bumper (see Package_CCSystem)

% d) Positioning of bumpers
% Adjustment of position due to plot functions (Object3d_Block)
p_bu_f=[v.dimensions.p_bumper{1}(1,1)-0.5*Lx_bu_f; 0.5*(v.dimensions.GY.vehicle_width-Ly_bu_f);v.dimensions.p_bumper{1}(1,3)-0.5*Lz_bu_f];
p_bu_r=[v.dimensions.GX.wheelbase-v.dimensions.p_bumper{2}(1,1)-0.5*Lx_bu_r; 0.5*(v.dimensions.GY.vehicle_width-Ly_bu_r);v.dimensions.p_bumper{2}(1,3)-0.5*Lz_bu_r]; % x-start = wheelbase - (x-pos bumper) due to negative relative position to rear axle
eu_bu=[0;0;0]; %Rotation Matrix for Euler Angles

%% 2) Plot of components
%Plot Cooler
[F_co,V_co]=Object3d_Block(p_co,eu_co,Lx_co,Ly_co,Lz_co);
patch(axis,'Faces', F_co, 'Vertices', V_co, 'FaceColor', [162 173 0]./255); %use patch to plot power electronics

% Plot front bumper
[F_buf,V_buf]=Object3d_Block(p_bu_f,eu_bu,Lx_bu_f,Ly_bu_f,Lz_bu_f);
patch(axis,'Faces', F_buf, 'Vertices', V_buf, 'FaceColor', [227 114 34]./255); %use patch to plot bumper

% Plot rear bumper
[F_bur,V_bur]=Object3d_Block(p_bu_r,eu_bu,Lx_bu_r,Ly_bu_r,Lz_bu_r);
patch(axis,'Faces', F_bur, 'Vertices', V_bur, 'FaceColor', [227 114 34]./255); %use patch to plot bumper


