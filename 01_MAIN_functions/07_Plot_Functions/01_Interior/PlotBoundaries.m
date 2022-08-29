function PlotBoundaries(v,axis)
%% Description:
% Designed by: Adrian König (FTM, Technical University of Munich)
%-------------
% Created on: 18.11.2021
% ------------
% Version: Matlab2020b
%-------------
% Description: This function plots the boundaries from interior and front/rear wagon
% ------------
% Sources:  [1] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - vehicle: struct with includes the data of the vehicle
%           - axis: figure where vehicle is plotted
% ------------
% Output:   - Plotted boundaries
% ------------

%% 1) Plot interior boundaries
%a) Read surfaces
int_boundary_f_new=v.interior.int_boundary_f_new; % initialize interior front boundary
int_boundary_r_new=v.interior.int_boundary_r_new; % initialize interior rear boundary
z_f=1:1:length(int_boundary_f_new); % z-vector for plot (size --> max height)
z_r=1:1:length(int_boundary_r_new); % z-vector for plor (size --> max height)
a_1=v.interior.a_1; % Side distance to interior front
a_2=v.interior.a_2; % Side distance to interior rear
%b) Plot surfaces: [x-contour,x-contour],[y-value 1,y-value 2],[z-values,z-values] Note: y-values 1 and 2 fill one vector, z-values help to locate x-contour (one column of x vector == 1mm in z)
surf(axis,[int_boundary_f_new';int_boundary_f_new'],[(a_1*ones(size(int_boundary_f_new)))';((v.interior.int_y_front+a_1)*ones(size(int_boundary_f_new)))'],[z_f;z_f],'FaceAlpha',0.7,'LineStyle','none','FaceColor',[0.8 0.4 0])
surf(axis,[int_boundary_r_new';int_boundary_r_new'],[(a_2*ones(size(int_boundary_r_new)))';((v.interior.int_y_rear+a_2)*ones(size(int_boundary_r_new)))'],[z_r;z_r],'FaceAlpha',0.7,'LineStyle','none','FaceColor',[0.8 0.4 0])

%% 2) Plot powertrain boundaries
%a) Read surfaces
p_boundaries_f=v.interior.p_boundaries_f; % initialize front axle boundary
p_boundaries_r=v.interior.p_boundaries_r; % initialize rear axle boundary
%b) Plot surfaces
if v.settings.plot_boundary % plot axles boundaries if option is selected
    for i=1:length(p_boundaries_f)-1
        z_help=(1:1:(size(p_boundaries_f{3,i})));
        surf(axis,[p_boundaries_f{3,i}';p_boundaries_f{3,i}'],[p_boundaries_f{2,i}*ones(size(p_boundaries_f{3,i}))';p_boundaries_f{2,i+1}*ones(size(p_boundaries_f{3,i}))'],[z_help;z_help],'Linestyle', 'none','FaceColor',[0 0 1],'FaceAlpha',0.5)
    end
    for i=1:length(p_boundaries_r)-1
        z_help=(1:1:(size(p_boundaries_r{3,i})));
        surf(axis,[p_boundaries_r{3,i}';p_boundaries_r{3,i}'],[p_boundaries_r{2,i}*ones(size(p_boundaries_r{3,i}))';p_boundaries_r{2,i+1}*ones(size(p_boundaries_r{3,i}))'],[z_help;z_help],'Linestyle', 'none','FaceColor',[0 0 1],'FaceAlpha',0.5)
    end
end
end

