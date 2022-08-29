%% Script for analyzing the optimization results
%% Description:
% Designed by: Adrian König (FTM, Technical University of Munich)
%-------------
% Created on: 05.05.2022
% ------------
% Version: Matlab2020b
%-------------
% Description: This function reads the finished simulation data and collects the optima 
% ------------
% Sources:  [1] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - Simulation data (e.g. 220502_Optimization_1_01.mat)
% ------------
% Output:   - Table with results
% ------------

%% Locate folder
InputPath=uigetdir;
file=dir(fullfile(InputPath,'*Optimization*.mat'));

for i=1:length(file)
   load(fullfile(file(i).folder,file(i).name)); 
   Rename_eq=sprintf('state%02d=state;',i);
   eval(Rename_eq);
   clear('state');
end

%Initialize array with results (best for each objective)
best_par=zeros(length(file),4);

%Initialize vehicle struct
vehicle_opt=struct;

%% Read parameters
for j=1:length(file)
    %Check if optimium was probably reached
    state=eval(sprintf('state%02d',j));
    optimum_check=CHECK_optimum(state);
    best_par(j,5)=optimum_check;
    Parameters=eval(sprintf('state%02d.Parameters',j));
    
    for i=1:2
        l=eval(sprintf('size(state%02d.result.bestPosition,1)',j));
        if i==1
            var=eval(sprintf('state%02d.result.bestPosition',j));
            par=eval(sprintf('state%02d.result.bestFitness',j));
            comp=[par var];
            comp=sortrows(comp);
            best_par(j,1:2)=comp(1,1:2);
            var=comp(1,3:end);
        elseif i==2
            var=eval(sprintf('state%02d.result.bestPosition',j));
            par=eval(sprintf('state%02d.result.bestFitness',j));
            comp=[par var];
            comp=sortrows(comp,2);
            best_par(j,3:4)=comp(1,1:2);
            var=comp(1,3:end);
        end
               
        %% Calculate vehicle
        vehicle=MAIN_AuVeCoDe(Parameters,var);
        eval(sprintf('vehicle_opt.Parameters.Parameters%02d_%d=Parameters',j,i));
        eval(sprintf('vehicle_opt.vehicles.vehicle%02d_%d=vehicle',j,i));
        eval(sprintf('vehicle_opt.var.var%02d_%d=var',j,i));
        
        %% Plot vehicle
        figure('Name',sprintf('Vehicle Package %02d_%02d',j,i),'NumberTitle','off');
        clf
        ax1 = axes;
        %Optional fixed axes, better zoom but not true to scale after zoom
        ax1.Clipping = "off";
        ax1.Projection = 'perspective';
        ax1.Visible= 'off';  %disable axes
        ax1.View=[-40, 20];
        DISPLAY_vehicle(vehicle,Parameters,ax1);
        
    end
end

format longG
par;