function plot_result_LDS_Results(veh)
% Designed by: Adrian König (FTM, Technical University of Munich), Lorenzo Nicoletti (FTM, Technical University of Munich), Korbinian Moller
%-------------
% Created on: 07.01.2022
% ------------
% Version: Matlab2020b
%-------------
% Description: This function can be called to plot the results of the LDS once the lds is done calculating
% ------------
% Input: v: The vehicle structure -> Stores the calculated component volumes and masses
% ------------
% Output: None
% ------------
%% Implementation
% 0) Initialize inputs
% 1) Plot speed for the acceleration simulation
% 2) Plot resistance forces during acceleration simulation
% 3) Plot speed profile of the selected cycle
% 4) Plot energy consumption of the selected cycle
% 5) Plot the machines and their operating points
%% 0) Initialize inputs
Color=[0,101,189]./255;    %Color for calculated line

sim_acc  = veh.LDS.sim_acc;     %Structure containing the results of the acceleration simulation
sim_cons = veh.LDS.sim_cons;    %Structure containing the results of the consumption simulation

%It could be that the LDS structure has been clared to save space. In this case it is not possible to plot the results
if ~isfield(sim_cons,'v')
    disp('The option clear struct for the LDS is activated. Therefore it is not possible to plot the results. To plot the result uncomment the line vehicle=clear_struct(vehicle) in the function calc_longitudinal_simulation and then simulate again')
    return
end

%Create figure to plot the results
%fig=figure('Name','Results LDS','Renderer', 'Painters'); fig.Units='Centimeters';
fig=figure('Name','Results LDS'); fig.Units='Centimeters';
set(gcf, 'Position', [10, 20, 25, 10]);

%% 5) Plot the machines and their operating points
mot=veh.LDS.MOTOR;
filled_axles=[~isempty(veh.LDS.MOTOR{1}),~isempty(veh.LDS.MOTOR{2})];
for i=find(filled_axles)
    
    subplot(1,2,i);
    
    n_mot=mot{i}.diagram.n_scaled; % in 1/min
    T_mot=mot{i}.diagram.T_scaled; % in Nm 
    eta=mot{i}.diagram.etages;     % in percent
    
    [C,h]=contourf(n_mot,T_mot,eta,[0.3,0.7,0.9,0.93,0.95,0.96,0.965,0.97,0.9725,0.975,0.977,0.978,0.979,0.980,0.985]);
    hold on
    [C,h]=contourf(n_mot,-T_mot,eta,[0.3,0.7,0.9,0.93,0.95,0.96,0.965,0.97,0.9725,0.975,0.977,0.978,0.979]);
    if i==1
        scatter(sim_cons.n_mot_f,sim_cons.T_mot_f,15,'filled','MarkerFaceColor',Color,'MarkerFaceAlpha',0.1)
        str='Frontmotor';
    else
        scatter(sim_cons.n_mot_r,sim_cons.T_mot_r,15,'filled','MarkerFaceColor',Color,'MarkerFaceAlpha',0.05)
        str='Heckmotor';
    end
    
    %Set Axis Labels
    ax=gca; ax.XLabel.String='Drehzahl in 1/min'; ax.YLabel.String='Drehmoment in Nm'; ax.Title.String=str;
    %set(gca, 'Visible', 'off');
end
end