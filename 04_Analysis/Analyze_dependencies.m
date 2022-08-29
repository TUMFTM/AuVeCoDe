function Analyze_dependencies()
%% Description:
% Designed by: Adrian König (FTM, Technical University of Munich)
%-------------
% Created on: 01.01.2021
% ------------
% Version: Matlab2020b
%-------------
% Description:  This function displays the objectives and variables and their dependency at the
%               final result of the optimization
% ------------
% Sources:  [1] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - Parameters_Pareto: struct with input and constant values
%           - variable_matrix: Variables
%           - objective_matrix: Objectives
% ------------
% Output:   - vehicle: struct with includes the data of the vehicle

clf
load('Parameters_Pareto')
load('variable_matrix')
load('objective_matrix')
[opt,Parameters] = nsga_options(Parameters);
figure
for i=1:opt.numVar
    subplot(2,ceil(opt.numVar/2),i)
    scatter(variable_matrix(:,i),objective_matrix(:,1));
    xlim([opt.lb(i) opt.ub(i)])
    title(opt.nameVar{i})
    sgtitle(Parameters.optimization.goal_1_name)
    hold on
end
hold off

figure
for i=1:opt.numVar
    subplot(2,ceil(opt.numVar/2),i)
    scatter(variable_matrix(:,i),objective_matrix(:,2));
    xlim([opt.lb(i) opt.ub(i)])
    title(opt.nameVar{i})
    sgtitle(Parameters.optimization.goal_2_name)
    hold on
end
hold off
end