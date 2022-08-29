function gearbox = set_laysh_transmission(gearbox)
%% 1) Description
% This function calculates gearbox data for a non-shiftable two-stage 
% helical-gear electric transmission in lay-shaft design.

%% 1.1) Authors
% Peter Koehler
% Date: 24.08.2021

%% 1.2) Sources:
% [1] Master Thesis Koehler: "Semi-physikalische Modellierung von Antriebsstrangkomponenten fuer Elektrofahrzeuge", Institute of Automotive Technology, TUM, Munich, 2021
% [2] Niemann and Winter: "Maschinenelemente - Band 2: Getriebe allgemein, Zahnradgetriebe", Springer Verlag, 2003, ISBN: 978-3-662-11874-0
% [3] Dissertation Parlow: "Entwicklung einer Methode zum anforderungsgerechten Entwurf von Stirnradgetrieben", Institute of Machine Elements, TUM, 2016
% [4] Bachelor Thesis Zaehringer: "Erstellung eines analytischen Ersatzmodells fuer Getriebe von Elektrofahrzeugen", Institute of Automotive Technology, TUM, Munich, 2018

%% 2) Initialization required values:
i_tot = gearbox.Input.i_tot;            % Total transmission ratio [-]
S_H_min = gearbox.MinSafety.S_H_min;    % Minimum safety factor against flank break (Niemann and Winter, p. 344) [-]
S_F_min = gearbox.MinSafety.S_F_min;    % Minimum safety factor against root break (Niemann and Winter, p. 344) [-]
manTR = gearbox.Input.manTR;            % For manual number of teeth as input
% Number of iterations for selection of transmission ratios of both stages
n = 5;                 % has to be uneven!
step = 0.2;

%% 3) Calculation of gearbox

if (manTR==0) %The user did not give a desired number of teeth, this has to be derived!
    % Calculation and storage of initial transmission ratio of both stages (Parlow pp. 6)
    i_12 = 1.1260*(i_tot^(0.5103));     % formula for minimum gear mass
    gearbox.results.i_12 = i_12 + step*floor(n/2);
    gearbox.results.i_34 = i_tot/gearbox.results.i_12;

    % Optimization of the transmission ratios in relation to tooth safety factors (Zaehringer, p. 92)
    i_12_new = gearbox.results.i_12;
    err_S_H_12 = zeros(1,n);
    err_S_H_34 = zeros(1,n);
    err_S_F_12 = zeros(1,n);
    err_S_F_34 = zeros(1,n);
    
    for i=1:n
        gearbox = set_lay_shaft(gearbox);
        gearbox_arr(:,i) = gearbox;
        
        % Absolute errors in the safety factors of both stages
        err_S_H_12(1,i) = gearbox.results.S_H_12-S_H_min;
        err_S_H_34(1,i) = gearbox.results.S_H_34-S_H_min;
        err_S_F_12(1,i) = gearbox.results.S_F_12-S_F_min;
        err_S_F_34(1,i) = gearbox.results.S_F_34-S_F_min;
        
        gearbox.results.i_12 = i_12_new-(step*i);
        gearbox.results.i_34 = i_tot/gearbox.results.i_12;
    end

    % Calculation of dataset with minimum delta of the safety factors
    err_sum = err_S_H_12+err_S_H_34+err_S_F_12+err_S_F_34;
    [~, min_idx] = min(err_sum);

    gearbox = gearbox_arr(1,min_idx);

else %The user has manually given the desired number of teeth

    z_1 = gearbox.Input.z_1;             % Number of teeth of first wheel       
    z_2 = gearbox.Input.z_2;             % Number of teeth of second wheel
    z_3 = gearbox.Input.z_3;             % Number of teeth of third wheel
    z_4 = gearbox.Input.z_4;             % Number of teeth of forth wheel
    
    % Definition of transmission ratios and number of teeth for manual selection
    gearbox.results.i_12 = z_2/z_1;
    gearbox.results.i_34 = z_4/z_3;
    gearbox.gears_12.z_1 = z_1;
    gearbox.gears_12.z_2 = z_2;
    gearbox.gears_34.z_3 = z_3;
    gearbox.gears_34.z_4 = z_4;
    
    gearbox = set_lay_shaft(gearbox);
end
    
end
