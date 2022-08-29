function gearbox = set_plan_transmission (gearbox)
%% 1) Description:
% This function computes the transmission ratios between all gears in the
% planetary gearbox and calls the set_planetary-function.

%% 1.1) Authors
% Peter Koehler
% Date: 24.08.2021

%% 1.2) Sources:
% [1] Master Thesis Koehler: "Semi-physikalische Modellierung von Antriebsstrangkomponenten fuer Elektrofahrzeuge", Institute of Automotive Technology, TUM, Munich, 2021
% [2] Mueller: "Die Umlaufgetriebe", Springer Verlag, 1998, ISBN: 978-3-642-58725-2
% [3] Bachelor Thesis Zaehringer: "Erstellung eines analytischen Ersatzmodells fuer Getriebe von Elektrofahrzeugen", Institute of Automotive Technology, TUM, Munich, 2018
% [4] Master Thesis Koehler: "Semi-physikalische Modellierung von Antriebsstrangkomponenten fuer Elektrofahrzeuge", Institute of Automotive Technology, TUM, Munich, 2021

%% 2) Initialization of required values:
i_tot = gearbox.Input.i_tot(1);         % Total transmission ratio [-]
manTR = gearbox.Input.manTR;            % For manual number of teeth as input 
if manTR
    z_1 = gearbox.Input.z_1;            % Number of teeth of sun gear      
    z_p1 = gearbox.Input.z_2;           % Number of teeth of first planet
    z_p2 = gearbox.Input.z_3;           % Number of teeth of second planet
    z_2 = gearbox.Input.z_4;            % Number of teeth of ring gear
end
% Number of iterations for selection of transmission ratios of both stages
n = 5;                 % has to be uneven!
step = 0.1;
% Preallocation of error arrays  
err_S_H_12 = zeros(1,n);
err_S_H_34 = zeros(1,n);
err_S_F_12 = zeros(1,n);
err_S_F_34 = zeros(1,n);
err_i_tot = zeros(1,n);


%% 3) Calculation of the transmission ratios according (Mueller, p.34)
if (manTR==0)
    % Total transmission ratio is between input (sun) shaft and planet carrier (index s)
    i_1s = i_tot;
    % Stationary gear ratio (between sun shaft and ring gear)
    i_0 = 1-i_1s;

    % Determination of the transmission ratio of the first stage from the total ratio (Koehler, p. 56):
    i_1p1 = i_1s^(1/4);

    % Storage of initial tranmission ratios for further calculation
    gearbox.results.i_1s = i_1s;
    gearbox.results.i_0 = i_0;

    % Optimization of the transmission ratios in relation to tooth safety factors (Zaehringer, p. 92)
    i_1p1_max = i_1p1+step*floor(n/2);

    for i=n:-1:1
        gearbox.results.i_1p1 = i_1p1_max-(step*(n-i));
        gearbox.results.i_p22 = i_0/gearbox.results.i_1p1;
        
        % Calculation of the gearbox with given stage transmission ratios
        gearbox = set_planetary(gearbox);
        gearbox_selection(1,i) = gearbox;

        % Determination of errors in safety factors and transmission ratio
        err_S_H_12(1,i) = abs(gearbox.results.S_H_1p1-gearbox.results.S_H_p22);
        err_S_H_34(1,i) = abs(gearbox.results.S_H_1p1-gearbox.results.S_H_p22);
        err_S_F_12(1,i) = abs(gearbox.results.S_F_1p1-gearbox.results.S_F_p22);
        err_S_F_34(1,i) = abs(gearbox.results.S_F_1p1-gearbox.results.S_F_p22);
        err_i_tot(1,i) = abs(gearbox.Input.i_tot-gearbox.results.i_1s); 
    end

    % Calculation of dataset with minimum delta of the safety factors
    err_sum = err_S_H_12+err_S_H_34+err_S_F_12+err_S_F_34+err_i_tot;
    [~, min_idx] = min(err_sum);

    gearbox = gearbox_selection(1,min_idx); 
    
else        % Storing of numbers of teeth and transmission ratios for manual selection
    gearbox.gears.z_1 = z_1;
    gearbox.gears.z_p1 = z_p1;
    gearbox.gears.z_p2 = z_p2;
    gearbox.gears.z_2 = z_2;
    gearbox.results.i_1p1 = z_2/z_1;
    gearbox.results.i_p22 = z_2/z_p2;
    gearbox.results.i_0 = gearbox.results.i_1p1*gearbox.results.i_p22;
    gearbox.results.i_1s = 1-gearbox.results.i_0;
    
    % Calculation of the gearbox with given transmission ratio
    gearbox = set_planetary(gearbox);
end
    
end
