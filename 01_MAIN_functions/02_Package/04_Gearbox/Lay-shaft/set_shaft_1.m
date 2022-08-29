function gearbox = set_shaft_1(gearbox)
%% 1) Decription:
% This function dimensions the first shaft of the lay-shaft gearbox.

%% 1.1) Authors
% Peter Koehler
% Date: 24.08.2021

%% 1.2) Sources:
% [1] Master Thesis Koehler: "Semi-physikalische Modellierung von Antriebsstrangkomponenten fuer Elektrofahrzeuge", Institute of Automotive Technology, TUM, Munich, 2021
% [2] Niemann and Winter: "Maschinenelemente - Band 2: Getriebe allgemein, Zahnradgetriebe", Springer Verlag, 2003, ISBN: 978-3-662-11874-0
% [3] Schaeffler: "Waelzlager", Bearing Catalogue, 2019
% [4] Semesterarbeit Koehler: "Semi-physikalische Modellierung von Antriebsstrangkomponenten fuer Elektrofahrzeuge", Institute of Automotive Technology, TUM, Munich, 2020

%% 2) Initialization of required values:
b_d_1 = gearbox.GearingConst.b_d_1;         % Ratio of width to pitch diameter of first wheel [-]
S_H_min = gearbox.MinSafety.S_H_min;        % Minimum safety factor against flank break (Niemann and Winter, p. 344) [-]
S_F_min = gearbox.MinSafety.S_F_min;        % Minimum safety factor against root break (Niemann and Winter, p. 344) [-]

% Loading and sorting of the bearing catalogue
dgb = gearbox.BearCtlg.ball;                % Loading of Schaeffler ball bearing catalogue (starting on p. 246)
% Filter out bearings with insufficient inner diameter for coaxial design
if (strcmpi(gearbox.Input.axles,'coaxial'))
    dgb(dgb.d<(gearbox.shafts.d_sh_3+4+7.5),:) = [];
end
d_ctlg = dgb.d;                             % List of inner bearing diameters [mm]
d_A_ctlg = dgb.d_A;                         % List of outer bearing diameters [mm]
b_ctlg = dgb.b;                             % List of bearing widths [mm]
C_dyn_ctlg = dgb.C_dyn;                     % List of dynamic load ratings [N] 
m_ctlg = dgb.m;                             % List of bearing masses [kg]
f0_ctlg = dgb.f0;                           % List of bearing factors f0 [-]
C_stat_ctlg = dgb.C_stat;                   % List of static load ratings [N]
d_1_ctlg = dgb.d_1;                         % List of diameters of inner bearing ring [mm]
name_ctlg = dgb.Name;                       % List of bearing codes

shaft = 1;                                  % Switch for sub-functions
iteration = 0;                              % Initialization of iteration counter

%% 3) Calculation of shaft 1 data:
while 1
    % Calculation of actual number of teeth and corresponding wheel dimensions
    gearbox = calc_wheel_data (shaft, gearbox);

    % Bearing hollow shaft calculation for shaft 1 according to input load and desired lifetime
    gearbox = set_bearings_shaft_1(shaft, gearbox, d_ctlg, d_A_ctlg, b_ctlg, C_dyn_ctlg, m_ctlg, f0_ctlg, C_stat_ctlg, d_1_ctlg, name_ctlg);

    % Determination of calculation factors needed for safety coefficients
    gearbox = calc_factors(shaft, gearbox);

    % Calculation of safety factors for critical pinion on shaft 1
    gearbox = calc_S_F(shaft, gearbox);
    gearbox = calc_S_H(shaft, gearbox);

    % Verification of safety factors, increase of gear width and diameter according to b/d-ratio
    if (gearbox.results.S_H_12<S_H_min || gearbox.results.S_F_12<S_F_min)
        gearbox.gears_12.d_1 = gearbox.gears_12.d_1+0.25/b_d_1;
        iteration = iteration+1;
    end

    % Iterative decrease of normal module if safety factors are sufficient
    cond_2 = 1;
    if (gearbox.results.S_H_12>(S_H_min+0.2) && gearbox.results.S_F_12>(S_F_min+0.2) && gearbox.gears_12.m_n_1>gearbox.GearingConst.m_n_1_min)
        gearbox.gears_12.m_n_1 = gearbox.gears_12.m_n_1-0.1;
        cond_2 = 0;
    end
        
    if (gearbox.results.S_H_12>=S_H_min && gearbox.results.S_F_12>=S_F_min)
        cond_1 = 1;
    elseif (iteration>150)
        cond_1 = 1;
    else
        cond_1 = 0;
    end

    % End condition for the calculation loop
    if (cond_1 && cond_2)
        break;
    end
end

% Selection of maximum diameter of shaft 1 for outer gearbox dimensions
d_1_arr = [gearbox.gears_12.d_a1 gearbox.bearings_1.d_A_A gearbox.bearings_1.d_A_B];
d_1_max = max(d_1_arr);

%% 4) Output assignment:
gearbox.shafts.d_1_max = d_1_max;               % Maximum diameter of shaft 1 in mm
gearbox.factors.it_sh_1 = iteration;            % Number of iterations of shaft 1

end
