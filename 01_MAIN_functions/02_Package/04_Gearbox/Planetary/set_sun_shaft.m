function gearbox = set_sun_shaft (gearbox)
%% 1) Description:
% This function dimensions the sun shaft of the planetary gearbox.

%% 1.1) Authors
% Peter Koehler
% Date: 24.08.2021

%% 1.2) Sources:
% [1] Master Thesis Koehler: "Semi-physikalische Modellierung von Antriebsstrangkomponenten fuer Elektrofahrzeuge", Institute of Automotive Technology, TUM, Munich, 2021
% [2] Niemann and Winter: "Maschinenelemente - Band 2: Getriebe allgemein, Zahnradgetriebe", Springer Verlag, 2003, ISBN: 978-3-662-11874-0
% [3] Schaeffler: "Waelzlager", Bearing Catalogue, 2019
% [4] Semesterarbeit Koehler: "Semi-physikalische Modellierung von Antriebsstrangkomponenten fuer Elektrofahrzeuge", Institute of Automotive Technology, TUM, Munich, 2020

%% 2) Initialization of required values:
S_H_min = gearbox.MinSafety.S_H_min;        % Minimum safety factor against flank break (Niemann and Winter, p. 344) [-]
S_F_min = gearbox.MinSafety.S_F_min;        % Minimum safety factor against root break (Niemann and Winter, p. 344) [-]
m_n_plan_min = 1.3;                         % Minimum normal module of planetary gears from BEV gearbox database (Koehler [1]) [mm]

% Loading of the bearing catalogue
ab = gearbox.BearCtlg.ang_ball;             % Loading of Schaeffler angular ball bearing catalogue (starting on p. 272)
% Filter out bearings with insufficient inner diameter for coaxial design
ab(ab.d<(gearbox.shafts.d_sh_3+4+7.5),:) = [];
d_ctlg = ab.d;                              % List of inner bearing diameters [mm]
d_A_ctlg = ab.d_A;                          % List of outer bearing diameters [mm]
b_ctlg = ab.b;                              % List of bearing widths [mm]
a_ctlg = ab.a;                              % List of bearing point distances [mm]
C_dyn_ctlg = ab.C_dyn;                      % List of dynamic load ratings [N] 
m_ctlg = ab.m;                              % List of bearing masses [kg]
C_stat_ctlg = ab.C_stat;                    % List of static load ratings [N]
d_1_ctlg = ab.d_1;                          % List of diameters of inner bearing ring [mm]
name_ctlg = ab.Name;                        % List of bearing codes

shaft = 1;                                  % Switch for sub-functions
iteration = 0;                              % Initialization of iteration counter

%% 3) Calculation of sun shaft data:
while 1
    % Calculation of actual number of teeth and corresponding wheel dimensions
    gearbox = calc_wheel_data_plan (gearbox);
    
    % Bearing and hollow shaft calculation for sun shaft according to input load and desired lifetime
    gearbox = set_bearings_sun (shaft, gearbox, d_ctlg, d_A_ctlg, b_ctlg, a_ctlg, C_dyn_ctlg, m_ctlg, C_stat_ctlg, d_1_ctlg, name_ctlg);

    % Determination of calculation factors needed for safety coefficients
    gearbox = calc_factors_plan(shaft, gearbox);

    % Calculation of safety factors for critical pinion on sun gear
    gearbox = calc_S_F_plan(shaft, gearbox);
    gearbox = calc_S_H_plan(shaft, gearbox);

    % Verification of safety factors, increase of gear width if not sufficient
    if (gearbox.results.S_H_1p1<S_H_min || gearbox.results.S_F_1p1<S_F_min)
        gearbox.gears.b_1 = gearbox.gears.b_1+0.2;
        iteration = iteration+1;
    end
 
    % Iterative decrease of normal module if safety factors are sufficient and overlap ratio is <2 until minimum module
    cond_2 = 1;
    if (gearbox.results.S_H_1p1>(S_H_min+0.1) && gearbox.results.S_F_1p1>(S_F_min+0.1) && gearbox.gears.m_n_1>m_n_plan_min)
        gearbox.gears.m_n_1 = gearbox.gears.m_n_1-0.1;
        cond_2 = 0;
    end
        
    if (gearbox.results.S_H_1p1>=S_H_min && gearbox.results.S_F_1p1>=S_F_min)
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

%% 4) Output assignment:
gearbox.factors.it_sh_1 = iteration;            % Number of iterations of sun shaft

end
