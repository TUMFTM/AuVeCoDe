function gearbox = set_planets (gearbox)
%% 1) Description:
% This function computes the planet shaft of the planetary gearbox.

%% 1.1) Authors
% Peter Koehler
% Date: 24.08.2021

%% 1.2) Sources:
% [1] Master Thesis Koehler: "Semi-physikalische Modellierung von Antriebsstrangkomponenten fuer Elektrofahrzeuge", Institute of Automotive Technology, TUM, Munich, 2021
% [2] Niemann and Winter: "Maschinenelemente - Band 2: Getriebe allgemein, Zahnradgetriebe", Springer Verlag, 2003, ISBN: 978-3-662-11874-0
% [3] Schaeffler: "Waelzlager", Bearing Catalogue, 2019
% [4] Semesterarbeit Koehler: "Semi-physikalische Modellierung von Antriebsstrangkomponenten fuer Elektrofahrzeuge", Institute of Automotive Technology, TUM, Munich, 2020

%% 2) Initialization of required values:
S_H_min = gearbox.MinSafety.S_H_min;    % Minimum safety factor against flank break (Niemann and Winter, p. 344) [-]
S_F_min = gearbox.MinSafety.S_F_min;    % Minimum safety factor against root break (Niemann and Winter, p. 344) [-]

% Loading of the bearing catalogue
tr = gearbox.BearCtlg.roller;           % Loading of Schaeffler taper roller bearing catalogue (starting on p. 590)
d_ctlg = tr.d;                          % List of inner bearing diameters [mm]
d_A_ctlg = tr.d_A;                      % List of outer bearing diameters [mm]
b_ctlg = tr.b;                          % List of bearing widths [mm]
C_dyn_ctlg = tr.C_dyn;                  % List of dynamic load ratings [N] 
m_ctlg = tr.m;                          % List of bearing masses [kg]
a_ctlg = tr.a;                          % List of bearing distances [mm]
e_ctlg = tr.e;                          % List of bearing factors e [-]
Y_ctlg = tr.Y;                          % List of bearing factors Y [-]
d_1_ctlg = tr.d_1;                      % List of diameters of inner bearing ring [mm]
C_stat_ctlg = tr.C_stat;                % List of static load ratings [N]
Y_0_ctlg = tr.Y_0;                      % List of bearing factors Y_0 [-]
name_ctlg = tr.Name;                    % List of bearing codes

shaft = 2;                              % Switch for sub-functions
iteration = 0;                          % Initialization of iteration counter

%% 3) Calculation of planet data:
while 1
    % Bearing hollow shaft calculation for shaft 2 according to input load and desired lifetime
    gearbox = set_bearings_planet(shaft, gearbox, d_ctlg, d_A_ctlg, b_ctlg, C_dyn_ctlg, m_ctlg, a_ctlg, e_ctlg, Y_ctlg, C_stat_ctlg, Y_0_ctlg, d_1_ctlg, name_ctlg);

    % Determination of calculation factors needed for safety coefficients
    gearbox = calc_factors_plan(shaft, gearbox);

    % Calculation of safety factors for critical pinion on shaft 2
    gearbox = calc_S_F_plan(shaft, gearbox);
    gearbox = calc_S_H_plan(shaft, gearbox);

    % Verification of safety factors, increase of gear width
    if (gearbox.results.S_H_p22<S_H_min || gearbox.results.S_F_p22<S_F_min)
        gearbox.gears.b_2 = gearbox.gears.b_2+1;
        iteration = iteration+1;
    end

    % Condition for end of loop: safety factors and overlap ratio okay, or iterations>150
    if (gearbox.results.S_H_p22>=S_H_min && gearbox.results.S_F_p22>=S_F_min)
        cond_1 = 1;
    elseif (iteration>150)
        cond_1 = 1;
    else
    cond_1 = 0;
    end

    % End condition for the calculation loop
    if (cond_1)
        break;
    end
end

%% 4) Output assignment:
gearbox.factors.it_sh_2 = iteration;            % Number of iterations of shaft 2
end
