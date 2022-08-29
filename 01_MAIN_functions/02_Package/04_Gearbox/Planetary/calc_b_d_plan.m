function gearbox = calc_b_d_plan (gearbox)
%% 1) Description:
% This function computes initial gear dimensions for the given transmission
% ratio according to the diameter of the output shaft.

%% 1.1) Authors
% Peter Koehler
% Date: 24.08.2021

%% 1.2) Sources:
% [1] Master Thesis Koehler: "Semi-physikalische Modellierung von Antriebsstrangkomponenten fuer Elektrofahrzeuge", Institute of Automotive Technology, TUM, Munich, 2021
% [2] Decker and Kabus: "Maschinenelemente - Formeln", Carl Hanser Verlag, 2011, ISBN: 978-3-446-42990-1

%% 2) Initialization of required values:
epsilon_beta = gearbox.GearingConst.epsilon_beta;   

%% 3) Calculation of initial gear widths:
% Approximation of first stage gear width with helix angle (30 deg) and
% minimum normal module (1.3) in mm (Decker, p. 136)
b_1 = epsilon_beta*1.3*pi/sin(pi/6);
% Approximation of second stage gear width according to empirical
% stage-width-ratio in mm (Koehler, p. 51)
b_2 = 1.6*b_1;

%% 4) Output Assignment:
gearbox.gears.b_1 = b_1;        % Width of first stage gears in mm
gearbox.gears.b_2 = b_2;        % Width diameter of second stage gears in mm

end
