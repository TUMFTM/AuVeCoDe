function gearbox = set_lay_shaft(gearbox)
%% 1) Description
% This function computes data for two-stage transmissions in lay-shaft design
% for given transmission ratios

%% 1.1) Authors
% Peter Koehler
% Date: 24.08.2021

%% 1.2) Sources:
% [1] Master Thesis Koehler: "Semi-physikalische Modellierung von Antriebsstrangkomponenten fuer Elektrofahrzeuge", Institute of Automotive Technology, TUM, Munich, 2021
% [2] Kirchner: "Leistungsuebertragung in Fahrzeuggetrieben", Springer Verlag, 2007, ISBN: 978-3-540-35288-4

%% 2) Initialization required values:
i_tot = gearbox.Input.i_tot;                % Total transmission ratio [-]
T_max = gearbox.Input.T_max*1000;           % Maximum torque of el. motor [Nmm]
tau_tw = gearbox.MatProp.tau_tw;            % Torsional fatigue strength 16MnCr5 (Kirchner, p. 158) [N/mm^2]

% Initialization of normal modules as maximum values depending on the maximum torque (Koehler, pp. 54) [mm]
if (gearbox.Input.T_max<200)
    gearbox.gears_12.m_n_1 = 1.8;           % Reduced maximum normal module of stage 1 for low input torque [mm]
    gearbox.gears_34.m_n_3 = 2.4;           % Reduced maximum normal module of stage 2 for low input torque [mm]
elseif (gearbox.Input.T_max<400)
    gearbox.gears_12.m_n_1 = 2.2;           % Reduced maximum normal module of stage 1 for low input torque [mm]
    gearbox.gears_34.m_n_3 = gearbox.GearingConst.m_n_3_max; % Maximum normal module of stage 2 [mm]
else
    gearbox.gears_12.m_n_1 = gearbox.GearingConst.m_n_1_max; % Maximum normal module of stage 1 [mm]
    gearbox.gears_34.m_n_3 = gearbox.GearingConst.m_n_3_max; % Maximum normal module of stage 2 [mm]

end
gearbox.results.i_tot = i_tot;

%% 3) Calculation of gearbox data:
% Maximum torque at output shaft in Nmm
T_out = T_max*i_tot(1)/2;

% Approximation of output shaft diameter (Kirchner, p.159)
d_sh_3 = ((16*T_out)/(pi*tau_tw))^(1/3);
gearbox.shafts.d_sh_3 = d_sh_3;

% Calculation main dimensions of the gears
gearbox = calc_b_d(gearbox);

% Dimensioning of the first shaft
gearbox = set_shaft_1(gearbox);

% Dimensioning of the second shaft
gearbox = set_shaft_2(gearbox);

% Dimensioning of the differential
gearbox = set_diff(gearbox);

% Calculation of final total transmission ratio
gearbox.results.i_tot = gearbox.results.i_12*gearbox.results.i_34;

end
