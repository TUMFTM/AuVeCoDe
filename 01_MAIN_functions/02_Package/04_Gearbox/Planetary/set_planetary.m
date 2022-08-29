function gearbox = set_planetary (gearbox)
%% 1) Description:
% This Function computes a planetary gearbox for given input data specified
% in the Parameters_gear-file.

%% 1.1) Authors
% Peter Koehler
% Date: 24.08.2021

%% 1.2) Sources:
% [1] Master Thesis Koehler: "Semi-physikalische Modellierung von Antriebsstrangkomponenten fuer Elektrofahrzeuge", Institute of Automotive Technology, TUM, Munich, 2021
% [2] Kirchner: "Leistungsuebertragung in Fahrzeuggetrieben", Springer Verlag, 2007, ISBN: 978-3-540-35288-4

%% 2) Initialization of required values:
i_tot = gearbox.results.i_1s;                   % Total transmission ratio [-]
T_max = gearbox.Input.T_max*1000;               % Maximum torque of el. motor [Nm]
tau_tw = gearbox.MatProp.tau_tw;                % Torsional fatigue strength 16MnCr5 (Kirchner, p. 158) [N/mm^2]

% Initialization of normal modules as maximum values depending on the maximum torque (Koehler, pp. 54) [mm]
if (gearbox.Input.T_max<200)
    gearbox.gears.m_n_1 = 1.6;                  % Reduced maximum normal module of stage 1 for low input torque [mm]
elseif (gearbox.Input.T_max<400)
    gearbox.gears.m_n_1 = 1.8;                  % Reduced maximum normal module of stage 1 for low input torque [mm]
else
    gearbox.gears.m_n_1 = 2.0;   % Maximum normal module of stage 1 [mm]
end
%% 3) Calculation of gearbox data
% Maximum torque at output shaft in Nmm
T_out = T_max*i_tot/2;
% Approximation of output shaft diameter in mm (Kirchner, p.159)
d_sh_3 = ((16*T_out)/(pi*tau_tw))^(1/3);
% Save output shaft diameter in gearbox struct
gearbox.shafts.d_sh_3 = d_sh_3;

% Calculation main dimensions of the gears
gearbox = calc_b_d_plan(gearbox);

% Calculation of the sun shaft
gearbox = set_sun_shaft(gearbox);

% Calculation of the planets
gearbox = set_planets(gearbox);

% Calculation of the differential gear
gearbox = set_diff_plan(gearbox);

% Final total transmission ratio
gearbox.results.i_tot = 1-(gearbox.results.i_1p1*gearbox.results.i_p22);

end
