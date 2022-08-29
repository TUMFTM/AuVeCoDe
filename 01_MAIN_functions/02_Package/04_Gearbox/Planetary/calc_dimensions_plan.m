function gearbox = calc_dimensions_plan(gearbox)
%% 1) Description:
% This function computes the main dimensions of the planetary gearbox.

%% 1.1) Authors
% Peter Koehler
% Date: 24.08.2021

%% 1.2) Sources:
% [1] Master Thesis Koehler: "Semi-physikalische Modellierung von Antriebsstrangkomponenten fuer Elektrofahrzeuge", Institute of Automotive Technology, TUM, Munich, 2021
% [2] Semesterarbeit Koehler: "Semi-physikalische Modellierung von Antriebsstrangkomponenten fuer Elektrofahrzeuge", Institute of Automotive Technology, TUM, Munich, 2020

%% 2) Initialization of required values:
b_gap = gearbox.ConstDim.b_gap;                 % Gap between components on the shafts [mm]
t_housing = gearbox.ConstDim.t_housing;         % Housing thickness as mean of database ([1], p. 60) [mm]
d_housing = gearbox.ConstDim.d_housing;         % Distance between gears and housing as mean of database (Koehler, p. 60) [mm]
b_seal = gearbox.ConstDim.b_seal;               % Width of seals on output shafts as mean of database (Koehler, p. 60) [mm]
d_flange = gearbox.ConstDim.d_flange;           % Additional space for screws on housing flange as average of database (Koehler, p. 60) [mm]
m_n_1 = gearbox.gears.m_n_1;                    % Normal module of the first stage [mm]
m_n_2 = gearbox.gears.m_n_2;                    % Normal module of the second stage [mm]
d_1 = gearbox.gears.d_1;                        % Pitch diameter of sun gear [mm]
d_p1 = gearbox.gears.d_p1;                      % Pitch diameter of the planet 1 [mm]
d_ap2 = gearbox.gears.d_ap2;                    % Outside diameter of planet 2 [mm]
d_f2 = gearbox.gears.d_f2;                      % Root diameter of ring gear [mm]
b_1 = gearbox.gears.b_1;                        % Width of first stage gears [mm]
b_2 = gearbox.gears.b_2;                        % Width of second stage gears [mm]
d_s = gearbox.gears.d_s;                        % Diameter of planet circle [mm]
b_C = gearbox.bearings_2.b_C;                   % Width of bearing C [mm]
b_D = gearbox.bearings_2.b_D;                   % Width of bearing D [mm]
b_E = gearbox.bearings_3.b_E;                   % Width of bearing E [mm]
b_F = gearbox.bearings_3.b_F;                   % Width of bearing F [mm]
if (gearbox.Input.num_EM==1)
    d_diffcage = gearbox.diff.d_diffcage;       % Diameter of differential cage [mm]
    b_diffcage = gearbox.diff.b_diffcage;       % Width of differential cage [mm]
elseif (gearbox.Input.num_EM==2)
    d_diffcage = gearbox.bearings_3.d_1_E;
    b_diffcage = b_gap;
end

%% 3) Calculation of the gearbox' dimensions:
%% 3.1) Calculation of gearbox width in mm ([2], pp. 56)
t_plancarrier = b_F+5+b_C+b_gap+b_1+b_gap+b_2+b_gap+b_D+5;
t_diff = b_diffcage+b_E;

if (gearbox.Input.num_EM==1)
    % Integration of differential in planet carrier if planet circle is large enough
    if (d_s>d_diffcage+4+d_ap2)
        t_gearing = t_plancarrier+t_diff-(5+b_D+b_gap+b_2);
    else
        t_gearing = t_plancarrier+t_diff;
    end
elseif (gearbox.Input.num_EM==2)
    t_gearing = t_plancarrier+t_diff;
end

% Width of housing for mass calculations in mm ([1], p. 61)
t_gearbox = b_seal+t_gearing+b_seal;

%% 3.2) Determination of gearbox diameter in mm ([2], pp. 56)
% Maximum outer diameter of gears of the first stage in mm
d_1_max = d_1+2*d_p1+2*m_n_1+2*d_housing;
% Maximum diameter of the ring gear (second stage) in mm
d_2_max = d_f2+2*6*m_n_2; 
% Critical diameter for outer gearbox dimensions in mm
d_gearing = max(d_1_max, d_2_max);

% Calculation of gearbox length and height in mm  ([1], p. 61)
d_gearbox = d_gearing+2*t_housing+2*d_flange;

%% 4) Output assignment:
gearbox.hous_dim.d_gearbox = d_gearbox;     % Diameter of the gearbox housing according to dimensional chains in mm
gearbox.hous_dim.t_gearbox = t_gearbox;     % Width of the gearbox housing according to dimensional chains in mm
gearbox.hous_dim.d_gearing = d_gearing;     % Largest diameter of all gears in mm
gearbox.hous_dim.t_gearing = t_gearing;     % Width of all components inside the gearbox housing in mm
gearbox.results.d_gearbox = d_gearbox;      % Diameter of the gearbox housing according to dimensional chains in mm
gearbox.results.t_gearbox = t_gearbox;      % Width of the gearbox housing according to dimensional chains in mm
gearbox.diff.t_diff = t_diff;               % Width of differential in mm
gearbox.shafts.d_1_max = d_1_max;           % Maximum outer diameter of gears of the first stage [mm]
gearbox.shafts.d_2_max = d_2_max;           % Maximum diameter of the ring gear (second stage) [mm]

end
