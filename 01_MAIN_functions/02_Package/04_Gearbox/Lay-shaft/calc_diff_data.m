function gearbox = calc_diff_data(gearbox)
%% 1) Description:
% This function calculates dimensions and gear data of the differtial gear
% according to Zaehringer, starting p. 53

%% 1.1) Authors
% Peter Koehler
% Date: 24.08.2021

%% 1.2) Sources:
% [1] Bachelor Thesis Zaehringer: "Erstellung eines analytischen Ersatzmodells fuer Getriebe von Elektrofahrzeugen", Institute of Automotive Technology, TUM, Munich, 2018
% [2] Semesterarbeit Koehler: "Semi-physikalische Modellierung von Antriebsstrangkomponenten fuer Elektrofahrzeuge", Institute of Automotive Technology, TUM, Munich, 2020
% [3] Master Thesis Koehler: "Semi-physikalische Modellierung von Antriebsstrangkomponenten fuer Elektrofahrzeuge", Institute of Automotive Technology, TUM, Munich, 2021

%% 2) Initialization of required values:
b_d_bevel = gearbox.GearingConst.b_d_bevel;     % Empirical ratio of bevel gear width and outer diameter (Koehler, p.41) [-]
b_3 = gearbox.gears_34.b_3;                     % Width of wheels 3&4 [mm]
d_bev_l = gearbox.diff.d_bev_l;                 % Diameter of larger bevel gears [mm]
t_diffcage = gearbox.ConstDim.t_diffcage;       % Minimum thickness of differential housing [mm]

%% 3) Calculation of differential data:
% Determination of the differential cage width in mm
if strcmpi(gearbox.Input.axles,'coaxial')
    b_diffcage = d_bev_l+2*t_diffcage-0.3*b_3;
else
    b_diffcage = d_bev_l+2*t_diffcage;
end

% Determination of the empirical width of bevel gearing in mm
b_bev = b_d_bevel*d_bev_l;

%% 4) Output assignment:
gearbox.diff.b_diffcage = b_diffcage;       % width of the differential cage in mm
gearbox.diff.b_bev = b_bev;                 % width of bevel gearing in mm

end
