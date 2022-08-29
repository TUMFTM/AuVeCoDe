function gearbox = set_bearings_sun (shaft, gearbox, d_ctlg, d_A_ctlg, b_ctlg, a_ctlg, C_dyn_ctlg, m_ctlg, C_stat_ctlg, d_1_ctlg, name_ctlg)
%% 1) Description:
% This function selects ball bearings for the first shaft of 
% the gearbox from the Schaeffler bearing catalog.
% SHAFT 1:
% Fixed bearing combination in O arrangement with two angular ball bearings
%           _
%   B----A-|_|
%
%        |-a|  (bearing distance)
%
% System of coordinates for calculation of forces: 
%   y ↑
%     |
%   z ⦿--> x

%% 1.1) Authors
% Peter Koehler
% Date: 24.08.2021

%% 1.2) Sources:
% [1] Master Thesis Koehler: "Semi-physikalische Modellierung von Antriebsstrangkomponenten fuer Elektrofahrzeuge", Institute of Automotive Technology, TUM, Munich, 2021
% [2] Stahl: "Formelsammlung zur Vorlesung Maschinenelemente", Wintersemester 2015/16
% [3] Kirchner: "Leistungsuebertragung in Fahrzeuggetrieben", Springer Verlag, 2007, ISBN: 978-3-540-35288-4
% [4] Bachelor Thesis Zaehringer: "Erstellung eines analytischen Ersatzmodells fuer Getriebe von Elektrofahrzeugen", Institute of Automotive Technology, TUM, Munich, 2018
% [5] Schaeffler: "Waelzlager", Bearing Catalogue, 2019

%% 2) Initialization of required values:
T_nom = gearbox.Input.T_nom*1000;                   % Nominal torque of el. motor [Nmm]
overload_factor = gearbox.Input.overload_factor;    % Overload factor of el. motor [-]
m_n_1 = gearbox.gears.m_n_1;                        % Normal module of first stage [mm]
alpha_t_1 = gearbox.gears.alpha_t_1;                % Pressure angle in transverse section [rad]
beta_1 = gearbox.gears.beta_1;                      % Helix angle of gearing [rad]
L_10 = gearbox.CalcFactors.L_10;                    % Lifetime factor regarding input speed [10^6 revolutions]
S_Dt_min = gearbox.MinSafety.S_Dt_min;              % Minimum safety factor against torsional fatigue fracture (Stahl, p. 33) [-]
z_1 = gearbox.gears.z_1;                            % Number of teeth of sun gear
z_p1 = gearbox.gears.z_p1;                          % Number of teeth of planet 1
d_1 = gearbox.gears.d_1;                            % Pitch diameter of sun gear [mm]
b_1 = gearbox.gears.b_1;                            % Width of first stage gears [mm]
d_p1 = gearbox.gears.d_p1;                          % Pitch diameter of planet 1 [mm]
d_sh_3 = gearbox.shafts.d_sh_3;                     % Diameter of output shafts [mm]

% Initialization of bearing matrices for vectorial calculations
% Variation over the rows means variation of bearing A, variation over the columns varies bearing B
m = length(d_ctlg);
% m x m matrices where the static and dynamic load ratings of the bearings are varied over the rows
C_stat_mat = repmat(C_stat_ctlg,1,m);
C_dyn_mat = repmat(C_dyn_ctlg,1,m);

%% 3) Calculation of bearing loads and shaft design:
%% 3.1) Calculation of pressure angle and gearing forces:
% Calculation of the operating pressure angle in rad (Kirchner, p. 189)
alpha_wt_1 = acos((((z_1+z_p1)*m_n_1)/(d_1+d_p1))*(cos(alpha_t_1)/cos(beta_1)));

% Calculation of occuring gearing forces in N (Stahl, p. 2)
F_u_1 = 2*T_nom/d_1;                % Circumferential force on the sun gear
F_ax_1 = F_u_1*tan(beta_1);         % Axial force on the sun gear

% Minimum diameter of sun shaft in mm
d_sh_min = d_sh_3+4+10;         % output shafts + 2 mm gap + 5 mm shaft thickness

%% 3.1) Calculation loop for verification of bearing forces and distances l and s:
% Axial force only on bearing A in N, no radial forces (central shaft)
F_axA = F_ax_1;
    
%% 3.2) Calculation of bearing loads (Stahl, pp. 82)
%% Bearing A
% Static load ratings for bearing A (Schaeffler, p. 268)
P_stat_A = 0.26*F_axA*overload_factor;      % Equivalent static load rating in N (in relation to T_max)
C_stat_req_A = 2.1*P_stat_A;                % Required static load rating in N, Safety factor according to Stahl, p. 83 and Zaehringer, p. 29
    
% Dynamic load ratings for bearing A (Schaeffler, p. 267)
P_dyn_A = 0.57*F_axA;                       % Equivalent dynamic load rating in N (in relation to T_nom)
C_dyn_req_A = (L_10^(1/3))*P_dyn_A;         % Required equivalent dynamic load rating in N

% Check for static (cond_A1) and dynamic (cond_A2) load for bearing A
cond_A1 = (C_stat_mat>=C_stat_req_A);
cond_A2 = (C_dyn_mat>=C_dyn_req_A);
      
%% Bearing B
% Static load ratings for bearing B (Schaeffler, p. 268)
P_stat_B = 0;                               % No nominal radial or axial forces on bearing B
C_stat_req_B = 2.1*P_stat_B;                % Required static load rating in N, Safety factor according to Stahl, p. 83 and Zaehringer, p. 29
        
% Dynamic load ratings (Stahl, p. 84)
P_dyn_B = 0;                                % No nominal radial or axial forces on bearing B
C_dyn_req_B = (L_10^(1/3))*P_dyn_B;         % Required equivalent dynamic load rating in N

% Check for static (cond_B1) and dynamic (cond_B2) load for bearing B
cond_B1 = (C_stat_mat'>=C_stat_req_B);
cond_B2 = (C_dyn_mat'>=C_dyn_req_B);

%% 3.5) Calculation of shaft:
% Calculation of shaft safety and inner diameter of hollow shaft
gearbox = calc_d_sh_plan(shaft, gearbox, d_ctlg);
sh_mat = gearbox.shafts.shaft_1;

% Check if shaft endures torsional load
S_sh_min = min(repmat(sh_mat(:,2),1,m),repmat(sh_mat(:,2)',m,1));
cond_s1 = (S_sh_min>S_Dt_min);

% Check if minimum bearing diameter is larger than the minimum shaft diameter
d_bear_min = min(repmat(d_ctlg,1,m),repmat(d_ctlg',m,1));
cond_s2 = (d_bear_min>=d_sh_min);

%% 3.6) Check if all conditions are met for bearing combinations and selection of optimum
% Bearing and shaft conditions are combined
all_cond = cond_A1.*cond_A2.*cond_B1.*cond_B2.*cond_s1.*cond_s2;

% Find best bearing combination
[poss_A, poss_B] = find(all_cond);
poss = sum([poss_A,poss_B],2);
[~,idx] = min(poss);

% Print error message if no bearings are found
if isempty(poss)
    disp('Error! No bearing combination was found for sun shaft!')
    gearbox.error.ratio_C_dyn_A = C_dyn_mat./C_dyn_req_A;
    gearbox.error.ratio_C_dyn_B = C_dyn_mat'./C_dyn_req_B;
end

%% 3.7) Calculation of the bearing distances for calc_factors
% Distances a and b on shaft 1 for chosen bearing combination in mm
a = (b_ctlg(poss_A(idx))-a_ctlg(poss_A(idx)))+5+b_1/2;

%% 4) Output assignment:
gearbox.gears.alpha_wt_1 = alpha_wt_1;              % working transverse pressure angle in rad
gearbox.bearings_1.a = a;                           % distance between wheel 1 and bearing point A in mm
gearbox.bearings_1.d_sh_A = d_ctlg(poss_A(idx));    % inner diameter of bearing A in mm
gearbox.bearings_1.d_sh_B = d_ctlg(poss_B(idx));    % inner diameter of bearing B in mm
gearbox.bearings_1.m_A = m_ctlg(poss_A(idx));       % mass of bearing A in kg
gearbox.bearings_1.m_B = m_ctlg(poss_B(idx));       % mass of bearing B in kg
gearbox.bearings_1.d_A_A = d_A_ctlg(poss_A(idx));   % outer diameter of bearing A in mm
gearbox.bearings_1.d_A_B = d_A_ctlg(poss_B(idx));   % outer diameter of bearing B in mm
gearbox.bearings_1.b_A = b_ctlg(poss_A(idx));       % width of bearing A in mm
gearbox.bearings_1.b_B = b_ctlg(poss_B(idx));       % width of bearing B in mm
gearbox.bearings_1.d_1_A = d_1_ctlg(poss_A(idx));   % outer diameter of inner bearing ring of bearing A in mm
gearbox.bearings_1.d_1_B = d_1_ctlg(poss_B(idx));   % outer diameter of inner bearing ring of bearing B in mm
gearbox.bearings_1.ID_A = name_ctlg(poss_A(idx));   % Name of bearing A from bearing catalog
gearbox.bearings_1.ID_B = name_ctlg(poss_B(idx));   % Name of bearing B from bearing catalog
gearbox.forces.F_u_1 = F_u_1;                       % Circumferential force on sun gear in N
gearbox.forces.F_ax_1 = F_ax_1;                     % Axial force on sun gear in N
gearbox.shafts.d_sh_1 = min(sh_mat(min(poss_A(idx),poss_B(idx)),1),gearbox.gears.d_f1); % Critical diameter of sun shaft in mm
gearbox.shafts.S_dt_1 = sh_mat(min(poss_A(idx),poss_B(idx)),2);                         % Safety factor against torsional fatigue of sun shaft
gearbox.shafts.d_inn_1 = sh_mat(min(poss_A(idx),poss_B(idx)),3);                        % Inner diameter of sun shaft in mm

end
