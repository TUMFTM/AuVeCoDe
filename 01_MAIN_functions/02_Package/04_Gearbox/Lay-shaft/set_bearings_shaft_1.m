function gearbox = set_bearings_shaft_1(shaft, gearbox, d_ctlg, d_A_ctlg, b_ctlg, C_dyn_ctlg, m_ctlg, f0_ctlg, C_stat_ctlg, d_1_ctlg, name_ctlg)
%% 1) Description:
% This function selects deep groove ball bearings for the first shaft of 
% the gearbox from the Schaeffler bearing catalog.
% SHAFT 1:
% Fixed/floating bearings with two deep groove ball bearings
%        _
%   A---|_|--B
%
%   |-a--|-b-|  (bearing distances) 
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
T_nom = gearbox.Input.T_nom*1000;                       % Nominal torque of el. motor [Nmm]
overload_factor = gearbox.Input.overload_factor;        % Overload factor of el. motor [-]
m_n_1 = gearbox.gears_12.m_n_1;                         % Normal module of first stage [mm]
alpha_t = gearbox.gears_12.alpha_t_1;                   % Pressure angle in transverse section of first stage [rad]
beta_1 = gearbox.gears_12.beta_1;                       % Helix angle of first stage [rad]
b_park = gearbox.ConstDim.b_park;                       % Width of parking gear [mm]
b_gap = gearbox.ConstDim.b_gap;                         % Gap between components on the shafts [mm]
L_10 = gearbox.CalcFactors.L_10;                        % Lifetime factor regarding input speed [10^6 revolutions]
S_Dt_min = gearbox.MinSafety.S_Dt_min;                  % Minimum safety factor against torsional fatigue fracture (Stahl, p. 33) [-]
z_1 = gearbox.gears_12.z_1;                             % Number of teeth of wheel 1
z_2 = gearbox.gears_12.z_2;                             % Number of teeth of wheel 2
b_1 = gearbox.gears_12.b_1;                             % Width of wheel 1 [mm]
d_1 = gearbox.gears_12.d_1;                             % Pitch diameter of wheel 1 [mm]
a_12 = gearbox.results.a_12;                            % Distance between shafts 1&2 [mm]
if strcmpi(gearbox.Input.axles,'parallel')
    d_sh_min = 0;
else
    d_sh_min = gearbox.shafts.d_sh_3+4+10;
end

% Initialization of bearing matrices for vectorial calculations
% Variation over the rows means variation of bearing A, variation over the columns varies bearing B
m = length(d_ctlg);
b_vert = repmat(b_ctlg,1,m);            % m x m matrix where the width of bearing A is varied over the rows
b_hor = b_vert';                        % m x m matrix where the width of bearing B is varied over the columns
% m x m matrices where the static and dynamic load ratings of the bearings are varied over the rows
C_stat_mat = repmat(C_stat_ctlg,1,m);   
C_dyn_mat = repmat(C_dyn_ctlg,1,m);

%% 3) Calculation of bearing loads and shaft design:
%% 3.1) Calculation of pressure angle and gearing forces:
% Calculation of the operating pressure angle in rad (Kirchner, p. 189)
alpha_wt = acos((((z_1+z_2)*m_n_1)/(2*a_12))*(cos(alpha_t)/cos(beta_1)));

% Calculation of occuring gearing forces in N
F_u_1 = 2*T_nom/d_1;
F_ax_1 = F_u_1*tan(beta_1);
F_rad_1 = F_u_1*tan(alpha_wt);

%% 3.2) Calculation of bearing distances a and b for bearing force calculation:
if (strcmpi(gearbox.Input.axles, 'parallel'))
    a = b_vert/2+b_gap+b_park+b_gap+b_1/2;              % Distance between force application points of bearing A and wheel 1
elseif (strcmpi(gearbox.Input.axles, 'coaxial'))
    gearbox.shafts.d_sh_1 = gearbox.shafts.d_sh_3+4+7.5;% Initialization of shaft diameter for coaxial design
    a = b_vert/2+b_gap+b_1/2;                           % Distance between force application points of bearing A and wheel 1 (no parking gear)
end
b = b_1/2+b_gap+b_hor/2;                                % Distance between force application points of wheel 1 and bearing B

%% 3.3) Calculation of bearing forces:
% Bearing reaction forces in N in Y-direction
F_Ay = 1./(a+b).*(F_rad_1*b+F_ax_1*d_1/2);
F_By = F_rad_1-F_Ay;
% Bearing reaction forces in N in Z-direction    
F_Az = 1./(a+b).*(F_u_1*b);
F_Bz = F_u_1-F_Az;
% Radial forces in N
F_radA = sqrt((F_Az.^2)+(F_Ay.^2));
F_radB = sqrt((F_Bz.^2)+(F_By.^2));
        
% Distribution of axial forces on bearing with lower radial load (Zaehringer, p. 28/29)
F_axA = (F_radA<F_radB).*F_ax_1;
F_axB = (F_radA>=F_radB).*F_ax_1;
    
%% 3.4) Calculation of bearing loads (Stahl, starting p. 82)
%% Bearing A
% Definition of axial to radial load ratio e (Stahl, p.84)
val = f0_ctlg.*F_axA./C_stat_ctlg;
e = (val<0.5)*0.22+(val>=0.5 & val<0.9)*0.24+(val>=0.9 & val<1.6)*0.28...
    +(val>=1.6 & val<3)*0.32+(val>=3 & val<6)*0.36+(val>=6)*0.43;

% Determination of radial and axial factors (X and Y) of bearing (Stahl, p.84)
par = F_axA./F_radA;
X = (par<=e)+(par>e)*0.56;
Y = (par>e & e==0.22)*2+(par>e & e==0.24)*1.8+(par>e & e==0.28)*1.6...
    +(par>e & e==0.32)*1.4+(par>e & e==0.36)*1.2+(par>e & e==0.43);
        
% Static load ratings (Stahl, p. 82/83)
P_stat_A = (0.6*F_radA+0.5*F_axA)*overload_factor;  % Equivalent static load rating in N (in relation to T_max)
C_stat_req_A = 2.1*P_stat_A;                        % Required static load rating in N, Safety factor according to Stahl, p. 83 and Zaehringer, p. 29

% Dynamic load ratings (Stahl, p. 84)
P_next_A = (X.*F_radA)+(Y.*F_axA);                  % Equivalent dynamic load rating in N (in relation to T_nom)
C_dyn_req_A = (L_10^(1/3))*P_next_A;                % Required equivalent dynamic load rating in N

% Check for static (cond_A1) and dynamic (cond_A2) load for bearing A
cond_A1 = (C_stat_mat>=C_stat_req_A);
cond_A2 = (C_dyn_mat>=C_dyn_req_A);
        
%% Bearing B        
% Definition of axial to radial load ratio e (Stahl, p.84)
val = f0_ctlg'.*F_axB./C_stat_ctlg';
e = (val<0.5)*0.22+(val>=0.5 & val<0.9)*0.24+(val>=0.9 & val<1.6)*0.28...
    +(val>=1.6 & val<3)*0.32+(val>=3 & val<6)*0.36+(val>=6)*0.43;

% Determination of radial and axial factors (X and Y) of bearing (Stahl, p.84)
par = F_axB./F_radB;
X = (par<=e)+(par>e)*0.56;
Y = (par>e & e==0.22)*2+(par>e & e==0.24)*1.8+(par>e & e==0.28)*1.6...
    +(par>e & e==0.32)*1.4+(par>e & e==0.36)*1.2+(par>e & e==0.43);
        
% Static load ratings (Stahl, p. 82/83)
P_stat_B = (0.6*F_radB+0.5*F_axB)*overload_factor;  % Equivalent static load rating in N (in relation to T_max)
C_stat_req_B = 2.1*P_stat_B;                        % Required static load rating in N, Safety factor according to Stahl, p. 83 and Zaehringer, p. 29
% Dynamic load ratings (Stahl, p. 84)
P_next_B = (X.*F_radB)+(Y.*F_axB);                  % Equivalent dynamic load rating in N (in relation to T_nom)
C_dyn_req_B = (L_10^(1/3))*P_next_B;                % Required equivalent dynamic load rating in N

% Check for static (cond_B1) and dynamic (cond_B2) load for bearing B
cond_B1 = (C_stat_mat'>=C_stat_req_B);
cond_B2 = (C_dyn_mat'>=C_dyn_req_B);

%% 3.5) Calculation of shaft:
% Calculation of shaft safety and inner diameter of hollow shaft
gearbox = calc_d_sh(shaft, gearbox, d_ctlg);
sh_mat = gearbox.shafts.shaft_1;

% Check if shaft endures torsional load
S_sh_min = min(repmat(sh_mat(:,2),1,m),repmat(sh_mat(:,2)',m,1));
cond_s1 = (S_sh_min>S_Dt_min);

% Check if minimum bearing diameter is larger than the minimum shaft diameter (otherwise the shaft safety is not sufficient!)
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
    disp('Error! No bearing combination was found for shaft 1!')
    gearbox.error.ratio_C_dyn_A = C_dyn_mat./C_dyn_req_A;
    gearbox.error.ratio_C_dyn_B = C_dyn_mat'./C_dyn_req_B;
end

%% 3.7) Calculation of the bearing distances for calc_factors
% Distances a and b on shaft 1 for chosen bearing combination in mm
a = a(poss_A(idx),1);
b = b(1,poss_B(idx));
% Distance of the bearing points on shaft 1 in mm
l_1 = a+b;
% Distances of the force application point and the middle of the bearings for wheel 1 in mm
s_1 = (a-b)/2;

%% 4) Output assignment:
gearbox.gears_12.alpha_wt_1 = alpha_wt;                 % working transverse pressure angle in rad
gearbox.results.a_12 = a_12;                            % distance between shafts 1&2 in mm
gearbox.bearings_1.s_1 = s_1;                           % relative position to centre of bearing distance in mm
gearbox.bearings_1.l_1 = l_1;                           % bearing distance in mm
gearbox.bearings_1.a = a;                               % distance between bearing point A and wheel 1 in mm
gearbox.bearings_1.b = b;                               % distance between wheel 1 and bearing point B in mm
gearbox.bearings_1.d_sh_A = d_ctlg(poss_A(idx));        % inner diameter of bearing A in mm
gearbox.bearings_1.d_sh_B = d_ctlg(poss_B(idx));        % inner diameter of bearing B in mm
gearbox.bearings_1.m_A = m_ctlg(poss_A(idx));           % mass of bearing A in kg
gearbox.bearings_1.m_B = m_ctlg(poss_B(idx));           % mass of bearing B in kg
gearbox.bearings_1.d_A_A = d_A_ctlg(poss_A(idx));       % outer diameter of bearing A in mm
gearbox.bearings_1.d_A_B = d_A_ctlg(poss_B(idx));       % outer diameter of bearing B in mm
gearbox.bearings_1.b_A = b_ctlg(poss_A(idx));           % width of bearing A in mm
gearbox.bearings_1.b_B = b_ctlg(poss_B(idx));           % width of bearing B in mm
gearbox.bearings_1.d_1_A = d_1_ctlg(poss_A(idx));       % outer diameter of inner bearing ring of bearing A in mm
gearbox.bearings_1.d_1_B = d_1_ctlg(poss_B(idx));       % outer diameter of inner bearing ring of bearing B in mm
gearbox.bearings_1.ID_A = name_ctlg(poss_A(idx));       % Name of bearing A from bearing catalog
gearbox.bearings_1.ID_B = name_ctlg(poss_B(idx));       % Name of bearing B from bearing catalog
gearbox.forces.F_u_1 = F_u_1;                           % Circumferential force on wheel 1 in N
gearbox.forces.F_rad_1 = F_rad_1;                       % Radial force on wheel 1 in N
gearbox.forces.F_ax_1 = F_ax_1;                         % Axial force on wheel 1 in N
gearbox.shafts.d_sh_1 = sh_mat(min(poss_A(idx),poss_B(idx)),1);     % Critical diameter of shaft 1 in mm
gearbox.shafts.S_dt_1 = sh_mat(min(poss_A(idx),poss_B(idx)),2);     % Safety factor against torsional fatigue of shaft 1
gearbox.shafts.d_inn_1 = sh_mat(min(poss_A(idx),poss_B(idx)),3);    % Inner diameter of shaft 1 in mm

end
