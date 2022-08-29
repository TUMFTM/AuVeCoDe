function gearbox = set_bearings_shaft_2(shaft, gearbox, d_ctlg, d_A_ctlg, b_ctlg, C_dyn_ctlg, m_ctlg, f0_ctlg, C_stat_ctlg, d_1_ctlg, name_ctlg)
%% 1) Description:
% This function selects deep groove ball bearings for the second shaft of 
% the gearbox from the Schaeffler bearing catalog.
% SHAFT 2:
% Fixed/floating bearings with two deep groove ball bearings
%       _
%      | |  _
%   C--|2|-|3|--D
%      |_|
%
%   |-a-|-b-|-c-|   (bearing distances)
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
m_n_3 = gearbox.gears_34.m_n_3;                     % Normal module of second stage [mm]
alpha_t = gearbox.gears_34.alpha_t_2;               % Pressure angle in transverse section of second stage [rad]
beta_2 = gearbox.gears_34.beta_2;                   % Helix angle of second stage [rad]
b_gap = gearbox.ConstDim.b_gap;                     % Gap between components on the shafts [mm]
z_3 = gearbox.gears_34.z_3;                         % Number of teeth of wheel 3
z_4 = gearbox.gears_34.z_4;                         % Number of teeth of wheel 4
b_B = gearbox.bearings_1.b_B;                       % Width of bearing B [mm]
b_1 = gearbox.gears_12.b_1;                         % Width of wheel 1 [mm]
b_3 = gearbox.gears_34.b_3;                         % Width of wheel 3 [mm]               
d_2 = gearbox.gears_12.d_2;                         % Pitch diameter of wheel 2 [mm]
d_3 = gearbox.gears_34.d_3;                         % Pitch diameter of wheel 3 [mm]
a_23 = gearbox.results.a_23;                        % Distance between shafts 2&3 [mm]
i_12 = gearbox.results.i_12;                        % Transmission ratio of first stage [-]
L_10 = gearbox.CalcFactors.L_10/i_12;               % Lifetime factor regarding speed of second shaft [10^6 revolutions]
S_Dt_min = gearbox.MinSafety.S_Dt_min;              % Minimum safety factor against torsional fatigue fracture (Stahl, p. 33) [-]
F_u_1 = gearbox.forces.F_u_1;                       % Circumferential force on wheel 1 [N]
F_rad_1 = gearbox.forces.F_rad_1;                   % Radial force on wheel 1 [N]
F_ax_1 = gearbox.forces.F_ax_1;                     % Axial force on wheel 1 [N]

% Initialization of bearing matrices for vectorial calculations
% Variation over the rows means variation of bearing C, variation over the columns varies bearing D
m = length(d_ctlg);
b_vert = repmat(b_ctlg,1,m);            % m x m matrix where the width of bearing C is varied over the rows
b_hor = b_vert';                        % m x m matrix where the width of bearing D is varied over the columns
% m x m matrices where the static and dynamic load ratings of the bearings are varied over the rows
C_stat_mat = repmat(C_stat_ctlg,1,m);   
C_dyn_mat = repmat(C_dyn_ctlg,1,m);

%% 3) Calculation of bearing loads and shaft design:
%% 3.1) Calculation of pressure angle and gearing forces:
% Calculation of the operating pressure angle in rad (Kirchner, p. 189)
alpha_wt = acos((((z_3+z_4)*m_n_3)/(2*a_23))*(cos(alpha_t)/cos(beta_2)));

% Calculation of occuring gearing forces in N
F_u_2 = F_u_1;
F_ax_2 = F_ax_1;
F_rad_2 = F_rad_1;
F_u_3 = 2*(i_12*T_nom)/d_3;
F_ax_3 = F_u_3*tan(beta_2);
F_rad_3 = F_u_3*tan(alpha_wt);

%% 3.2) Calculation of bearing distances a, b and c for bearing force calculation:
a = b_vert/2+b_gap+b_1/2;
if (strcmpi(gearbox.Input.axles, 'parallel'))
    b = b_1/2+b_gap+b_3/2;
elseif (strcmpi(gearbox.Input.axles, 'coaxial'))
    b = b_1/2+b_gap+b_B+b_gap+35+b_gap+b_3/2;       % distance is larger for coaxial gearboxes
end
c = b_3/2+b_gap+b_hor/2;

%% 3.3) Calculation of bearing forces:
% Bearing reaction forces in N in Y-direction
F_Cy = 1./(a+b+c).*(F_rad_3*c-F_rad_2*(b+c)+F_ax_3*d_3/2+F_ax_2*d_2/2);
F_Dy = F_rad_3-F_rad_2-F_Cy;
% Bearing reaction forces in N in Z-direction
F_Cz = 1./(a+b+c).*(F_u_3*c-F_u_2*(b+c));
F_Dz = F_u_3-F_u_2-F_Cz;  
% Radial forces in N
F_radC = sqrt((F_Cz.^2)+(F_Cy.^2));
F_radD = sqrt((F_Dz.^2)+(F_Dy.^2));
        
% Distribution of axial forces on bearing with lower radial load (Zaehringer, p. 28/29)
F_axC = (F_radC<=F_radD).*(F_ax_3-F_ax_2);
F_axD = (F_radC>F_radD).*(F_ax_3-F_ax_2);
    
%% 3.4) Calculation of bearing loads (Stahl, starting p. 82)
%% Bearing C
% Definition of axial to radial load ratio e (Stahl, p.84)
val = f0_ctlg.*F_axC./C_stat_ctlg;
e = (val<0.5)*0.22+(val>=0.5 & val<0.9)*0.24+(val>=0.9 & val<1.6)*0.28...
    +(val>=1.6 & val<3)*0.32+(val>=3 & val<6)*0.36+(val>=6)*0.43;

% Determination of radial and axial factors (X and Y) of bearing (Stahl, p.84)
par = F_axC./F_radC;
X = (par<=e)+(par>e)*0.56;
Y = zeros(m)+(par>e & e==0.22)*2+(par>e & e==0.24)*1.8+(par>e & e==0.28)*1.6...
    +(par>e & e==0.32)*1.4+(par>e & e==0.36)*1.2+(par>e & e==0.43);
         
% Static load ratings (Stahl, p. 82/83)
P_stat_C = (0.6*F_radC+0.5*F_axC)*overload_factor;  % Equivalent static load rating in N (in relation to T_max)
C_stat_req_C = 2.1*P_stat_C;                        % Required static load rating in N, Safety factor according to Stahl, p. 83 and Zaehringer, p. 29
% Dynamic load ratings (Stahl, p. 84)
P_next_C = (X.*F_radC)+(Y.*F_axC);                  % Equivalent dynamic load rating in N (in relation to T-nom)
C_dyn_req_C = (L_10^(1/3))*P_next_C;                % Required equivalent dynamic load rating in N

% Check for static (cond_C1) and dynamic (cond_C2) load for bearing C
cond_C1 = (C_stat_mat>=C_stat_req_C);
cond_C2 = (C_dyn_mat>=C_dyn_req_C);
        
%% Bearing D:       
% Definition of axial to radial load ratio e (Stahl, p.84)
val = f0_ctlg'.*F_axD./C_stat_ctlg';
e = (val<0.5)*0.22+(val>=0.5 & val<0.9)*0.24+(val>=0.9 & val<1.6)*0.28...
    +(val>=1.6 & val<3)*0.32+(val>=3 & val<6)*0.36+(val>=6)*0.43;

% Determination of radial and axial factors (X and Y) of bearing (Stahl, p.84)
par = F_axD./F_radD;
X = (par<=e)+(par>e)*0.56;
Y = zeros(m)+(par>e & e==0.22)*2+(par>e & e==0.24)*1.8+(par>e & e==0.28)*1.6...
    +(par>e & e==0.32)*1.4+(par>e & e==0.36)*1.2+(par>e & e==0.43);
         
% Static load ratings (Stahl, p. 82/83)
P_stat_D = (0.6*F_radD+0.5*F_axD)*overload_factor;  % Equivalent static load rating in N (in relation to T_max)
C_stat_req_D = 2.1*P_stat_D;                        % Required static load rating in N, Safety factor according to Stahl, p. 83 and Zaehringer, p. 29
% Dynamic load ratings (Stahl, p. 84)
P_next_D = (X.*F_radD)+(Y.*F_axD);                  % Equivalent dynamic load rating in N (in relation to T_nom)
C_dyn_req_D = (L_10^(1/3))*P_next_D;                % Required equivalent dynamic load rating in N

% Check for static (cond_D1) and dynamic (cond_D2) load for bearing D
cond_D1 = (C_stat_mat'>=C_stat_req_D);
cond_D2 = (C_dyn_mat'>=C_dyn_req_D);
        
%% 3.5) Calculation of shaft:
% Calculation of shaft safety and inner diameter of hollow shaft
gearbox = calc_d_sh(shaft, gearbox, d_ctlg);
sh_mat = gearbox.shafts.shaft_2;

% Check if shaft endures torsional load
S_sh_min = min(repmat(sh_mat(:,2),1,m),repmat(sh_mat(:,2)',m,1));
cond_s = (S_sh_min>S_Dt_min);
        
%% 3.6) Check if all conditions are met for bearing combinations and select optimum
% Bearing and shaft conditions are combined
all_cond = cond_C1.*cond_C2.*cond_D1.*cond_D2.*cond_s;

% Find best bearings combination
[poss_C, poss_D] = find(all_cond);
poss = sum([poss_C,poss_D],2);
[~,idx] = min(poss);

% Print error message if no bearings are found
if isempty(poss)
    disp('Error! No bearing combination was found for shaft 2!')
    gearbox.error.ratio_C_dyn_C = C_dyn_mat./C_dyn_req_C;
    gearbox.error.ratio_C_dyn_D = C_dyn_mat'./C_dyn_req_D;
end

%% 3.7) Calculation of the bearing distances for calc_factors
% Distances a and c on shaft 2 for chosen bearing combination in mm
a = a(poss_C(idx),1);
c = c(1,poss_D(idx));
% Distance of the bearing points on shaft 2 in mm
l_2 = a+b+c;
% Distances of the force application points and the middle of the bearings for wheel 2 and 3 in mm
s_2 = (-a+b+c)/2;
s_3 = (a+b-c)/2;

%% 5) Output assignment:
gearbox.gears_34.alpha_wt_2 = alpha_wt;                 % working transverse pressure angle in rad
gearbox.results.a_23 = a_23;                            % distance between shafts 3&4 in mm
gearbox.bearings_2.s_2 = s_2;                           % distance between middle of the shaft and wheel 2
gearbox.bearings_2.s_3 = s_3;                           % distance between middle of the shaft and wheel 3
gearbox.bearings_2.l_2 = l_2;                           % bearing distance on shaft 2 in mm
gearbox.bearings_2.a = a;                               % distance between bearing point C and wheel 2 in mm
gearbox.bearings_2.b = b;                               % distance between wheel 2 and 3 in mm
gearbox.bearings_2.c = c;                               % distance between wheel 3 and bearing point D in mm
gearbox.bearings_2.d_sh_C = d_ctlg(poss_C(idx));        % inner diameter of bearing C in mm
gearbox.bearings_2.d_sh_D = d_ctlg(poss_D(idx));        % inner diameter of bearing D in mm
gearbox.bearings_2.m_C = m_ctlg(poss_C(idx));           % mass of bearing C in kg
gearbox.bearings_2.m_D = m_ctlg(poss_D(idx));           % mass of bearing D in kg
gearbox.bearings_2.d_A_C = d_A_ctlg(poss_C(idx));       % outer diameter of bearing C in mm
gearbox.bearings_2.d_A_D = d_A_ctlg(poss_D(idx));       % outer diameter of bearing D in mm
gearbox.bearings_2.b_C = b_ctlg(poss_C(idx));           % width of bearing C in mm
gearbox.bearings_2.b_D = b_ctlg(poss_D(idx));           % width of bearing D in mm
gearbox.bearings_2.d_1_C = d_1_ctlg(poss_C(idx));       % outer diameter of inner bearing ring of bearing C in mm
gearbox.bearings_2.d_1_D = d_1_ctlg(poss_D(idx));       % outer diameter of inner bearing ring of bearing D in mm
gearbox.bearings_2.ID_C = name_ctlg(poss_C(idx));       % Name of bearing C from bearing catalog
gearbox.bearings_2.ID_D = name_ctlg(poss_D(idx));       % Name of bearing D from bearing catalog
gearbox.forces.F_u_2 = F_u_2;                           % Circumferential force of gear 2 in N
gearbox.forces.F_ax_2 = F_ax_2;                         % Axial force of shaft 2 in N
gearbox.forces.F_rad_2 = F_rad_2;                       % Radial force of shaft 2 in N
gearbox.forces.F_u_3 = F_u_3;                           % Circumferential force of gear 3 in N
gearbox.forces.F_ax_3 = F_ax_3;                         % Axial force of shaft 3 in N
gearbox.forces.F_rad_3 = F_rad_3;                       % Radial force of shaft 3 in N
gearbox.shafts.d_sh_2 = sh_mat(min(poss_C(idx),poss_D(idx)),1);     % Critical diameter of shaft 2 in mm
gearbox.shafts.S_dt_2 = sh_mat(min(poss_C(idx),poss_D(idx)),2);     % Safety factor against torsional fatigue of shaft 2
gearbox.shafts.d_inn_2 = sh_mat(min(poss_C(idx),poss_D(idx)),3);    % Inner diameter of shaft 2 in mm

end
