function gearbox = set_bearings_planet(shaft, gearbox, d_ctlg, d_A_ctlg, b_ctlg, C_dyn_ctlg, m_ctlg, a_ctlg, e_ctlg, Y_ctlg, C_stat_ctlg, Y_0_ctlg, d_1_ctlg, name_ctlg)
%% 1) Description:
% This function selects taper roller bearings for the second shaft of 
% the gearbox from the Schaeffler bearing catalog.
% SHAFT 2:
% Fixed bearing combination in X arrangement with two taper roller bearings
%        _
%       | |  _
%   -C--|1|-|2|--D-
%       |_|
%
%    |-a-|-b-|-c-|
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
overload_factor = gearbox.Input.overload_factor;    % Overload factor of el. motor [-]
m_n_1 = gearbox.gears.m_n_1;                        % Normal module of first stage [mm]
m_n_2 = gearbox.gears.m_n_2;                        % Normal module of second stage [mm]
alpha_t_1 = gearbox.gears.alpha_t_1;                % Pressure angle in transverse section of the first stage [rad]
alpha_t_2 = gearbox.gears.alpha_t_2;                % Pressure angle in transverse section of the second stage [rad]
beta_1 = gearbox.gears.beta_1;                      % Helix angle of first stage [rad]
beta_2 = gearbox.gears.beta_2;                      % Helix angle of second stage [rad]
b_gap = gearbox.ConstDim.b_gap;                     % Gap between components on the shafts [mm]
z_1 = gearbox.gears.z_1;                            % Number of teeth of sun gear
z_p1 = gearbox.gears.z_p1;                          % Number of teeth of planet 1
z_p2 = gearbox.gears.z_p2;                          % Number of teeth of planet 2
z_2 = gearbox.gears.z_2;                            % Number of teeth of ring gear (negative)
b_1 = gearbox.gears.b_1;                            % Width of sun gear [mm]
b_2 = gearbox.gears.b_2;                            % Width of planet 2 [mm]
d_1 = gearbox.gears.d_1;                            % Pitch diameter of sun gear [mm]
d_p1 = gearbox.gears.d_p1;                          % Pitch diameter of planet 1 [mm]
d_p2 = gearbox.gears.d_p2;                          % Pitch diameter of planet 2 [mm]
d_2 = gearbox.gears.d_2;                            % Pitch diameter of ring gear [mm]
i_p_rel = gearbox.results.i_p_rel;                  % Relative transmission ratio of the planets [-]
L_10 = gearbox.CalcFactors.L_10/i_p_rel;            % Lifetime factor regarding speed of planets [10^6 revolutions]
S_Dt_min = gearbox.MinSafety.S_Dt_min;              % Minimum safety factor against torsional fatigue fracture (Stahl, p. 33) [-]
F_u_1 = gearbox.forces.F_u_1;                       % Circumferential force on sun gear [N]
d_sh_p = gearbox.ConstDim.d_planbolt;               % Diameter of planet bolt [mm]

% Initialization of bearing matrices for vectorial calculations
% Variation over the rows means variation of bearing C, variation over the columns varies bearing D
m = length(d_ctlg);
b_vert = repmat(b_ctlg,1,m);            % m x m matrix where the width of bearing C is varied over the rows
b_hor = b_vert';                        % m x m matrix where the width of bearing D is varied over the columns
a_vert = repmat(a_ctlg,1,m);            % m x m matrix where the bearing point distance of bearing C is varied over the rows
a_hor = a_vert';                        % m x m matrix where the bearing point distance of bearing D is varied over the columns
e_mat = repmat(e_ctlg,1,m);             % m x m matrix where the axial to radial load ratio of the bearings is varied over the rows
% m x m matrices where the static and dynamic load ratings of the bearings are varied over the rows
C_stat_mat = repmat(C_stat_ctlg,1,m);
C_dyn_mat = repmat(C_dyn_ctlg,1,m);

%% 3) Calculation of bearing loads and shaft design:
%% 3.1) Calculation of pressure angle and gearing forces:
% Calculation of the operating pressure angle in rad according (Kirchner, p. 189)
alpha_wt_1 = acos((((z_1+z_p1)*m_n_1)/(d_1+d_p1))*(cos(alpha_t_1)/cos(beta_1)));
alpha_wt_2 = acos((((z_p2+z_2)*m_n_2)/(d_p2-d_2))*(cos(alpha_t_2)/cos(beta_2)));

% Calculation of occuring gearing forces in N according (Stahl, p. 2)
F_u_p1 = 1/3*F_u_1;                     % Circumferential force of planet gear 1
F_ax_p1 = F_u_p1*tan(beta_1);           % Axial force of planet gear 1
F_rad_p1 = F_u_p1*tan(alpha_wt_1);      % Radial force of planet gear 1
F_u_p2 = F_u_p1*d_p1/d_p2;              % Circumferential force of planet gear 2
F_ax_p2 = F_u_p2*tan(beta_2);           % Axial force of planet gear 2
F_rad_p2 = F_u_p2*tan(alpha_wt_2);      % Radial force of planet gear 2

% Calculation of the circumferential and axial forces on the planet as a whole in N
F_u_sp = F_u_p2-F_u_p1;                 % Circumferential force on the planet bolt
F_ax_p = F_ax_p2-F_ax_p1;               % Axial force on the planet bolt

%% 3.2) Calculation of bearing distances a, b and c:
% Auxiliary parameters of distances on planet shaft
a = b_vert-a_vert+b_gap+b_1/2;
b = (b_1+b_2)/2+b_gap;
c = b_hor-a_hor+b_gap+b_2/2;

%% 3.3) Calculation of bearing forces:
% Bearing reaction forces in N in Y-direction
F_Cy = (1./(a+b+c)).*(F_rad_p2*c-F_rad_p1*(b+c)+F_ax_p1*d_p1/2+F_ax_p2*d_p2/2);
F_Dy = F_rad_p2-F_rad_p1-F_Cy;
% Bearing reaction forces in N in Z-direction
F_Cz = (1./(a+b+c)).*(F_u_p2*c-F_u_p1*(b+c)-F_u_sp*(b/2+c));
F_Dz = -F_Cz-F_u_p1-F_u_sp+F_u_p2; 
% Radial bearing forces in N
F_radC = sqrt((F_Cy.^2)+(F_Cz.^2));
F_radD = sqrt((F_Dy.^2)+(F_Dz.^2));    
        
% Determination of the distribution of axial forces (Stahl, pp. 82)
val_C = F_radC./Y_ctlg;
val_D = F_radD./Y_ctlg';

% Axial bearing forces in N
F_axC = (val_C<val_D & (abs(F_ax_p)<=abs(0.5*val_D-val_C))).*(0.5*val_D-F_ax_p);
F_axD = (val_C>=val_D).*(F_ax_p+(0.5*val_C))...
    +(val_C<val_D & (abs(F_ax_p)>abs(0.5*val_D-val_C))).*(F_ax_p+(0.5*val_C));
    
%% 3.4) Calculation of bearing loads (Stahl, pp. 82)
%% Bearing C
% Static load ratings (Stahl, pp. 82)
P_stat_C = (0.5*F_radC+Y_0_ctlg.*F_axC)*overload_factor;    % Equivalent static load rating in N (in relation to T_max)
C_stat_req_C = 4.1*P_stat_C;                                % Required static load rating in N, Safety factor according to Stahl, p. 83 and Zaehringer, p. 29
        
% Determination of equivalent dynamic load ratings in N (Stahl, p. 84) (in relation to T_nom)
par_C = F_axC./F_radC;
P_next_C = (par_C<=e_mat).*(F_radC+(1.12*Y_ctlg.*F_axC))...
    +(par_C>e_mat).*((0.67*F_radC)+(1.68*Y_ctlg.*F_axC));
C_dyn_req_C = (L_10^(3/10))*P_next_C;                       % Required equivalent dynamic load rating in N 

% Check for static (cond_C1) and dynamic (cond_C2) load for bearing C
cond_C1 = (C_stat_mat>=C_stat_req_C);
cond_C2 = (C_dyn_mat>=C_dyn_req_C);

%% Bearing D
% Static load ratings (Schaeffler, p. 268)
P_stat_D = (0.5*F_radD+Y_0_ctlg'.*F_axD)*overload_factor;   % Equivalent static load rating in N (in relation to T_max)
C_stat_req_D = 4.1*P_stat_D;                                % Required static load rating in N, Safety factor according to Stahl, p. 83 and Zaehringer, p. 29
        
% Determination of equivalent dynamic load ratings in N (Stahl, p. 84) (in relation to T_nom)
par_D = F_axD./F_radD;
P_next_D = (par_D<=e_mat').*(F_radD+(1.12*Y_ctlg'.*F_axD))...
    +(par_D>e_mat').*((0.67*F_radD)+(1.68*Y_ctlg'.*F_axD));
C_dyn_req_D = (L_10^(3/10))*P_next_D;                       % Required equivalent dynamic load rating in N

% Check for static (cond_D1) and dynamic (cond_D2) load for bearing D
cond_D1 = (C_stat_mat'>=C_stat_req_D);
cond_D2 = (C_dyn_mat'>=C_dyn_req_D);

%% 3.5) Calculation of shaft:
% Calculation of shaft safety and inner diameter of hollow shaft
gearbox = calc_d_sh_plan(shaft, gearbox, d_ctlg);
sh_mat = gearbox.shafts.shaft_p;

% Check if shaft endures torsional load
S_sh_min = min(repmat(sh_mat(:,2),1,m),repmat(sh_mat(:,2)',m,1));
cond_s1 = (S_sh_min>S_Dt_min);

% Check if minimum bearing diameter is larger than the minimum shaft diameter
d_bear_min = min(repmat(d_ctlg,1,m),repmat(d_ctlg',m,1));
cond_s2 = (d_bear_min>=d_sh_p);

%% 3.6) Check if all conditions are met for bearing combinations and select optimum
% Bearing and shaft conditions are combined
all_cond = cond_C1.*cond_C2.*cond_D1.*cond_D2.*cond_s1.*cond_s2;

% Find best bearings combination
[poss_C, poss_D] = find(all_cond);
poss = sum([poss_C,poss_D],2);
[~,idx] = min(poss);

% Print error message if no bearings are found
if isempty(poss)
    disp('Error! No bearing combination was found for planets!')
    gearbox.error.ratio_C_dyn_C = C_dyn_mat./C_dyn_req_C;
    gearbox.error.ratio_C_dyn_D = C_dyn_mat'./C_dyn_req_D;
end

%% 3.7) Calculation of the bearing distances for calc_factors
% Distances a and c on planet shaft for chosen bearing combination in mm
a = a(poss_C(idx),1);
c = c(1,poss_D(idx));
% Distance of the bearing points on planet shaft in mm
l_2 = a+b+c;
% Distances of the force application points and the middle of the bearings for planet wheel 1 and 2 in mm
s_p1 = (-a+b+c)/2;
s_p2 = (a+b-c)/2;

%% 5) Output assignment:
gearbox.gears.alpha_wt_2 = alpha_wt_2;                  % working transverse pressure angle in rad
gearbox.bearings_2.s_p1 = s_p1;                         % distance between middle of the shaft and planet 1 in mm
gearbox.bearings_2.s_p2 = s_p2;                         % distance between middle of the shaft and planet 2 in mm
gearbox.bearings_2.l_2 = l_2;                           % bearing distance in mm
gearbox.bearings_2.a = a;                               % distance between bearing point C and planet 1 in mm
gearbox.bearings_2.b = b;                               % distance between the middles of planet 1 and 2 in mm
gearbox.bearings_2.c = c;                               % distance between planet 2 and bearing point D in mm
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
gearbox.forces.F_u_p1 = F_u_p1;                         % Circumferential force on planet 1 in N
gearbox.forces.F_ax_p1 = F_ax_p1;                       % Axial force of planet 1 in N
gearbox.forces.F_rad_p1 = F_rad_p1;                     % Radial force of planet 1 in N
gearbox.forces.F_u_p2 = F_u_p2;                         % Circumferential force of planet 2 in N
gearbox.forces.F_ax_p2 = F_ax_p2;                       % Axial force of planet 2 in N
gearbox.forces.F_rad_p2 = F_rad_p2;                     % Radial force of planet 2 in N
gearbox.forces.F_ax_p = F_ax_p;                         % Resulting axial force on every planet in N
gearbox.shafts.d_sh_p = sh_mat(min(poss_C(idx),poss_D(idx)),1);     % Critical diameter of planet bolts in mm
gearbox.shafts.S_dt_p = sh_mat(min(poss_C(idx),poss_D(idx)),2);     % Safety factor against torsional fatigue of planet bolts
gearbox.shafts.d_inn_p = sh_mat(min(poss_C(idx),poss_D(idx)),3);    % Inner diameter of planet bolts in mm

end
