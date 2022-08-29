function gearbox = set_diff_plan (gearbox)
%% 1) Desription:
% This function computes a bevel gear differential for gearbox with angular
% ball bearings.

%% 1.1) Authors
% Peter Koehler
% Date: 24.08.2021

%% 1.2) Sources:
% [1] Semesterarbeit Koehler: "Semi-physikalische Modellierung von Antriebsstrangkomponenten fuer Elektrofahrzeuge", Institute of Automotive Technology, TUM, Munich, 2020
% [2] Bachelor Thesis Zaehringer: "Erstellung eines analytischen Ersatzmodells fuer Getriebe von Elektrofahrzeugen", Institute of Automotive Technology, TUM, Munich, 2018
% [3] Schaeffler: "Waelzlager", Bearing Catalogue, 2019
% [4] Master Thesis Koehler: "Semi-physikalische Modellierung von Antriebsstrangkomponenten fuer Elektrofahrzeuge", Institute of Automotive Technology, TUM, Munich, 2021
% [5] Stahl: "Formelsammlung zur Vorlesung Maschinenelemente", Wintersemester 2015/16

%% 2) Initialization of required values:
b_d_bevel = gearbox.GearingConst.b_d_bevel;             % Empirical ratio of bevel gear width and outer diameter (Koehler [1], p.41) [-]
d_bevel = gearbox.GearingConst.d_bevel;                 % Ratio between smaller and larger bevel gears of the diffferential (Koehler [1], p.47) [-]
T_max = gearbox.Input.T_max;                            % Maximum torque of el. motor [Nm]
t_diffcage = gearbox.ConstDim.t_diffcage;               % Thickness of differential housing [mm]
b_gap = gearbox.ConstDim.b_gap;                         % Gap between components on the shafts [mm]
i_1s = gearbox.results.i_1s;                            % Total transmission ratio of planetary gearbox [-]
b_1 = gearbox.gears.b_1;                                % Width of first stage gears [mm]
b_2 = gearbox.gears.b_2;                                % Width of second stage gears [mm]
d_s = gearbox.gears.d_s;                                % Diameter of the planet circle [mm]
d_ap2 = gearbox.gears.d_ap2;                            % Outer diameter of planet 2 [mm]
b_C = gearbox.bearings_2.b_C;                           % Width of bearing C [mm]
b_D = gearbox.bearings_2.b_D;                           % Width of bearing D [mm]
overload_factor = gearbox.Input.overload_factor;        % Overload factor of el. motor [-]
d_sh_3 = gearbox.shafts.d_sh_3;                         % Outer diameter of output shaft [mm]
L_10 = gearbox.CalcFactors.L_10/(gearbox.results.i_1s); % Lifetime factor regarding differential shaft speed [10^6 revolutions]
F_ax_p = gearbox.forces.F_ax_p;                         % Resulting axial force on each planet [N]
d_A_B = gearbox.bearings_1.d_A_B;                       % Outer diameter of bearing A [mm]
regr_d_bevel = gearbox.Regression.d_bevel.eq;           % Regression formula for the diameter of the larger differential gears in mm ([1], p. 46)

% Loading of the bearing catalogue
ab = gearbox.BearCtlg.ang_ball;         % Loading of Schaeffler angular ball bearing catalogue (starting on p. 272)
d_ctlg = ab.d;                          % List of inner bearing diameters [mm]
d_A_ctlg = ab.d_A;                      % List of outer bearing diameters [mm]
b_ctlg = ab.b;                          % List of bearing widths [mm]
a_ctlg = ab.a;                          % List of bearing point distances [mm]
C_dyn_ctlg = ab.C_dyn;                  % List of dynamic load ratings [N] 
m_ctlg = ab.m;                          % List of bearing masses [kg]
d_1_ctlg = ab.d_1;                      % List of diameters of inner bearing ring [mm]
C_stat_ctlg = ab.C_stat;                % List of static load ratings [N]
name_ctlg = ab.Name;                    % List of bearing codes

% Initialization of bearing matrices for vectorial calculations
% Variation over the rows means variation of bearing E, variation over the columns varies bearing F
m = length(d_ctlg);
d_vert = repmat(d_ctlg,1,m);            % m x m matrix where the inner diameter of bearing E is varied over the rows
d_hor = d_vert';                        % m x m matrix where the inner diameter of bearing F is varied over the columns
% m x m matrices where the static and dynamic load ratings of the bearings are varied in over the rows
C_stat_mat = repmat(C_stat_ctlg,1,m);
C_dyn_mat = repmat(C_dyn_ctlg,1,m);

%% 3) Calculation of differential dimensions and gear data
% Calculation of differential data for one EM on the axle
if (gearbox.Input.num_EM==1)
    % Empirical diameters of differential bevel gears in mm (Koehler [1], pp .46)
    d_bev_l = regr_d_bevel(T_max*i_1s);         % Empirical diameter of larger bevel gears in mm
    d_bev_s = d_bevel*d_bev_l;                  % Empirical diameter of smaller bevel gears in mm

    % Diameter of differential cage in mm (Koehler [4], p.59)
    d_diffcage = d_bev_l+10+2*t_diffcage;

    % Determination of the empirical width of bevel gearing in mm (Koehler [1], p.47)
    b_bev = b_d_bevel*d_bev_l;

    % Determination of the differential cage width in mm
    b_diffcage = d_bev_s+2*t_diffcage;
else % substitute values for plotting of planetary gearboxes without differential
    b_diffcage = gearbox.ConstDim.b_gap;
end

%% 4) Selection of bearings according to input load and desired lifetime
% Definition of initial inner bearing diameters in mm
if (gearbox.Input.num_EM==1)
    d_sh_E_min = d_sh_3+4+10;
elseif (gearbox.Input.num_EM==2)
    d_sh_E_min = d_sh_3;
end
d_sh_F_min = d_A_B+10;

% Bearings nominally experience no radial load (central shafts in planetary gears)!
% Definition of axial Forces on planet carrier
F_ax_s = 3*F_ax_p;
F_axF = F_ax_s;

%% 4.1) Bearing E: Static load ratings in N (Stahl, pp. 82)
P_stat_E = 0;                               % Equivalent static load rating in N (in relation to T_max; only axial Forces)
C_stat_req_E = 2.1*P_stat_E;                % Required static load rating in N, Safety factor according to Stahl, p. 83
        
% Determination of equivalent dynamic load ratings in N (Stahl, p. 84) (in relation to T_nom)
P_dyn_E = 0;
C_dyn_req_E = (L_10^(1/3))*P_dyn_E;         % Required equivalent dynamic load rating in N

% Check for static (cond_E1) and dynamic (cond_E2) load for bearing E
cond_E1 = (C_stat_mat>=C_stat_req_E);
cond_E2 = (C_dyn_mat>=C_dyn_req_E);

% Check for minimum bearing E diameter 
cond_s1 = (d_vert>=d_sh_E_min);

%% 4.2) Bearing F: Static load ratings in N (Stahl, pp. 82)
P_stat_F = 0.26*F_axF*overload_factor;      % No nominal radial or axial forces on bearing F 
C_stat_req_F = 2.1*P_stat_F;                % Required static load rating in N, Safety factor according to Stahl, p. 83
        
% Determination of equivalent dynamic load ratings in N (Stahl, p. 84) (in relation to T_nom)
P_dyn_F = 0.57*F_axF;                       % No nominal radial forces on bearing F
C_dyn_req_F = (L_10^(1/3))*P_dyn_F;         % Required equivalent dynamic load rating in N 

% Check for static (cond_F1) and dynamic (cond_F2) load for bearing F
cond_F1 = (C_stat_mat>=C_stat_req_F);
cond_F2 = (C_dyn_mat>=C_dyn_req_F);

% Check for minimum bearing F diameter 
cond_s2 = (d_hor>=d_sh_F_min);

%% 3.6) Check if all conditions are met for bearing combinations and selection of optimum
% Bearing and shaft conditions are combined
all_cond = cond_E1.*cond_E2.*cond_F1.*cond_F2.*cond_s1.*cond_s2;

% Find best bearing combination
[poss_E, poss_F] = find(all_cond);
poss = sum([poss_E,poss_F],2);
[~,idx] = min(poss);

% Print error message if no bearings are found
if isempty(poss)
    disp('Error! No bearing combination was found for planet carrier!')
    gearbox.error.ratio_C_dyn_E = C_dyn_mat./C_dyn_req_E;
    gearbox.error.ratio_C_dyn_F = C_dyn_mat'./C_dyn_req_F;
end

%% 3.7) Calculation of the bearing distances for calc_factors
% Distances a and b on shaft 1 for chosen bearing combination in mm
if (gearbox.Input.num_EM==1)
    % Integration of differential in planet carrier if planet circle is large enough
    if (d_s>d_diffcage+4+d_ap2)
        a = (b_ctlg(poss_E(idx),1)-a_ctlg(poss_E(idx),1))+b_diffcage+b_gap/2;
    else
        a = (b_ctlg(poss_E(idx),1)-a_ctlg(poss_E(idx),1))+b_diffcage+5+b_D+b_gap+b_2+b_gap/2;
    end
elseif (gearbox.Input.num_EM==2)
    a = (b_ctlg(poss_E(idx),1)-a_ctlg(poss_E(idx),1))+5+b_D+b_gap+b_2+b_gap/2;
end
b = (b_ctlg(poss_F(idx))-a_ctlg(poss_F(idx)))+5+b_C+b_gap+b_1+b_gap/2;

%% 4) Output Assignment:
gearbox.diff.b_diffcage = b_diffcage;               % Width of the differential cage in mm
gearbox.bearings_3.a = a;                           % distance between bearing point E and the middle of the planets in mm
gearbox.bearings_3.b = b;                           % distance between the middle of the planets and bearing point F in mm
gearbox.bearings_3.d_sh_E = d_ctlg(poss_E(idx));    % inner diameter of bearing E in mm
gearbox.bearings_3.d_sh_F = d_ctlg(poss_F(idx));    % inner diametet of bearing F in mm
gearbox.bearings_3.m_E = m_ctlg(poss_E(idx));       % mass of bearing E in kg
gearbox.bearings_3.m_F = m_ctlg(poss_F(idx));       % mass of bearing F in kg
gearbox.bearings_3.d_A_E = d_A_ctlg(poss_E(idx));   % outer diameter of bearing E in mm
gearbox.bearings_3.d_A_F = d_A_ctlg(poss_F(idx));   % outer diameter of bearing F in mm
gearbox.bearings_3.b_E = b_ctlg(poss_E(idx));       % width of bearing E in mm
gearbox.bearings_3.b_F = b_ctlg(poss_F(idx));       % width of bearing F in mm
gearbox.bearings_3.d_1_E = d_1_ctlg(poss_E(idx));   % outer diameter of inner bearing ring of bearing E in mm
gearbox.bearings_3.d_1_F = d_1_ctlg(poss_F(idx));   % outer diameter of inner bearing ring of bearing F in mm
gearbox.bearings_3.ID_E = name_ctlg(poss_E(idx));   % name of bearing E from bearing catalog
gearbox.bearings_3.ID_F = name_ctlg(poss_F(idx));   % name of bearing F from bearing catalog

if (gearbox.Input.num_EM==1)
    gearbox.diff.d_diffcage = d_diffcage;           % Diameter of differential cage in mm
    gearbox.diff.d_bev_l = d_bev_l;                 % Diameter of larger bevel gears in mm
    gearbox.diff.d_bev_s = d_bev_s;                 % Diameter of smaller bevel gears in mm
    gearbox.diff.b_bev = b_bev;                     % Width of bevel gearing in mm
else
    gearbox.diff.d_diffcage = gearbox.bearings_3.d_1_E; % Substitute diameter for plotfunction in mm
end
