function gearbox = set_bearings_diff(gearbox, d_ctlg, d_A_ctlg, b_ctlg, C_dyn_ctlg, m_ctlg, a_ctlg, e_ctlg, Y_ctlg, d_1_ctlg, C_stat_ctlg, Y_0_ctlg, name_ctlg)
%% 1) Description:
% This function selects taper roller bearings for the differential of 
% the gearbox from the Schaeffler bearing catalog.
% SHAFT 3:
% Fixed bearing combination with two taper roller bearings in X arrangement
%         _                         _
%       _| |                       | |
%   -E-|_|4|--F-      OR:      -E--|4|--F-  (for two EMs on vehicle axle)
%        |_|                       |_|
%
%    |-a--|-b-|                 |-a-|-b-|   (bearing distances)
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

%% 2) Initialization of required values:
overload_factor = gearbox.Input.overload_factor;        % Overload factor of el. motor [-]
d_sh_3 = gearbox.shafts.d_sh_3;                         % Outer diameter of output shaft [mm]
b_1 = gearbox.gears_12.b_1;                             % Width of wheels 1&2 [mm]
b_3 = gearbox.gears_34.b_3;                             % Width of wheels 3&4 [mm]
d_4 = gearbox.gears_34.d_4;                             % Pitch diameter of wheel 4 [mm]
i_12 = gearbox.results.i_12;                            % Transmission ratio of first stage
i_34 = gearbox.results.i_34;                            % Transmission ratio of second stage
L_10 = gearbox.CalcFactors.L_10/(i_12*i_34);            % Lifetime factor regarding differential shaft speed [10^6 revolutions]
b_gap = gearbox.ConstDim.b_gap;                         % Gap between components on the shafts [mm]
F_u_4 = gearbox.forces.F_u_3;                           % Circumferential force of gear 3&4 [N]
F_ax_4 = gearbox.forces.F_ax_3;                         % Axial force of gear 3&4 [N]
F_rad_4 = gearbox.forces.F_rad_3;                       % Radial force of gear 3&4 [N]
if (gearbox.Input.num_EM==1)
    b_diffcage = gearbox.diff.b_diffcage;               % Width of the differential cage [mm]
    diff_orientation = gearbox.diff.diff_orientation;   % Orientation of the differential
    d_sh_min = d_sh_3+4+10;                             % Minimum inner diameter of bearings [mm]
else
    d_sh_min = d_sh_3;
end

% Initialization of bearing matrices for vectorial calculations
% Variation over the rows means variation of bearing E, variation over the columns varies bearing F
m = length(d_ctlg);
b_vert = repmat(b_ctlg,1,m);            % m x m matrix where the width of bearing E is varied over the rows
b_hor = b_vert';                        % m x m matrix where the width of bearing F is varied over the columns
a_vert = repmat(a_ctlg,1,m);            % m x m matrix where the bearing point distance of bearing E is varied over the rows
a_hor = a_vert';                        % m x m matrix where the bearing point distance of bearing F is varied over the columns
e_mat = repmat(e_ctlg,1,m);             % m x m matrix where the axial to radial load ratio of the bearings is varied over the rows
% m x m matrices where the static and dynamic load ratings of the bearings are varied in over the rows
C_stat_mat = repmat(C_stat_ctlg,1,m);
C_dyn_mat = repmat(C_dyn_ctlg,1,m);
d_mat = repmat(d_ctlg,1,m);             % m x m matrix where the inner diameter of the bearings is varied over the rows

%% 3) Calculation of bearing:
% Definition of initial inner bearing diameter 
if (gearbox.Input.num_EM==1)
    a = (b_vert-a_vert)+b_diffcage+b_3/2;
    if (strcmpi(gearbox.Input.axles,'parallel') && strcmpi(diff_orientation,'out'))        
        b = b_3/2+b_gap+b_1+b_gap+(b_hor-a_hor);        % extra shaft segment
    else
        b = b_3/2+b_gap+(b_hor-a_hor);
    end
elseif (gearbox.Input.num_EM==2)
    a = (b_vert-a_vert)+b_gap+b_3/2;
    b = b_3/2+b_gap+(b_hor-a_hor);
end

%% 3.1) Calculation of bearing forces:
% Bearing reaction forces in N in Y-direction
F_Ey = 1./(a+b).*(F_ax_4*d_4/2-F_rad_4*b);
F_Fy = -F_rad_4-F_Ey;
% Bearing reaction forces in N in Z-direction
F_Ez = 1./(a+b).*(-F_u_4*b);
F_Fz = -F_Ez-F_u_4;
% Radial bearing forces in N
F_radE = sqrt((F_Ez.^2)+(F_Ey.^2));
F_radF = sqrt((F_Fz.^2)+(F_Fy.^2));

%% 3.2) Calculation of bearing loads (Stahl, starting p. 82)
%% Bearing E
% Determination of the distribution of axial forces (Stahl, starting p. 82)
val_E = F_radE./Y_ctlg;
val_F = F_radF./Y_ctlg';

F_axE = (val_E<val_F & (abs(F_ax_4)<=abs(0.5*val_F-val_E))).*(0.5*val_F-F_ax_4);
F_axF = (val_E>=val_F).*(F_ax_4+(0.5*val_E))...
    +(val_E<val_F & (abs(F_ax_4)>abs(0.5*val_F-val_E))).*(F_ax_4+(0.5*val_E));

% Static load ratings in N (Stahl, p. 82/83)
P_stat_E = (0.5*F_radE+Y_0_ctlg.*F_axE)*overload_factor;    % Equivalent static load rating in N (in relation to T_max)
C_stat_req_E = 4.1*P_stat_E;                                % Required static load rating in N, Safety factor according to Stahl, p. 83
        
% Determination of equivalent dynamic load ratings in N (Stahl, p. 84) (in relation to T_nom)
par_E = F_axE./F_radE;
P_next_E = (par_E<=e_mat).*(F_radE+(1.12*Y_ctlg.*F_axE))...
    +(par_E>e_mat).*((0.67*F_radE)+(1.68*Y_ctlg.*F_axE));
C_dyn_req_E = (L_10^(3/10))*P_next_E;                       % Required equivalent dynamic load rating in N

% Check for static (cond_E1) and dynamic (cond_E2) load for bearing E
cond_E1 = (C_stat_mat>=C_stat_req_E);
cond_E2 = (C_dyn_mat>=C_dyn_req_E);

%% Bearing F: 
% Static load ratings in N (Stahl, p. 82/83)
P_stat_F = (0.5*F_radF+Y_0_ctlg'.*F_axF)*overload_factor;   % Equivalent static load rating in N (in relation to T_max)
C_stat_req_F = 4.1*P_stat_F;                                % Required static load rating in N, Safety factor according to Stahl, p. 83
        
% Determination of equivalent dynamic load ratings in N (Stahl, p. 84) (in relation to T_nom)
par_F = F_axF./F_radF;
P_next_F = (par_F<=e_mat').*(F_radF+(1.12*Y_ctlg'.*F_axF))...
    +(par_F>e_mat').*((0.67*F_radF)+(1.68*Y_ctlg'.*F_axF));
C_dyn_req_F = (L_10^(3/10))*P_next_F;                       % Required equivalent dynamic load rating in N 

% Check for static (cond_F1) and dynamic (cond_F2) load for bearing F
cond_F1 = (C_stat_mat'>=C_stat_req_F);
cond_F2 = (C_dyn_mat'>=C_dyn_req_F);

%% 3.3) Check if minimum bearing diameter is larger than the minimum shaft diameter:
% Check if inner bearing diameter is larger than the minimum possible diameter
d_bear_min = min(d_mat,d_mat');
cond_s = d_bear_min>=d_sh_min;

%% 3.4) Check if all conditions are met for bearing combinations and selection of optimum
% Combination of bearing and shaft conditions
all_cond = cond_E1.*cond_E2.*cond_F1.*cond_F2.*cond_s;

% Find minimal bearings that met all conditions
[poss_E, poss_F] = find(all_cond);
poss = sum([poss_E,poss_F],2);
[~,idx] = min(poss);

% Print error message if no bearings are found
if isempty(poss)
    disp('Error! No bearing combination was found for shaft 3!')
    gearbox.error.ratio_C_dyn_E = C_dyn_mat./C_dyn_req_E;
    gearbox.error.ratio_C_dyn_F = C_dyn_mat'./C_dyn_req_F;
end

%% 3.7) Calculation of the bearing distances for calc_factors
% Distances a and b on shaft 3 for chosen bearing combination in mm
a = a(poss_E(idx),1);
b = b(1,poss_F(idx));
% Distance of the bearing points on shaft 33 in mm
l_3 = a+b;
% Distances of the force application point and the middle of the bearings for wheel 4 in mm
s_4 = (a-b)/2;

%% 4) Output assignment:
gearbox.bearings_3.l_3 = l_3;                           % bearing distance in mm
gearbox.bearings_3.s_4 = s_4;                           % distance between middle of the shaft and wheel 4 in mm
gearbox.bearings_3.a = a;                               % distance between bearing point E and wheel 4 in mm
gearbox.bearings_3.b = b;                               % distance between wheel 4 and bearing point F in mm
gearbox.bearings_3.d_sh_E = d_ctlg(poss_E(idx));        % inner diameter of bearing E in mm
gearbox.bearings_3.d_sh_F = d_ctlg(poss_F(idx));        % inner diametet of bearing F in mm
gearbox.bearings_3.m_E = m_ctlg(poss_E(idx));           % mass of bearing E in kg
gearbox.bearings_3.m_F = m_ctlg(poss_F(idx));           % mass of bearing F in kg
gearbox.bearings_3.d_A_E = d_A_ctlg(poss_E(idx));       % outer diameter of bearing E in mm
gearbox.bearings_3.d_A_F = d_A_ctlg(poss_F(idx));    	% outer diameter of bearing F in mm
gearbox.bearings_3.b_E = b_ctlg(poss_E(idx));        	% width of bearing E in mm
gearbox.bearings_3.b_F = b_ctlg(poss_F(idx));        	% width of bearing F in mm
gearbox.bearings_3.d_1_E = d_1_ctlg(poss_E(idx));    	% outer diameter of inner bearing ring of bearing E in mm
gearbox.bearings_3.d_1_F = d_1_ctlg(poss_F(idx));    	% outer diameter of inner bearing ring of bearing F in mm
gearbox.bearings_3.ID_E = name_ctlg(poss_E(idx));    	% name of bearing E from bearing catalog
gearbox.bearings_3.ID_F = name_ctlg(poss_F(idx));    	% name of bearing F from bearing catalog

end
