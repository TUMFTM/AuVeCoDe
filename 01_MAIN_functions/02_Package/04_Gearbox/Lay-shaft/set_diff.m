function gearbox = set_diff(gearbox)
%% 1) Desription:
% This function computes a bevel gear differential for gearbox with taper roller bearings.

%% 1.1) Authors
% Peter Koehler
% Date: 24.08.2021

%% 1.2) Sources:
% [1] Semesterarbeit Koehler: "Semi-physikalische Modellierung von Antriebsstrangkomponenten fuer Elektrofahrzeuge", Institute of Automotive Technology, TUM, Munich, 2020
% [2] Bachelor Thesis Zaehringer: "Erstellung eines analytischen Ersatzmodells fuer Getriebe von Elektrofahrzeugen", Institute of Automotive Technology, TUM, Munich, 2018
% [3] Schaeffler: "Waelzlager", Bearing Catalogue, 2019
% [4] Master Thesis Koehler: "Semi-physikalische Modellierung von Antriebsstrangkomponenten fuer Elektrofahrzeuge", Institute of Automotive Technology, TUM, Munich, 2021

%% 2) Initialization of required values:
b_1 = gearbox.gears_12.b_1;             % Width of wheels 1&2 [mm]
b_3 = gearbox.gears_34.b_3;             % Width of wheels 3&4 [mm]
b_gap = gearbox.ConstDim.b_gap;         % Gap between components on the shafts (Zaehringer, p. 64) [mm]
if (gearbox.Input.num_EM==1 && strcmpi(gearbox.Input.axles,'parallel'))
    diff_orientation = gearbox.diff.diff_orientation;   % Orientation of the differential
end
% Loading of the bearing catalogue
tr = gearbox.BearCtlg.roller;           % Loading of Schaeffler taper roller bearing catalogue (starting on p. 590)
d_ctlg = tr.d;                          % List of inner bearing diameters [mm]
d_A_ctlg = tr.d_A;                      % List of outer bearing diameters [mm]
b_ctlg = tr.b;                          % List of bearing widths [mm]
C_dyn_ctlg = tr.C_dyn;                  % List of dynamic load ratings [N] 
m_ctlg = tr.m;                          % List of bearing masses [kg]
a_ctlg = tr.a;                          % List of bearing distances [mm]
e_ctlg = tr.e;                          % List of bearing factors e [-]
Y_ctlg = tr.Y;                          % List of bearing factors Y [-]
d_1_ctlg = tr.d_1;                      % List of diameters of inner bearing ring [mm]
C_stat_ctlg = tr.C_stat;                % List of static load ratings [N]
Y_0_ctlg = tr.Y_0;                      % List of bearing factors Y_0 [-]
name_ctlg = tr.Name;                    % List of bearing codes

%% 3) Calculation of output shaft data:
% Switch for number of EMs on vehicle axle (differential only for one EM)
if (gearbox.Input.num_EM==1)
    % Calculation of differential dimensions and gear data
    gearbox = calc_diff_data(gearbox);

    % Selection of bearings according to input load and desired lifetime
    gearbox = set_bearings_diff(gearbox, d_ctlg, d_A_ctlg, b_ctlg, C_dyn_ctlg, m_ctlg, a_ctlg, e_ctlg, Y_ctlg, d_1_ctlg, C_stat_ctlg, Y_0_ctlg, name_ctlg);

    % Calculation of the width of the differential with bearings in mm 
    if (strcmpi(gearbox.Input.axles,'parallel')==0)
        b_diff = gearbox.bearings_3.b_E+gearbox.diff.b_diffcage+b_3+b_gap+gearbox.bearings_3.b_F;
    elseif (strcmpi(diff_orientation,'out'))
        b_diff = gearbox.bearings_3.b_E+gearbox.diff.b_diffcage+b_3+b_gap+b_1+b_gap+gearbox.bearings_3.b_F;
    else
        b_diff = gearbox.bearings_3.b_E+gearbox.diff.b_diffcage+b_3+b_gap+gearbox.bearings_3.b_F;
    end
    
elseif (gearbox.Input.num_EM==2)
    % Selection of bearings according to input load and desired lifetime
    gearbox = set_bearings_diff(gearbox, d_ctlg, d_A_ctlg, b_ctlg, C_dyn_ctlg, m_ctlg, a_ctlg, e_ctlg, Y_ctlg, d_1_ctlg, C_stat_ctlg, Y_0_ctlg, name_ctlg);
    
    % Calculation of the width of the third shaft in mm 
    b_sh_3 = gearbox.bearings_3.b_E+b_gap+b_3+b_gap+gearbox.bearings_3.b_F;
end

%% 4) Output assignment:
if (gearbox.Input.num_EM==1)
    gearbox.diff.b_diff = b_diff;           % Width of the differential in mm
elseif (gearbox.Input.num_EM==2)
    gearbox.shafts.b_sh_3 = b_sh_3;         % Width of the third shaft in mm
    gearbox.diff.d_diffcage = gearbox.bearings_3.d_1_E;
    gearbox.diff.b_diffcage = b_gap;
end
