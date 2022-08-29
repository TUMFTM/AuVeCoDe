function gearbox = calc_J(gearbox)
%% 1) Description:
% This function computes the moment of inertia with regard to input of the and output shaft speed of the spinning 
% components of the gearbox in kg*mm^2 according to Gross et al.: "Technische Mechanik 2" (starting p. 80 and 134).

%% 1.1) Authors
% Peter Koehler
% Date: 24.08.2021

%% 1.2) Sources:
% [1] Gross et al.: "Technische Mechanik 2", Springer Vieweg, 2021, ISBN: 978-3-662-61862-2
% [2] Bachelor Thesis Zaehringer: "Erstellung eines analytischen Ersatzmodells fuer Getriebe von Elektrofahrzeugen", Institute of Automotive Technology, TUM, Munich, 2018
% [3] Master Thesis Koehler: "Semi-physikalische Modellierung von Antriebsstrangkomponenten fuer Elektrofahrzeuge", Institute of Automotive Technology, TUM, Munich, 2021

%% 2) Initialization of required values:
b_gap = gearbox.ConstDim.b_gap;                 % Gap between components on the shafts (Zaehringer, p. 64) [mm]
b_park = gearbox.ConstDim.b_park;               % Width of parking gear (Zaehringer, p. 64) [mm]
b_seal = gearbox.ConstDim.b_seal;               % Width of seals on output shafts as average of database [mm]
d_diffbolt = gearbox.ConstDim.d_diffbolt;       % Diameter of differential bolt (Zaehringer, p. 55) [mm]
rho_gear = gearbox.MatProp.rho_gear/(10^9);     % Density 16MnCr5 (Zaehringer, p. 67) [kg/mm^3]
m_n_1 = gearbox.gears_12.m_n_1;                 % Normal module of first stage [mm]
m_n_3 = gearbox.gears_34.m_n_3;                 % Normal module of second stage [mm]
i_12 = gearbox.results.i_12;                    % Transmission ratio of first stage [-]
i_34 = gearbox.results.i_34;                    % Transmission ratio of second stage [-]
d_inn_1 = gearbox.shafts.d_inn_1;               % Inner diameter of hollow shaft 1 [mm]
d_inn_2 = gearbox.shafts.d_inn_2;               % Inner diameter of hollow shaft 2 [mm]
d_1 = gearbox.gears_12.d_1;                     % Pitch diameter of wheel 1 [mm]
d_2 = gearbox.gears_12.d_2;                     % Pitch diameter of wheel 2 [mm]
d_3 = gearbox.gears_34.d_3;                     % Pitch diameter of wheel 3 [mm]
d_4 = gearbox.gears_34.d_4;                     % Pitch diameter of wheel 4 [mm]
b_1 = gearbox.gears_12.b_1;                     % Width of wheels 1&2 [mm]
b_3 = gearbox.gears_34.b_3;                     % Width of wheels 3&4 [mm]
d_f2 = gearbox.gears_12.d_f2;                   % Root diameter of wheel 2 [mm]
d_f4 = gearbox.gears_34.d_f4;                   % Root diameter of wheel 4 [mm]
b_A = gearbox.bearings_1.b_A;                   % Width of bearing A [mm]
b_B = gearbox.bearings_1.b_B;                   % Width of bearing B [mm]
b_C = gearbox.bearings_2.b_C;                   % Width of bearing C [mm]
b_D = gearbox.bearings_2.b_D;                   % Width of bearing D [mm]
b_E = gearbox.bearings_3.b_E;                   % Width of bearing E [mm]
b_F = gearbox.bearings_3.b_F;                   % Width of bearing F [mm]
d_sh_A = gearbox.bearings_1.d_sh_A;             % Inner diameter of bearing A [mm]
d_sh_B = gearbox.bearings_1.d_sh_B;             % Inner diameter of bearing B [mm]
d_sh_C = gearbox.bearings_2.d_sh_C;             % Inner diameter of bearing C [mm]
d_sh_D = gearbox.bearings_2.d_sh_D;             % Inner diameter of bearing D [mm]
d_sh_E = gearbox.bearings_3.d_sh_E;             % Inner diameter of bearing E [mm]
d_sh_F = gearbox.bearings_3.d_sh_F;             % Inner diameter of bearing F [mm]
d_1_A = gearbox.bearings_1.d_1_A;               % Outer diameter of As inner bearing ring [mm]
d_1_B = gearbox.bearings_1.d_1_B;               % Outer diameter of Bs inner bearing ring [mm]
d_1_C = gearbox.bearings_2.d_1_C;               % Outer diameter of Cs inner bearing ring [mm]
d_1_D = gearbox.bearings_2.d_1_D;               % Outer diameter of Ds inner bearing ring [mm]
d_1_E = gearbox.bearings_3.d_1_E;               % Outer diameter of Es inner bearing ring [mm]
d_1_F = gearbox.bearings_3.d_1_F;               % Outer diameter of Fs inner bearing ring [mm]
if (gearbox.Input.num_EM==1)
    d_diffcage = gearbox.diff.d_diffcage;                   % Diameter of differential cage [mm]
    d_bev_l = gearbox.diff.d_bev_l;                         % Diameer of larger bevel gears in differential [mm]
    d_bev_s = gearbox.diff.d_bev_s;                         % Diameer of smaller bevel gears in differential [mm]
    b_bev = gearbox.diff.b_bev;                             % Width of bevel gears in differential [mm]
    if strcmpi(gearbox.Input.axles,'parallel')
        diff_orientation = gearbox.diff.diff_orientation;   % Orientation of the differential
    end
end
d_sh_3 = gearbox.shafts.d_sh_3;                 % Outer diameter of output shaft [mm]

%% 3) Calculation of the moment of inertia in kg*mm^2:
% Shaft 1:
% Lengths of shaft segments left and right of wheel in mm
l_1_1 = b_seal+b_A+b_gap+b_park+b_gap;
l_1_2 = b_gap+b_B;
% Moment of inertia of shaft segments in kg*mm^2
J_1_1 = pi/2*((d_sh_A/2)^4-(d_inn_1/2)^4)*l_1_1*rho_gear;
J_g1 = pi/2*((d_1/2)^4-(d_inn_1/2)^4)*b_1*rho_gear;
J_1_2 = pi/2*((d_sh_B/2)^4-(d_inn_1/2)^4)*l_1_2*rho_gear;
% Total moment of inertia of shaft 1
J_1 = J_1_1+J_g1+J_1_2;

% Shaft 2:
d_2a = d_f2-(6*m_n_1);
l_2_1 = b_C+b_gap;
l_2_2 = b_gap+b_D;
% Moment of inertia of shaft segments in kg*mm^2
J_2_1 = pi/2*((d_sh_C/2)^4-(d_inn_2/2)^4)*l_2_1*rho_gear;
J_g2 = pi/2*((d_2/2)^4-(d_inn_2/2)^4)*b_1*rho_gear;
J_g2_red = pi/2*((d_2a/2)^4-(d_1_C/2)^4)*(0.3*b_1)*rho_gear;
J_2_2 = pi/2*((0.5*min(d_sh_C,d_sh_D))^4-(0.5*d_inn_2)^4)*b_gap*rho_gear;
J_g3 = pi/2*((d_3/2)^4-(d_inn_2/2)^4)*b_3*rho_gear;
J_2_3 = pi/2*((d_sh_D/2)^4-(d_inn_2/2)^4)*l_2_2*rho_gear;
% Total moment of inertia of shaft 2
J_2 = J_2_1+J_g2-2*J_g2_red+J_2_2+J_g3+J_2_3;

% Shaft 3:
if (gearbox.Input.num_EM==1)
    d_4a = d_f4-(6*m_n_3);    
    % Shaft 3 side of bearing F:
    if (strcmpi(gearbox.Input.axles,'parallel') && strcmpi(diff_orientation,'in'))
        J_3_1 = pi/2*((d_sh_F/2)^4-(d_sh_3/2+2)^4)*b_F*rho_gear+pi/2*((d_1_F/2)^4-(d_sh_3/2+2)^4)*b_gap*rho_gear;
    else
        J_3_1 = pi/2*((d_sh_F/2)^4-(d_sh_3/2+2)^4)*b_F*rho_gear+pi/2*((d_1_F/2)^4-(d_sh_F/2+2)^4)*(b_gap+b_1+b_gap)*rho_gear;
    end
    % Gear 4:
    J_g4 = pi/2*((d_4/2)^4-(d_sh_3/2+2)^4)*b_3*rho_gear;
    J_g4_red_1 = pi/2*((d_4a/2)^4-(d_1_F/2)^4)*0.3*b_3*rho_gear;
    J_g4_red_2 = pi/2*(d_4a/2)^4*0.3*b_3*rho_gear;
    % Differential cage
    J_3_dc_1 = pi/2*((d_diffcage/2)^4-(d_bev_l/2)^4)*d_bev_s*rho_gear;
    J_3_dc_2 = pi/2*(((d_diffcage+d_1_E)/4)^4-(d_sh_3/2+5)^4)*10*rho_gear;
    % Shaft 3 side of bearing E:
    J_3_2 = pi/2*((d_sh_E/2)^4-(d_sh_3/2+2)^4)*b_E*rho_gear;
    % Bevel gears (smaller bevel gears approximated as rotating point masses)
    J_3_bev_l = pi/2*(((d_bev_l+d_bev_l-b_bev)/4)^4-(d_sh_3/2)^4)*b_bev*rho_gear;
    m_bev_s = (((d_bev_s+d_bev_s-b_bev)/4)^2)*pi*b_bev*rho_gear;
    J_3_bev_s = pi/2*(((d_bev_s+d_bev_s-b_bev)/4)^4)*b_bev*rho_gear+m_bev_s*((d_bev_l+d_bev_l-b_bev)/4)^2;    
    % Differential bolt
    m_db = (d_diffbolt/2)^2*pi*(d_bev_l-b_bev)*rho_gear;
    J_3_db = 1/4*m_db*(d_diffbolt/2)^2+1/12*m_db*(d_bev_l-b_bev)^2;
    % Total shaft 3 with differential
    J_3 = J_3_1+(J_g4-J_g4_red_1-J_g4_red_2)+J_3_dc_1+J_3_dc_2+J_3_2+2*J_3_bev_l+2*J_3_bev_s+J_3_db;
    
elseif (gearbox.Input.num_EM==2)
    d_4a = d_f4-(6*m_n_3);
    J_3_1 = pi/2*((d_sh_F/2)^4-(d_sh_3/2+2)^4)*b_F*rho_gear+pi/2*((d_1_F/2)^4-(d_sh_3/2+2)^4)*b_gap*rho_gear;
    J_g4 = pi/2*((d_4/2)^4-(d_sh_3/2+2)^4)*b_3*rho_gear;
    J_g4_red_1 = pi/2*((d_4a/2)^4-(d_1_F/2)^4)*0.3*b_3*rho_gear;
    J_g4_red_2 = pi/2*((d_4a/2)^4-(d_1_E/2)^4)*0.3*b_3*rho_gear;
    J_3_2 = pi/2*((d_sh_E/2)^4-(d_sh_3/2+2)^4)*b_E*rho_gear+pi/2*((d_1_E/2)^4-(d_sh_3/2+2)^4)*b_gap*rho_gear;
    J_3 = J_3_1+J_g4-J_g4_red_1-J_g4_red_2+J_3_2;
    
end

% Moment of inertia of the bearings (approximated as spinning inner ring)
J_A = pi/2*((d_1_A/2)^4-(d_sh_A/2)^4)*b_A*rho_gear;
J_B = pi/2*((d_1_B/2)^4-(d_sh_B/2)^4)*b_B*rho_gear;
J_C = pi/2*((d_1_C/2)^4-(d_sh_C/2)^4)*b_C*rho_gear;
J_D = pi/2*((d_1_D/2)^4-(d_sh_D/2)^4)*b_D*rho_gear;
J_E = pi/2*((d_1_E/2)^4-(d_sh_E/2)^4)*b_E*rho_gear;
J_F = pi/2*((d_1_F/2)^4-(d_sh_F/2)^4)*b_F*rho_gear;

% Total reduced moment of inertia based on input shaft speed
J_tot_in = (J_1+J_A+J_B)+((J_2+J_C+J_D)/(i_12^2))+((J_3+J_E+J_F)/((i_12^2)*(i_34^2)));

% Total reduced moment of inertia based on wheel speed
J_tot_out = ((J_1+J_A+J_B)*(i_12^2)*(i_34^2))+((J_2+J_C+J_D)*(i_34^2))+(J_3+J_E+J_F);

%% 4) Output assignment:
gearbox.results.J_tot_in = J_tot_in;        % Moment of inertia with regard to input rotational speed in kg*mm^2
gearbox.results.J_tot_out = J_tot_out;      % Moment of inertia with regard to output rotational speed in kg*mm^2

end
