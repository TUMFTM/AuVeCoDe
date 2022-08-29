function gearbox = calc_J_plan(gearbox)
%% 1) Description:
% This function computes the moment of inertia with regard to input of the  and output shaft speed of the spinning components of the planetary 
% gearbox in kg*mm^2 according to Gross et al.: "Technische Mechanik 2" (starting p. 80 and 134).

%% 1.1) Authors
% Peter Koehler
% Date: 24.08.2021

%% 1.2) Sources:
% [1] Gross et al.: "Technische Mechanik 2", Springer Vieweg, 2021, ISBN: 978-3-662-61862-2
% [2] Bachelor Thesis Zaehringer: "Erstellung eines analytischen Ersatzmodells fuer Getriebe von Elektrofahrzeugen", Institute of Automotive Technology, TUM, Munich, 2018
% [3] Master Thesis Koehler: "Semi-physikalische Modellierung von Antriebsstrangkomponenten fuer Elektrofahrzeuge", Institute of Automotive Technology, TUM, Munich, 2021

%% 2) Initialization of required values:
b_gap = gearbox.ConstDim.b_gap;                 % Gap between components on the shafts (Zaehringer, p. 64) [mm]
t_housing = gearbox.ConstDim.t_housing;         % Housing thickness [mm]
d_diffbolt = gearbox.ConstDim.d_diffbolt;       % Diameter of differential bolt (Zaehringer, p. 55) [mm]
d_planbolt = gearbox.ConstDim.d_planbolt;       % Diameter of planet bolt [mm]
b_seal = gearbox.ConstDim.b_seal;               % Width of sealings of the housing [mm]
rho_gear = gearbox.MatProp.rho_gear/(10^9);     % Density 16MnCr5 (Zaehringer, p. 67) [kg/m^3]
m_n_1 = gearbox.gears.m_n_1;                    % Normal module of first stage [mm]
i_0 = gearbox.results.i_0;                      % Stationary gear ratio [-]
i_1s = gearbox.results.i_1s;                    % Transmission ratio between sun gear and planet carrier [-]
i_p_rel = gearbox.results.i_p_rel;              % Relative transmission ratio of the planets [-]
d_inn_1 = gearbox.shafts.d_inn_1;               % Inner diameter of hollow sun shaft [mm]
d_inn_p = gearbox.shafts.d_inn_p;               % Inner diameter of hollow planet shafts [mm]
d_1 = gearbox.gears.d_1;                        % Pitch diameter of sun gear [mm]
d_p1 = gearbox.gears.d_p1;                      % Pitch diameter of first planet [mm]
d_p2 = gearbox.gears.d_p2;                      % Pitch diameter of second planet [mm]
d_s = gearbox.gears.d_s;                        % Diameter of planet circle [mm]
b_1 = gearbox.gears.b_1;                        % Width of first stage gears [mm]
b_2 = gearbox.gears.b_2;                        % Width of second stage gears [mm]
d_fp1 = gearbox.gears.d_fp1;                    % Root diameter of first planet [mm]
d_fp2 = gearbox.gears.d_fp2;                    % Root diameter of second planet [mm]
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
d_A_A = gearbox.bearings_1.d_A_A;               % Outer diameter of bearing A [mm]
d_A_C = gearbox.bearings_2.d_A_C;               % Outer diameter of bearing C [mm]
d_A_D = gearbox.bearings_2.d_A_D;               % Outer diameter of bearing D [mm]
d_diffcage = gearbox.diff.d_diffcage;           % Diameter of differential cage [mm]
d_sh_3 = gearbox.shafts.d_sh_3;                 % Outer diameter of output shaft [mm]
m_planet = gearbox.masses.m_planet;             % Mass of each planet with bearings [kg]
m_C = gearbox.bearings_2.m_C;                   % Mass of bearing C [kg]
m_D = gearbox.bearings_2.m_D;                   % Mass of bearing D [kg]
if (gearbox.Input.num_EM==1)
    d_bev_l = gearbox.diff.d_bev_l;             % Diameer of larger bevel gears in differential [mm]
    d_bev_s = gearbox.diff.d_bev_s;             % Diameer of smaller bevel gears in differential [mm]
    b_bev = gearbox.diff.b_bev;                 % Width of bevel gears in differential [mm]
end

%% 3) Calculation of the moment of inertia in kg*mm^2:
% Sun shaft:
% Lengths of shaft segments left and right of wheel in mm
l_1_1 = b_seal+b_F+5+b_C+b_gap-5-b_A;
l_1_2 = b_A;
l_1_3 = 5;
% Moment of inertia of shaft segments in kg*mm^2
J_1_1 = pi/2*((d_sh_B/2)^4-(d_inn_1/2)^4)*l_1_1*rho_gear;
J_1_2 = pi/2*((d_sh_A/2)^4-(d_inn_1/2)^4)*l_1_2*rho_gear;
J_1_3 = pi/2*((d_1_A/2)^4-(d_inn_1/2)^4)*l_1_3*rho_gear;
J_g1 = pi/2*((d_1/2)^4-(d_inn_1/2)^4)*b_1*rho_gear;
% Total moment of inertia of the sun shaft in kg*mm^2
J_1 = J_1_1+J_1_2+J_1_3+J_g1;

% Planets about planet axis:
d_p1a = d_fp1-(6*m_n_1);
J_p_1 = pi/2*((d_sh_C/2)^4-(d_inn_p/2)^4)*b_C*rho_gear;
J_p_2 = pi/2*((d_1_C/2)^4-(d_inn_p/2)^4)*b_gap*rho_gear;
J_gp1 = pi/2*((d_p1/2)^4-(d_inn_p/2)^4)*b_1*rho_gear;
J_gp1_sub = pi/2*((d_p1a/2)^4-(d_1_C/2)^4)*(0.3*b_1)*rho_gear;
J_p_3 = pi/2*((d_fp2/2)^4-(d_inn_p/2)^4)*b_gap*rho_gear;
J_gp2 = pi/2*((d_p2/2)^4-(d_inn_p/2)^4)*b_2*rho_gear;
J_p_4 = pi/2*((d_1_D/2)^4-(d_inn_p/2)^4)*b_gap*rho_gear;
J_p_5 = pi/2*((d_sh_D/2)^4-(d_inn_p/2)^4)*b_D*rho_gear;
% Moment of inertia of the bearings on the planet axle
J_C = pi/2*((d_1_C/2)^4-(d_sh_C/2)^4)*b_C*rho_gear;
J_D = pi/2*((d_1_D/2)^4-(d_sh_D/2)^4)*b_D*rho_gear;
% Total moment of inertia of one planet (about the planet axis)
J_p = J_p_1+J_p_2+(J_gp1-2*J_gp1_sub)+J_p_3+J_gp2+J_p_4+J_p_5+J_C+J_D; 

% Moment of inertia of one planet about the center axis (plus pc bolts):
m_p = m_planet-m_C-m_D+((((d_1_C/2)^2-(d_sh_C/2)^2)*pi*b_C+((d_1_D/2)^2-(d_sh_D/2)^2)*pi*b_D)*rho_gear);
J_p_center = J_p+(d_s/2)^2*m_p;

% Planet carrier with differential (planet carrier approximated as prisma with 3 sides):
g_1 = (d_s/2+2*(d_A_C/2+5)*(1+sqrt(3)/2));
g_2 = (d_s/2+2*(d_A_D/2+5)*(1+sqrt(3)/2));
J_pc_1 = ((sqrt(3)*g_1^4)/48-pi/2*(d_A_A/2)^4)*t_housing*rho_gear;
% Planet carrier bolts
l_pcb = b_gap+b_1+b_gap+b_2+b_gap;
m_pcb = (d_planbolt/2)^2*pi*l_pcb*rho_gear;
J_pcb = pi/2*(d_planbolt/2)^4*l_pcb*rho_gear+m_pcb*(d_s/2)^2;

if (gearbox.Input.num_EM==1)
    J_pc_2 = ((sqrt(3)*g_2^4)/48-pi/2*(d_diffcage/2)^4)*t_housing*rho_gear;
    % Differential cage
    J_dc_1 = pi/2*((d_diffcage/2)^4-(d_bev_l/2)^4)*d_bev_s*rho_gear;
    J_dc_2 = pi/2*(((d_diffcage+d_1_E)/4)^4-(d_sh_3/2+5)^4)*10*rho_gear;
    J_dc_3 = pi/2*((d_diffcage/2)^4-(d_sh_3/2+5)^4)*t_housing*rho_gear;
    % Planet carrier side of bearing E:
    J_pc_E = pi/2*((d_sh_E/2)^4-(d_sh_E/2-5)^4)*b_E*rho_gear;
    % Planet carrier side of bearing F:
    J_pc_F = pi/2*((d_sh_F/2)^4-(d_A_A/2)^4)*b_F*rho_gear+pi/2*((d_1_F/2)^4-(d_A_A/2)^4)*5*rho_gear;
    % Bevel gears
    J_bev_l = pi/2*(((d_bev_l+d_bev_l-b_bev)/4)^4-(d_sh_3/2)^4)*b_bev*rho_gear;
    m_bev_s = (((d_bev_s+d_bev_s-b_bev)/4)^2)*pi*b_bev*rho_gear;
    J_bev_s = pi/2*(((d_bev_s+d_bev_s-b_bev)/4)^4)*b_bev*rho_gear+m_bev_s*((d_bev_l+d_bev_l-b_bev)/4)^2;    
    % Differential bolt
    m_db = (d_diffbolt/2)^2*pi*(d_bev_l-b_bev)*rho_gear;
    J_db = 1/4*m_db*(d_diffbolt/2)^2+1/12*m_db*(d_bev_l-b_bev)^2;
    % Total moment of inertia of the planet carrier with differential
    J_pc = J_pc_1+J_pc_2+J_pcb+J_dc_1+J_dc_2+J_dc_3+2*J_bev_l+2*J_bev_s+J_db+J_pc_E+J_pc_F;
    
elseif (gearbox.Input.num_EM==2)
    J_pc_2 = (sqrt(3)*g_2^4)/48*t_housing*rho_gear;
    % Bearing carriers differential shaft
    J_pc_E = pi/2*(d_sh_E/2)^4*(b_E+b_seal)*rho_gear;
    J_pc_F = pi/2*((d_sh_F/2)^4-(d_A_A/2)^4)*b_F*rho_gear+pi/2*((d_1_F/2)^4-(d_A_A/2)^4)*5*rho_gear;
    % Total moment of inertia of the planet carrier
    J_pc = J_pc_1+J_pc_2+J_pcb+J_pc_E+J_pc_F;
end

% Moment of inertia of the bearings (approximated as spinning inner ring)
J_A = pi/2*((d_1_A/2)^4-(d_sh_A/2)^4)*b_A*rho_gear;
J_B = pi/2*((d_1_B/2)^4-(d_sh_B/2)^4)*b_B*rho_gear;
J_E = pi/2*((d_1_E/2)^4-(d_sh_E/2)^4)*b_E*rho_gear;
J_F = pi/2*((d_1_F/2)^4-(d_sh_F/2)^4)*b_F*rho_gear;

% Relative transmission ratio of the planets in relation to the planet carrier
i_sp = (-2*i_0)/(i_0+1);

% Total reduced moment of inertia based on input shaft speed
J_tot_in = (J_1+J_A+J_B)+((3*J_p)/(i_p_rel^2))+((3*J_p_center+J_E+J_F+J_pc)/(i_1s^2));

% Total reduced moment of inertia based on wheel speed
J_tot_out = ((J_1+J_A+J_B)*(i_1s^2))+((3*J_p)*(i_sp^2))+(3*J_p_center+J_E+J_F+J_pc);

%% 4) Output assignment:
gearbox.results.J_tot_in = J_tot_in;        % Moment of inertia with regard to input rotational speed in kg*mm^2
gearbox.results.J_tot_out = J_tot_out;      % Moment of inertia with regard to output rotational speed in kg*mm^2

end
