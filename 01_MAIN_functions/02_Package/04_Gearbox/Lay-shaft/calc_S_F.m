function gearbox = calc_S_F(shaft, gearbox)
%% 1) Description:
% This function computes the safety factor against root break according to
% Decker, p. 150

%% 1.1) Authors
% Peter Koehler
% Date: 24.08.2021

%% 1.2) Sources:
% [1] Decker and Kabus: "Maschinenelemente - Formeln", Carl Hanser Verlag, 2011, ISBN: 978-3-446-42990-1
% [2] DIN 3990-1: "Tragfaehigkeitsberechnung von Stirnraedern", 1987
% [3] Bachelor Thesis Zaehringer: "Erstellung eines analytischen Ersatzmodells fuer Getriebe von Elektrofahrzeugen", Institute of Automotive Technology, TUM, Munich, 2018
% [4] Semesterarbeit Koehler: "Semi-physikalische Modellierung von Antriebsstrangkomponenten fuer Elektrofahrzeuge", Institute of Automotive Technology, TUM, Munich, 2020
% [5] Master Thesis Koehler: "Semi-physikalische Modellierung von Antriebsstrangkomponenten fuer Elektrofahrzeuge", Institute of Automotive Technology, TUM, Munich, 2021

%% 2) Initialization of required values:
K_app = gearbox.CalcFactors.K_app;                  % Application factor for static strength verification (DIN 3990-1, p.55) [-]
sigma_fe = gearbox.MatProp.sigma_fe;                % Fatigue strength of 16MnCr5 (Zaehringer, p. xiv)[N/mm^2]

if (shaft==1)
    beta = gearbox.gears_12.beta_1;                 % Helix angle of first stage [rad]
    beta_b = gearbox.gears_12.beta_b_1;             % Base helix angle of first stage [rad]
    b = gearbox.gears_12.b_1;                       % Width of first wheel [mm]
    m_n = gearbox.gears_12.m_n_1;                   % Normal module of first stage [mm]
    epsilon_beta = gearbox.factors.epsilon_beta_1;  % Overlap ratio of first stage [-]
    z_1 = gearbox.gears_12.z_1;                     % Number of teeth of first wheel [-]
    F_n_t = gearbox.factors.F_n_t_1;                % Circumferential tooth force of first stage [N]
    Y_epsilon = gearbox.factors.Y_epsilon_1;        % Overlap factor for tooth root load capacity for first stage [-]
    K_v = gearbox.factors.K_v_1;                    % Dynamic factor for first stage [-]
    K_f_beta = gearbox.factors.K_f_beta_1;          % Width factor for root load capacity for first stage [-]
    K_f_alpha = gearbox.factors.K_f_alpha_1;        % Boundary condition for tooth root load capacity for first stage [-]
elseif (round(shaft)==2)
    beta = gearbox.gears_34.beta_2;                 % Helix angle of second stage [rad]
    beta_b = gearbox.gears_34.beta_b_2;             % Base helix angle of second stage [rad]
    z_1 = gearbox.gears_34.z_3;                     % Number of teeth of third wheel [-]
    b = gearbox.gears_34.b_3;                       % Width of third wheel [mm]
    m_n = gearbox.gears_34.m_n_3;                   % Normal module of second stage [mm]
    epsilon_beta = gearbox.factors.epsilon_beta_2;  % Overlap ratio of second stage [-]
    F_n_t = gearbox.factors.F_n_t_2;                % Circumferential tooth force of second stage [N]
    Y_epsilon = gearbox.factors.Y_epsilon_2;        % Overlap factor for tooth root load capacity for second stage [-]
    K_v = gearbox.factors.K_v_2;                    % Dynamic factor for second stage [-]
    K_f_beta = gearbox.factors.K_f_beta_2;          % Width factor for root load capacity for second stage [-]
    K_f_alpha = gearbox.factors.K_f_alpha_2;        % Boundary condition for tooth root load capacity for second stage [-]
end

%% 3) Calculation of safety factors against root break:
% Substitute number of teeth (Decker, p. 133)
z_n = z_1/((cos(beta_b)^2)*cos(beta));
% Determination of helix angle factor, overlap ratio set to 1 if >1 (Decker, p. 150)
if (epsilon_beta>1)
    epsilon_beta = 1;
end
Y_beta = 1-(epsilon_beta*((beta*180/pi)/120));

% Form factor depending on substitute number of teeth for a profile
% modification of 0 (Zaehringer, p. xiii)
if (z_n<15)
    Y_Fa = 3.36;
elseif (z_n>=15 && z_n<16)
    Y_Fa = 3.25;
elseif (z_n>=16 && z_n<17)
    Y_Fa = 3.16;
elseif (z_n>=17 && z_n<18)
    Y_Fa = 3.09;
elseif (z_n>=18 && z_n<19)
    Y_Fa = 3.02;
elseif (z_n>=19 && z_n<20)
    Y_Fa = 2.96;
elseif (z_n>=20 && z_n<21)
    Y_Fa = 2.91;
elseif (z_n>=21 && z_n<22)
    Y_Fa = 2.87;
elseif (z_n>=22 && z_n<23)
    Y_Fa = 2.83;
elseif (z_n>=23 && z_n<24)
    Y_Fa = 2.80;
elseif (z_n>=24 && z_n<25)
    Y_Fa = 2.75;
elseif (z_n>=25 && z_n<30)
    Y_Fa = 2.72;
elseif (z_n>=30 && z_n<40)
    Y_Fa = 2.60;
elseif (z_n>=40 && z_n<50)
    Y_Fa = 2.45;
elseif (z_n>=50 && z_n<60)
    Y_Fa = 2.36;
elseif (z_n>=60 && z_n<100)
    Y_Fa = 2.32;
elseif (z_n>=100 && z_n<200)
    Y_Fa = 2.21;
elseif (z_n>=200 && z_n<400)
    Y_Fa = 2.14;
else
    Y_Fa = 2.10;
end

% Stress coefficient (Zaehringer, p. xiv)
if (z_n<18)
    Y_Sa = 1.57;
elseif (z_n>=18 && z_n<19)
    Y_Sa = 1.58;
elseif (z_n>=19 && z_n<20)
    Y_Sa = 1.59;
elseif (z_n>=20 && z_n<22)
    Y_Sa = 1.60;
elseif (z_n>=22 && z_n<24)
    Y_Sa = 1.63;
elseif (z_n>=24 && z_n<26)
    Y_Sa = 1.64;
elseif (z_n>=26 && z_n<28)
    Y_Sa = 1.66;
elseif (z_n>=28 && z_n<30)
    Y_Sa = 1.68;
elseif (z_n>=30 && z_n<35)
    Y_Sa = 1.69;
elseif (z_n>=35 && z_n<40)
    Y_Sa = 1.73;
elseif (z_n>=40 && z_n<45)
    Y_Sa = 1.75;
elseif (z_n>=45 && z_n<50)
    Y_Sa = 1.78;
elseif (z_n>=50 && z_n<60)
    Y_Sa = 1.80;
elseif (z_n>=60 && z_n<70)
    Y_Sa = 1.84;
elseif (z_n>=70 && z_n<80)
    Y_Sa = 1.87;
elseif (z_n>=80 && z_n<100)
    Y_Sa = 1.90;
elseif (z_n>=100 && z_n<150)
    Y_Sa = 1.94;
elseif (z_n>=150 && z_n<200)
    Y_Sa = 2.02;
elseif (z_n>=200 && z_n<400)
    Y_Sa = 2.06;
else
    Y_Sa = 2.17;
end

% Root stresses and safety factor (Decker, p. 150)
% Root nominal stress in N/mm^2
sigma_f_0 = (F_n_t/(b*m_n))*Y_Fa*Y_Sa*Y_beta*Y_epsilon;
% Root stress in N/mm^2
sigma_f = (((sigma_f_0*K_app)*K_v)*K_f_beta)*K_f_alpha;

% Safety factor against against root break with Y_NT=1.8 (Zaehringer, p. xv)
S_F = sigma_fe*1.8/sigma_f;

%% 4) Output assignment:
if (shaft==1)
    gearbox.results.S_F_12 = S_F;       % Safety factor against root break for first stage
elseif (shaft==2)
    gearbox.results.S_F_34 = S_F;       % Safety factor against root break for second stage
end

end
