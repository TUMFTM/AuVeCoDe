function gearbox = calc_factors_plan (shaft, gearbox)
%% 1) Description:
% This function computes calculation factors for the determination of the
% gears' safety coefficients for planetary gearboxes according to Decker,
% starting p. 148

%% 1.1) Authors
% Peter Koehler
% Date: 24.08.2021

%% 1.2) Sources:
% [1] DIN 3990-1: "Tragfaehigkeitsberechnung von Stirnraedern", 1987
% [2] Decker and Kabus: "Maschinenelemente - Formeln", Carl Hanser Verlag, 2011, ISBN: 978-3-446-42990-1
% [3] Niemann and Winter: "Maschinenelemente - Band 2: Getriebe allgemein, Zahnradgetriebe", Springer Verlag, 2003, ISBN: 978-3-662-11874-0
% [4] Bachelor Thesis Zaehringer: "Erstellung eines analytischen Ersatzmodells fuer Getriebe von Elektrofahrzeugen", Institute of Automotive Technology, TUM, Munich, 2018
% [5] Master Thesis Koehler: "Semi-physikalische Modellierung von Antriebsstrangkomponenten fuer Elektrofahrzeuge", Institute of Automotive Technology, TUM, Munich, 2021

%% 2) Initialization of required values:
T_max = 1/3*gearbox.Input.T_max*1000;               % Maximum torque of el. motor [Nmm]
alpha_n = gearbox.GearingConst.alpha_n;             % Normal pressure angle [rad] --> 20deg
K_app = gearbox.CalcFactors.K_app;                  % Application factor for static strength verification (DIN 3990-1, p.55) [-]
K_1 = gearbox.CalcFactors.K_1;                      % Factor for calculation of dynamic factor K_v (DIN 3990-1, p. 18) [-]
K_2 = gearbox.CalcFactors.K_2;                      % Factor for calculation of dynamic factor K_v (DIN 3990-1, p. 18) [-]
c_gamma = gearbox.MatProp.c_gamma;                  % Mesh stiffness steel/steel (Decker, p. 148) [N/mm*mym]
n_nom = gearbox.Input.n_nom;                        % Nominal engine speed [rpm]
if (shaft==1)
    alpha_t = gearbox.gears.alpha_t_1;              % Pressure angle in transverse section of first stage [rad]
    beta = gearbox.gears.beta_1;                    % Helix angle of first stage [rad]
    m_n = gearbox.gears.m_n_1;                      % Normal module of first stage [mm]
    b = gearbox.gears.b_1;                          % Width of wheel 1&2 [mm]
    z_1 = gearbox.gears.z_1;                        % Number of teeth of wheel 1 [-]
    d_1 = gearbox.gears.d_1;                        % Pitch diameter of wheel 1 [mm]
    d_a1 = gearbox.gears.d_a1;                      % Outside diameter of wheel 1 [mm]
    d_a2 = gearbox.gears.d_ap1;                     % Outside diameter of first planet [mm]
    d_b1 = gearbox.gears.d_b1;                      % Base diameter of wheel 1 [mm]
    d_b2 = gearbox.gears.d_bp1;                     % Base diameter of first planet [mm]
    i_12 = gearbox.results.i_1p1;                   % Transmission ratio between sun shaft and first planet [-]
    alpha_wt = gearbox.gears.alpha_wt_1;            % Working transverse pressure angle of first stage [rad]
else
    i_1p1 = gearbox.results.i_1p1;                  % Stationary gear ratio of the planetary transmission [-]
    i_1s = gearbox.results.i_1s;                    % Total transmission of the gearbox [-]
    n_nom = n_nom/i_1p1*(1-1/i_1s);                 % Nominal speed of the planets [rpm]
    T_max = T_max*i_1p1;                            % Maximum torque of the planets [Nmm]
    alpha_t = gearbox.gears.alpha_t_2;              % Pressure angle in transverse section of second stage [rad]
    beta = gearbox.gears.beta_2;                    % Helix angle of second stage [rad]
    m_n = gearbox.gears.m_n_2;                      % Normal module of stage 2 [mm]
    b = gearbox.gears.b_2;                          % Width of planet 2 [mm]
    z_1 = gearbox.gears.z_p2;                       % Number of teeth of planet 2 [-]
    d_1 = gearbox.gears.d_p2;                       % Pitch diameter of planet 2 [mm]
    d_a1 = gearbox.gears.d_ap2;                     % Outside diameter of planet 2 [mm]
    d_a2 = gearbox.gears.d_a2;                      % Inside diameter of ring gear [mm]
    d_b1 = gearbox.gears.d_bp2;                     % Base diameter of planet 2 [mm]
    d_b2 = gearbox.gears.d_b2;                      % Base diameter of ring gear [mm]
    i_12 = gearbox.results.i_p22;                   % Transmission ratio between sun and first planet [-]
    alpha_wt = gearbox.gears.alpha_wt_2;            % Working transverse pressure angle of second stage [rad]
end

%% 3) Calculation of calculation factors for safety coefficients:
% Calculation of base helix angle in rad (Decker, p. 133)
beta_b = acos(sin(alpha_n)/sin(alpha_t));

% Definition of root clearance in mm (Decker, p. 133)
c = 0.25*m_n;

% Distance between planetary axle and main gearbox axle
a = (gearbox.gears.d_1+gearbox.gears.d_p1)/2;

% Determination of dynamic factor K_v according to Decker, p. 148
v = ((d_1/1000)*pi)*(n_nom/60);                     % circumferential speed in tooth contact point [m/s]
F_n_t = 2*T_max/d_1;                                % circumferential tooth force [N]
K_v = 1+((K_1/(K_app*F_n_t/b))+K_2)*((z_1*v/100)*sqrt((i_12^2)/(1+(i_12^2))));

% Determination of width factors K_h_beta (flank) und K_f_beta (root) (Decker, pp. 149)
if (b<=20)
    f_h_beta = 8;
elseif (b>20 && b<=40)
    f_h_beta = 9;
elseif (b>40 && b<=100)
    f_h_beta = 10;
else
    f_h_beta = 11;
end
% Average circumferential force at the pitch circle in N (Decker, p. 148)
F_m = F_n_t*K_v*K_app;

% Approximated flank line deviation in mym (Niemann and Winter, p. 324)
if (F_n_t/b<200)
    if (b<20)
        f_sh = 5;
    elseif (b<40)
        f_sh = 6.5;
    else            % Width always smaller than 100 mm!
        f_sh = 7;
    end
elseif (F_n_t/b<1000)
    if (b<20)
        f_sh = 6;
    elseif (b<40)
        f_sh = 7;
    else
        f_sh = 8;
    end
else
    if (b<20)
        f_sh = 10;
    elseif (b<40)
        f_sh = 13;
    else
        f_sh = 18;
    end
end

% Effective flank line deviation before running-in in mym (Decker, p. 149)
F_beta_x = (1.33*f_sh)+(f_h_beta);
% Running-in value in mym (with maximum of 6 mym) (Zaehringer, p. xii)
y_beta = 0.15*F_beta_x;                 
if (y_beta>6)
    y_beta = 6;
end
% Effective flank line deviation after running-in in mym (Decker, p. 149)
F_beta_y = F_beta_x-y_beta;

% Calculation of flank width factor K_h_beta (Decker, p. 148)
par = (F_beta_y*c_gamma)/(2*F_m/b);     % auxiliary parameter for determination of K_h_beta
if (par>=1)
    K_h_beta = sqrt((2*F_beta_y*c_gamma)/(F_m/b));
else
    K_h_beta = 1+(F_beta_y*c_gamma)/(2*F_m/b);
end

% Calculation of width factor for root load capacity K_f_beta (Decker, p. 149)
h = (2*m_n)+c;                          % tooth height in mm (Decker, p.149)
N_f = ((b/h)^2)/(1+(b/h)+((b/h)^2));    
K_f_beta = K_h_beta^(N_f);

% Definition of transverse factors K_f_alpha and K_h_alpha (Decker, pp. 149)
% Decisive transverse circumferential force in N
F_t_h = F_n_t*K_v*K_app*K_h_beta;
% Permitted base-pitch deviation from table (Zaehringer, p. xii)
if (d_1<=50)
    if (m_n<=3.55)
        f_pe = 7;
    elseif (m_n>3.55 && m_n<=6)
        f_pe = 8;
    else
        f_pe = 10;
    end
elseif (d_1>50 && d_1<=125)
    if (m_n<=3.55)
        f_pe = 7;
    elseif (m_n>3.55 && m_n<=6)
        f_pe = 9;
    elseif (m_n>6 && m_n<=10)
        f_pe = 10;
    else
        f_pe = 12;
    end
else
    if (m_n<=3.55)
        f_pe = 8;
    elseif (m_n>3.55 && m_n<=6)
        f_pe = 9;
    elseif (m_n>6 && m_n<10)
        f_pe = 11;
    else
        f_pe = 16;
    end
end

% Running-in value y_alpha (Zaehringer, p. 45)
y_alpha = 0.075*f_pe;

% Calculation of Total overlap
p_et = m_n*pi*cos(alpha_t)/cos(beta);           % transverse normal base pitch (Decker, p. 135)
% Transverse contact ratio (Decker, p. 136)
if (shaft==1)
    epsilon_alpha = (sqrt((d_a1^2)-(d_b1^2))+sqrt((d_a2^2)-(d_b2^2))-(2*a*sin(alpha_wt)))/(2*p_et);
else
    epsilon_alpha = (sqrt((d_a1^2)-(d_b1^2))-sqrt((d_a2^2)-(d_b2^2))-(2*(-a)*sin(alpha_wt)))/(2*p_et);
end
epsilon_beta = b*sin(beta)/(m_n*pi);            % overlap ratio (Decker, p. 136)
epsilon_gamma = epsilon_beta+epsilon_alpha;     % total overlap (Decker, p. 136)

% Boundary condition for tooth root and pitting load capacity K_f_alpha and K_h_alpha (Decker, p.150)
if (epsilon_gamma>2)
    K_f_alpha = 0.9+(0.4*(sqrt((2*(epsilon_gamma-1))/epsilon_gamma)*(c_gamma*(f_pe-y_alpha)/(F_t_h/b))));
    K_h_alpha = K_f_alpha;
else
    K_f_alpha = (epsilon_gamma/2)*(0.9+(0.4*(c_gamma*(f_pe-y_alpha)/(F_t_h/b))));
    K_h_alpha = K_f_alpha;
end

% Overlap factor for tooth root load capacity (Decker, p. 150)
Y_epsilon = 0.25+(0.75/(epsilon_alpha/(cos(beta_b)^2)));

% Overlap factor for calculation of Z_epsilon (Decker, p. 150) 
% (epsilon_beta is set to one for the calculation, if > 1)
epsilon_beta_new = epsilon_beta;
if (epsilon_beta_new>1)
    epsilon_beta_new = 1;
end
% Overlap factor for pitting load capacity (Decker, p. 150)
Z_epsilon = sqrt((((4-epsilon_alpha)/3)*(1-epsilon_beta_new))+(epsilon_beta_new/epsilon_alpha));

%% 4) Output assignment:
if (shaft==1)
    gearbox.gears.beta_b_1 = beta_b;                    % Base helix angle of first stage in rad
    gearbox.factors.K_v_1 = K_v;                        % Dynamic factor for first stage
    gearbox.factors.K_h_beta_1 = K_h_beta;              % Flank width factor for first stage
    gearbox.factors.K_h_alpha_1 = K_h_alpha;            % Boundary condition for pitting load capacity for first stage
    gearbox.factors.K_f_beta_1 = K_f_beta;              % Width factor for root load capacity for first stage
    gearbox.factors.K_f_alpha_1 = K_f_alpha;            % Boundary condition for tooth root load capacity for first stage
    gearbox.factors.Y_epsilon_1 = Y_epsilon;            % Overlap factor for tooth root load capacity for first stage
    gearbox.factors.Z_epsilon_1 = Z_epsilon;            % Overlap factor for pitting load capacity for first stage
    gearbox.factors.epsilon_beta_1 = epsilon_beta;      % Overlap factor for first stage
    gearbox.factors.epsilon_alpha_1 = epsilon_alpha;    % Transverse contact ratio of first stage
    gearbox.factors.F_n_t_1 = F_n_t;                    % Circumferential tooth force of first stage [N]
else 
    gearbox.gears.beta_b_2 = beta_b;                    % Base helix angle of second stage in rad
    gearbox.factors.K_v_2 = K_v;                        % Dynamic factor for second stage
    gearbox.factors.K_h_beta_2 = K_h_beta;              % Flank width factor for second stage
    gearbox.factors.K_h_alpha_2 = K_h_alpha;            % Boundary condition for pitting load capacity for second stage
    gearbox.factors.K_f_beta_2 = K_f_beta;              % Width factor for root load capacity for second stage
    gearbox.factors.K_f_alpha_2 = K_f_alpha;            % Boundary condition for tooth root load capacity for second stage
    gearbox.factors.Y_epsilon_2 = Y_epsilon;            % Overlap factor for tooth root load capacity for second stage
    gearbox.factors.Z_epsilon_2 = Z_epsilon;            % Overlap factor for pitting load capacity for second stage
    gearbox.factors.epsilon_beta_2 = epsilon_beta;      % Overlap factor for second stage
    gearbox.factors.epsilon_alpha_2 = epsilon_alpha;    % Transverse contact ratio of second stage
    gearbox.factors.F_n_t_2 = F_n_t;                    % Circumferential tooth force of second stage [N]
end

end
