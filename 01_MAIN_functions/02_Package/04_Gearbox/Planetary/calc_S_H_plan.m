function gearbox = calc_S_H_plan(shaft, gearbox)
%% 1) Decription:
% This function computes the safety factor against flank break for 
% planetary gearboxes, according to Decker, starting p. 151

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
Z_E = gearbox.CalcFactors.Z_E;                      % Elasticity factor for material combination steel/steel (Decker, p. 151) [(N/mm^2)^1/2]
sigma_Hlim = gearbox.MatProp.sigma_Hlim;            % Fatigue strength against gear flank pressure 16MnCr5 (Zaehringer, p. xiv) [N/mm^2]

if (shaft==1)
    i_12 = gearbox.results.i_1p1;                   % Transmission ratio of first stage [-]
    beta = gearbox.gears.beta_1;                    % Helix angle of first stage [rad]
    beta_b = gearbox.gears.beta_b_1;                % Base helix angle of first stage [rad]
    alpha_t = gearbox.gears.alpha_t_1;              % Pressure angle in transverse section of first stage [rad]
    alpha_wt = gearbox.gears.alpha_wt_1;            % Working transverse pressure angle of first stage [rad]
    b = gearbox.gears.b_1;                          % Width of first wheel [mm]
    d_1 = gearbox.gears.d_1;                        % Pitch diameter of sun gear [mm]
    Z_epsilon = gearbox.factors.Z_epsilon_1;        % Overlap factor for pitting load capacity for first stage [-]
    K_v = gearbox.factors.K_v_1;                    % Dynamic factor for first stage [-]
    F_n_t = gearbox.factors.F_n_t_1;                % Circumferential tooth force of first stage [N]
    K_h_beta = gearbox.factors.K_h_beta_1;          % Flank width factor for first stage [-]
    K_h_alpha = gearbox.factors.K_h_alpha_1;        % Boundary condition for pitting load capacity for first stage [-]
else
    i_12 = abs(gearbox.results.i_p22);              % Transmission ratio of second stage [-]
    beta = gearbox.gears.beta_2;                    % Helix angle of second stage [rad]
    beta_b = gearbox.gears.beta_b_2;                % Base helix angle of second stage [rad]
    alpha_t = gearbox.gears.alpha_t_2;              % Pressure angle in transverse section of second stage [rad]
    alpha_wt = gearbox.gears.alpha_wt_2;            % Working transverse pressure angle of second stage [rad]
    b = gearbox.gears.b_2;                          % Width of planet 2 [mm]
    d_1 = gearbox.gears.d_p2;                       % Pitch diameter of planet 2 [mm]
    Z_epsilon = gearbox.factors.Z_epsilon_2;        % Overlap factor for pitting load capacity for second stage [-]
    K_v = gearbox.factors.K_v_2;                    % Dynamic factor for second stage [-]
    F_n_t = gearbox.factors.F_n_t_2;                % Circumferential tooth force of second stage [N]
    K_h_beta = gearbox.factors.K_h_beta_2;          % Flank width factor for second stage [-]
    K_h_alpha = gearbox.factors.K_h_alpha_2;        % Boundary condition for pitting load capacity for second stage [-]
end
    
%% 3) Calculation of safety factors against flank break:
% Zone coefficient (Decker, p. 151)
Z_h = sqrt((2*cos(beta_b))/((cos(alpha_t)^2)*tan(alpha_wt)));

% Helix angle factor (Decker, p. 151)
Z_beta = sqrt(cos(beta));

% Nominal tooth pressure (Decker, p.151)
sigma_h0 = sqrt(((i_12+1)/i_12)*(F_n_t/(d_1*b)))*Z_h*Z_E*Z_epsilon*Z_beta;

% Decisive tooth pressure (Decker, p.152) (Z_B = 1)
sigma_h = sigma_h0*sqrt(K_app*K_v*K_h_beta*K_h_alpha);

% Safety factor against flank break with Z_NT = 1.6 (Zaehringer, p. xv)
S_H = sigma_Hlim*1.6/sigma_h;

%% 4) Output assignment:
if (shaft==1)
    gearbox.results.S_H_1p1 = S_H;      % Safety factor against flank break for first stage
else
    gearbox.results.S_H_p22 = S_H;      % Safety factor against flank break for second stage
end

end
