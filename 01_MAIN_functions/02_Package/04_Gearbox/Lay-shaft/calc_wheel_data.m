function gearbox = calc_wheel_data (shaft, gearbox)
%% 1) Description
% This function computes the number of teeth according to the chosen 
% transmission ratios and calculates the wheels' dimensions.

%% 1.1) Authors
% Peter Koehler
% Date: 24.08.2021

%% 1.2) Sources:
% [1] Semesterarbeit Koehler: "Semi-physikalische Modellierung von Antriebsstrangkomponenten fuer Elektrofahrzeuge", Institute of Automotive Technology, TUM, Munich, 2020
% [2] Master Thesis Koehler: "Semi-physikalische Modellierung von Antriebsstrangkomponenten fuer Elektrofahrzeuge", Institute of Automotive Technology, TUM, Munich, 2021
% [3] Decker and Kabus: "Maschinenelemente - Formeln", Carl Hanser Verlag, 2011, ISBN: 978-3-446-42990-1
% [4] Bachelor Thesis Zaehringer: "Erstellung eines analytischen Ersatzmodells fuer Getriebe von Elektrofahrzeugen", Institute of Automotive Technology, TUM, Munich, 2018
% [5] Stahl: "Formelsammlung zur Vorlesung Maschinenelemente", Wintersemester 2015/16

%% 2) Initialization required variables:
epsilon_beta = gearbox.GearingConst.epsilon_beta;   % Desired overlap ratio [-]
alpha_n = gearbox.GearingConst.alpha_n;             % Normal pressure angle [rad] --> 20deg
if (shaft==1)
    d_1 = gearbox.gears_12.d_1;                 % Initial pitch diameter of wheel 1 [mm]
    i_12 = gearbox.results.i_12;                % Initial transmission ratio of first stage [-]
    m_n = gearbox.gears_12.m_n_1;               % Normal module of first stage [mm]
    b = gearbox.gears_12.b_1;                   % Width of first stage [mm]
else
    d_1 = gearbox.gears_34.d_3;                 % Initial pitch diameter of wheel 3 [mm]
    i_12 = gearbox.results.i_34;                % Initial transmission ratio of second stage [-]
    m_n = gearbox.gears_34.m_n_3;               % Normal module of second stage [mm]
    b = gearbox.gears_34.b_3;                   % Width of second stage [mm]
end

%% 3) Calculation Wheel dimensions:
% Calculation of helix angle of the stage in rad (Decker, p. 136)
beta = asin((epsilon_beta*m_n*pi)/b);
if (beta>(pi/6))
    b = 2*epsilon_beta*m_n*pi;
    beta = asin((epsilon_beta*m_n*pi)/b);
end

% Calculation of pressure angle in transverse section in rad (Decker, p. 132)
alpha_t = atan(tan(alpha_n)/cos(beta));

% Definition of root clearance (Decker, p. 133)
c = 0.25*m_n;

% Calculation of the number of teeth for automatic selection (Decker, p.135)
if ~gearbox.Input.manTR
    z_1 = d_1*cos(beta)/m_n;
    if (shaft==1)
        z_1 = round(z_1);
    else
        z_1 = ceil(z_1);
    end
    
    % Minimum number of teeth (Decker, p. 135)
    if (z_1<15)
        z_1 = 15;
    end
    
    % Calculation of pitch diameters with integer tooth number in mm (Decker, p. 132 and Zaehringer, p. 25)
    d_1 = z_1*m_n/cos(beta);
    d_2 = i_12*d_1;
    % Calculation of number of teeth of second wheel as for wheel 1
    z_2 = d_2*cos(beta)/m_n;
    if (shaft==1)
        z_2 = round(z_2);
    else
        z_2 = ceil(z_2);
    end

    % Check if number of teeth corresponds to transmission ratio, 3
    % alternative transmission variants are created
    i_12_1 = z_2/z_1;
    i_12_2 = (z_2+1)/z_1;
    i_12_3 = z_2/(z_1+1);
    % Calculation of errors between desired transmission ratio and
    % transmission ratios generated from number of teeth
    err_1 = abs(i_12-i_12_1);
    err_2 = abs(i_12-i_12_2);
    err_3 = abs(i_12-i_12_3);

    % Check that the number of teeth is not an integral multiple: if the number
    % of teeth of the wheels of a stage are intregral multiples, a penalty
    % is added so that another pair of number of teeth is chosen
    if (mod(z_2, z_1)==0)
        err_1 = err_1+1;
    elseif (mod(z_2+1, z_1)==0)
        err_2 = err_2+1;
    elseif (mod(z_2, z_1+1)==0)
        err_3 = err_3+1;
    end
    % Search for minimum error of desired to calculated transmission ratio
    err_arr = [err_1; err_2; err_3];
    opt = min(err_arr);
    % Definition of number of teeth and transmission ratio according to criteria
    if (opt==err_1)
        i_12 = i_12_1;
    elseif (opt==err_2)
        i_12 = i_12_2;
        z_2 = z_2+1;
    else
        i_12 = i_12_3;
        z_1 = z_1+1;
    end
else            % Switch for manual transmission ratios
    if (shaft==1)
        z_1 = gearbox.gears_12.z_1;
        z_2 = gearbox.gears_12.z_2;
    else
        z_1 = gearbox.gears_34.z_3;
        z_2 = gearbox.gears_34.z_4;
    end
end

% Calculation of new gear dimensions with updated transmission ratios
if (strcmpi(gearbox.Input.axles, 'coaxial') && shaft==2)
    d_1 = (2*gearbox.results.a_12)/(1+i_12);    % New pitch diameter of wheel 3 according to updated transmission ratio in mm
    d_2 = (2*gearbox.results.a_12)-d_1;         % New pitch diameter of wheel 4 with fixed axle distance in mm
    a_12 = gearbox.results.a_12;                % Distance between shafts 2&3 stays the same
    z_1 = round(d_1*cos(beta)/m_n);             % New number of teeth of wheel 3
    z_2 = round(d_2*cos(beta)/m_n);             % New number of teeth of wheel 4
    i_12 = z_2/z_1;                             % New transmission ratio of second stage
else
    d_1 = z_1*m_n/cos(beta);                    % New pitch diameter of pinion in mm (Decker, p. 132)
    d_2 = i_12*d_1;                             % New pitch diameter of wheel in mm (Zaehringer, p. 25)
    a_12 = (d_1+d_2)/2;
end

% Calculation of outside, root and base diameters of pinion and wheel (Stahl, p. 106)
d_a1 = d_1+(2*m_n);
d_a2 = d_2+(2*m_n);

d_f1 = d_1-(2*(m_n+c));
d_f2 = d_2-(2*(m_n+c));

d_b1 = d_1*cos(alpha_t);
d_b2 = d_2*cos(alpha_t);

%% 4) Output assignment:
if (shaft==1)
    gearbox.gears_12.beta_1 = beta;         % Helix angle of first stage in rad
    gearbox.gears_12.alpha_t_1 = alpha_t;   % Pressure angle in transverse section of first stage in rad
    gearbox.gears_12.d_1 = d_1;             % Pitch diameter of wheel 1 in mm
    gearbox.gears_12.d_a1 = d_a1;           % Outside diameter of wheel 1 in mm
    gearbox.gears_12.d_f1 = d_f1;           % Root diameter of wheel 1 in mm
    gearbox.gears_12.d_b1 = d_b1;           % Base diameter of wheel 1 in mm
    gearbox.gears_12.z_1 = z_1;             % Number of teeth of wheel 1
    gearbox.gears_12.d_2 = d_2;             % Pitch diameter of wheel 2 in mm
    gearbox.gears_12.d_a2 = d_a2;           % Outside diameter of wheel 2 in mm
    gearbox.gears_12.d_f2 = d_f2;           % Root diameter of wheel 2 in mm
    gearbox.gears_12.d_b2 = d_b2;           % Base diameter of wheel 2 in mm
    gearbox.gears_12.z_2 = z_2;             % Number of teeth of wheel 2
    gearbox.gears_12.b_1 = b;               % Width of first stage gears in mm
    gearbox.results.a_12 = a_12;            % Updated distance between shafts 1&2 in mm
    gearbox.results.i_12 = i_12;            % updated transmission ratio of first stage
else
    gearbox.gears_34.beta_2 = beta;         % Helix angle of second stage in rad
    gearbox.gears_34.alpha_t_2 = alpha_t;   % Pressure angle in transverse section of second stage in rad
    gearbox.gears_34.d_3 = d_1;             % Pitch diameter of wheel 3 in mm
    gearbox.gears_34.d_a3 = d_a1;           % Outside diameter of wheel 3 in mm
    gearbox.gears_34.d_f3 = d_f1;           % Root diameter of wheel 3 in mm
    gearbox.gears_34.d_b3 = d_b1;           % Base diameter of wheel 3 in mm
    gearbox.gears_34.z_3 = z_1;             % Number of teeth of wheel 3
    gearbox.gears_34.d_4 = d_2;             % Pitch diameter of wheel 4 in mm
    gearbox.gears_34.d_a4 = d_a2;           % Outside diameter of wheel 4 in mm
    gearbox.gears_34.d_f4 = d_f2;           % Root diameter of wheel 4 in mm
    gearbox.gears_34.d_b4 = d_b2;           % Base diameter of wheel 4 in mm
    gearbox.gears_34.z_4 = z_2;             % Number of teeth of wheel 4
    gearbox.gears_34.b_3 = b;               % Width of second stage gears in mm
    gearbox.results.a_23 = a_12;            % Updated distance between shafts 2&3 in mm
    gearbox.results.i_34 = i_12;            % Updated transmission ratio of second stage
end

end
