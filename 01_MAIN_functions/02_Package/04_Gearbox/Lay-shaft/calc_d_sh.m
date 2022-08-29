function gearbox = calc_d_sh(shaft, gearbox, d_ctlg)
%% 1) Description:
% This function computes the inner diameter of the hollow shaft and ensures
% sufficient torsional strength according to Stahl.

%% 1.1) Authors
% Peter Koehler
% Date: 24.08.2021

%% 1.2) Sources:
% [1] Master Thesis Koehler: "Semi-physikalische Modellierung von Antriebsstrangkomponenten fuer Elektrofahrzeuge", Institute of Automotive Technology, TUM, Munich, 2021
% [2] Stahl: "Formelsammlung zur Vorlesung Maschinenelemente", Wintersemester 2015/16
% [3] Bachelor Thesis Zaehringer: "Erstellung eines analytischen Ersatzmodells fuer Getriebe von Elektrofahrzeugen", Institute of Automotive Technology, TUM, Munich, 2018

%% 2) Initialization of required values:
R_mn = gearbox.MatProp.R_mn;                % Yield strength 16MnCr5 (Stahl, p. 135) [N/mm^2]
T_max = gearbox.Input.T_max;                % Maximum torque of el. motor [Nm]
i_12 = gearbox.results.i_12;                % Transmission ratio of first stage [-]
S_Dt_min = gearbox.MinSafety.S_Dt_min;      % Minimum safety factor against torsional fatigue fracture (Stahl, p. 33) [-]
if (shaft==1)
    T_rel = T_max*1000;                     % Maximum torque of shaft 1 [Nmm]
    d_f1 = gearbox.gears_12.d_f1;           % Root diameter of wheel 1 [mm]
    m_n = gearbox.gears_12.m_n_1;           % Normal module of stage 1 [mm]
elseif shaft==2
    T_rel = T_max*1000*i_12;                % Maximum torque of shaft 2 [Nmm]
    d_f1 = gearbox.gears_34.d_f3;           % Root diameter of wheel 3 [mm]
    m_n = gearbox.gears_34.m_n_3;           % Normal module of stage 2 [mm]
end
if (strcmpi(gearbox.Input.axles,'coaxial'))
    d_sh_3 = gearbox.shafts.d_sh_3;
end

m = length(d_ctlg);

%% 3) Calculation of inner diameter of hollow shaft:
% Selection of minimum shaft diameter in mm (either wheel 1 or bearing)
d_sh_rel = [d_ctlg, d_f1*ones(m,1)];
d_sh_min = min(d_sh_rel,[],2);
% Resistance to alternating stress for standard dimensions in N/mm^2 (Stahl, p. 19)
tau_wsn = 0.58*0.4*R_mn;                    % Factors (Stahl: Table 12, p. 20)
% Technological size factor K_dm according to Stahl, p. 10
K_dm = (1-(0.7686*0.5*log10(d_sh_min/7.5)))/(1-(0.7686*0.5*log10(11/7.5))); 
% Resistance to alternating stress in part in N/mm^2 (Stahl, p. 20)
tau_ws = tau_wsn*K_dm;                      
% Neglect of notches for approximation (Zaehringer, p. 37)
tau_ak = tau_ws;

% Initialization of array with possible inner shaft diameters in mm for coaxial and parallel axles
% with empirical boundaries for shaft thickness and minimum bore diameter
if (shaft==1 && strcmpi(gearbox.Input.axles, 'coaxial'))
    % Minimum inner shaft diameter is either determined by wheel 1 or
    % bearing with smaller inner diameter
    if ((d_f1-6*m_n)>(d_sh_3+4))
        d_inn_u = zeros(size(d_sh_min,1),10);
        d_inn_min = (d_sh_min==d_f1).*(d_f1-6*m_n)+(d_sh_min~=d_f1).*(d_sh_min-7);
        for i=1:size(d_sh_min,1)
            d_inn_u(i,:) = linspace(d_inn_min(i),(d_sh_3+4),10);
        end
    else
        % If minimum shaft diameter is wheel 1, minimun inner shaft
        % diameter is determined by coaxial output shafts
        d_inn_u = d_sh_3+4;
    end
else
    % Creation of array with possible inner diameters for each bearing
    % diameter d_sh
    space = linspace(1,7.5/(d_f1-6*m_n),9);
    % Minimum inner shaft diameter is either determined by wheel 1 or
    % smaller inner bearing diameter
    d_inn_u = (d_sh_min==d_f1).*(d_f1-6*m_n)*space+(d_sh_min~=d_f1).*(d_sh_min-7)*space;
    % Addition of column with zeros for shafts without hollow bores
    d_inn_u = [d_inn_u,zeros(m,1)];
end

d_sh_min = repmat(d_sh_min,1,10);
% Torsional resistance of a ring surface area in mm^3 (Stahl, p. 137)
wt = pi*((d_sh_min.^4)-(d_inn_u.^4))./(16*d_sh_min);
% Occuring torsional stress in N/mm^2 (Stahl, p. 8)
tau_occ = T_rel./wt;
% Safety factor against torsional fatigue (Stahl, p. 33)
S_dt = tau_ak./tau_occ;

% Selection of inner shaft diameter with minimum shaft thickness that fits
% safety criterion of S_dt>2 (Zaehringer, p. 38)
S_dt(S_dt<S_Dt_min) = NaN;

% If no value fulfills safety criterion, the outer diameter has to increase --> parameter int stays 0
if (isnan(S_dt))            
    gearbox.error = {'Shaft does not support torsional load'};
else
    d_inn = (S_dt>=S_Dt_min).*d_inn_u;
    d_inn = max(d_inn,[],2);
    S_dt = min(S_dt,[],2);
    sh_mat = [d_ctlg, S_dt, d_inn];
end

%% 4) Output assignment:
if (shaft==1)
    gearbox.shafts.shaft_1 = sh_mat;        % Matrix of possible first shaft configuarations
elseif (shaft==2)
    gearbox.shafts.shaft_2 = sh_mat;        % Matrix of possible second shaft configuarations
end

end
    