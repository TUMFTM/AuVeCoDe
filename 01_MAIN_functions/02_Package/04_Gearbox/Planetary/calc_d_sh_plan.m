function gearbox = calc_d_sh_plan(shaft, gearbox, d_ctlg)
%% 1) Description:
% This function computes the inner diameter of hollow shafts and ensures
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
T_max = gearbox.Input.T_max*1000;           % Maximum torque of el. motor [Nmm]
S_Dt_min = gearbox.MinSafety.S_Dt_min;      % Minimum safety factor against torsional fatigue fracture (Stahl, p. 33) [-]
if (shaft==1)
    d_f1 = gearbox.gears.d_f1;              % Root diameter of wheel 1 [mm]
    m_n = gearbox.gears.m_n_1;              % Normal module of stage 1 [mm]
    d_sh_3 = gearbox.shafts.d_sh_3;         % Outer diameter of output shafts [mm]
else % if shaft==2
    i_1p1 = gearbox.results.i_1p1;
    T_max = 1/3*T_max*i_1p1;                % Maximum torque on planet [Nmm]
    d_f1 = gearbox.gears.d_fp2;             % Root diameter of planet [mm]
    m_n = gearbox.gears.m_n_2;              % Normal module of stage 2 [mm]
end

m = length(d_ctlg);

%% 3) Calculation of inner diameter of hollow shaft:
% Selection of critical shaft diameter in mm
if (shaft==1)
    d_sh_min = repmat(d_f1,m,1);
else
    d_sh_rel = [d_ctlg, repmat(d_f1,m,1)];
    d_sh_min = min(d_sh_rel,[],2);
end

% Resistance to alternating stress for standard dimensions in N/mm^2 (Stahl, p. 19)
tau_wsn = 0.58*0.4*R_mn;                    % Factors (Stahl, Table 12, p. 20)
% Technological size factor K_dm (Stahl, p. 10)
K_dm = (1-(0.7686*0.5*log10(d_sh_min/7.5)))/(1-(0.7686*0.5*log10(11/7.5))); 
% Resistance to alternating stress in part in N/mm^2 (Stahl, p. 20)
tau_ws = tau_wsn*K_dm;                      
% Neglect of notches for approximation (Zaehringer, p. 37)
tau_ak = tau_ws;

% Initialization of array with possible inner shaft diameters in mm for coaxial and parallel axles
% with empirical boundaries for shaft thickness and minimum bore diameter
if (shaft==1)
    % Minimum inner shaft diameter is determined by wheel 1
    if ((d_f1-6*m_n)>(d_sh_3+4))
        space = linspace(1,(d_sh_3+4)/(d_f1-6*m_n),10);
        d_inn_u = (d_sh_min==d_f1).*(d_f1-6*m_n)*space+(d_sh_min~=d_f1).*space;
    else
        % If minimum shaft diameter is wheel 1, minimun inner shaft
        % diameter is determined by coaxial output shafts
        d_inn_u = d_sh_3+4;
    end
else
    % Creation of array with possible inner diameters for each bearing diameter d_sh
    space = linspace(1,7.5/(d_f1-6*m_n),9);
    % Minimum inner shaft diameter is either determined by wheel 1 or smaller inner bearing diameter
    d_inn_u = (d_sh_min==d_f1).*(d_f1-6*m_n)*space+(d_sh_min~=d_f1).*(d_sh_min-7)*space;
    % Addition of column with zeros for shafts without hollow bores
    d_inn_u = [d_inn_u,zeros(m,1)];
end

% Torsional resistance of a ring surface area in mm^3 (Stahl, p. 137)
wt = pi*((d_sh_min.^4)-(d_inn_u.^4))./(16*d_sh_min);
% Occuring torsional stress in N/mm^2 (Stahl, p. 8)
tau_occ = T_max./wt;
% Safety factor against torsional fatigue (Stahl, p. 33)
S_dt = tau_ak./tau_occ;

% Selection of inner shaft diameter with minimum shaft thickness that fits
% safety criterion of S_dt>2 (Zaehringer, p. 38)
S_dt(S_dt<S_Dt_min) = NaN;

% If no value fulfills safety criterion, the outer diameter has to increase --> parameter int stays 0
if (isnan(S_dt))            
    plot_display('Shaft does not support torsional load');
end
d_inn = (S_dt>=S_Dt_min).*d_inn_u;
d_inn = max(d_inn,[],2);
S_dt = min(S_dt,[],2);
sh_mat = [d_ctlg, S_dt, d_inn];


%% 4) Output assignment:
if (shaft==1)
    gearbox.shafts.shaft_1 = sh_mat;        % Matrix of possible sun shaft configuarations
elseif (shaft==2)
    gearbox.shafts.shaft_p = sh_mat;        % Matrix of possible planet shaft configuarations
end

end
    