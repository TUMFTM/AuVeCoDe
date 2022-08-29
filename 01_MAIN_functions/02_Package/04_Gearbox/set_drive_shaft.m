function drive_shaft = set_drive_shaft (gearbox,e_machine_length)
%% 1) Description:
% This function computes the mass of both drive shafts of one propelled axle 
% according to data from the Gearbox function.

%% 1.1) Authors
% Peter Koehler
% Date: 24.08.2021

%% 1.2) Sources:
% [1] Semesterarbeit Koehler: "Semi-physikalische Modellierung von Antriebsstrangkomponenten fuer Elektrofahrzeuge", Institute of Automotive Technology, TUM, Munich, 2020
% [2] Bachelor Thesis Zaehringer: "Erstellung eines analytischen Ersatzmodells fuer Getriebe von Elektrofahrzeugen", Institute of Automotive Technology, TUM, Munich, 2018
% [3] Master Thesis Koehler: "Semi-physikalische Modellierung von Antriebsstrangkomponenten fuer Elektrofahrzeuge", Institute of Automotive Technology, TUM, Munich, 2021

%% 2) Initialization of required values:
T_max = gearbox.Input.T_max;                    % Maximum torque of electric machine [Nm]
i_tot = gearbox.results.i_tot(1);               % Total transmission ratio of the gearbox [-]
rho_gear = gearbox.MatProp.rho_gear/10^9;       % Density 16MnCr5 (Zaehringer, p. 67) [kg/mm^3]
d_sh_3 = gearbox.shafts.d_sh_3;                 % Diameter of output shafts [mm]
regr_m_out = gearbox.Regression.m_out.eq;  % Regression formula for the mass of the output shafts (Koehler, p. 67) [kg]

%% 3) Calculation of drive shaft weight:
% Resulting maximum torque on each drive shaft in Nm
T_out = T_max*i_tot/2;

% Total mass of both drive shafts of one propelled axle calculated by a
% linear regression of gearbox database in kg (Koehler, p. 67)
m_regr = regr_m_out(T_out);

% If the number of EM on the axle is 2, the mass of the output shaft is
% reduced by the length of the EMs and gearboxes
if (gearbox.Input.num_EM==2)
    m_regr = m_regr-2*((0.5*d_sh_3)^2*pi*(e_machine_length+gearbox.results.t_gearbox)*rho_gear);
end

%% 4) Output assignment:
drive_shaft.mass_regression = m_regr;

end
