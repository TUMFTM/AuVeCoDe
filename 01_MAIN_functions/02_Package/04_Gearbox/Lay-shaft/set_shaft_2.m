function gearbox = set_shaft_2(gearbox)
%% 1) Description:
% This function computes the second shaft of the lay-shaft gearbox.

%% 1.1) Authors
% Peter Koehler
% Date: 24.08.2021

%% 1.2) Sources:
% [1] Master Thesis Koehler: "Semi-physikalische Modellierung von Antriebsstrangkomponenten fuer Elektrofahrzeuge", Institute of Automotive Technology, TUM, Munich, 2021
% [2] Niemann and Winter: "Maschinenelemente - Band 2: Getriebe allgemein, Zahnradgetriebe", Springer Verlag, 2003, ISBN: 978-3-662-11874-0
% [3] Schaeffler: "Waelzlager", Bearing Catalogue, 2019
% [4] Semesterarbeit Koehler: "Semi-physikalische Modellierung von Antriebsstrangkomponenten fuer Elektrofahrzeuge", Institute of Automotive Technology, TUM, Munich, 2020

%% 2) Initialization of required values:
b_d_3 = gearbox.GearingConst.b_d_3;         % Ratio of width to pitch diameter of first wheel [-]
d_bevel = gearbox.GearingConst.d_bevel;     % Empirical ratio between smaller and larger bevel gears of the diffferential ([4], pp. 46) [-]
T_max = gearbox.Input.T_max;                % Maximum torque of el. motor [Nm]
S_H_min = gearbox.MinSafety.S_H_min;        % Minimum safety factor against flank break (Niemann and Winter, p. 344) [-]
S_F_min = gearbox.MinSafety.S_F_min;        % Minimum safety factor against root break (Niemann and Winter, p. 344) [-]
t_diffcage = gearbox.ConstDim.t_diffcage;   % Thickness of differential housing [mm]
i_12 = gearbox.results.i_12;                % Transmission ratio of first stage [-]
i_34 = gearbox.results.i_34;                % Transmission ratio of second stage [-]    
a_23 = gearbox.results.a_23;                % Distance between shafts 2&3 [mm]
d_a2 = gearbox.gears_12.d_a2;               % Outer diameter of wheel 2 [mm]
regr_d_bevel = gearbox.Regression.d_bevel.eq;           % Regression formula for the diameter of the larger differential gears in mm ([1], p. 46)

% Loading of the bearing catalogue
dgb = gearbox.BearCtlg.ball;                % Loading of Schaeffler ball bearing catalogue (starting on p. 246)
d_ctlg = dgb.d;                             % List of inner bearing diameters [mm]
d_A_ctlg = dgb.d_A;                         % List of outer bearing diameters [mm]
b_ctlg = dgb.b;                             % List of bearing widths [mm]
C_dyn_ctlg = dgb.C_dyn;                     % List of dynamic load ratings [N] 
m_ctlg = dgb.m;                             % List of bearing masses [kg]
f0_ctlg = dgb.f0;                           % List of bearing factors [-]
C_stat_ctlg = dgb.C_stat;                   % List of static load ratings [N]
d_1_ctlg = dgb.d_1;                         % List of diameters of inner bearing ring [mm]
name_ctlg = dgb.Name;                       % List of bearing codes

shaft = 2;                                  % Switch for sub-functions
iteration = 0;                              % Initialization of iteration counter

%% 3) Verification of space for differential cage:
if (gearbox.Input.num_EM==1)
    % Calculate dimensions of the differential gears and the differential cage ([4], pp. 46)
    d_bev_l = regr_d_bevel(T_max*i_12*i_34);                    % Empirical diameter of larger bevel gears in mm
    d_bev_s = d_bevel*d_bev_l;                                  % Empirical diameter of smaller bevel gears in mm  
    d_diffcage = d_bev_l+(sqrt(2)-1)*d_bev_s+2*t_diffcage;      % Diameter of differential cage in mm
    
    % Check differential orientation ([1], pp. 51)
    if (strcmpi(gearbox.Input.axles, 'parallel'))
        % Check if there is a given differential orientation
        if isempty(gearbox.diff.diff_orientation)
            % Increase of stage 2 wheel diameters if differential does not fit (applies only for parallel axles)
            if ((d_diffcage/2+d_a2/2+5)<=a_23)
                gearbox.diff.diff_orientation = {'in'};
            else
                gearbox.diff.diff_orientation = {'out'};
            end
        else
            % Check if the differential cage and wheel 2 fit within the second stage axle distance 
            if (strcmpi(gearbox.diff.diff_orientation,'in') && ((d_diffcage/2+d_a2/2+5)<a_23))
                % Axle distance of second stage is increased to fit differenial orientation
                a_23 = d_diffcage/2+d_a2/2+5;
                % Update of pitch diameters of the second stage with new axle distance
                d_3 = (2*a_23)/(1+i_34);
                d_4 = i_34*d_3;
                % Storing of updated pitch diameters and distance between shafts
                gearbox.results.a_23 = a_23;
                gearbox.gears_34.d_3 = d_3;
                gearbox.gears_34.d_4 = d_4;
            else %An 'in' position was chosen but the differential does not fit // the chosen option is 'out' -> set 'out'
                gearbox.diff.diff_orientation = {'out'};
            end
        end
    else
        gearbox.diff.diff_orientation = 0;
    end
else
    gearbox.diff.diff_orientation = 0;
end

%% 4) Calculation of shaft 2 data:
while 1
    % Calculation of actual number of teeth and corresponding wheel dimensions
    gearbox = calc_wheel_data(shaft, gearbox);

    % Bearing hollow shaft calculation for shaft 2 according to input load and desired lifetime
    gearbox = set_bearings_shaft_2(shaft, gearbox, d_ctlg, d_A_ctlg, b_ctlg, C_dyn_ctlg, m_ctlg, f0_ctlg, C_stat_ctlg, d_1_ctlg, name_ctlg);

    % Determination of calculation factors needed for safety coefficients
    gearbox = calc_factors(shaft, gearbox);

    % Calculation of safety factors for critical pinion on shaft 2
    gearbox = calc_S_F(shaft, gearbox);
    gearbox = calc_S_H(shaft, gearbox);

    % Verification of safety factors, increase of gear width and diameter according to b/d-ratio
    if (gearbox.results.S_H_34<S_H_min || gearbox.results.S_F_34<S_F_min)
        %gearbox.gears_34.b_3 = gearbox.gears_34.b_3+0.25;
        if (strcmpi(gearbox.Input.axles,'coaxial')==0)
            gearbox.gears_34.d_3 = gearbox.gears_34.d_3+0.25/b_d_3;
        end
        iteration = iteration+1;
    end
    
    % Iterative decrease of normal module if safety factors are sufficient
    cond_2 = 1;
    if (gearbox.results.S_H_34>(S_H_min+0.2) && gearbox.results.S_F_34>(S_F_min+0.2) && gearbox.gears_34.m_n_3>gearbox.GearingConst.m_n_3_min)
        gearbox.gears_34.m_n_3 = gearbox.gears_34.m_n_3-0.2;
        cond_2 = 0;
    end

    % Condition for end of loop: safety factors and overlap ratio sufficient or iterations>150
    if (gearbox.results.S_H_34<=5 || gearbox.results.S_F_34<=5 || iteration>100)
        if (gearbox.results.S_H_34>=S_H_min && gearbox.results.S_F_34>=S_F_min)
            cond_1 = 1;
        elseif (iteration>150)
            cond_1 = 1;
        else
        cond_1 = 0;
        end
    else
        cond_1 = 0;
    end

    % End condition for the calculation loop
    if (cond_1 && cond_2)
        break;
    end

end

%% Output assignment:
if (gearbox.Input.num_EM==1)
    gearbox.diff.d_bev_l = d_bev_l;                 % Diameter of larger differential bevel gears in mm
    gearbox.diff.d_bev_s = d_bev_s;                 % Diameter of smaller differential bevel gears in mm
    gearbox.diff.d_diffcage = d_diffcage;           % Diameter of differential cage in mm
end
gearbox.factors.it_sh_2 = iteration;                % Number of iterations of shaft 2
end
