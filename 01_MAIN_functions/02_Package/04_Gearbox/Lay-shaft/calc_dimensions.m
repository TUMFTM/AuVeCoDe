function gearbox = calc_dimensions(gearbox)
%% 1) Description:
% This function computes the main dimensions of the lay-shaft gearbox.

%% 1.1) Authors
% Peter Koehler
% Date: 24.08.2021

%% 1.2) Sources:
% [1]: Master Thesis Koehler: "Semi-physikalische Modellierung von Antriebsstrangkomponenten fuer Elektrofahrzeuge", Institute of Automotive Technology, TUM, Munich, 2021

%% 2) Initialization of required values:
b_gap = gearbox.ConstDim.b_gap;             % Gap between components on the shafts [mm]
l_park = gearbox.ConstDim.l_park;           % Additional space for parking lock mechanism (empirical, Koehler, p. 60) [mm]
b_park = gearbox.ConstDim.b_park;           % Width of parking gear [mm]
t_housing = gearbox.ConstDim.t_housing;     % Housing thickness [mm]
d_housing = gearbox.ConstDim.d_housing;     % Distance between gears and housing [mm]
d_flange = gearbox.ConstDim.d_flange;       % Additional space for screws on housing flange as average of database [mm]
b_seal = gearbox.ConstDim.b_seal;           % Width of seals on output shafts as average of database [mm]
d_a2 = gearbox.gears_12.d_a2;               % Outer diameter of wheel 2 [mm]
d_a3 = gearbox.gears_34.d_a3;               % Outer diameter of wheel 3 [mm]
d_a4 = gearbox.gears_34.d_a4;               % Outer diameter of wheel 4 [mm]
b_1 = gearbox.gears_12.b_1;                 % Width of first stage gears [mm]
b_3 = gearbox.gears_34.b_3;                 % Width of second stage gears [mm]
d_1_max = gearbox.shafts.d_1_max;           % Maximum diameter on shaft 1 [mm]
a_12 = gearbox.results.a_12;                % Distance between shafts 1&2 [mm]
a_23 = gearbox.results.a_23;                % Distance between shafts 1&2 [mm]
b_A = gearbox.bearings_1.b_A;               % Width of bearing A [mm]
b_B = gearbox.bearings_1.b_B;               % Width of bearing B [mm]
b_C = gearbox.bearings_2.b_C;               % Width of bearing C [mm]
b_D = gearbox.bearings_2.b_D;               % Width of bearing D [mm]
b_E = gearbox.bearings_3.b_E;               % Width of bearing E [mm]
b_F = gearbox.bearings_3.b_F;               % Width of bearing F [mm]

if (gearbox.Input.num_EM==1)
    b_diffcage = gearbox.diff.b_diffcage;               % Width of differential cage [mm]
    b_diff = gearbox.diff.b_diff;                       % Width of the complete differential shaft [mm]
    if strcmpi(gearbox.Input.axles,'parallel')
        diff_orientation = gearbox.diff.diff_orientation;   % Orientation of the differential
    end
elseif (gearbox.Input.num_EM==2)
    b_sh_3 = gearbox.shafts.b_sh_3;                     % Width of the third shaft [mm]
end

%% 3) Calculation of the gearbox dimensions:
%% 3.1) Calculation of the gearbox dimensions with parallel axles in mm:
if strcmpi(gearbox.Input.axles, 'parallel')
    % Determination of auxiliary dimensions for gearbox width (dividing plane on right side of wheel 1)
    t_1_left = b_seal+b_A+b_gap+b_park+b_gap+b_1;
    t_1_right = b_gap+b_B+t_housing;
    t_2_left = t_housing+b_C+b_gap+b_1;
    t_2_right = b_gap+b_3+b_gap+b_D+t_housing;
    if (gearbox.Input.num_EM==1)
        if strcmpi(diff_orientation,'in')
            t_3_left = b_seal+b_E+b_diffcage-b_gap;
            t_3_right = b_gap+b_3+b_gap+b_F+b_seal;
        else
            t_3_left = b_seal+b_F+b_gap+b_1;
            t_3_right = b_gap+b_3+b_diffcage+b_E+b_seal;
        end
    elseif (gearbox.Input.num_EM==2)
        t_3_left = b_seal+b_E;
        t_3_right = b_gap+b_3+b_gap+b_F+t_housing;
    end
    
    % Selection of the largest widths of the shafts left and right of the dividing plane
    t_left_arr = [t_1_left t_2_left t_3_left];
    t_left_max = max(t_left_arr);
    t_right_arr = [t_1_right t_2_right t_3_right];
    t_right_max = max(t_right_arr);
    
    % Total gearbox width in mm
    t_gearbox = t_left_max+t_right_max;
    % Storage of housing width in output struct
    gearbox.hous_dim.t_gearbox = t_gearbox;
    
    % Calculation of the inclination of the transmission shafts in deg
    gearbox = calc_inc(gearbox);

    % Calculation of gearbox length and height (veh. x & z) in mm
    theta = gearbox.results.inc*pi/180;         % Angle between connecting lines of shafts 1-2 and 1-3 in rad
    zeta = asin((a_12/a_23)*sin(theta));        % Angle between connecting lines of shafts 3-1 and 3-2 in rad
    a_13 = a_12*cos(theta)+(a_23*cos(zeta));    % Distance between shafts 1&3 in mm

    % Length of gearing without housing in mm
    l_gearing = (d_1_max/2)+a_13+(d_a4/2);
    
    % Calcultion of total gearbox length in mm
    % with length of parking lock mechanism - gearing - housing clearance - housing thickness - housing flange
    %l_gearbox = l_gearing+2*d_housing+2*t_housing+2*d_flange;
    l_gearbox = l_park+l_gearing+d_housing+t_housing+d_flange;
    
    % Determination of gearing height without housing in mm
    if (((d_a2/2)+(a_12*sin(theta))) > (d_a4/2))
        h_gearing = (d_a2/2)+(d_a4/2)+a_12*sin(theta);
    else
        h_gearing = d_a4;
    end
    
    % Addition of housing thickness for total gearbox height in mm
    h_gearbox = h_gearing+2*d_housing+2*t_housing+2*d_flange; 
    
    % Storage of angles between the axles in rad
    gearbox.results.theta = theta;
    gearbox.results.zeta = zeta;

%% 3.2) Calculation of the gearbox dimensions with coaxial in-/output shaft in mm:  
elseif strcmpi(gearbox.Input.axles, 'coaxial')
    % Calculation of the gearbox thickness (vehicle y-dim.) in mm
    t_1_left = b_seal+b_A+b_gap+b_1;
    t_1_right = b_gap+b_B+b_gap;
    t_2_left = t_housing+b_C+b_gap+b_1;
    if (gearbox.Input.num_EM==1)
        t_3 = b_diff+b_seal;
    elseif (gearbox.Input.num_EM==2)
        t_3 = b_sh_3+b_seal;
    end

    t_left_arr = [t_1_left t_2_left];
    t_left_max = max(t_left_arr);
    
    % Width of housing for mass calculations in mm
    t_gearbox = t_left_max+t_1_right+t_3;
    % Storage of housing width in output struct
    gearbox.hous_dim.t_gearbox = t_gearbox;
    
    % Calculation of gearbox length and height in mm with auxiliary parameters
    max_d = max([d_a2, d_a4]);
    
    l_gearing = a_12+d_a4/2;
    h_gearing = max_d;
    % length of the dimensional chain of wheel 2 with housing and parking gear
    l_chain_wheel_2 = d_a2/2+d_housing+t_housing+d_flange;
    l_chain_park = d_a3/2+l_park;
    
    % Calculation of gearbox length and height in mm
    l_gearbox = max(l_chain_park,l_chain_wheel_2)+l_gearing+d_housing+t_housing+d_flange;
    h_gearbox = h_gearing+2*t_housing+2*d_housing+2*d_flange;
end

%% 4) Output assignment:
gearbox.hous_dim.l_gearbox = l_gearbox;     % length of the gearbox housing according to dimensional chains in mm
gearbox.hous_dim.h_gearbox = h_gearbox;     % height of the gearbox housing according to dimensional chains in mm
gearbox.hous_dim.t_gearbox = t_gearbox;     % width of the gearbox housing according to dimensional chains in mm
gearbox.results.l_gearbox = l_gearbox;      % length of the gearbox housing according to dimensional chains in mm
gearbox.results.h_gearbox = h_gearbox;      % height of the gearbox housing according to dimensional chains in mm
gearbox.results.t_gearbox = t_gearbox;      % width of the gearbox housing according to dimensional chains in mm
if strcmpi(gearbox.Input.axles,'parallel')
    gearbox.results.a_13 = a_13;
end

end