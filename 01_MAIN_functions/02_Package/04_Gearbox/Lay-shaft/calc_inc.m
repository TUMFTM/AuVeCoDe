function gearbox = calc_inc(gearbox)
%% 1) Description
% This function computes the inclination of between the shafts of a parallel 
% lay-shaft gearbox according to the optimization goal (Height/Length/Mass/Manual).

%% 1.1) Authors
% Peter Koehler
% Date: 24.08.2021

%% 1.2) Sources:
% [1] Semesterarbeit Koehler: "Semi-physikalische Modellierung von Antriebsstrangkomponenten fuer Elektrofahrzeuge", Institute of Automotive Technology, TUM, Munich, 2020
% [2] Master Thesis Koehler: "Semi-physikalische Modellierung von Antriebsstrangkomponenten fuer Elektrofahrzeuge", Institute of Automotive Technology, TUM, Munich, 2021

%% 2) Initialization of required values:
d_4 = gearbox.gears_34.d_4;                 % Pitch diameter of wheel 4 [mm]
a_12 = gearbox.results.a_12;                % Distance between shafts 1&2 [mm]
a_23 = gearbox.results.a_23;                % Distance between shafts 2&3 [mm]
d_sh_3 = gearbox.shafts.d_sh_3;             % Outer diameter of output shaft [mm]
d_mot = gearbox.Input.d_mot;                % Diameter of el. motor [mm]
inc_init = gearbox.Input.inc_init;          % Angle between shafts with manual selection [deg]
opt_gears = gearbox.Input.opt_gears;        % Optimization goal
d_1 = gearbox.gears_12.d_1;                 % Pitch diameter of wheel 1 [mm]
d_2 = gearbox.gears_12.d_2;                 % Pitch diameter of wheel 2 [mm]

inc = 0;                                    % Initialization of inclination parameter
angle = (0:60)';                            % Initialization of possible angles as a vector [deg]

%% 3) Calculation of auxiliary parameters:
% Definition of characteristic dimensions 
theta = angle.*pi./180;                     % Possible angles between the connecting lines of axles 1-2 and 1-3 [rad]
zeta = asin((a_12/a_23).*sin(theta));       % Possible angles between the connecting lines of axles 3-1 and 3-2 [rad]
z_1 = (a_12.*cos(theta))+(a_23.*cos(zeta)); % Distance between shafts 1&3 in mm
z = z_1-(d_sh_3/2);                         % Distance between shaft 1 and outer limit of output shaft in mm
val = (a_12.*cos(theta))+(d_1/2);           % Distance between outer limit of wheel 1 and shaft 2 in x/y-plane

% Definition of general geometric criteria: output shaft fits next to el. motor, wheel 1 is outer limit, wheel 1&4 do not cross
test_matrix = [z>(d_mot/2+5), val>(d_2/2), z_1>((d_4/2)+(d_2/2))];
test_cond = sum(test_matrix,2);
poss_angle = find(test_cond==3);

%% 4) Switch for selected optimization
if isempty(poss_angle)   
    disp('The drive unit cannot be mounted on the axle, since the machine on the Input shaft has such a big diameter, that it collides with the output shaft!!');
    disp('The drive unit will be assembled anyway although this will lead to collisions!!!');
    inc=0;
    
    %Check if the actual number of Error stored in Errorlog
    id=numel(gearbox.Errorlog);
    gearbox.Errorlog{id+1}='Drive unit unfeasible due to collision';
    
else
    if strcmpi(opt_gears, 'Length')
        % Determine the lengths that fit geometric criteria
        z_poss = z_1(poss_angle(1):poss_angle(end)); 
        [~, inc_idx] = min(z_poss);
        inc = angle(inc_idx);
    elseif strcmpi(opt_gears, 'Manual')
        % Angle is set to desired value
        inc = inc_init;
    elseif strcmpi(opt_gears, 'Height')      
        % Determines minimum length for minimum height = d_4
        if ((d_2>d_4) || isempty(poss_angle))
            inc = 0;
        else
            h = d_2/2+a_12.*sin(theta); 
            h_poss = h(poss_angle(1):poss_angle(end));
            inc_idx = find(h_poss<d_4/2,1, 'last');
            inc = angle(inc_idx);
        end
    elseif strcmpi(opt_gears, 'Mass')
        % Determines angle for minimum housing mass
        angle = angle(poss_angle(1):poss_angle(end));
        for t=1:length(angle)                           % Calculate mass for all angles that fit geometric criteria
            gearbox.results.theta = theta(t);
            gearbox.results.zeta = zeta(t);
            if strcmpi(gearbox.Input.axles,'parallel')
                gearbox = calc_mass(gearbox);
            else
                gearbox = calc_mass_2sp(gearbox);
            end
            m_er(t) = gearbox.results.m_gearbox;        % Array with possible masses in kg
        end
        [~, inc_idx] = min(m_er);
        inc = angle(inc_idx);                           % Choose angle with minimum mass in kg
    end

end

%% 5) Output assignment
gearbox.results.inc = inc;          % Inclination of the gearbox on y-axis in deg

end
