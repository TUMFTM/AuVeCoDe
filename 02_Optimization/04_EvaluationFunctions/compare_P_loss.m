function [DriveStruct] = compare_P_loss(varargin)
    %% Description
    % Function to compare loss power of motor from multiple vehicle concepts
    
    % Author: Fabian Liemawan Adji
    % Date: July 2021
    
    %% Input
    % varargin: struct containing multiple concepts as fields
    %           -e.g. veh.K3, veh.K5
    
    %% Output
    % DriveStruct: struct containing torque, power, and efficiency fields
    
    % define name of concepts    
    Name = ["K3", "K5"];
    
    % initialize Antrieb struct
    DriveStruct = struct;
    for i = 1:nargin
        % define Antrieb name
        DriveStruct(i).name = Name(i);
        switch string(varargin{i}.LDS.settings.drive)
            % case all wheel
            case "all_wheel"
                %% Torque
                % get torques values
                DriveStruct(i).torque.front_motor = varargin{i}.LDS.sim_cons.T_mot_f;
                DriveStruct(i).torque.rear_motor = varargin{i}.LDS.sim_cons.T_mot_r;
                DriveStruct(i).torque.wheel = varargin{i}.LDS.sim_cons.T_wheels;
                DriveStruct(i).torque.front_dist = varargin{i}.LDS.sim_cons.torque_distribution.trq_front;  
                DriveStruct(i).torque.rear_dist = varargin{i}.LDS.sim_cons.torque_distribution.trq_rear;

                %% motor rev
                % get engine speed values
                DriveStruct(i).rev.front_motor = varargin{i}.LDS.sim_cons.n_mot_f;
                DriveStruct(i).rev.rear_motor = varargin{i}.LDS.sim_cons.n_mot_r;
                DriveStruct(i).rev.wheel = varargin{i}.LDS.sim_cons.n_wheels;

                %% Power
                % calculate power for both motors and wheels
                DriveStruct(i).power.front_motor = calc_P(DriveStruct(i).torque.front_motor, ...
                    DriveStruct(i).rev.front_motor);

                DriveStruct(i).power.rear_motor = calc_P(DriveStruct(i).torque.rear_motor, ...
                    DriveStruct(i).rev.rear_motor);

                DriveStruct(i).power.wheel = calc_P(DriveStruct(i).torque.wheel, ...
                    DriveStruct(i).rev.wheel);

                %% Loss in Motor
                % calculate power loss in motor
                % power loss = power from motor - power needed in motor
                % power needed in motor = power in motor to supply needed
                % power at wheel
                
                % calculate torque needed in motors
                % T = T_wheel * torque_split / (eta*i_gearbox)
                DriveStruct(i).torque.front_motor_need = DriveStruct(i).torque.wheel .* ...
                    (DriveStruct(i).torque.front_dist ./ 100) ...
                    ./ varargin{i}.LDS.GEARBOX{1}.eta...
                    ./ varargin{i}.LDS.GEARBOX{1, 1}.i_gearbox;

                DriveStruct(i).torque.rear_motor_need = DriveStruct(i).torque.wheel .* ...
                    (DriveStruct(i).torque.rear_dist./ 100) ...
                    ./ varargin{i}.LDS.GEARBOX{2}.eta...
                    ./ varargin{i}.LDS.GEARBOX{1, 2}.i_gearbox;

                % calculate power needed in motor
                DriveStruct(i).power.front_motor_need =calc_P(DriveStruct(i).torque.front_motor_need, ...
                    DriveStruct(i).rev.front_motor);
                DriveStruct(i).power.rear_motor_need =calc_P(DriveStruct(i).torque.rear_motor_need, ...
                    DriveStruct(i).rev.rear_motor);
                
                % power loss = power from motor - power needed in motor
                 DriveStruct(i).loss.front_motor = DriveStruct(i).power.front_motor -...
                    DriveStruct(i).power.front_motor_need;
                DriveStruct(i).loss.rear_motor = DriveStruct(i).power.rear_motor -...
                    DriveStruct(i).power.rear_motor_need;

                %% Loss after gearbox
                % calculate power loss from motor to after gearbox &
                % differential
                % power loss = power loss motor + power loss gearbox + diff
                % power needed after gearbox = power after gearbox + diff to 
                % supply needed power at wheel
                
                % torque needed after gearbox
                % T = T_wheel * torque_split;
                DriveStruct(i).torque.front_after_gearbox_need = DriveStruct(i).torque.wheel .* ...
                    (DriveStruct(i).torque.front_dist ./ 100);
                DriveStruct(i).torque.rear_after_gearbox_need = DriveStruct(i).torque.wheel .* ...
                    (DriveStruct(i).torque.rear_dist ./ 100);

                % torque from motor after gearbox + diff
                % T = T_motor * i_gearbox
                DriveStruct(i).torque.front_after_gearbox = DriveStruct(i).torque.front_motor .* ...
                    varargin{i}.LDS.GEARBOX{1, 1}.i_gearbox;
                DriveStruct(i).torque.rear_after_gearbox = DriveStruct(i).torque.rear_motor .* ...
                    varargin{i}.LDS.GEARBOX{1, 2}.i_gearbox;

                % calculate power after gearbox + diff from motor
                DriveStruct(i).power.after_gearbox_front = calc_P(DriveStruct(i).torque.front_after_gearbox,...
                    DriveStruct(i).rev.wheel);
                DriveStruct(i).power.after_gearbox_rear = calc_P(DriveStruct(i).torque.rear_after_gearbox,...
                    DriveStruct(i).rev.wheel);

                % calculate power needed after gearbox + diff
                 DriveStruct(i).power.after_gearbox_front_need = calc_P(DriveStruct(i).torque.front_after_gearbox_need,...
                    DriveStruct(i).rev.wheel);
                DriveStruct(i).power.after_gearbox_rear_need = calc_P(DriveStruct(i).torque.rear_after_gearbox_need,...
                    DriveStruct(i).rev.wheel);

                % power loss = power loss motor + power loss (gearbox+diff)
                DriveStruct(i).loss.gearbox_front = DriveStruct(i).power.after_gearbox_front - ...
                    DriveStruct(i).power.after_gearbox_front_need;
                DriveStruct(i).loss.gearbox_rear = DriveStruct(i).power.after_gearbox_rear - ...
                    DriveStruct(i).power.after_gearbox_rear_need;
            case "front_wheel"
                %% Torque
                % get torques values
                DriveStruct(i).torque.front_motor = varargin{i}.LDS.sim_cons.T_mot_f;
                DriveStruct(i).torque.wheel = varargin{i}.LDS.sim_cons.T_wheels;
                DriveStruct(i).torque.front_dist = 100;  

                %% motor rev
                % get engine speed values
                DriveStruct(i).rev.front_motor = varargin{i}.LDS.sim_cons.n_mot_f;
                DriveStruct(i).rev.rear_motor = 0;
                DriveStruct(i).rev.wheel = varargin{i}.LDS.sim_cons.n_wheels;

                %% Power
                % calculate power for front motor and wheels
                DriveStruct(i).power.front_motor = calc_P(DriveStruct(i).torque.front_motor, ...
                    DriveStruct(i).rev.front_motor);
                DriveStruct(i).power.wheel = calc_P(DriveStruct(i).torque.wheel, ...
                    DriveStruct(i).rev.wheel);

                %% Loss in Motor
                % calculate power loss in motor
                % power loss = power from motor - power needed in motor
                % power needed in motor = power in motor to supply needed
                % power at wheel
                
                % calculate torque needed in motors
                % T = T_wheel * torque_split / (eta*i_gearbox)
                DriveStruct(i).torque.front_motor_need = DriveStruct(i).torque.wheel .* ...
                    (DriveStruct(i).torque.front_dist ./ 100) ...
                    ./ varargin{i}.LDS.GEARBOX{1}.eta...
                    ./ varargin{i}.LDS.GEARBOX{1, 1}.i_gearbox;

                % calculate power needed in motor
                DriveStruct(i).power.front_motor_need =calc_P(DriveStruct(i).torque.front_motor_need, ...
                    DriveStruct(i).rev.front_motor);
                
                % power loss = power from motor - power needed in motor
                DriveStruct(i).loss.front_motor = DriveStruct(i).power.front_motor -...
                    DriveStruct(i).power.front_motor_need;
                % set value for rear motor to zero
                DriveStruct(i).loss.rear_motor = 0;

               %% Loss after gearbox
                % calculate power loss from motor to after gearbox &
                % differential
                % power loss = power loss motor + power loss gearbox + diff
                % power needed after gearbox = power after gearbox + diff to 
                % supply needed power at wheel
                
                % torque needed after gearbox + diff
                % T = T_wheel * torque_split;
                DriveStruct(i).torque.front_after_gearbox_need = DriveStruct(i).torque.wheel .* ...
                    (DriveStruct(i).torque.front_dist ./ 100);

                DriveStruct(i).torque.front_after_gearbox = DriveStruct(i).torque.front_motor .* ...
                    varargin{i}.LDS.GEARBOX{1, 1}.i_gearbox;

                % calculate power after gearbox + diff from motor
                DriveStruct(i).power.after_gearbox_front = calc_P(DriveStruct(i).torque.front_after_gearbox,...
                    DriveStruct(i).rev.wheel);


                % calculate power needed after gearbox + diff
                DriveStruct(i).power.after_gearbox_front_need = calc_P(DriveStruct(i).torque.front_after_gearbox_need,...
                    DriveStruct(i).rev.wheel);

                % power loss = power loss motor + power loss (gearbox+diff)
                DriveStruct(i).loss.gearbox_front = DriveStruct(i).power.after_gearbox_front - ...
                    DriveStruct(i).power.after_gearbox_front_need;
                % set value for rear motor to zero
                DriveStruct(i).loss.gearbox_rear = 0;
            case "rear_wheel"
                %% Torque
                % get torques values
                DriveStruct(i).torque.rear_motor = varargin{i}.LDS.sim_cons.T_mot_r;
                DriveStruct(i).torque.wheel = varargin{i}.LDS.sim_cons.T_wheels;
                DriveStruct(i).torque.rear_dist = 100;

                %% motor rev
                % get engine speed values
                DriveStruct(i).rev.rear_motor = varargin{i}.LDS.sim_cons.n_mot_r;
                DriveStruct(i).rev.wheel = varargin{i}.LDS.sim_cons.n_wheels;

                %% Power
                % calculate power for rear motor and wheels
                DriveStruct(i).power.rear_motor = calc_P(DriveStruct(i).torque.rear_motor, ...
                    DriveStruct(i).rev.rear_motor);

                DriveStruct(i).power.wheel = calc_P(DriveStruct(i).torque.wheel, ...
                    DriveStruct(i).rev.wheel);

                %% Loss in Motor
                % calculate power loss in motor
                % power loss = power from motor - power needed in motor
                % power needed in motor = power in motor to supply needed
                % power at wheel
                
                % calculate torque needed in motors
                % T = T_wheel * torque_split / (eta*i_gearbox)
                DriveStruct(i).torque.rear_motor_need = DriveStruct(i).torque.wheel .* ...
                    (DriveStruct(i).torque.rear_dist./ 100) ...
                    ./ varargin{i}.LDS.GEARBOX{2}.eta...
                    ./ varargin{i}.LDS.GEARBOX{1, 2}.i_gearbox;

                % calculate power needed in rear motor
                DriveStruct(i).power.rear_motor_need =calc_P(DriveStruct(i).torque.rear_motor_need, ...
                    DriveStruct(i).rev.rear_motor);
                
                % power loss = power from motor - power needed in motor
                DriveStruct(i).loss.rear_motor = DriveStruct(i).power.rear_motor -...
                    DriveStruct(i).power.rear_motor_need;
                % set value for front motor to zero
                DriveStruct(i).loss.front_motor = 0;

                 %% Loss after gearbox
                % calculate power loss from motor to after gearbox &
                % differential
                % power loss = power loss motor + power loss gearbox+diff
                % power needed after gearbox = power after gearbox+diff to 
                % supply needed power at wheel
                
                % torque needed after gearbox + diff
                % T = T_wheel * torque_split;
                DriveStruct(i).torque.rear_after_gearbox_need = DriveStruct(i).torque.wheel .* ...
                    (DriveStruct(i).torque.rear_dist ./ 100);

                % torque from motor after gearbox + diff
                % T = T_motor * i_gearbox
                DriveStruct(i).torque.rear_after_gearbox = DriveStruct(i).torque.rear_motor .* ...
                    varargin{i}.LDS.GEARBOX{1, 2}.i_gearbox;

                % calculate power after gearbox + diff from motor
                DriveStruct(i).power.after_gearbox_rear = calc_P(DriveStruct(i).torque.rear_after_gearbox,...
                    DriveStruct(i).rev.wheel);

                % calculate power needed after gearbox + diff
                DriveStruct(i).power.after_gearbox_rear_need = calc_P(DriveStruct(i).torque.rear_after_gearbox_need,...
                    DriveStruct(i).rev.wheel);

                % power loss = power loss motor + power loss (gearbox+diff)
                DriveStruct(i).loss.gearbox_rear = DriveStruct(i).power.after_gearbox_rear - ...
                    DriveStruct(i).power.after_gearbox_rear_need;
                % set value for front motor to 0
                DriveStruct(i).loss.gearbox_front = 0;
        end
    end
end
    
function [P] = calc_P (T,n)
    %% Description
    % Function to calculate Power based on torque and engine speed
    
    %% Input:
    % T: torque in Nm
    % n: engine speed in 1/min
    
    %% Output:
    % P: power in kW
    P = T .* n .* (2 * pi / (60 * 1000)); % in kW
end