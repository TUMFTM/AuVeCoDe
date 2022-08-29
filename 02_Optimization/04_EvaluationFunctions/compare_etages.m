function [Compare_Drive] = compare_etages(DriveStruct)

    %% Description
    % Function to compare efficiency using calculated loss power in
    % compare_P_loss
    
    % Author: Fabian Liemawan Adji
    % Date: September 2021
    
    %% Input
    % DriveStruct: struct containing torque, power, and efficiency fields
    %           -ideally results of compare_P_loss
    
    %% Output
    % Compare_Drive: table showing comparison results of loss powers and
    % efficiencies
    
    % initialize table for CompareDrive
    Compare_Drive = table;
    % length(DriveStruct) corresponds to the concepts in DriveStruct
    for i = 1:length(DriveStruct)
        % get and set concept name
        Compare_Drive.Concept(i) = DriveStruct(i).name;
        % calculate the mean of power loss in front motor
        Compare_Drive.loss_front_mot(i) = mean(DriveStruct(i).loss.front_motor);
        % calculate the mean of power loss in rear motor
        Compare_Drive.loss_rear_mot(i) = mean(DriveStruct(i).loss.rear_motor);
        % calculate the mean of power loss after front gearbox &
        % differential
        Compare_Drive.loss_front_gear(i) = mean(DriveStruct(i).loss.gearbox_front);
        % calculate the mean of power loss after rear gearbox &
        % differential
        Compare_Drive.loss_rear_gear(i) = mean(DriveStruct(i).loss.gearbox_rear);

        try
            % if denominator is not zero
            % calculate eta for front motor
            Compare_Drive.eta_front_mot(i) = mean(DriveStruct(i).power.front_motor_need) / ...
                mean(DriveStruct(i).power.front_motor);
            % calculate eta for between before front motor and after gearbox+diff
            Compare_Drive.eta_front_gear(i) = mean(DriveStruct(i).power.after_gearbox_front_need) / ...
                mean(DriveStruct(i).power.after_gearbox_front);
        catch
            % if denominator zero (no motor in this axle)
            % set eta to -
            Compare_Drive.eta_front_mot(i) = "-";
            Compare_Drive.eta_front_gear(i) = "-";
        end

        try
            % if denominator is not zero
            % calculate eta for rear motor
            Compare_Drive.eta_rear_mot(i) = mean(DriveStruct(i).power.rear_motor_need) / ...
                mean(DriveStruct(i).power.rear_motor);
            % calculate eta for between before rear motor and after gearbox+diff
            Compare_Drive.eta_rear_gear(i) = mean(DriveStruct(i).power.after_gearbox_rear_need) / ...
                mean(DriveStruct(i).power.after_gearbox_rear);
        catch
            % if denominator zero (no motor in this axle)
            % set eta to -
            Compare_Drive.eta_rear_mot(i) = "-";
            Compare_Drive.eta_rear_gear(i) = "-";
        end

end
    
    
    