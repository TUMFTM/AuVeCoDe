function [Compare] = compare_veh(veh_struct)

    %% Description
    % Function to compare vehicle concepts by vehicle properties
    
    % Author: Fabian Liemawan Adji
    % Date: September 2021
    
    %% Input:
    % veh_struct: struct containing multiples structs of vehicle concepts
    % e.g.      veh_struct.X_GM1, veh_struct.GM_X1, veh.struct_GMGM1
    
    %% Output:
    % Compare: struct with comparison tables as fields
    % Result.xlsx: Sheet Überblick shows comparison of vehicle properties 
    
    % initialize table
    Properties = table;
    % get fieldnames containing all concept names in string
    structlist = string(fieldnames(veh_struct));
    % collect values for vehicle properties for each concept
    for i = 1:length(structlist)
        % manufacture cost
        Properties.Cost(i) = veh_struct.(structlist(i)).Cost.Manufacturing_Cost;
        % battery capacity
        Properties.BatCap(i) = veh_struct.(structlist(i)).battery.energy_is_gross_in_kWh;
        % range
        Properties.Range(i) = veh_struct.(structlist(i)).LDS.range;
        % max speed
        Properties.MaxSpeed(i) = veh_struct.(structlist(i)).LDS.sim_speed.max_speed_is;
        % empty weight
        Properties.EmptyWeight(i) = veh_struct.(structlist(i)).masses.vehicle_empty_weight_EU;
        % max weight
        Properties.MaxWeight(i) = veh_struct.(structlist(i)).masses.vehicle_max_weight;
        % vehicle length
        Properties.Length(i) = veh_struct.(structlist(i)).dimensions.GX.vehicle_length;
        % vehicle width
        Properties.Width(i) = veh_struct.(structlist(i)).dimensions.GY.vehicle_width;
        % vehicle height
        Properties.Height(i) = veh_struct.(structlist(i)).dimensions.GZ.vehicle_height;
        % vehicle front overhang
        Properties.OverhangFr(i) = veh_struct.(structlist(i)).dimensions.GX.vehicle_overhang_f;
        % vehicle rear overhang
        Properties.OverhangRe(i) = veh_struct.(structlist(i)).dimensions.GX.vehicle_overhang_r;
        % vehicle wheelbase
        Properties.Wheelbase(i) = veh_struct.(structlist(i)).dimensions.GX.wheelbase;

        % get power and torque of every axle to get sum of both in
        % properties
        T_max = zeros(2,1);
        P_max = zeros(2,1);
        for ii = 1:2
            % if not empty in current axle then multiply torque and power
            % each machine with number of machine in the axle
            if ~isempty(veh_struct.(structlist(i)).e_machine{1, ii})
                T_max(ii) = veh_struct.(structlist(i)).e_machine{1, ii}.T_max * veh_struct.(structlist(i)).e_machine{1, ii}.quantity;
                P_max(ii) = veh_struct.(structlist(i)).e_machine{1, ii}.P_max_mech * veh_struct.(structlist(i)).e_machine{1, ii}.quantity;
            end
        end
        % sum of torque
        Properties.TorqueTotal(i) = sum(T_max);
        % sum of power
        Properties.PowerTotal(i) = sum(P_max);
        % consumption
        Properties.Consumption(i) = veh_struct.(structlist(i)).LDS.sim_cons.consumption100km;
    end
    
    % exhange rows and columns
    Properties = rows2vars(Properties);
    % add vehicle concept names into columns beside category
    Properties.Properties.VariableNames = ["Category",structlist'];
    % save result in Result.xlsx
    writetable(Properties, "Result.xlsx", "Sheet", "Überblick");
    
    % save vehicle properties in Compare
    Compare.Prop = Properties;

    [Compare.Cost] = table_Cost(veh_struct);
end