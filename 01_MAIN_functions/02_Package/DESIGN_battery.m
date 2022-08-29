function [v] = DESIGN_battery(v,Parameters)
%% Description:
% Designed by: Adrian König (FTM, Technical University of Munich), Daniel Telschow (Technical University of Munich)
%-------------
% Created on: 01.01.2021
% ------------
% Version: Matlab2020b
%-------------
% Description: This function designs the battery pack in the available space.
% ------------
% Sources:  [1] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - Parameters: struct with input and constant values
%           - vehicle: struct with includes the data of the vehicle
% ------------
% Output:   - vehicle: struct with includes the data of the vehicle
% ------------

%% Implementation:
%1) Calculation of the maximum available space
%2) Calculation of the maximum available capacity in the space from 1)
%3) Reduction of the battery (starting from the second level batteries) to
%   reach the required capacity and avoid oversizing

%% 1) Calculate battery available space
%Initialize possible cell combinations
v=Package_cell_combinations(v,Parameters);

%Calc the dimensions of the battery installation space in the underfloor of the v
v = Package_battery_dimensions(v,Parameters);

%% 2) Calculate required and available battery capacity
% Required battery energy (in kWh) to reach the given electric range (net and gross)
battery_capacity_required_gross_net = v.LDS.sim_cons.consumption100km*v.Input.range/100; % req. energy net [kWh] = (consumption [kWh/100km] * Req. Range [km]) / 100km
battery_capacity_required_gross= battery_capacity_required_gross_net/Parameters.battery.net_to_gross_capacity_factor; % req. energy gross = net/net_to_gross_factor
v.battery.energy_required_gross_in_kWh=battery_capacity_required_gross;

% Calculate the maximum installable capacity (underfloor + second level)
v.settings.fill_tunnel=1;
v.settings.fill_second_level=1;
v= Package_battery(v,Parameters);
battery_capacity_max_gross=v.battery.energy_is_gross_in_kWh; %Resulting maximum installable capacity (in kWh)


%% 3) Reduce battery to avoid oversizing
% If the maximum capacity exceeds the required capacity, the capacity is
% reduced to meet the desired range as close as possible. Too high upward or
% downward deviations will lead to the exclusion of the v (see check_LDS_change)  

% Only proceed with resizing if not range optimization is selected. Otherwise keep the max. possible capacity
if Parameters.optimization.range_opt==0
    
    %% 3.1) The max. battery capacity is higher than the required one, reduce the battery space
    if battery_capacity_max_gross>battery_capacity_required_gross
        % Check if underfloor battery is sufficient for range --> set second level fill to 0
        v.settings.fill_second_level=0;
        v.settings.fill_tunnel=0; % Tunnel = Second Level 2
        
        v=Package_battery(v,Parameters); % Calculate max. battery capacity only with underfloor
        battery_capacity_just_underfloor_gross=v.battery.energy_is_gross_in_kWh;
        
        % Set underbody dimensions to zero if no cell can be fitted
        if battery_capacity_just_underfloor_gross==0
            v.battery.installationspace.CX_batt_underfloor=0;
            v.battery.installationspace.CY_batt_underfloor=0;
            v.battery.installationspace.CZ_batt_underfloor=0;
            U=table(0,0,0,v.battery.installationspace.EX_batt_underfloor, v.battery.installationspace.EZ_batt_underfloor);
            U.Properties.VariableNames={'CX','CY','CZ','EX','EZ'};
            U.Properties.RowNames={'underfloor'};
            v.battery.spacetable(1,:)=U;
            
            %Reduce bottom cover from second level space since now it is not included into the underfloor already
            h_sl1=v.battery.installationspace.CZ_batt_second_level_1; %current height
            p_z_sl1=v.battery.installationspace.EZ_batt_second_level_1; %current z-position
            v.battery.installationspace.CZ_batt_second_level_1=h_sl1-v.dimensions.CZ.batt_bottom_cover; %reduce height with bottom cover thickness
            v.battery.installationspace.EZ_batt_second_level_1=p_z_sl1+v.dimensions.CZ.batt_bottom_cover; %increase z-position with bottom cover thickness
            h_sl2=v.battery.installationspace.CZ_batt_second_level_2;  %current z-position
            p_z_sl2=v.battery.installationspace.EZ_batt_second_level_2; %current z-position
            if ~strcmp(v.Input.int_type,'btb') %Only if not back2back
                v.battery.installationspace.EZ_batt_second_level_2=p_z_sl2+v.dimensions.CZ.batt_bottom_cover; %reduce z-position with bottom cover thickness
                v.battery.installationspace.CZ_batt_second_level_2=h_sl2-v.dimensions.CZ.batt_bottom_cover; %increase z-position with bottom cover thickness
            end
            
            %Write values in table
            v.battery.spacetable.CZ(3)=v.battery.installationspace.CZ_batt_second_level_1;
            v.battery.spacetable.CZ(4)=v.battery.installationspace.CZ_batt_second_level_2;
            v.battery.spacetable.EZ(3)=v.battery.installationspace.EZ_batt_second_level_1;
            v.battery.spacetable.EZ(4)=v.battery.installationspace.EZ_batt_second_level_2;
        end
        
        %% 3.1.1) Max. battery capacity just underfloor is higher than required
        if  battery_capacity_just_underfloor_gross>battery_capacity_required_gross
            % Set Second Level Dimension to zero (to avoid plot)
            v.battery.installationspace.CX_batt_second_level_1=0;
            v.battery.installationspace.CY_batt_second_level_1=0;
            v.battery.installationspace.CX_batt_second_level_2=0;
            v.battery.installationspace.CY_batt_second_level_2=0;
            
            % Reduce underfloor dimension until its capacity is equal to required capacity
            while  v.battery.energy_is_gross_in_kWh > battery_capacity_required_gross
                % Save temporary battery dimensions (to return to last feasible)
                v=store_temp_battery(v);
                
                % Calculate target/actual battery ratio for faster reduction of battery
                bat_ratio=1-battery_capacity_required_gross/v.battery.energy_is_gross_in_kWh;
                bat_y_ratio=2*v.battery.installationspace.CY_batt_underfloor/(v.battery.installationspace.CX_batt_underfloor+v.battery.installationspace.CY_batt_underfloor); %ratio of y measurement in relation to perimeter of battery footprint
                bat_x_ratio=(2-bat_y_ratio); %x-ratio (x+y ratio = 2) => if x and y length are the same, the x and y ratio =1
                if bat_ratio<0.2 %if deviation is less then 20 percent
                    bat_red_uf_x=Parameters.settings.dim_red_bat*bat_x_ratio;
                    bat_red_uf_y=Parameters.settings.dim_red_bat*bat_y_ratio;
                else %if bat_ratio is more than 0.2, enlarge reduction rate in relation to overfilling
                    %Curvefitting used: every 20% the dim_red_bat is doubled till 60% (20%:5mm, 40%:10mm, 60%:20mm)
                    %The equation has to be used with bat_ratio (since bat_ratio <=1!!) and afterwards multiplied with bata_x/y_ratio!
                    bat_red_uf_x=(62.5*bat_ratio^2-12.5*bat_ratio+Parameters.settings.dim_red_bat)*bat_x_ratio;
                    bat_red_uf_y=(62.5*bat_ratio^2-12.5*bat_ratio+Parameters.settings.dim_red_bat)*bat_y_ratio;
                end
                
                % Reduce undefloor x and y dimension (z dim not reduced!)
                v.battery.installationspace.CX_batt_underfloor=v.battery.spacetable.CX(1)-bat_red_uf_x;
                v.battery.installationspace.CY_batt_underfloor=v.battery.spacetable.CY(1)-bat_red_uf_y;
                
                % Reposition battery in x-direction (middle of available space)
                x_pos=v.battery.installationspace.EX_batt_underfloor;
                v.battery.installationspace.EX_batt_underfloor=x_pos+bat_red_uf_x*0.5; %Movment to new middle point
                
                % Assign New Values to Spacetable
                v.battery.spacetable.CX(1)=v.battery.installationspace.CX_batt_underfloor;
                v.battery.spacetable.CY(1)=v.battery.installationspace.CY_batt_underfloor;
                v.battery.spacetable.EX(1)=v.battery.installationspace.EX_batt_underfloor;
                
                v=Package_battery(v,Parameters); % recalculate battery capacity with reduced dimensions
                
                % Return to last feasible config
                if v.battery.energy_is_gross_in_kWh < battery_capacity_required_gross
                    %calculate differences between previous (temp) and current version
                    delta_temp=v.battery.energy_is_gross_in_kWh_temp-battery_capacity_required_gross;
                    delta_new=battery_capacity_required_gross-v.battery.energy_is_gross_in_kWh;
                    if delta_temp>=delta_new %Current capacity is nearer to required capacity then previous capacity
                        % Keep undefloor x and y dimension of current config
                    else %Previous capacity was nearer to required capacity
                        % Set undefloor x and y dimension to last feasible config
                        v.battery.installationspace.CX_batt_underfloor=v.battery.installationspace.CX_batt_underfloor_temp;
                        v.battery.installationspace.CY_batt_underfloor=v.battery.installationspace.CY_batt_underfloor_temp;
                        % Assign New Values to Spacetable
                        v.battery.spacetable.CX(1)=v.battery.installationspace.CX_batt_underfloor;
                        v.battery.spacetable.CY(1)=v.battery.installationspace.CY_batt_underfloor;
                        v.battery.spacetable.EX(1)=v.battery.installationspace.EX_batt_underfloor;
                        v=Package_battery(v,Parameters); % recalculate battery capacity
                    end
                    %Set tunnel to 0 for correct calculation of weight
                    v.battery.spacetable.CZ(3)=0;
                    v.battery.spacetable.CZ(4)=0;
                    return %Calculation finished
                end
            end
            %% 3.1.2) Maximum capacity of underfloor does not achieve req. capacity => Second level needed
        else
            % set filling to 1
            v.settings.fill_tunnel=1;
            v.settings.fill_second_level=1;
            
            % Calculate the amount of energy that needs to be reduced
            extra_energy_sec_lev=battery_capacity_max_gross-battery_capacity_required_gross;
            
            v.battery.energy_is_gross_in_kWh=battery_capacity_max_gross; %Assign max. installable capacity as currently installed battery capacity
            while extra_energy_sec_lev>0
                % Save temporary battery dimensions (to return to last feasible)
                v=store_temp_battery(v);
                
                %% Second level 1
                
                % Calculate target/actual battery ratio for faster reduction of battery
                bat_ratio=1-battery_capacity_required_gross/v.battery.energy_is_gross_in_kWh;
                bat_y_ratio=2*v.battery.installationspace.CY_batt_second_level_1/(v.battery.installationspace.CX_batt_second_level_1+v.battery.installationspace.CY_batt_second_level_1); %ratio of y measurement in relation to perimeter of battery footprint
                bat_x_ratio=(2-bat_y_ratio); %x-ratio (x+y ratio = 2) => if x and y length are the same, the x and y ratio =1
                if bat_ratio<0.2 %if deviation is less then 20 percent
                    bat_red_s1_x=Parameters.settings.dim_red_bat*bat_x_ratio;
                    bat_red_s1_y=Parameters.settings.dim_red_bat*bat_y_ratio;
                else %if bat_ratio is more than 0.2, enlarge reduction rate in relation to overfilling
                    %Curvefitting used: every 20% the dim_red_bat is doubled till 60% (20%:5mm, 40%:10mm, 60%:20mm)
                    %The equation has to be used with bat_ratio (since bat_ratio <=1!!) and afterwards multiplied with bata_x/y_ratio!
                    bat_red_s1_x=(62.5*bat_ratio^2-12.5*bat_ratio+Parameters.settings.dim_red_bat)*bat_x_ratio;
                    bat_red_s1_y=(62.5*bat_ratio^2-12.5*bat_ratio+Parameters.settings.dim_red_bat)*bat_y_ratio;
                end
                
                % Reduce second level 1 x and y dimension (z dim not reduced!)
                v.battery.installationspace.CX_batt_second_level_1=v.battery.installationspace.CX_batt_second_level_1-bat_red_s1_x;
                v.battery.installationspace.CY_batt_second_level_1=v.battery.installationspace.CY_batt_second_level_1-bat_red_s1_y;
                
                %Check if negative values occur
                v.battery.installationspace.CX_batt_second_level_1=max(v.battery.installationspace.CX_batt_second_level_1,0);
                v.battery.installationspace.CY_batt_second_level_1=max(v.battery.installationspace.CY_batt_second_level_1,0);
                
                % Reposition battery in x-direction (as near as possible to interior) Only necessary for level 1, level 2 is already near to interior (EX stays the same)
                %Differentiate between vis-a-vis, b2b and conventional
                if strcmp(v.Input.int_type,'con') || strcmp(v.Input.int_type,'sr')
                    %no movement so that battery is directly behind front feet area
                elseif strcmp(v.Input.int_type,'vav') %battery has to be moved to the back so that it is near interior
                    x_pos=v.battery.installationspace.EX_batt_second_level_1_temp;
                    v.battery.installationspace.EX_batt_second_level_1=x_pos+bat_red_s1_x; %Movement to most near point to interior
                elseif strcmp(v.Input.int_type,'btb') %battery has to be moved with half of length so it stays in the middle (analogue underfloor battery)
                    x_pos=v.battery.installationspace.EX_batt_second_level_1_temp;
                    v.battery.installationspace.EX_batt_second_level_1=x_pos+bat_red_s1_x*0.5; %Movement to the middle
                end
                
                %% Second level 2
                % Calculate target/actual battery ratio for faster reduction of battery
                bat_y_ratio=2*v.battery.installationspace.CY_batt_second_level_2/(v.battery.installationspace.CX_batt_second_level_2+v.battery.installationspace.CY_batt_second_level_2); %ratio of y measurement in relation to perimeter of battery footprint
                bat_x_ratio=(2-bat_y_ratio); %x-ratio (x+y ratio = 2) => if x and y length are the same, the x and y ratio =1
                if bat_ratio<0.2 %if deviation is less then 20 percent
                    bat_red_s2_x=Parameters.settings.dim_red_bat;
                    bat_red_s2_y=Parameters.settings.dim_red_bat;
                else %if bat_ratio is more than 0.2, enlarge reduction rate in relation to overfilling
                    %Curvefitting used: every 20% the dim_red_bat is doubled till 60% (20%:5mm, 40%:10mm, 60%:20mm)
                    %The equation has to be used with bat_ratio (since bat_ratio <=1!!) and afterwards multiplied with bata_x/y_ratio!
                    bat_red_s2_x=(62.5*bat_ratio^2-12.5*bat_ratio+Parameters.settings.dim_red_bat)*bat_x_ratio;
                    bat_red_s2_y=(62.5*bat_ratio^2-12.5*bat_ratio+Parameters.settings.dim_red_bat)*bat_y_ratio;
                end
                
                if strcmp(v.Input.int_type,'btb')
                    x_pos=v.battery.installationspace.EX_batt_second_level_2_temp;
                    v.battery.installationspace.EX_batt_second_level_2=x_pos+bat_red_s2_x*0.5; %Movement to the middle
                end
                
                % Reduce second level 2 x and y dimension (z dim not reduced!)
                v.battery.installationspace.CX_batt_second_level_2=v.battery.installationspace.CX_batt_second_level_2-bat_red_s2_x;
                v.battery.installationspace.CY_batt_second_level_2=v.battery.installationspace.CY_batt_second_level_2-bat_red_s2_y;
                
                %Check if negative values occur
                v.battery.installationspace.CX_batt_second_level_2=max(v.battery.installationspace.CX_batt_second_level_2,0);
                v.battery.installationspace.CY_batt_second_level_2=max(v.battery.installationspace.CY_batt_second_level_2,0);
                
                %% Assign New Values to Spacetable
                % 3=Second_Level=Second Level 1 = front underseat battery
                % 4=Tunnel=Second Level 2 = rear underseat battery
                v.battery.spacetable.CX(3)=v.battery.installationspace.CX_batt_second_level_1;
                v.battery.spacetable.CX(4)=v.battery.installationspace.CX_batt_second_level_2;
                v.battery.spacetable.CY(3)=v.battery.installationspace.CY_batt_second_level_1;
                v.battery.spacetable.CY(4)=v.battery.installationspace.CY_batt_second_level_2;
                v.battery.spacetable.EX(3)=v.battery.installationspace.EX_batt_second_level_1;
                v.battery.spacetable.EX(4)=v.battery.installationspace.EX_batt_second_level_2;
                
                % Recalculate battery capacity with reduced dimensions
                v=Package_battery(v,Parameters);
                extra_energy_sec_lev=v.battery.energy_is_gross_in_kWh-battery_capacity_required_gross;
                
                %% Return to last feasible config
                if extra_energy_sec_lev<0
                    %calculate differences between previous (temp) and current version
                    delta_temp=v.battery.energy_is_gross_in_kWh_temp-battery_capacity_required_gross;
                    delta_new=battery_capacity_required_gross-v.battery.energy_is_gross_in_kWh;
                    if delta_temp>=delta_new %Current capacity is nearer to required capacity then previous capacity
                        % Keep undefloor x and y dimension of current config
                    else
                        % Set Second Level 1 x and y dimension to last feasible config
                        v.battery.installationspace.CX_batt_second_level_1=v.battery.installationspace.CX_batt_second_level_1_temp;
                        v.battery.installationspace.CY_batt_second_level_1=v.battery.installationspace.CY_batt_second_level_1_temp;
                        
                        % Set Second Level 2 x and y dimension to last feasible config
                        v.battery.installationspace.CX_batt_second_level_2=v.battery.installationspace.CX_batt_second_level_2_temp;
                        v.battery.installationspace.CY_batt_second_level_2=v.battery.installationspace.CY_batt_second_level_2_temp;
                        
                        % Assign New Values to Spacetable
                        % 3=Second_Level=Second Level 1 = front underseat battery
                        % 4=Tunnel=Second Level 2 = rear underseat battery
                        v.battery.spacetable.CX(3)=v.battery.installationspace.CX_batt_second_level_1;
                        v.battery.spacetable.CX(4)=v.battery.installationspace.CX_batt_second_level_2;
                        v.battery.spacetable.CY(3)=v.battery.installationspace.CY_batt_second_level_1;
                        v.battery.spacetable.CY(4)=v.battery.installationspace.CY_batt_second_level_2;
                        v.battery.spacetable.EX(3)=v.battery.installationspace.EX_batt_second_level_1;
                        v.battery.spacetable.EX(4)=v.battery.installationspace.EX_batt_second_level_2;
                        
                        % Recalculate battery capacity with reduced dimensions
                        v=Package_battery(v,Parameters);
                    end
                    return
                end
                
                
            end
        end
        %% 3.2) Range not achieved with maximal battery capacity
    else
        v.Error=4;
        return
    end
    
end
end


function  v=store_temp_battery(v)
% Subfunction stores temporary battery dimensions (to return to last feasible)
% Only y- and x-values, since z-Values are not changed (capacity is most sensitive to changes in z-direction)
v.battery.installationspace.CX_batt_underfloor_temp=v.battery.installationspace.CX_batt_underfloor;
v.battery.installationspace.CY_batt_underfloor_temp=v.battery.installationspace.CY_batt_underfloor;

v.battery.installationspace.CX_batt_second_level_1_temp=v.battery.installationspace.CX_batt_second_level_1;
v.battery.installationspace.CY_batt_second_level_1_temp=v.battery.installationspace.CY_batt_second_level_1;
v.battery.installationspace.EX_batt_second_level_1_temp=v.battery.installationspace.EX_batt_second_level_1;

v.battery.installationspace.CX_batt_second_level_2_temp=v.battery.installationspace.CX_batt_second_level_2;
v.battery.installationspace.CY_batt_second_level_2_temp=v.battery.installationspace.CY_batt_second_level_2;
v.battery.installationspace.EX_batt_second_level_2_temp=v.battery.installationspace.EX_batt_second_level_2;

v.battery.energy_is_gross_in_kWh_temp=v.battery.energy_is_gross_in_kWh;

end

