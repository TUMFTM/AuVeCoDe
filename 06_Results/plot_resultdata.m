function plot_resultdata(vehicle)
%% Description:
%This function shows quickly the most important data and checks
%plausibility
 
%% Input:
% v: The vehicle struct
%% Output
% The updated v display all important data
 
%Author Adrian König, 23.09.202    Adrian Koenig FTM, TUM
 
%% Implementation

for i=1:nargin

Result{1,1}='Vehicle index';
Result{1,2}='Desired range set in km';
Result{1,3}='Achieved range in km';
Result{1,4}='Desired acceleration 0-100km/h in s';
Result{1,5}='Achieved acceleration 0-100km/h in s';
Result{1,6}='Desired max. speed in km/h';
Result{1,7}='Achieved max. speed in km/h';
Result{1,8}='Desired tire radius in mm';
Result{1,9}='Achieved tire radius in mm';
Result{1,10}='Tire width in mm';
Result{1,11}='Rim diameter in inch';
Result{1,12}='Gross battery capacity gross in kWh';
Result{1,13}='Net battery capacity gross in kWh';
Result{1,14}='Consumption in kWh/100km';
Result{1,15}='Torque front in Nm';
Result{1,16}='Power front in kW';
Result{1,17}='Torque rear in Nm';
Result{1,18}='Power rear in kW';
Result{1,19}='Length in mm';
Result{1,20}='Width in mm';
Result{1,21}='Height in mm';
Result{1,22}='Wheelbase in mm';
Result{1,23}='Mass in kg';
Result{1,24}='Cost in €';

%Differentiate if only one or more vehicle are input
j=i+1; %Row count for values starts from second row
if nargin==1
    Result{2,1}='Vehicle';
else
    Result{j,1}=sprintf('Vehicle %d',i);
end
Result{j,2}='Desired range set in km';
Result{j,3}='Achieved range in km';
Result{j,4}='Desired acceleration 0-100km/h in s';
Result{j,5}='Achieved acceleration 0-100km/h in s';
Result{j,6}='Desired max. speed in km/h';
Result{j,7}='Achieved max. speed in km/h';
Result{j,8}='Desired tire radius in mm';
Result{j,9}='Achieved tire radius in mm';
Result{j,10}='Tire width in mm';
Result{j,11}='Rim diameter in inch';
Result{j,12}='Gross battery capacity gross in kWh';
Result{j,13}='Net battery capacity gross in kWh';
Result{j,14}='Consumption in kWh/100km';
try
Result{j,15}='Torque front in Nm';
Result{j,16}='Power front in kW';
catch
    Result{j,15}='-';
    Result{j,16}='-';
end
try
    Result{j,17}='-';
    Result{j,18}='-';
catch
end
Result{j,19}='Length in mm';
Result{j,20}='Width in mm';
Result{j,21}='Height in mm';
Result{j,22}='Wheelbase in mm';
Result{j,23}='Mass in kg';
Result{j,24}='Cost in €';

end
Msg{1}=sprintf('Range_set = %0.1f',vehicle.Input.range);
Msg{2}=sprintf('Range_is = %0.1f' ,  vehicle.LDS.range);
Msg{3}=sprintf('Acceleration_set = %0.2f',vehicle.Input.acceleration_time_req);
Msg{4}=sprintf('Acceleration_is = %0.2f' ,  vehicle.LDS.sim_acc.acc_time_is);
Msg{5}=sprintf('Max_speed_set = %0.0f',vehicle.Input.max_speed);
Msg{6}=sprintf('Max_speed_is = %0.0f' ,  vehicle.LDS.sim_speed.max_speed_is );
Msg{7}=sprintf('Tire_Radius_set = %0.0f',vehicle.Input.r_tire);
Msg{8}=sprintf('Tire_Radius_is = %0.0f' ,  vehicle.LDS.wheel.r_tire);
Msg{9}=sprintf('Tire_Width_is = %0.0f' ,  vehicle.dimensions.CY.wheel_f_width);
Msg{10}=sprintf('Rim_Diameter_is = %0.0f' ,  vehicle.dimensions.CX.rim_f_diameter);
Msg{11}=sprintf('Battery_cap_gross_is = %0.2f',vehicle.battery.energy_is_gross_in_kWh  );
Msg{12}=sprintf('Battery_cap_net_is = %0.2f',vehicle.battery.energy_is_net_in_kWh  );
try
Msg{13}=sprintf('Torque_Front_is = %0.1f' ,  vehicle.LDS.MOTOR{1, 1}.T_max);
Msg{14}=sprintf('Power_Front_is = %0.1f' ,  vehicle.LDS.MOTOR{1, 1}.P_max_mech);
catch
end
try 
Msg{15}=sprintf('Torque_Rear_is = %0.1f',vehicle.LDS.MOTOR{1, 2}.T_max);
Msg{16}=sprintf('Power_Rear_is = %0.1f' ,  vehicle.LDS.MOTOR{1, 2}.P_max_mech);
catch
end
Msg{17}=sprintf('Length_is = %0.0f' ,  vehicle.dimensions.GX.vehicle_length  );
Msg{18}=sprintf('Width_is = %0.0f' ,  vehicle.dimensions.GY.vehicle_width  );
Msg{19}=sprintf('Height_is = %0.0f' ,  vehicle.dimensions.GZ.vehicle_height);
try
Msg{20}=sprintf('Cost_is = %0.0f' ,  vehicle.Cost.Manufacturing_Cost);
catch
end
Msg{21}=sprintf('Wheelbase_is = %0.0f' ,vehicle.dimensions.GX.wheelbase);
Msg{22}=sprintf('Consumption_is = %0.2f' ,vehicle.LDS.sim_cons.consumption100km);
Msg{23}=sprintf('Mass_is = %0.1f' ,vehicle.masses.vehicle_empty_weight_EU);
Msg{24}=sprintf('Overhang front: %4.2f',vehicle.dimensions.GX.vehicle_overhang_f);
Msg{25}=sprintf('Overhang rear: %4.2f',vehicle.dimensions.GX.vehicle_overhang_r);
Msg{26}=sprintf('Trunk front DIN: %4.2f',vehicle.dimensions.trunk_vol_front_DIN/(1000^2));
Msg{27}=sprintf('Trunk rear DIN: %4.2f',vehicle.dimensions.trunk_vol_rear_DIN/(1000^2));
msgbox(Msg)
 
 


