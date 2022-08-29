function [veh] = Auxiliary_SensECU(veh,Par)
%% Description:
% Designed by: Adrian König (FTM, Technical University of Munich)
%-------------
% Created on: 10.04.2022
% ------------
% Version: Matlab2020b
%-------------
% Description: Calculate energy consumption due to Sensors and ECUs
% ------------
% Sources:  [1] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
%           [2] Z. Liu, H. Tan, X. Kuang, H. Hao, und F. Zhao, “The Negative Impact of Vehicular Intelligence on Energy Consumption,” Journal of Advanced Transportation, Bd. 2019, S. 1–11, 2019.
% ------------
% Input:    - Par: struct with input and constant values
%           - veh: struct including the data of the vehicle
% ------------
% Output:   - veh: struct including the data of the vehicle
% ------------

%% Implementation:
%1) Read sensor and ECU setup
%2) Calculate auxiliary use
%3) Write values in vehicle struct

%% 1) Read setup

n_V2X       =   veh.Equipment.Sensors.C2X;
n_GPS       =   veh.Equipment.Sensors.GPS.GPS_receiver;
n_ECU       =   veh.Equipment.Sensors.Computer;
n_Cam       =   veh.Equipment.Sensors.Front_Camera;
n_Cam       =   n_Cam+veh.Equipment.Sensors.Surround_Camera;
n_Lid_low   =   veh.Equipment.Sensors.Lidar.n_lidar_mems;
n_Lid_high  =   veh.Equipment.Sensors.Lidar.n_lidar_mech;
n_Radar     =   veh.Equipment.Sensors.Radar;
n_Ultra     =   veh.Equipment.Sensors.Sonar;

%% 2) Calculate energy consumption and additional weight (later used)
for i=1:2
    if i==1 %Consumption in W, Values from [2]
        E_Cam       =   Par.aux.Sensor_ECU_Table.Camera(1)*n_Cam;
        E_Radar     =   Par.aux.Sensor_ECU_Table.Radar(1)*n_Radar;
        E_Lid_low   =   Par.aux.Sensor_ECU_Table.Lidar_low(1)*n_Lid_low;
        E_Lid_high  =   Par.aux.Sensor_ECU_Table.Lidar_high(1)*n_Lid_high;
        E_Ultra     =   Par.aux.Sensor_ECU_Table.Ultrasonic(1)*n_Ultra;
        E_GPS       =   Par.aux.Sensor_ECU_Table.GNSS(1)*n_GPS;
        E_ECU       =   Par.aux.Sensor_ECU_Table.ECU(1)*n_ECU; 
        E_V2X       =   Par.aux.Sensor_ECU_Table.V2X(1)*n_V2X;
        
        E_sum       =   E_Cam + E_Radar + E_Lid_low + E_Lid_high ...
                        + E_Ultra + E_GPS + E_ECU + E_V2X;
    else %Weight in kg, Values from [2]
        m_Cam       =   Par.aux.Sensor_ECU_Table.Camera(2)*n_Cam;
        m_Radar     =   Par.aux.Sensor_ECU_Table.Radar(2)*n_Radar;
        m_Lid_low   =   Par.aux.Sensor_ECU_Table.Lidar_low(2)*n_Lid_low;
        m_Lid_high  =   Par.aux.Sensor_ECU_Table.Lidar_high(2)*n_Lid_high;
        m_Ultra     =   Par.aux.Sensor_ECU_Table.Ultrasonic(2)*n_Ultra;
        m_GPS       =   Par.aux.Sensor_ECU_Table.GNSS(2)*n_GPS;
        m_ECU       =   Par.aux.Sensor_ECU_Table.ECU(2)*n_ECU; 
        m_V2X       =   Par.aux.Sensor_ECU_Table.V2X(2)*n_V2X;
        
        m_sum       =   m_Cam + m_Radar + m_Lid_low + m_Lid_high ...
                        + m_Ultra + m_GPS + m_ECU + m_V2X;
    end
end
%% 3) Write values in vehicle struct
veh.aux.SensECU_con     =   E_sum/1000; %Consumption in kW
veh.masses.SensECU      =   m_sum;
    


