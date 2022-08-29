function plot_weight(vehicle)
%% Description:
%Plot the distribution of all masses
%Author:        Adrian Koenig FTM, TUM
%Date:          22.11.2021

%% Initialize needed information
Font        =   'Arial';
FontSize    =   10; 
Subsystems  =   ["powertrain" "chassis" "exterior" "EE" "frame" "interior"];
for i=1:6
   name=Subsystems(i);
   input=eval(strcat('vehicle.masses.',name));
   plot_mass_ratio(input,Font,FontSize) 
       
end

end

