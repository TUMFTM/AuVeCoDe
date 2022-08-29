function [no_cells_electric,no_cells_serial,battery_voltage]=calc_electric_scheme(no_cells_geometric,Parameters)
%% Description:
% Designed by: Adrian König (FTM, Technical University of Munich)
%-------------
% Created on: 01.05.2022
% ------------
% Version: Matlab2020b
%-------------
% Description:  This function calculate the electrical scheme of the battery. 
%               The working principle is simple: for each cell number possibility, the
%               code checks the following features:
%               - Is the number of cells sufficient for the min voltage?
%               - How many parallel cells and how many serial cells?
% ------------
% Sources:  [1] Peter Köhler, Semi-physikalische Modellierung von Antriebsstrangkomponenten für Elektrofahrzeuge , Master Thesis, Institute of Automotive Technology, TUM, 2021
%           [2] Lorenzo Nicoletti, "Parametric Modeling of Battery Electric Vehicles in the Early Development Phase", Technical University of Munich, Institute of Automotive Technology, 2022
%           [3] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - Parameters : struct with input and constant values
%           - no_cells_geometric: The number of cells that would fit in the given space (without considering the electrical scheme feasibility)
% ------------
% Output:   - Number of remaining cells after checking the electric scheme
%           - Number of remaining cells after checking the electric scheme (serial)
%           - Resulting battery voltage in V
% ------------

%% Implementation
% 1) Read parameters
% 2) Calculation
% 3) Assign output

%% 1) Read parameters
battery_voltage_min = Parameters.battery.battery_voltage_min;           % Minimum allowed voltage of the battery in V
battery_voltage_max = Parameters.battery.battery_voltage_max;           % Maximum allowed voltage of the battery in V
cell_voltage = Parameters.battery.cell_voltage;                         % Voltage of the cell in V

%% 2) Calculation
% Minimum and maximum number of cells in serial, depending on required battery voltage
cells_serial_min = ceil(battery_voltage_min / cell_voltage);
cells_serial_max = floor(battery_voltage_max / cell_voltage);

% Since the vector with the geometric cells contains a certain number of cells 
% more than once. Therefore to speed up the calculation reduce the vector
no_cell_geometric_reduced=unique(no_cells_geometric); 
no_cell_geometric_reduced_orig=no_cell_geometric_reduced;

% It is essential to initialize these variables otherwise MATLAB gives an "out of memory" error
cells_serial = zeros(size(no_cell_geometric_reduced,1),1);   % number of serial cells
cells_parallel = zeros(size(no_cell_geometric_reduced,1),1); % number of parallel cells 

for i=1:size(no_cell_geometric_reduced,1)    
    % Start values for iteration: minimum number of cells all in serial so we can barely reach the voltage level
    cells_parallel(i) = 0;
    
    while cells_parallel(i) == 0    % We impose the condition that at least one parallel strand is required
        
        if (no_cell_geometric_reduced(i) < cells_serial_min)
            % Number of cells insufficient to reach the minimum voltage
            break;        
        end

        % Possible values for cells in serial are between previously calculated min and max
        possibilities_serial = cells_serial_max:-1:cells_serial_min;
        
        % Check for a division without a remainder -> cell number multiple of one of the possibilites serial
        remainders = rem(no_cell_geometric_reduced(i), possibilities_serial);
        
        possible_configuration_id = find(remainders == 0);
        
        if ~isnan(possible_configuration_id)
            % We found a configuration without remainder. Get values for parallel and serial and leave loop
            cells_serial(i) = possibilities_serial(possible_configuration_id(1));
            cells_parallel(i) = no_cell_geometric_reduced(i) / cells_serial(i);
            break;      
        end
        
        % Try again with one cell less
        no_cell_geometric_reduced(i) = no_cell_geometric_reduced(i) - 1;       
    end
end

%% 3) Assign output
no_cells_electric_reduced=cells_serial.*cells_parallel;
no_cells_electric = zeros(size(no_cells_geometric,1),1);   % Number of remaining cells after checking the electric scheme
no_cells_serial=zeros(size(no_cells_geometric,1),1);       % Number of remaining cells after checking the electric scheme
battery_voltage = zeros(size(no_cells_geometric,1),1);     % Resulting battery voltage in V

% Reassign the calculated values of no_cells_geometric_reduced to cells_geometric
for i=1:size(no_cell_geometric_reduced_orig,1)   
    
    idx=find(no_cell_geometric_reduced_orig(i)==no_cells_geometric);
    no_cells_electric(idx,1)=no_cells_electric_reduced(i);
    battery_voltage(idx,1)=cells_serial(i)*cell_voltage;
    no_cells_serial(idx,1)=cells_serial(i);
end

end