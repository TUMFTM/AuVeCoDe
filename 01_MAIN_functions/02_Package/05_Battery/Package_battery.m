function v=Package_battery(v,Parameters)
%% Description:
% Designed by: Adrian König (FTM, Technical University of Munich), Lorenzo Nicoletti (FTM, Technical University of Munich)
%-------------
% Created on: 01.02.2022
% ------------
% Version: Matlab2020b
%-------------
% Description:  This function fills the previously calculated battery space with cells and
%               derives the resulting battery capacity. Different cell orientation and
%               configurations are tested to find the configuration with the highest
%               energy.
% ------------
% Sources:  [1] Peter Köhler, Semi-physikalische Modellierung von Antriebsstrangkomponenten für Elektrofahrzeuge , Master Thesis, Institute of Automotive Technology, TUM, 2021
%           [2] Lorenzo Nicoletti, "Parametric Modeling of Battery Electric Vehicles in the Early Development Phase", Technical University of Munich, Institute of Automotive Technology, 2022
%           [2] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - Parameters (Par): struct with input and constant values
%           - vehicle(v): struct with data of the vehicle
% ------------
% Output:   - vehicle(v): struct with data of the vehicle
% ------------

%% Implementation
%1) Read in parameters
%2) Calculate the number of cells fitting
%3) Calculare the electrical scheme
%4) Assign Outputs
    
%% 1) Read in paramters:
cell_type = v.battery.cell.cell_type;                                       % Cell type: prismatic/pouch/cylindrical
energy_density_volumetric = v.battery.cell.energy_density_vol;              % Cell volumetric energy density in Wh/l
topology = v.topology.battery_topology;                                     % Battery integration principle (lowfloor/highfloor/mixedfloor)
net2gross = Parameters.battery.net_to_gross_capacity_factor;                % Conversion factor from net to gross capacity (Koehler, pp. 63) [-]

%Assign the previously calculated battery spaces:
Tspace=v.battery.spacetable;                                                % Table containing the previously calculated battery spaces
Ftable=v.battery.filltable;                                                 % Table containing the number of fitting cells for each space

%Retrieve the previously calculated battery spaces
underbody_dim = table2array(Tspace('underfloor',{'CX','CY','CZ'}));         % Dimensions UNDERBODY in mm
second_level_dim = table2array(Tspace('second level',{'CX','CY','CZ'}));    % Dimensions SECOND LEVEL in mm
tunnel_dim = table2array(Tspace('tunnel',{'CX','CY','CZ'}));                % Dimensions TUNNEL in mm
sov_dim = table2array(Tspace('small overlap',{'CX','CY','CZ'}));            % Dimensions SOV in mm

%Check if the warning have to be suppressed or not
suppress = v.settings.suppress_warnings;                                    %0: Do not suppress warning; 1: suppress warnings


%% 2) Calculate number of cells
% To fill the space it is required to distinguish between lowfloor and
% highfloor integration principles
    switch topology
        case 'lowfloor'   
            % Assign the cell vector which will be used to fill the spaces
            cell_dim = v.battery.cell.allcelldim; 

            % Fill the second level with cells
            [cell_dim_second_level,pack_dim_second_level,num_cell_second_level] = calc_filling_second_level(v,second_level_dim);

            % Fill the tunnel with cells
            [cell_dim_tunnel,pack_dim_tunnel,num_cell_tunnel] = calc_filling_tunnel(v,tunnel_dim);

            % Sum up the to derive the total amount of cells fitting
            no_cells_geometric = prod(num_cell_second_level,2)+prod(num_cell_tunnel,2);

        otherwise % A distinction between highfloor and mixedfloor is not required yet
            % Calculate the cells fitting in the underfloor
            [cell_dim_underfloor,pack_dim_underfloor,num_cell_underfloor] = calc_filling_underfloor(v, underbody_dim);

            % The cells not fitting in the underfloor have been filtered out. Use the "filtered" vector to fill the other spaces
            cell_dim = cell_dim_underfloor;

            % Fill the small overlap (only if the option is activated)
            [v,cell_dim_sov,pack_dim_sov,num_cell_sov] = calc_filling_small_overlap(v,sov_dim,cell_dim_underfloor);

            % Fill the second level
            [cell_dim_second_level,pack_dim_second_level,num_cell_second_level] = calc_filling_second_level(v,second_level_dim,cell_dim_underfloor);

            % Fill the tunnel
            [cell_dim_tunnel,pack_dim_tunnel,num_cell_tunnel] = calc_filling_tunnel(v,tunnel_dim,cell_dim_underfloor);

            % Sum up the to derive the total amount of cells fitting
            no_cells_geometric = prod(num_cell_underfloor,2)+prod(num_cell_second_level,2)+prod(num_cell_tunnel,2)+prod(num_cell_sov,2);
    end

    %% 3) Calculate the electrical scheme
    % Calculate the required number of cells to reach the required voltage
    [no_cells_electric,no_cells_serial,battery_voltage] = calc_electric_scheme(no_cells_geometric,Parameters);
    
    if sum(no_cells_electric)==0 
        
        if suppress==0 %Do not suppress the output
            disp('THE CHOOSEN VOLTAGE RANGE CANNOT BE REACHED');
        end
        
        v.battery.energy_is_gross_in_kWh = 0;
        v.battery.battery_voltage = 0;
    return
    end      
    
    %The number of cells which had to be esliminated in order to reach an acceptable battery voltage
    lost_cells = no_cells_electric-no_cells_geometric;

    % Calculate the cell volume (in l) and derive the cell and battery energy (Koehler, pp. 69)
    cell_volume = calc_cell_volume(cell_dim,cell_type);
    cell_energy = cell_volume * energy_density_volumetric;
    battery_energy = no_cells_electric.*cell_energy/1000;

    % Choose cell with the biggest resulting battery energy
    [battery_energy_in_kWh,index_max] = max(battery_energy);

    % To be done: here actually I would take the first 10 top results and choose the one where there are no cell losts!
    index_max = index_max(1);
    
    % Error handling section
    if isempty(index_max)
        
        if suppress==0 %Do not suppress the output
            disp('NO CELL FITS IN THE GIVEN SPACE. ERROR!!');
        end
        
        v.battery.energy_is_gross_in_kWh=0;
    return            
    end

    %% 4) Assign the Outputs
    
    if ~strcmp(topology,'lowfloor') %The topology is whether a highfloor or a mixedfloor
        %The underfloor and small overlap have to be filled only for mixedfloor or highfloor topology
        Ftable('underfloor',:) = table(num_cell_underfloor(index_max,1),num_cell_underfloor(index_max,2),num_cell_underfloor(index_max,3),...
        cell_dim_underfloor(index_max,1),cell_dim_underfloor(index_max,2),cell_dim_underfloor(index_max,3),...
        pack_dim_underfloor(index_max,1),pack_dim_underfloor(index_max,2),pack_dim_underfloor(index_max,3));

        Ftable('small overlap',:) = table(num_cell_sov(index_max,1),num_cell_sov(index_max,2),num_cell_sov(index_max,3),...
        cell_dim_sov(index_max,1),cell_dim_sov(index_max,2),cell_dim_sov(index_max,3),...
        pack_dim_sov(index_max,1),pack_dim_sov(index_max,2),pack_dim_sov(index_max,3));    
    end

    % Assign the cell fitting in second level
    Ftable('second level',:) = table(num_cell_second_level(index_max,1),num_cell_second_level(index_max,2),num_cell_second_level(index_max,3),...
    cell_dim_second_level(index_max,1),cell_dim_second_level(index_max,2),cell_dim_second_level(index_max,3),...
    pack_dim_second_level(index_max,1),pack_dim_second_level(index_max,2),pack_dim_second_level(index_max,3));

    % Assign the cell fitting in the tunnel
    Ftable('tunnel',:)=table(num_cell_tunnel(index_max,1),num_cell_tunnel(index_max,2),num_cell_tunnel(index_max,3),...
    cell_dim_tunnel(index_max,1),cell_dim_tunnel(index_max,2),cell_dim_tunnel(index_max,3),...
    pack_dim_tunnel(index_max,1),pack_dim_tunnel(index_max,2),pack_dim_tunnel(index_max,3));

    % Update the table storing the number of cells
    v.battery.filltable=Ftable;

    % Save the resulting number of cells and the lost cells (if any) due to the electrical layout calculation 
    v.battery.num_of_cells=no_cells_electric(index_max,:);
    v.battery.num_of_cells_serial=no_cells_serial(index_max,:);
    v.battery.lost_cells=lost_cells(index_max,:);

    % Save the cell characteristics and dimensions
    v.battery.cell.energy=cell_energy(index_max,:);                       %Resulting energy of the cell in Wh
    v.battery.energy_is_gross_in_kWh=battery_energy_in_kWh;               %The total energy of the battery in kWh (gross, i.e. not all this energy can be used)
    v.battery.energy_is_net_in_kWh=battery_energy_in_kWh*net2gross;       %The usable energy of the battery in kWh (net)
    v.battery.battery_voltage=battery_voltage(index_max);                 %Total battery_voltage in V 
end

%% Subfunctions
function cell_volume = calc_cell_volume(cell_dim,cell_type)
%% Description:
%This function returns one vector containing the volume of every possible
%cell combination, distinguishing between prismatic/pouch and cylindrical
%cells for the volume calculation [1, pp. 69].

if strcmp(cell_type,'cylindrical')
    %The cell height is the array value, which is different from the other two values
    pos_height=~[cell_dim(:,2)-cell_dim(:,3) cell_dim(:,1)-cell_dim(:,3) cell_dim(:,1)-cell_dim(:,2)];
    cell_height=sum(cell_dim.*pos_height,2);

    %The cell diameter is the array value, which is equal to one of other two values
    pos_diameter=[cell_dim(:,1)==cell_dim(:,2),cell_dim(:,2)==cell_dim(:,3),cell_dim(:,3)==cell_dim(:,1)];
    cell_diameter=sum(cell_dim.*pos_diameter,2);

    %Cell volume in liter
    cell_volume = (cell_diameter./2).^2 .* pi.*cell_height/1e6;
else
    
    %Cell volume in liter
    cell_volume = (cell_dim(:,1).*cell_dim(:,2).*cell_dim(:,3))/1e6;
end
end