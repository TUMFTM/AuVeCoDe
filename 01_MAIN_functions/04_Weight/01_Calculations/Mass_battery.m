function [v] = Mass_battery(v,Par)
%% Description:
% Designed by: Adrian König (FTM, Technical University of Munich), Lorenzo Nicoletti (FTM, Technical University of Munich)
%-------------
% Created on: 01.04.2022
% ------------
% Version: Matlab2020b
%-------------
% Description: This function calculates the mass of the battery.
% ------------
% Sources: [1] Lorenzo Nicoletti, "Parametric Modeling of Battery Electric Vehicles in the Early Development Phase", Technical University of Munich, Institute of Automotive Technology, 2022
%          [2] A. Romano, „Data-based Analysis for Parametric Weight Estimation of new BEV Concepts,“Master thesis, Institute of Automotive Technology, Technical University of Munich, Munich, 2021.
%          [3] L. Nicoletti, A. Romano, A. König, P. Köhler, M. Heinrich and M. Lienkamp, „An Estimation of the Lightweight Potential of Battery Electric Vehicles,“ Energies, vol. 14, no. 15, p. 4655, 2021, DOI: 10.3390/en14154655.
%          [4] P. Köhler, „Semi-physikalische Modellierung von Antriebsstrangkomponenten für Elektrofahrzeuge,“Master thesis, Institute of Automotive Technology, Technical University of Munich, Munich, 2021
%          [5] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - Parameters (Par): struct with input and constant values
%           - vehicle(v): struct including the data of the vehicle
% ------------
% Output:   - vehicle(v): struct including the data of the vehicle
% ------------

%% Implementation:
%1) Initialize the required variables
%2) Caluclate the volumetric energy density of the battery
%3) Calculate the mass of the battery: Mass split into three shares: cells, cooling/electrical and structural components.

%% 1) Initialize the required variables
energy_density_gravimetric = v.battery.cell.energy_density_grav;    % in Wh/kg
battery_energy = v.battery.energy_is_gross_in_kWh*1000;             % in Wh
cell_type = v.battery.cell.cell_type;                               % prismatic, pouch or cylindrical

% Regression between battery housing (including tunnel and second level -if present- but no sills) (Koehler, p. 78)
regr_battery_structure = Par.regr.mass.battery_structural.eq;       

% Values for the Z dimensional chain (the battery cooling is alreadyincludedin CZ underfloor) (Koehler, pp. 73)
CZ_top_cover = v.dimensions.CZ.batt_top_cover;                              % in mm
CZ_bottom_cover = v.dimensions.CZ.batt_bottom_cover;                        % in mm
EZ_free_space_mod2cover = v.dimensions.EZ.free_space_module_to_top_cover;   % in mm

% Installation space underfloor and tunnel
TS=v.battery.spacetable;
installationspace_underfloor=table2array(TS('underfloor',{'CX','CY','CZ'}));    % in mm
installationspace_tunnel=table2array(TS('tunnel',{'CX','CY','CZ'}));            % in mm
installationspace_sec_lev=table2array(TS('second level',{'CX','CY','CZ'}));     % in mm

% Settings
fill_sec_lev=v.settings.fill_second_level;

% Installation space small overlap (all in mm)
CX_small_overlap=v.battery.installationspace.CX_batt_small_overlap;
CY_small_overlap=v.battery.installationspace.CY_batt_small_overlap;

% Masses of the battery electrics components (cables, relais, BMS) (Koehler, p. 78)
mass_elec_AWD=Par.masses.battery_electrics.AWD;         % Masses of battery if the vehicle is AWD in kg
mass_elec_noAWD=Par.masses.battery_electrics.noAWD;     % Masses if the vehicle is not AWD in kg

% Mass ratio from cell to module -> f_mod= (mass_cell+mass_module)/mass_cell
% Considers the added mass of the module housing and module cables (Koehler, p. 77)
f_mod_pouch = Par.battery.massfactor_cell_to_module.pouch;
f_mod_pris = Par.battery.massfactor_cell_to_module.prismatic;
f_mod_cyl = Par.battery.massfactor_cell_to_module.cylindrical;        

%% 2) Caluclate the volumetric energy density of the battery
% 2a) To calculate the volumetric density of the underfloor battery we also have to consider the volume of the battery cover and the cooling
    installationspace_underfloor(3)=installationspace_underfloor(3)+CZ_top_cover+CZ_bottom_cover+EZ_free_space_mod2cover;

    % Volume of the battery housing in liters
    volume_battery_underfloor = prod(installationspace_underfloor)/10^6;

% 2b) Volume of the tunnel in liters
    volume_tunnel=prod(installationspace_tunnel)/10^6;

% 2c) Volume of the small overlap area
    % Basis area of the small overlap section (simplified as trapeziodal section)
    basis_area_small_overlap=max(CX_small_overlap)*(max(CY_small_overlap)+min(CY_small_overlap))*0.5;

    % Volume of the small overlap area, also here correct the total height
    volume_small_overlap=basis_area_small_overlap*(installationspace_underfloor(3))/10^6;
    
% 2d) Volume of the second level

    % The second level space is calculated regardless whether is filled or
    % not-> Ensure that the option is activated. Otherwise do not count it
    % in the total volume
    if fill_sec_lev
        installationspace_sec_lev(3) = installationspace_sec_lev(3)+CZ_bottom_cover+EZ_free_space_mod2cover;

        %Volume of the second level in liters
        volume_battery_sec_lev = prod(installationspace_sec_lev)/10^6;
    else
        volume_battery_sec_lev=0;
    end

% 2e) Total volume of the battery and volumetric energy density
    volume_battery_total= volume_tunnel + volume_small_overlap + volume_battery_underfloor+volume_battery_sec_lev;

    % Volumetric energy density on the battery level
    volumetric_energy_density=battery_energy/volume_battery_total;

%% 3) Calculate the mass of the battery: Mass split into three shares: cells, electrical and structural components with cooling
% 3a) Mass of the cells and modules:

    % Total mass of cells in kg, calculated with the volumetric energy density (Koehler, p. 77)
    mass_cells =  battery_energy./ energy_density_gravimetric;
    
    % Mass of cell modules in kg, depending on the cell type the modules
    % have a different weight (Koehler, p. 77)
    switch cell_type
        case 'pouch'
            mass_modules = f_mod_pouch*mass_cells;
        case 'prismatic'
            mass_modules = f_mod_pris*mass_cells;
        case 'cylindrical'
            mass_modules = f_mod_cyl*mass_cells;
    end

% 3b) Mass electrical components (cables and so on) (Koehler, p. 78)
    if ~isempty(v.gearbox{1}) && ~isempty(v.gearbox{2}) %AWD
        
        mass_elec=mass_elec_AWD;
    else %RWD or FWD
        
        mass_elec=mass_elec_noAWD;
    end
    
% 3c) Mass of structural AND cooling components in kg (Koehler, p. 78)
    mass_structural = regr_battery_structure(volume_battery_total);

% 3d) Total gravimetric energy density at pack level
    gravimetric_energy_density = battery_energy/(mass_structural+mass_modules+mass_elec);
    
%% 4) Assign the outputs
v.masses.powertrain.battery_structural_components=mass_structural;
v.masses.powertrain.battery_cells=mass_modules;
v.masses.powertrain.battery_electrical=mass_elec;
v.battery.volumetric_energy_density=volumetric_energy_density;
v.battery.gravimetric_energy_density=gravimetric_energy_density;

end
