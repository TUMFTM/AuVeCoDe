function [v] = Package_cell_combinations(v,Par,varargin)
%% Description:
% Designed by: Adrian König (FTM, Technical University of Munich), Lorenzo Nicoletti (FTM, Technical University of Munich)
%-------------
% Created on: 01.02.2022
% ------------
% Version: Matlab2020b
%-------------
% Description:  This function has the main scope to calculate all the possible cell combination inside the given range length, width and height. Furthermore,
%               the function takes into account if the cells rotation have been blocked/unblocked and filters out the not needed cell rotations.
%               The calculated cells are used to fill the vehicle UNDERBODY. For the other battery spaces they have to be recalculated
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

%% Implementation:
%1) Initialize Inputs
%2) Calculate every possible (allowed) cell layout
%3) Initialize the tables where the battery model results will be stored
%4) Error Handling
%5) Assign Output

%% 1) Initialize Inputs
cell_type = v.Input.cell_type;        %Cell type: prismatic/pouch/cylindrical
cooling_in_Z=v.settings.cooling_in_z; %Binary value: 1=there is a cooling plate underneath the modules; 0: There is no cooling plate underneath the modules

%Check whether the cooling plate has to be considered for the Z-dimensional chain
if cooling_in_Z %Examples: Audi e-tron, Jaguar I-Pace, Mercedes EQC...
    CZ_batt_cooling=Par.dimensions.CZ.batt_cooling.cooling_in_Z;                      % Battery cooling tzhickness in Z in mm
    CZ_batt_bottom_cover = Par.dimensions.CZ.batt_bottom_cover.cooling_in_Z;       % Battery bottom cover in mm
else %Tesla Model 3 or vehicles with air cooling
    CZ_batt_cooling=Par.dimensions.CZ.batt_cooling.no_cooling_in_Z;                   % Battery cooling tzhickness in Z in mm
    CZ_batt_bottom_cover = Par.dimensions.CZ.batt_bottom_cover.no_cooling_in_Z;    % Battery bottom cover in mm
end

%Assign the cell parameters according to the choosen cell
switch cell_type
    case 'prismatic'

        packagefactor  =Par.battery.packagefactor.prismatic;       %Assign the package factors for prismatic cells
        energy_density_volumetric = Par.battery.cell_energy_density_vol.prismatic ;                                          %Wh/l -> example BMW i3
        energy_density_grav=Par.battery.cell_energy_density_grav.prismatic ;                                          %Wh/kg -> example BMW i3
        cell_x         = Par.battery.cell.dimensions_x.prismatic(1):Par.battery.cell.dimensions_x.prismatic(2);                                   %Pouch 230:340; Prismatic 40:80; Cylindric 18:21
        cell_y         = Par.battery.cell.dimensions_y.prismatic(1):Par.battery.cell.dimensions_y.prismatic(2);                                        %Pouch 7:14; Prismatic 150:170; -> For cylindric is not required!!
        cell_z         = Par.battery.cell.dimensions_z.prismatic(1):Par.battery.cell.dimensions_z.prismatic(2);                                 %Pouch 100:230; Prismatic 100:120; Cylindric 65:70
        cell_dim_possibilites = {cell_x , cell_y, cell_z};     %Get the ranges for length, width and height
        EZ_cell2module=Par.dimensions.EZ.offset_cell2module.prismatic;     %Difference in height (Z direction) between cell and module
    
    case 'pouch'
        packagefactor  =Par.battery.packagefactor.pouch;       %Assign the package factors for prismatic cells
        energy_density_volumetric = Par.battery.cell_energy_density_vol.pouch ;                                          %Wh/l -> example BMW i3
        energy_density_grav=Par.battery.cell_energy_density_grav.pouch ;                                          %Wh/kg -> example BMW i3
        cell_x         = Par.battery.cell.dimensions_x.pouch(1):Par.battery.cell.dimensions_x.pouch(2);                                   %Pouch 230:340; Prismatic 40:80; Cylindric 18:21
        cell_y         = Par.battery.cell.dimensions_y.pouch(1):Par.battery.cell.dimensions_y.pouch(2);                                        %Pouch 7:14; Prismatic 150:170; -> For cylindric is not required!!
        cell_z         = Par.battery.cell.dimensions_z.pouch(1):Par.battery.cell.dimensions_z.pouch(2);                                 %Pouch 100:230; Prismatic 100:120; Cylindric 65:70
        cell_dim_possibilites = {cell_x , cell_y, cell_z};     %Get the ranges for length, width and height
        EZ_cell2module=Par.dimensions.EZ.offset_cell2module.pouch;
    
    case 'cylindrical'
        packagefactor  =Par.battery.packagefactor.cylindrical;     %Assign the package factors for cylindrical cells 
        energy_density_volumetric= Par.battery.cell_energy_density_vol.cylindrical;                                    %Wh/l -> example Tesla Model 3
        energy_density_grav=Par.battery.cell_energy_density_grav.cylindrical;                                          %Wh/kg -> example Tesla Model 3
        cell_x         = Par.battery.cell.dimensions_x.cylindrical(1):Par.battery.cell.dimensions_x.cylindrical(2);    %Pouch 230:340; Prismatic 40:80; Cylindric 18:21
        cell_z         = Par.battery.cell.dimensions_z.cylindrical(1):Par.battery.cell.dimensions_z.cylindrical(2);    %Pouch 100:230; Prismatic 100:120; Cylindric 65:70
        cell_dim_possibilites = {cell_x , cell_z};                                                                     %For cilindrical cells only the length (diameter) and the height is required
        EZ_cell2module=Par.dimensions.EZ.offset_cell2module.cylindrical;
end

%% 2) Calculate every possible (allowed) cell layout
%The only allowed rotational degree for the cells is the one along the vertical direction
swap_width_length=1;   %1: The cell may be rotated so, that its lenght and width are swapped; 0: This rotation is blocked
swap_length_heigth=0;  %1: The cell may be rotated so, that its length is parallel to the Z-direction; 0: This rotation is blocked
swap_width_heigth =0;  %1: The cell may be rotated so, that its width  is parallel to the Z-direction; 0: This rotation is blocked

%Calculate every possible resulting cell sizes combination by varying width, length and height in the given ranges
if isempty(varargin)
    
    if strcmp(cell_type,'cylindrical')
        %X and Y dimensions are equal, since the section of the cell is a circle
        [x, z] = ndgrid(cell_dim_possibilites{:});
        cell_dim = [x(:) x(:) z(:)];   
    else 
        %Pouch and prismatic cells
        [x, y, z] = ndgrid(cell_dim_possibilites{:});
        cell_dim = [x(:) y(:) z(:)];
    end
    v.battery.imposed_cell_database='There is no imposed cell database';
else
    
    v.battery.imposed_cell_database=varargin{1};
    cell_dim=[varargin{1}.X,varargin{1}.Y,varargin{1}.Z];
    
end

%Create the Permutation matrix: Required for rotating the cells!
permmat(1:2,:)=[1,2,3;2,1,3]; %The cell is orientated so, that its height is placed parallel to the Z direction
permmat(3:4,:)=[3,2,1;2,3,1]; %The cell is orientated so, that its length is placed parallel to the Z direction
permmat(5:6,:)=[1,3,2;3,1,2]; %The cell is orientated so, that its width  is placed parallel to the Z direction

%Eliminate from the permutation matriy the rotations/swappings which are not allowed
filter_permmat=[swap_width_length,swap_width_length,swap_length_heigth,swap_length_heigth,swap_width_heigth,swap_width_heigth]';
if ~isempty(find(filter_permmat==0))
    permmat(find(filter_permmat==0),:)=[];    
end

%Number of possible combinations (layouts) for each cell (minimum 2, maximum 6)
combinations=size(permmat,1);

%Permutate the cell based on the roational degrees of freedom and calculate all possible cells layouts
for i=0:size(cell_dim,1)-1
    
    idx=i*(combinations)+1;
    orig_cell=[cell_dim(i+1,1),cell_dim(i+1,2),cell_dim(i+1,3)]; %Basis cell described as [Length, Width, Height]
    all_allowed_layouts=orig_cell(permmat);                      %Create the allowed rotations from the basis cells
    allcelldim(idx:idx+(combinations-1),:)=all_allowed_layouts;  %Append these rotation to the matrix storing all possible cells
end

%Remove duplicates (duplicates exist only if the x,y or z dimensions are in similar ranges)
allcelldim = unique(allcelldim,'rows');

%Calculate the resulting package dimensions for each cell
allpackdim = allcelldim.*packagefactor;

%Add cooling plate and cell module housing height to the package factor in Z direction
allpackdim(:,3)=allpackdim(:,3)+CZ_batt_cooling+EZ_cell2module;

%% 3) Initialize the tables where the battery model results will be stored
folderpath=mfilename('fullpath');
folderpath=folderpath(1:findstr(folderpath,mfilename)-1);
load([folderpath,'Battery_spacetable.mat']);
load([folderpath,'Battery_filltable.mat']);
v.battery.spacetable=spacetable;    %Will store the dimensions of the battery spaces
v.battery.filltable=filltable;      %Will store the number of cells fitting in each space 


%% 4) Error handling
%For some reason, the cell dimensions may contain NaN. If that is the case,
%the tool will enter an infinite loop. Avoid this by breaking the code here
if any(isnan(allpackdim(:,1))) || any(isnan(allpackdim(:,2))) || any(isnan(allpackdim(:,3)))
    disp('UNEXPECTED ERROR: SOME OF THE POSSIBLE CELL DIMENSIONS HAVE NaN VALUES: PLEASE CHECK THE INITIALIZATION IN THE FUNCTION: initialize_cell_combinations')
    return
    
elseif any((allpackdim(:,1).*allpackdim(:,2).*allpackdim(:,3))==0)
    disp('UNEXPECTED ERROR: SOME OF THE POSSIBLE CELL DIMENSIONS HAVE NULL VALUES: PLEASE CHECK THE INITIALIZATION IN THE FUNCTION: initialize_cell_combinations')
end

%% 5) Assign outputs
v.dimensions.CZ.batt_cooling = CZ_batt_cooling;               %Thickness of the cooling plate (in mm) along the Z direction
v.dimensions.CZ.batt_bottom_cover=CZ_batt_bottom_cover;       %Thickness of the battery bottom cover (in mm) along the Z direction
v.dimensions.EZ.cell2module = EZ_cell2module;                 %Difference in height (Z direction) between cell and module
v.battery.cell.packagefactor = packagefactor;                 %Packagefactor used for the calculation
v.battery.cell.allcelldim=allcelldim;                         %Cell dimensions of all the simulated cells
v.battery.cell.allpackdim=allpackdim;                         %Pack dimensions of all the simulated cells
v.battery.cell.energy_density_vol=energy_density_volumetric;  %Volumetric energy desnity at the cell level in Wh/l
v.battery.cell.energy_density_grav=energy_density_grav;       %Gravimetric energy density at the cell level in Wh/kg
v.battery.cell.cell_type= cell_type;                          %Type of cell (in this case always cylindrical


end

