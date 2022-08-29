function [cell_dim_underfloor,pack_dim_underfloor,no_cells_underfloor]=calc_filling_underfloor(v, underfloor_dim)
%% Description:
% Designed by: Adrian König (FTM, Technical University of Munich), Lorenzo Nicoletti (FTM, Technical University of Munich)
%-------------
% Created on: 01.02.2022
% ------------
% Version: Matlab2020b
%-------------
% Description:  This function fills the underfloor and calculates how many cell fit in in total.
% ------------
% Sources:  [1] Peter Köhler, Semi-physikalische Modellierung von Antriebsstrangkomponenten für Elektrofahrzeuge , Master Thesis, Institute of Automotive Technology, TUM, 2021
%           [2] Lorenzo Nicoletti, "Parametric Modeling of Battery Electric Vehicles in the Early Development Phase", Technical University of Munich, Institute of Automotive Technology, 2022
%           [2] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - underfloor_dim:  The dimensions of the tunnel in mm (as triple x,y,z)
%           - vehicle(v): struct including the data of the vehicle
% ------------
% Output:   - cell_dim_underfloor: Dimension of selected cells
%           - pack_dim_underfloor: Dimension of underfloor battery pack
%           - num_cell_underfloor: Number of cells
% ------------

%% Implementation
% 1) Read parameters
% 2) Calculation

%% 1) Read parameters
% underfloor_dim: The dimensions of the underfloor as vector [X,Y,Z]
cell_dim_underfloor = v.battery.cell.allcelldim; %All the possible cells
pack_dim_underfloor = v.battery.cell.allpackdim; %The corresponding pack dimensions

%% 2) Calculation
% How many cells can fit into the underfloor (calculate considering the package factors)
no_cells_underfloor = floor(underfloor_dim./pack_dim_underfloor);

% Identify the cell combinations, for which the number of cells in x,y or z direction is 0, and filter them out, as they do not need to be further simulated
empty_underfloor_id = find((no_cells_underfloor(:,1).*no_cells_underfloor(:,2).*no_cells_underfloor(:,3))==0);

if numel(empty_underfloor_id)==size(no_cells_underfloor,1)  % No cell fits in the underfloor
   
    no_cells_underfloor=int8(zeros(size(pack_dim_underfloor,1),3));
    
else % Some (or all) cells fit in the underfloor

    % Filter out the cells which do not fit in the underfloor. They will not
    % be used to fill the tunnel or second level either!
    no_cells_underfloor(empty_underfloor_id,:)=[];

    % Rewrite packdim and celldim keeping only the cells which fit in the underfloor
    pack_dim_underfloor(empty_underfloor_id,:)=[];
    cell_dim_underfloor(empty_underfloor_id,:)=[];   
end
