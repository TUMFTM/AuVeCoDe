function [cell_dim_second_level,pack_dim_second_level,num_cell_second_level]=calc_filling_second_level(v,second_level,varargin)
%% Description:
% Designed by: Adrian König (FTM, Technical University of Munich)
%-------------
% Created on: 01.05.2022
% ------------
% Version: Matlab2020b
%-------------
% Description:  This function fills the second level and calculates how many cells fit in
%               total. To fill the second level, the cells which fit in the underbody are
%               swapped in X and Y in their orientation to find the optimal configuration
% ------------
% Sources:  [1] Peter Köhler, Semi-physikalische Modellierung von Antriebsstrangkomponenten für Elektrofahrzeuge , Master Thesis, Institute of Automotive Technology, TUM, 2021
%           [2] Lorenzo Nicoletti, "Parametric Modeling of Battery Electric Vehicles in the Early Development Phase", Technical University of Munich, Institute of Automotive Technology, 2022
%           [3] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - varagin: is an optional variable, which is used only for highfloor and mixedfloor. It contains the cells which fit in the underfloor, and have therefore to be reused to fill the tunnel
%           - second_level:  The dimensions of the second level in mm (as triple x,y,z)
%           - vehicle(v): struct including the data of the vehicle
% ------------
% Output:   - cell_dim_second_level: Dimension of selected cells
%           - pack_dim_second_level: Dimension of second level battery pack
%           - num_cell_second_level: Number of cells
% ------------

%% Implementation
% 1) Read parameters
% 2) Calculation

%% 1) Read in parameters
batt_cooling = v.dimensions.CZ.batt_cooling;    % Height of the cooling plate in mm (Koehler, p. 76)
EZ_cell2module = v.dimensions.EZ.cell2module;   % Difference in height (Z-direction) between cell and module in mm (Koehler, p. 76)

%% 2) Calculation
%% Second level is deactivated
if v.settings.fill_second_level==0
    
    if ~isempty(varargin)
        cell_dim_second_level = int8(zeros(size(varargin{1})));
        pack_dim_second_level = int8(zeros(size(varargin{1})));
        num_cell_second_level = int8(zeros(size(varargin{1})));
    else
        cell_dim_second_level = v.battery.cell.allcelldim;
        pack_dim_second_level = v.battery.cell.allpackdim;
        num_cell_second_level = int8(zeros(size(pack_dim_second_level,1),3));
    end
    
%% Second level is activated
else

    if ~isempty(varargin) % The cells have to be swapped in x and y only if the come from the underfloor!!!
        % Retrieve cell dimensions from the underfloor
        cell_dim = varargin{1};
        packfactor = v.battery.cell.packagefactor;

        % Allow a swapping only in x and y direction -> Same strategy as the small Overlap
        cell_layout = zeros(size(cell_dim,1),6);
        cell_layout(:,1:3) = cell_dim;
        cell_layout(:,4:6) = [cell_dim(:,2),cell_dim(:,1),cell_dim(:,3)]; % Swap x and y directions

        % Calculate the corresponding pack layouts
        pack_layout = zeros(size(cell_dim,1),6);
        pack_layout(:,1:3) = cell_layout(:,1:3).*packfactor;
        pack_layout(:,4:6) = cell_layout(:,4:6).*packfactor;    % Swap x and y directions

        % Add to the pack dimensions the required dimensions for the battery cooling and the module housing
        pack_layout(:,3) = pack_layout(:,3)+batt_cooling+EZ_cell2module;
        pack_layout(:,6) = pack_layout(:,6)+batt_cooling+EZ_cell2module;

        % Calculate the number of cell fitting for each configuration
        num_cell_second_level_layout = [floor(second_level./pack_layout(:,1:3)),floor(second_level./pack_layout(:,4:6))];
        num_cell_second_level_total = [prod(num_cell_second_level_layout(:,1:3),2),prod(num_cell_second_level_layout(:,4:6),2)];

        % Find the column with the highest number of cells
        [~,id_max] = max(num_cell_second_level_total,[],2);

        % Identify for every i_th cell, the rotation which has the highest integration potential and assign it as an output
        num_cell_second_level = zeros(numel(id_max),3); % Preallocate the vector to speed up calculation
        cell_dim_second_level = num_cell_second_level;  % Preallocate the vector to speed up calculation
        pack_dim_second_level = num_cell_second_level;  % Preallocate the vector to speed up calculation

        for i=1:numel(id_max)

            id = id_max(i);
            num_cell_second_level(i,:) = num_cell_second_level_layout(i,id*3-2:id*3);
            cell_dim_second_level(i,:) = cell_layout(i,id*3-2:id*3);
            pack_dim_second_level(i,:) = pack_layout(i,id*3-2:id*3);

        end    

    else

        % If this option is activated, it means there is no underfloor compartement (lowfloor integration)
        cell_dim_second_level=v.battery.cell.allcelldim;
        pack_dim_second_level=v.battery.cell.allpackdim;

        % How many cells can fit into the underbody (calculate considering the package factors)
        num_cell_second_level = floor(second_level./pack_dim_second_level);    

    end

end
end
