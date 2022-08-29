function [v,cell_dim_sov,pack_dim_sov,num_cell_sov]=calc_filling_small_overlap(v,small_overlap,cell_dim)
%% Description:
% Designed by: Adrian König (FTM, Technical University of Munich), Lorenzo Nicoletti (FTM, Technical University of Munich)
%-------------
% Created on: 01.05.2022
% ------------
% Version: Matlab2020b
%-------------
% Description:  In the small overlap area, the same cells as the underfloor has to be
%               used. Nevertheless, the cells can be swapped with respect to the
%               underfloor.
% ------------
% Sources:  [1] Peter Köhler, Semi-physikalische Modellierung von Antriebsstrangkomponenten für Elektrofahrzeuge , Master Thesis, Institute of Automotive Technology, TUM, 2021
%           [2] Lorenzo Nicoletti, "Parametric Modeling of Battery Electric Vehicles in the Early Development Phase", Technical University of Munich, Institute of Automotive Technology, 2022
%           [3] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - cell_dim: The set of all the usable cells
%           - small_overlap: The dimensions of the small overlap as vector [X,Y,Z]
%           - vehicle(v): struct including the data of the vehicle
% ------------
% Output:   - cell_dim_sov: Dimension of selected cells
%           - pack_dim_sov: Dimension of second level battery pack
%           - num_cell_sov: Number of cells
%           - vehicle(v): struct including the data of the vehicle
% ------------

%% Implementation
% 1) Read parameters
% 2) Calculation

%% 1) Read in parameters
batt_cooling = v.dimensions.CZ.batt_cooling;      %Height of the cooling plate in mm (Koehler, p. 76)
EZ_cell2module = v.dimensions.EZ.cell2module;     %Difference in height (Z-direction) between cell and module in mm (Koehler, p. 76)


%% 2) Calculation
%% Small overlap is deactivated
if v.settings.fill_smalloverlapspace==0 %The small overlap space is not activated for filling -> set all cell dim and num to 0
    
    cell_dim_sov = int8(zeros(size(cell_dim)));
    pack_dim_sov = int8(zeros(size(cell_dim)));
    num_cell_sov = int8(zeros(size(cell_dim)));

    
%% Small overlap is activated
else

    % Initialize the variables (this increases the calculation speed)
    pack_layout = zeros(size(cell_dim,1),6);
    cell_layout = zeros(size(cell_dim,1),6);

    % In the small overlap, the cells can be swapped with respect to the underfloor
    cell_layout(:,1:3) = cell_dim;                                      % The first 3 rows of cell_layout contain the cell_layout of the underbody
    cell_layout(:,4:6) = [cell_dim(:,2),cell_dim(:,1),cell_dim(:,3)];   % The last 3 rows of cell_layout contain the cell layout like the underbody with swapped x and y dimensions

    % Calculate the corresponding pack layout
    packfactor = v.battery.cell.packagefactor;
    pack_layout(:,1:3) = cell_layout(:,1:3).*packfactor;                % The first 3 rows of pack_layout contain the pack_layout of the underbody
    pack_layout(:,4:6) = cell_layout(:,4:6).*packfactor;                % The last 3 rows of pack_layout contain the pack layout like the underbody with swapped x and y dimensions
    
    % Add to the pack dimensions the required dimensions for the battery cooling
    pack_layout(:,3) = pack_layout(:,3)+batt_cooling+EZ_cell2module;
    pack_layout(:,6) = pack_layout(:,6)+batt_cooling+EZ_cell2module;

    % Calculate the resulting number of cells
    num_cell_small_overlap_layout = [floor(small_overlap./pack_layout(:,1:3)),floor(small_overlap./pack_layout(:,4:6))];
    
    % Calculate total number of cells
    % -> First column: How many cells fit using the orientation of the underfloor
    % -> Second column: How many cells fit swapping the orientation (swapping x and y)
    num_cell_small_overlap_total = [prod(num_cell_small_overlap_layout(:,1:3),2),prod(num_cell_small_overlap_layout(:,4:6),2)];

    % Find the column with the highest number of cells
    [~,id_max] = max(num_cell_small_overlap_total,[],2);

    % Identify for every i_th cell, the rotation which has the highest intergation potential and assign it as an output
    num_cell_sov = zeros(numel(id_max),3);  % Preallocate the vector to speed up calculation
    cell_dim_sov = num_cell_sov;            % Preallocate the vector to speed up calculation
    pack_dim_sov = num_cell_sov;            % Preallocate the vector to speed up calculation

    for i=1:numel(id_max)

        id=id_max(i);
        num_cell_sov(i,:) = num_cell_small_overlap_layout(i,id*3-2:id*3);
        cell_dim_sov(i,:) = cell_layout(i,id*3-2:id*3);
        pack_dim_sov(i,:) = pack_layout(i,id*3-2:id*3);

    end

    if sum(prod(num_cell_sov,2))==0
        disp('No cells fits in the small overlap area so the option "fill small overlap" will be deactivated');
        v.settings.fill_smalloverlapspace = 0;
        cell_dim_sov = int8(zeros(size(cell_dim)));
        pack_dim_sov = int8(zeros(size(cell_dim)));
        num_cell_sov = int8(zeros(size(cell_dim)));
    end
end

end