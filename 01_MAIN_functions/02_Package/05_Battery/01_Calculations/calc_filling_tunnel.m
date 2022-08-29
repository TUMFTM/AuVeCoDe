function [cell_dim_tunnel,pack_dim_tunnel,num_cell_tunnel]=calc_filling_tunnel(v,tunnel,varargin)
%% Description:
% Designed by: Adrian König (FTM, Technical University of Munich), Lorenzo Nicoletti (FTM, Technical University of Munich)
%-------------
% Created on: 01.05.2022
% ------------
% Version: Matlab2020b
%-------------
% Description:  This function fills the tunnel and calculates how many cell fits in
%               total. To fill the tunnel, the cells which fit in the underbody are
%               swapped in each possible orientation to find the optimal configuration
% ------------
% Sources:  [1] Peter Köhler, Semi-physikalische Modellierung von Antriebsstrangkomponenten für Elektrofahrzeuge , Master Thesis, Institute of Automotive Technology, TUM, 2021
%           [2] Lorenzo Nicoletti, "Parametric Modeling of Battery Electric Vehicles in the Early Development Phase", Technical University of Munich, Institute of Automotive Technology, 2022
%           [3] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - varagin: is an optional variable, which is used only for highfloor and mixedfloor. It contains the cells which fit in the underfloor, and have therefore to be reused to fill the tunnel
%           - tunnel:  The dimensions of the tunnel in mm (as triple x,y,z)
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
batt_cooling = v.dimensions.CZ.batt_cooling;      % Height of the cooling plate in mm  (Koehler, p. 76)
EZ_cell2module = v.dimensions.EZ.cell2module;     % Difference in height (Z-direction) between cell and module in mm  (Koehler, p. 76)

%% 2) Calculation

%% Tunnel is deactivated
if v.settings.fill_tunnel==0 
    if ~isempty(varargin)
        cell_dim_tunnel = int8(zeros(size(varargin{1})));
        pack_dim_tunnel = int8(zeros(size(varargin{1})));
        num_cell_tunnel = int8(zeros(size(varargin{1})));
    else
        cell_dim_tunnel = v.battery.cell.allcelldim;
        pack_dim_tunnel = v.battery.cell.allpackdim;
        num_cell_tunnel = int8(zeros(size(pack_dim_tunnel,1),3));
    end    
   
%% Tunnel is activated
else
    
    if ~isempty(varargin)   % Use the cells which fit in the underfloor and swap them in every possible orientation -> Idea: we have to use the same cell, but not necessarly the same orientation!
    
        cell_dim = varargin{1}; %Cell which fit in the underfloor
        packfactor = v.battery.cell.packagefactor;

        if strcmpi(v.Input.cell_type,'cylindrical')
            
            % For cylindrical cells due to the simmetry of the cell there is no need to test all the swappings
            cell_layout(:,1:3) = cell_dim;
            cell_layout(:,4:6) = [cell_dim(:,1),cell_dim(:,3),cell_dim(:,2)];   % Swap y and z directions
            cell_layout(:,7:9) = [cell_dim(:,3),cell_dim(:,2),cell_dim(:,1)];   % Swap x and z directions

            % Calculate corresponding packlayouts
            pack_layout(:,1:3) = cell_layout(:,1:3).*packfactor;
            pack_layout(:,4:6) = cell_layout(:,4:6).*packfactor;    % Swap y and z directions
            pack_layout(:,7:9) = cell_layout(:,7:9).*packfactor;    % Swap y and z directions
            
            % Add the space in Z-direction required for the battery cooling
            pack_layout(:,3) = pack_layout(:,3)+batt_cooling+EZ_cell2module;
            pack_layout(:,6) = pack_layout(:,6)+batt_cooling+EZ_cell2module;
            pack_layout(:,9) = pack_layout(:,9)+batt_cooling+EZ_cell2module;
            
            % Calculate the number of cell fitting for each configuration
            num_cell_tunnel_layout = [floor(tunnel./pack_layout(:,1:3)),floor(tunnel./pack_layout(:,4:6)),floor(tunnel./pack_layout(:,7:9))];
            num_cell_tunnel_total = [prod(num_cell_tunnel_layout(:,1:3),2),prod(num_cell_tunnel_layout(:,4:6),2),prod(num_cell_tunnel_layout(:,7:9),2)];   

        elseif strcmpi(v.Input.cell_type,'prismatic')   % For prismatic cells enable all possible rotations

            % Rotate each cells in all direction -> the cells in the tunnel do not necessarly need to be positioned like the cells in the underbody
            cell_layout = zeros(size(cell_dim,1),18);
            cell_layout(:,1:3) = cell_dim;
            cell_layout(:,4:6) = [cell_dim(:,1),cell_dim(:,3),cell_dim(:,2)];   % Swap y and z directions
            cell_layout(:,7:9) = [cell_dim(:,2),cell_dim(:,1),cell_dim(:,3)];   % Swap x and y directions
            cell_layout(:,10:12) = [cell_dim(:,2),cell_dim(:,3),cell_dim(:,1)]; % Multiple swapping
            cell_layout(:,13:15) = [cell_dim(:,3),cell_dim(:,1),cell_dim(:,2)]; % Multiple swapping
            cell_layout(:,16:18) = [cell_dim(:,3),cell_dim(:,2),cell_dim(:,1)]; % Multiple swapping
 
            % Calculate corresponding packlayouts
            pack_layout = zeros(size(cell_dim,1),18);
            pack_layout(:,1:3) = cell_dim.*packfactor;
            pack_layout(:,4:6) = cell_layout(:,4:6).*packfactor;        % Swap y and z directions
            pack_layout(:,7:9) = cell_layout(:,7:9).*packfactor;        % Swap y and z directions
            pack_layout(:,10:12) = cell_layout(:,10:12).*packfactor;    % Swap y and z directions
            pack_layout(:,13:15) = cell_layout(:,13:15).*packfactor;    % Swap y and z directions
            pack_layout(:,16:18) = cell_layout(:,16:18).*packfactor;    % Swap y and z directions
            
            % Add the space required for the battery cooling along the z direction
            pack_layout(:,3) = pack_layout(:,3)+batt_cooling+EZ_cell2module;
            pack_layout(:,6) = pack_layout(:,6)+batt_cooling+EZ_cell2module;
            pack_layout(:,9) = pack_layout(:,9)+batt_cooling+EZ_cell2module;
            pack_layout(:,12) = pack_layout(:,12)+batt_cooling+EZ_cell2module;
            pack_layout(:,15) = pack_layout(:,15)+batt_cooling+EZ_cell2module;
            pack_layout(:,18) = pack_layout(:,18)+batt_cooling+EZ_cell2module;

            % Calculate the number of cell fitting for each orientation (expressed as triple x,y,z)
            num_cell_tunnel_layout = [floor(tunnel./pack_layout(:,1:3)),floor(tunnel./pack_layout(:,4:6)),...
                         floor(tunnel./pack_layout(:,7:9)),floor(tunnel./pack_layout(:,10:12)),...
                         floor(tunnel./pack_layout(:,13:15)),floor(tunnel./pack_layout(:,16:18))];

            % Calculate the total number of cells filling         
            num_cell_tunnel_total = [prod(num_cell_tunnel_layout(:,1:3),2),prod(num_cell_tunnel_layout(:,4:6),2),...
                                   prod(num_cell_tunnel_layout(:,7:9),2),prod(num_cell_tunnel_layout(:,10:12),2),...
                                   prod(num_cell_tunnel_layout(:,13:15),2),prod(num_cell_tunnel_layout(:,15:18),2)];   
         
        elseif strcmpi(v.Input.cell_type,'pouch')   % For pouch cells enable only rotation along the z direction

            % Rotate each cells in all direction -> the cells in the tunnel do not necessarly need to be positioned like the cells in the underbody
            cell_layout = zeros(size(cell_dim,1),6);
            cell_layout(:,1:3) = cell_dim;
            cell_layout(:,4:6) =[cell_dim(:,2),cell_dim(:,1),cell_dim(:,3)];    % Swap x and y directions
            
            % Calculate corresponding packlayouts
            pack_layout = zeros(size(cell_dim,1),6);
            pack_layout(:,1:3) = cell_dim.*packfactor;
            pack_layout(:,4:6) = cell_layout(:,4:6).*packfactor;    % Swap y and z directions
            
            % Add the space required for the battery cooling along the z direction
            pack_layout(:,3) = pack_layout(:,3)+batt_cooling+EZ_cell2module;
            pack_layout(:,6) = pack_layout(:,6)+batt_cooling+EZ_cell2module;

            % Calculate the number of cell fitting for each orientation (expressed as triple x,y,z)
            num_cell_tunnel_layout = [floor(tunnel./pack_layout(:,1:3)),floor(tunnel./pack_layout(:,4:6))];

            % Calculate the total number of cells filling         
            num_cell_tunnel_total = [prod(num_cell_tunnel_layout(:,1:3),2),prod(num_cell_tunnel_layout(:,4:6),2)];   
        end

        % Find the column with the highest number of cells
        [~,id_max] = max(num_cell_tunnel_total,[],2);

        % Identify for every i_th cell, the rotation which has the highest intergation potential and assign it as an output
        num_cell_tunnel = zeros(numel(id_max),3);
        cell_dim_tunnel = num_cell_tunnel;
        pack_dim_tunnel = num_cell_tunnel;

        for i=1:numel(id_max)
            id = id_max(i);
            num_cell_tunnel(i,:) = num_cell_tunnel_layout(i,id*3-2:id*3);
            cell_dim_tunnel(i,:) = cell_layout(i,id*3-2:id*3);
            pack_dim_tunnel(i,:) = pack_layout(i,id*3-2:id*3);
        end

    else
        
        % There are no cells in the underfloor. This configuration is the lowfloor variant       
        cell_dim_tunnel = v.battery.cell.allcelldim;
        pack_dim_tunnel = v.battery.cell.allpackdim;
        
        % How many cells can fit into the underbody (calculate considering the package factors)
        num_cell_tunnel = floor(tunnel./pack_dim_tunnel);    
        
    end
end

end
