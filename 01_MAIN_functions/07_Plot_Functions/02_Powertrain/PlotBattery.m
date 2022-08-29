function PlotBattery(v)
%% Description:
% Designed by: Adrian König (FTM, Technical University of Munich), Lorenzo Nicoletti (FTM, Technical University of Munich)
%-------------
% Created on: 01.04.2022
% ------------
% Version: Matlab2020b
%-------------
% Description: This function plots the battery cells
% ------------
% Sources:      [1] Lorenzo Nicoletti, "Parametric Modeling of Battery Electric Vehicles in the Early Development Phase", Technical University of Munich, Institute of Automotive Technology, 2022
%               [2] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - vehicle: struct with includes the data of the vehicle
% ------------
% Output:   - Plot of battery cells
% ------------

%% Implementation:
% 1) Read width of vehicle
% 2) Subfunctions

%% 1) Read width of vehicle
width=v.dimensions.GY.vehicle_width;

if v.battery.energy_is_gross_in_kWh>0
    
    cell_type=v.battery.cell.cell_type;
    ft=v.battery.filltable;
    st=v.battery.spacetable;
    cooling=v.dimensions.CZ.batt_cooling;

    for i=1:numel(ft.Properties.RowNames)
        
        str=ft.Properties.RowNames{i};
        plot_cells(cell_type,...
              table2array(ft(str,{'NumcellX','NumcellY','NumcellZ'})),...
              table2array(ft(str,{'CelldimX','CelldimY','CelldimZ'})),...
              table2array(ft(str,{'PackdimX','PackdimY','PackdimZ'})),...
              table2array(st(str,'EX')),table2array(st(str,'EZ')),...
              cooling,width);
    end

end


end
%% 2) Subfunctions
function plot_cells(cell_type,cell_num,cell_dim,pack_dim,EX_batt,EZ_batt,cooling,width)
spacing = (pack_dim-cell_dim)/2;                 % distance between 2 cells [X,Y,Z]

%Set the starting point for filling the battery (outermost foremost cell)
origin_initial = [EX_batt + spacing(1);                             % X origin           
                  (width-pack_dim(2)*cell_num(2))/2 + spacing(2);   % Y origin
                  EZ_batt + cooling];                               % Z origin

if strcmp(cell_type,'cylindrical') && prod(cell_num)>0    %For cylindrical cells, the origin needs to be adjusted -> see function description
    %The plot function for prismatic or pouch cells positions the origin (which
    %is used as reference to plot the cell in the vehicle) in the lowest angle
    %of the cube. In order to plot cylinders, the MATLAB function cylinder is
    %required. Latter positions the origin in the center of the cylinder, on
    %its symmetry axis. In order for the plot battery function to work, in the
    %case of cylindrical cells the origin has to be shifted of an offset equal
    %to the radius of the cylinder.

    %Identify the radius of the cell (in mm)
    radius = cell_dim([cell_dim(1)==cell_dim(2),cell_dim(2)==cell_dim(3),cell_dim(3)==cell_dim(1)])/2;

    %Identify the position of the cell vector, which have a value equal to the
    %radius. These position have to be corrected (see description above)
    id_circle=[cell_dim(1)==radius*2,cell_dim(2)==radius*2,cell_dim(3)==radius*2];

    %Correct the origin with an offset equal to the radius
    origin_initial(id_circle)=origin_initial(id_circle)+radius;
end

% calculate origin of each cell, shifting it to its [x y z] position in the grid
for x = 1:cell_num(1)
    
    for y = 1:cell_num(2)
        
        for z = 1:cell_num(3)
            %Recalculate new origin (for the soon to be plotted cell)
            origin = [origin_initial(1) + (x-1)*pack_dim(1); ... %X coordinate
                      origin_initial(2) + (y-1)*pack_dim(2); ... %Y coordinate
                      origin_initial(3) + (z-1)*pack_dim(3); ];  %Z coordinate

            if strcmp(cell_type,'cylindrical')       
                plot_cylidrical_cell(cell_dim,origin,x);   % Plot cylinders for cylindrical cells               
            else               
                plot_prismatic_cell(origin',cell_dim);   % Plot cuboids for prismatic and pouch cells   
            end
        end
        
    end
    
end
end

function plot_cylidrical_cell(cell_dim,origin,row)
%% Description
%This function plots the cylindrical cells starting from the MATLAB
%function cylinder. As the function cylinder defines the symmetry axis of
%the cylinder by default on the Z-axis, the resulting cylinder has to be
%rotated accroding to the calculated cell orientation.

%Orange color for the cells
Color=[227,114,34]./255;

radius = cell_dim([cell_dim(1)==cell_dim(2),cell_dim(2)==cell_dim(3),cell_dim(3)==cell_dim(1)])/2;

%Define cylinder having the same radius as the cell
[X0,Y0,Z0] = cylinder(radius,18);

% scale cylinder from unit height to cell height
height = cell_dim(~[cell_dim(2)-cell_dim(3) cell_dim(1)-cell_dim(3) cell_dim(1)-cell_dim(2)]);
Z0 = Z0*height;

%Find orientation of the cell and rotate the cylinder accordingly
if cell_dim(1)==height %the cell is rotated and its height is in X direction
    X=Z0+origin(1);
    Y=Y0+origin(2);
    Z=X0+origin(3);
elseif cell_dim(2)==height %the cell is rotated and its height is it Y direction
    X=X0+origin(1);
    Y=Z0+origin(2);
    Z=Y0+origin(3);
elseif cell_dim(3)==height %the cell is not rotated and its height is in Z direction
    X = X0 + origin(1);
    if rem(row,2)==0 %For the even row, shift the cells in Y direction!
        Y = Y0 + origin(2)+radius*cosd(45);
    else
        Y = Y0 + origin(2);
    end
    Z = Z0 + origin(3);
end

%Plot the cylinder 
surface(X,Y,Z,'FaceColor',Color);

end

function plot_prismatic_cell(origin,celldim)
%% Description
%This function plots the prismatic or pouch cell. It is based on the
%"voxel" function created by Suresh Joel.
%% Source
%Source: Suresh Joel (2020). Voxel (https://www.mathworks.com/matlabcentral/fileexchange/3280-voxel), MATLAB Central File Exchange. Retrieved July 16, 2020.

%Orange color for the cells
Color=[227,114,34]./255;

%Define all the position of the points of the prismatic/pouch cells
x=[origin(1)+[0 0 0 0 celldim(1) celldim(1) celldim(1) celldim(1)]; ...
        origin(2)+[0 0 celldim(2) celldim(2) 0 0 celldim(2) celldim(2)]; ...
        origin(3)+[0 celldim(3) 0 celldim(3) 0 celldim(3) 0 celldim(3)]]';

%Plot
for n=1:3
    if n==3
        x=sortrows(x,[n,1]);
    else
        x=sortrows(x,[n n+1]);
    end
    temp=x(3,:);
    x(3,:)=x(4,:);
    x(4,:)=temp;
    h=patch(x(1:4,1),x(1:4,2),x(1:4,3),Color,'LineWidth',0.05);
    set(h,'FaceAlpha',1);
    temp=x(7,:);
    x(7,:)=x(8,:);
    x(8,:)=temp;
    h=patch(x(5:8,1),x(5:8,2),x(5:8,3),Color,'LineWidth',0.05);
    set(h,'FaceAlpha',1);
    
end

end