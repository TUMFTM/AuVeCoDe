function Object3d_Cube(pos,dim,c,alpha,y_pos)
%% Description:
% Designed by: Adrian König (FTM, Technical University of Munich), Lorenzo Nicoletti (FTM, Technical University of Munich)
%-------------
% Created on: 01.05.2022
% ------------
% Version: Matlab2020b
%-------------
% Description: %This function plot a cube with the assumption, that the cube is simmetric
%               to the vehicle center (symmetric to the y axle). 
% ------------
% Sources:  [1] Lorenzo Nicoletti, "Parametric Modeling of Battery Electric Vehicles in the Early Development Phase", Technical University of Munich, Institute of Automotive Technology, 2022
%           [2] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% % ------------
% Input:    - pos: [x position and z_position] -> 1x2 vector containing the x and the y coordinates where the first face of the cube start. The y coordinate is not required, as we suppose symmetry towards this axle.
%           - dim: [length, width, height] -> vector containing the dimensions of the cube 
%           - c: Color of the cube expressed as string
%           - alpha: Transparency of the cube expressed as double (0 to 1)
% ------------
% Output:   - Plot of cube
% ------------



%% Implementation

%Thickness of the lines -> not visible in MATLAB but visible when you print
lwidth=0.05;

switch(nargin)
    case 0
        disp('Too few arguements for plotcube');
        return;
    case 1
        dim(1)=1;    %default length of side of voxel is 1
        dim(2)=1;
        dim(3)=1;
        c='b';  %default color of voxel is blue
        alpha=1;
    case 2
        c='b';
        alpha=1;
    case 3
        alpha=1;
    case 4
        %do nothing
    case 5
        %do nothing
    otherwise
        disp('Too many arguements for plotcube');
end

%define the position of the cube at the middle point of the yz surface
posx=pos(1);
posy_min=y_pos(1);
posy_max=y_pos(2);
posz=pos(2);

%Define length width and height
l=dim(1);
w=dim(2);
h=dim(3);

%Design the first face in the yz plane
faceyz(1,:) = [posx,posy_max,posz];
faceyz(2,:) = [posx,posy_min,posz];
faceyz(3,:) = [posx,posy_min,posz+h];
faceyz(4,:) = [posx,posy_max,posz+h];
%Plot both faces in yz plane
patch(faceyz(:,1),faceyz(:,2),faceyz(:,3),c,'FaceAlpha',alpha,'LineWidth',lwidth);
patch(faceyz(:,1)+l,faceyz(:,2),faceyz(:,3),c,'FaceAlpha',alpha,'LineWidth',lwidth);

%Design the first face in the xz plane
facexz(1,:) = faceyz(1,:); 
facexz(2,:) = faceyz(4,:);
facexz(3,:) = faceyz(4,:)+[l,0,0];
facexz(4,:) = faceyz(1,:)+[l,0,0];
%Plot both faces in xz plane
patch(facexz(:,1),facexz(:,2),facexz(:,3),c,'FaceAlpha',alpha,'LineWidth',lwidth);
patch(facexz(:,1),facexz(:,2)-w,facexz(:,3),c,'FaceAlpha',alpha,'LineWidth',lwidth);


%Design the first face in the xy plane
facexy(1,:) = faceyz(1,:);
facexy(2,:) = facexz(4,:);
facexy(3,:) = facexz(4,:)-[0,w,0];
facexy(4,:) = faceyz(2,:);
%Plot both faces in xy plane
patch(facexy(:,1),facexy(:,2),facexy(:,3),c,'FaceAlpha',alpha,'LineWidth',lwidth);
patch(facexy(:,1),facexy(:,2),facexy(:,3)+h,c,'FaceAlpha',alpha,'LineWidth',lwidth);

end