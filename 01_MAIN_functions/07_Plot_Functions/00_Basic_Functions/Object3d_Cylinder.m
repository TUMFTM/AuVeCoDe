function [F_c,F_c_b,V_c]=Object3d_Cylinder(p,eu,r,h,sc)
%% Description:
% Designed by: Adrian König (FTM, Technical University of Munich)
%-------------
% Created on: 18.12.2019
% ------------
% Version: Matlab2020b
%-------------
% Description: %Designs a 3d cylinder with the geometrical size defined by r, h and the
%               level of detail sc
% ------------
% Sources:  [1] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
%           [2] https://www.sky-engin.jp/MATLABAnimation/chap06/chap06.html
% ------------
% Input:    - eu=rotation with euler angles
%           - p=position offset in x,y, and z direction
%           - r =Radius
%           - h =Height
%           - sc = Sidecount of cylinder (surface detail level)
% ------------
% Output:   - F_c=Face of Cylinder
%           - V_c= Vertices of Cylinder
% ------------

%% Calculation of vertices and faces
% Reference orientation
R = Object3d_rotate(eu);

% Vertices
vertices_0 = zeros(2*sc, 3);
for i = 1:sc
    theta = 2*pi/sc*(i-1);
    vertices_0(i,:) = [r*cos(theta), r*sin(theta), 0];
    vertices_0(sc+i,:) = [r*cos(theta), r*sin(theta), h];
end

V_c = p' + vertices_0*R';

% Side faces
F_c = zeros(sc, 4);
for i = 1:(sc-1)
    F_c(i,:) = [i, i+1, sc+i+1, sc+i];
end
F_c(sc,:) = [sc, 1, sc+1, 2*sc];

% Bottom faces
F_c_b = [
    1:sc;
    (sc+1):2*sc];

