function [F_c,F_c_b,V_c] = Object3d_Cylinder_hollow(p,eu,r_o,r_i,h,sc)
%% Description:
% Designed by: Adrian König (FTM, Technical University of Munich), Korbinian Moller
%-------------
% Created on: 01.09.2021
% ------------
% Version: Matlab2020b
%-------------
% Description: %Designs a hollow 3d cylinder with the geometrical size defined by r_i, r_o, h and the
%               level of detail sc
% ------------
% Sources:  [1] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
%           [2] https://www.sky-engin.jp/MATLABAnimation/chap06/chap06.html
% ------------
% Input:    - eu=rotation with euler angles
%           - p=position offset in x,y, and z direction
%           - r_o    = Outer Radius [mm]
%           - r_i    = Inner Radius [mm]
%           - h =Height [mm]
%           - sc = Sidecount of cylinder (surface detail level)
% ------------
% Output:   - F_c=Face of Cylinder
%           - V_c= Vertices of Cylinder
% ------------

%% Calculation of vertices and faces
% Reference orientation
R = Object3d_rotate(eu);

% Vertices
vertices = zeros(4*sc, 3);
for i = 1:sc
    theta = 2*pi/sc*(i-1);
    vertices(i,:) = [r_i*cos(theta), r_i*sin(theta), 0];
    vertices(sc+i,:) = [r_i*cos(theta), r_i*sin(theta), h];
    vertices(2*sc+i,:) = [r_o*cos(theta), r_o*sin(theta), 0];
    vertices(3*sc+i,:) = [r_o*cos(theta), r_o*sin(theta), h]; 
end

V_c = p' + vertices * R';

% Side faces
F_c = zeros(2*sc, 4);
for i = 1:(sc-1)
    F_c(i,:) = [i, i+1, sc+i+1, sc+i];
    F_c(sc + i,:) = [2*sc+i, 2*sc+i+1, 3*sc+i+1, 3*sc+i];
end
F_c(sc,:) = [sc, 1, sc+1, 2*sc];
F_c(2*sc,:) = [3*sc, 2*sc + 1, 3*sc+1, 4*sc];

% Bottom faces
F_c_b = zeros(2*sc, 4);
for i = 1:(sc-1)
    F_c_b(i,:) = [i, i+1, 2*sc+i+1, 2*sc+i];
    F_c_b(sc + i,:) = [sc+i, sc+i+1, 3*sc+i+1, 3*sc+i];
end
F_c_b(sc,:) = [sc, 1, 2*sc+1, 3*sc];
F_c_b(2*sc,:) = [2*sc, sc+1, 3*sc+1, 4*sc];

end

