function [F_b,V_b]=Object3d_Block(p,eu,Lx,Ly,Lz)
%% Description:
% Designed by: Adrian König (FTM, Technical University of Munich)
%-------------
% Created on: 18.12.2019
% ------------
% Version: Matlab2020b
%-------------
% Description: %Designs a 3d block with the geometrical size defined by Lx,Ly,Lz
% ------------
% Sources:  [1] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
%           [2] https://www.sky-engin.jp/MATLABAnimation/chap06/chap06.html
% ------------
% Input:    - eu=rotation with euler angles
%           - p=position offset in x,y, and z direction
%           - Lx,Ly,Lx = Length in x,y,z Direction
% ------------
% Output:   - F_b=Face of Block
%           - V_b= Vertices of Block
% ------------

%% Calculation of vertices and faces

% Reference orientation
R = Object3d_rotate(eu);

% Vertices
vertices_0 = [
    0,   0,  0;  % #1
    Lx,  0,  0;  % #2
    0,  Ly,  0;  % #3
    0,   0, Lz;  % #4
    Lx, Ly,  0;  % #5
    0,  Ly, Lz;  % #6
    Lx,  0, Lz;  % #7
    Lx, Ly, Lz]; % #8

V_b = p' + vertices_0*R';

% Faces
F_b = [
    1, 2, 5, 3;  % #1
    1, 3, 6, 4;  % #2
    1, 4, 7, 2;  % #3
    4, 7, 8, 6;  % #4
    2, 5, 8, 7;  % #5
    3, 6, 8, 5]; % #6

