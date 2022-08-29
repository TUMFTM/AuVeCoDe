function R = Object3d_rotate(eu)
%% Description:
% Designed by: Adrian König (FTM, Technical University of Munich)
%-------------
% Created on: 18.12.2019
% ------------
% Version: Matlab2020b
%-------------
% Description: %Creates a 3dmatrix for rotation
% ------------
% Sources:  [1] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
%           [2] https://www.sky-engin.jp/MATLABAnimation/chap06/chap06.html
% ------------
% Input:    - eu=rotation with euler angles
% ------------
% Output:   - R=rotation matrix
% ------------

%% Calculation of vertices and faces
% Read in angles
a1=eu(1);
a2=eu(2);
a3=eu(3);

%Create rotation matrix
R1 = [
1, 0, 0;
0, cos(a1), -sin(a1);
0, sin(a1), cos(a1)];

R2 = [
cos(a2), 0, sin(a2);
0, 1, 0;
-sin(a2), 0, cos(a2)];

R3 = [
cos(a3), -sin(a3), 0;
sin(a3), cos(a3), 0;
0, 0, 1];

R = R1*R2*R3;

end