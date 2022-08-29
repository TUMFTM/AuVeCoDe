function [X, Y, Z] = Object3d_Drive_Shaft(R, sc,r1,r2)
%% Description:
% Designed by: Korbinian Moller, Adrian König (FTM, Technical University of Munich)
%-------------
% Created on: 01.10.2021
% ------------
% Version: Matlab2020b
%-------------
% Description: Returns the X,Y and Z coordinates of the desired drive shaft
% ------------
% Sources:  [1] https://www.mathworks.com/matlabcentral/fileexchange/5468-cylinder-between-2-points
%           [2] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - R = The vector of radii used to define the radius of the different segments of the drive shaft
%           - sc = Sidecount of cylinder 
% ------------
% Output:   - X/Y/Z = The x/y/z-coordinates of the drive shaft
% ------------

%% Implementation

% Set up an array of angles for the polygon.
theta = linspace(0,2*pi,sc);

%knuckle division
time = [0,50,51,100]./100;

m = length(R);                  % Number of radius values supplied.

if m == 1                       % Only one radius value supplied.
    R = [R; R];                 % Add a duplicate radius to make a cylinder.
    m = 2;                      
    time = [0, 1];              % if only one radius is given plot normal cylinder
end

X = zeros(m, sc);                % Preallocate memory.
Y = zeros(m, sc);
Z = zeros(m, sc);

v = (r2-r1)/sqrt((r2-r1)*(r2-r1)');    %Normalized vector -- cylinder axis described by: r(t)=r1+v*t for 0<t<1
R2 = rand(1,3);               %linear independent vector (of v)
x2 = v-R2/(R2*v');            %orthogonal vector to v
x2 = x2/sqrt(x2*x2');         %orthonormal vector to v
x3 = cross(v,x2);             %vector orthonormal to v and x2
x3 = x3/sqrt(x3*x3');

r1x = r1(1);r1y=r1(2);r1z=r1(3);
r2x = r2(1);r2y=r2(2);r2z=r2(3);
x2x = x2(1);x2y=x2(2);x2z=x2(3);
x3x = x3(1);x3y=x3(2);x3z=x3(3);


for j = 1 : m
  t = time(j);
  X(j, :) = r1x+(r2x-r1x)*t+R(j)*cos(theta)*x2x+R(j)*sin(theta)*x3x; 
  Y(j, :) = r1y+(r2y-r1y)*t+R(j)*cos(theta)*x2y+R(j)*sin(theta)*x3y; 
  Z(j, :) = r1z+(r2z-r1z)*t+R(j)*cos(theta)*x2z+R(j)*sin(theta)*x3z;
end

%surf(X, Y, Z, 'FaceColor', 'b');
end