%% Description:
% Designed by: Adrian König (FTM, Technical University of Munich)
%-------------
% Created on: 01.05.2021
% ------------
% Version: Matlab2020b
%-------------
% Description:  This function creates surface files out of stl files for the plot of the human
%               This function can be deleted and the main will still work, as long as the
%               MATLAB variable remains in the main
% ------------
% Sources:  [1] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - stl-files of Human
% ------------
% Output:   - trinangulated human parts
% ------------

%% Implementation:

%% 1) Read stl-Files
headstl=stlread('Human_Head_AuVeCoDe.stl');
torsostl=stlread('Human_Torso_AuVeCoDe.stl');
legstopstl=stlread('Human_LegsTop_AuVeCoDe.stl');
legslowstl=stlread('Human_LegsLow_AuVeCoDe.stl');
armstopstl=stlread('Human_ArmsTop_AuVeCoDe.stl');

head.cv=headstl.ConnectivityList;
head.points=headstl.Points;

torso.cv=torsostl.ConnectivityList;
torso.points=torsostl.Points;

legstop.cv=legstopstl.ConnectivityList;
legstop.points=legstopstl.Points;

legslow.cv=legslowstl.ConnectivityList;
legslow.points=legslowstl.Points;

armstop.cv=armstopstl.ConnectivityList;
armstop.points=armstopstl.Points;

% %Create rotation matriy with the given rotation angle in rad -> Rotation on the Y axis
% rot_angle=45;
% rot_angle_init=90;
% rot_angle=deg2rad(rot_angle-rot_angle_init);
% Rotmatrix=[cos(rot_angle),0,sin(rot_angle);0,1,0; -sin(rot_angle),0,cos(rot_angle)];
% 
% %Rotate the backseat and the headrest for the desired rotation angle
% rot=(Rotmatrix*[torso.points(:,1)';torso.points(:,2)';torso.points(:,3)'])';
% torso.points(:,1)=rot(:,1); %Rotated X coordinates of the backseat
% torso.points(:,2)=rot(:,2); %Rotated Y coordinates of the backseat-> These do not change
% torso.points(:,3)=rot(:,3); %Rotated Z coordinates of the backseat
% 
% rot=(Rotmatrix*[head.points(:,1)';head.points(:,2)';head.points(:,3)'])';
% head.points(:,1)=rot(:,1); %Rotated X coordinates of the backseat
% head.points(:,2)=rot(:,2); %Rotated Y coordinates of the backseat-> These do not change
% head.points(:,3)=rot(:,3); %Rotated Z coordinates of the backseat
% 
% rot=(Rotmatrix*[armstop.points(:,1)';armstop.points(:,2)';armstop.points(:,3)'])';
% armstop.points(:,1)=rot(:,1); %Rotated X coordinates of the backseat
% armstop.points(:,2)=rot(:,2); %Rotated Y coordinates of the backseat-> These do not change
% armstop.points(:,3)=rot(:,3); %Rotated Z coordinates of the backseat

%3) Plot
figure
hold on
TT=triangulation(head.cv,head.points);
trisurf(TT);
TT=triangulation(torso.cv,torso.points);
trisurf(TT);
TT=triangulation(legstop.cv,legstop.points);
trisurf(TT);
TT=triangulation(armstop.cv,armstop.points);
trisurf(TT);

