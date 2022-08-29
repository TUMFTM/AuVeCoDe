function [Volume_DIN,U,F,B,O] = Package_DIN_Volume(Trunk_Dim,dim,box)
%% Description:
% Designed by: Michael Mast, Adrian König (FTM, Technical University of Munich)
%-------------
% Created on: 01.06.2021
% ------------
% Version: Matlab2020b
%-------------
% Description:  This function places the reference bodys defined in box/dim
%               inside the compartments defined by Trunk_Dim according to ISO 3832 procedure
%               and returns the determined reference volume as well as the orientations of 
%               the objects in the different compartments
% ------------
% Sources:  [1] Michael Mast, “Packageplanung von autonomen Fahrzeugkonzepten im Vorder- und Hinterwagen,” Semester Thesis, Technical University of Munich, Institute of Automotive Technology, 2021
%           [2] Michael Mast, “Karosseriemodellierung autonomer Elektrofahrzeuge,” Master Thesis, Technical University of Munich, Institute of Automotive Technology, 2022
%           [3] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - Dimensions of available compartments 
%           - Dimensions of test body (in different orientations)
% ------------
% Output:   - Total volume according to DIN
%           - Orientation of compartment
% ------------
% Abbreviations [1]:
%   FA: "infornt of Axle"
%   BA: "behind Axle"
%   OA: "over  Axle"
%   UC: "Upper Compartment"

%% 1) Calculate number of possible test bodies in each compartment
layoutUC = floor(Trunk_Dim(4,:)'./dim); %(x-z,U)
RUC = Trunk_Dim(4,:)' - layoutUC .* dim; %free rest lenght to extend FA,BA,OA

FAnew = Trunk_Dim(1,:)'.*ones(1,6); %(x,U)
FAnew(3,:) = FAnew(3,:) + RUC(3,:); %extendet FA z dim. for all UC.
                    %(x,U,F)                     (x-z,F) perm (x-z,U,F)
layoutFA = floor((FAnew .*ones(1,1,6))./ permute(dim .* ones(1,1,6),[1 3 2]));
RFA = (FAnew .*ones(1,1,6)) - layoutFA .* permute(dim .* ones(1,1,6),[1 3 2]);

BAnew = Trunk_Dim(2,:)'.*ones(1,6); %(x,U)
BAnew(3,:) = BAnew(3,:) + RUC(3,:); %extendet BA z dim. for all UC.
                    %(x,U,B)                     (x-z,B) perm (x-z,U,B)
layoutBA = floor((BAnew .*ones(1,1,6))./ permute(dim .* ones(1,1,6),[1 3 2]));
RBA = (BAnew .*ones(1,1,6)) - layoutBA .* permute(dim .* ones(1,1,6),[1 3 2]);

OAnew = Trunk_Dim(3,:)' .*ones(1,6,6,6); %dimensions=(x-z,Or.UC,Or.FA,Or.BA)
                                   %(x-z,U)
OAnew(3,:,:,:) = OAnew(3,:,:,:) + (RUC(3,:) .*ones(1,1,6,6));
                                        %(x-z,U,F)
OAnew(1,:,:,:) = OAnew(1,:,:,:) + RFA(1,:,:) .*ones(1,1,6,6);
                                        %(x-z,U,B) perm (x-z,U,F,B)
OAnew(1,:,:,:) = OAnew(1,:,:,:) + permute(RBA(1,:,:) .*ones(1,1,1,6),[1 2 4 3]);
                    %(x-z,U,F,B,O)                  (x-z,O) perm (x-z,U,F,B,O)
layoutOA = floor((OAnew .*ones(1,1,1,1,6))./ permute(dim .* ones(1,1,6,6,6),[1 3 4 5 2]));

%% 2) Calculate volume due to number of bodies
Vol =                          prod(layoutOA,1)...   %(1,U,F,B,O)         
                        + permute(prod(layoutBA,1) .* ones(1,1,1,6,6),[1 2 4 3 5])... %(1,U,B)
                        + permute(prod(layoutFA,1) .* ones(1,1,1,6,6),[1 2 3 4 5])... %(1,U,F)
                        +         prod(layoutUC,1) .* ones(1,1,6,6,6); %(1,U)
Vol=Vol.*(prod(box)/10^6); %Volume correction

%% 3) Find orientations with maximum volume
[Volume_DIN,Orientations] = max(Vol,[],'all','linear');
[~,U,F,B,O] = ind2sub(size(Vol),Orientations);%V is always 1

end
