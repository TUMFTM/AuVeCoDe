function [v] = Mass_frame(v,Par)
%% Description:
% Designed by: Adrian König (FTM, Technical University of Munich), Lorenzo Nicoletti (FTM, Technical University of Munich)
%-------------
% Created on: 01.04.2022
% ------------
% Version: Matlab2020b
%-------------
% Description:  This function calculates the BIW weight. All possible body types are possible due to
%               the use of a substitute density of the body
%               Toggle alu_perc in initialize_weight_options
% ------------
% Sources:  [1] Lorenzo Nicoletti, "Parametric Modeling of Battery Electric Vehicles in the Early Development Phase", Technical University of Munich, Institute of Automotive Technology, 2022
%           [2] A. Romano, „Data-based Analysis for Parametric Weight Estimation of new BEV Concepts,“Master thesis, Institute of Automotive Technology, Technical University of Munich, Munich, 2021.
%           [3] L. Nicoletti, A. Romano, A. König, P. Köhler, M. Heinrich and M. Lienkamp, „An Estimation of the Lightweight Potential of Battery Electric Vehicles,“ Energies, vol. 14, no. 15, p. 4655, 2021, DOI: 10.3390/en14154655.
%           [4] S. Fuchs, Verfahren zur parameterbasierten Gewichtsabschätzung neuer Fahrzeugkonzepte, Ph.D. Thesis, Institute of Automotive Technology, Technical University of Munich, Munich, 2014.
%           [5] Michael Mast, “Karosseriemodellierung autonomer Elektrofahrzeuge,” Master Thesis, Technical University of Munich, Institute of Automotive Technology, 2022
%           [6] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - Parameters (Par): struct with input and constant values
%           - vehicle(v): struct including the data of the vehicle
% ------------
% Output:   - vehicle(v): struct including the data of the vehicle
% ------------

%% Implementation
%1) Initialize the variables
%2) BIW weight
%3) Weight of other frame components
%4) Assign outputs

%% 1) Initialize variables
%Vehicle measurements
gross_weight = v.masses.vehicle_max_weight;            % Maximum allowable mass in kg
overhang_f = v.dimensions.GX.vehicle_overhang_f;       % in mm
overhang_r = v.dimensions.GX.vehicle_overhang_r;       % in mm
width =v.dimensions.GY.vehicle_width;                  % in mm
height = v.dimensions.GZ.vehicle_height;               % in mm
wheelbase = v.dimensions.GX.wheelbase;                 % in mm

%Further vehicle data
perc_alu = v.masses.optional_extras.alu_perc_BIW;      % in %
frameform = v.topology.frameform;                      % SUV, Hatchback or Sedan

%%Regressions for the mass calculation of frame components
regr_BIW=Par.regr.mass.BIW.eq;
regr_other_frame_components=Par.regr.mass.frame_elements.eq;

%% 2) BIW weight
%Calculate substitute volume according to [4]
if strcmp(frameform,'Sedan')
    subs_volume_2 = ((0.5*overhang_f + wheelbase + 2*overhang_r/3)*width*height)/(10^9);   % [m^3]
else
    subs_volume_2 = ((0.5*overhang_f + wheelbase + 3*overhang_r/4)*width*height)/(10^9);   % [m^3]
end
v.body.vol_ref=(subs_volume_2*(10^9));

%Calculate Silhouette volume according to [5]
    front_wagon=((v.body.p_hood_f(1,3) * (v.body.p_hood_f(1,1)-v.body.p_grill_f(2,1))) - (((v.body.p_hood_f(1,1)-v.body.p_hood_f(2,1))*(v.body.p_hood_f(1,3)-v.body.p_hood_f(2,3)))/2)- (((v.body.p_grill_f(1,1)-v.body.p_grill_f(2,1))*(v.body.p_grill_f(1,3)-v.body.p_grill_f(2,3)))/2)- (((v.body.p_grill_f(1,1)-v.body.p_grill_f(2,1))*(v.body.p_hood_f(1,3)-v.body.p_hood_f(2,3))))) * v.dimensions.GY.vehicle_width; %(front and back) falls nicht integriert
    rear_wagon= ((v.body.p_hood_r(1,3) * (v.body.p_grill_r(2,1)-v.body.p_hood_r(1,1))) - (((v.body.p_hood_r(2,1)-v.body.p_hood_r(1,1))*(v.body.p_hood_r(1,3)-v.body.p_hood_r(2,3)))/2)- (((v.body.p_grill_r(2,1)-v.body.p_grill_r(1,1))*(v.body.p_grill_r(1,3)-v.body.p_grill_r(2,3)))/2)- (((v.body.p_grill_r(2,1)-v.body.p_grill_r(1,1))*(v.body.p_hood_r(1,3)-v.body.p_hood_r(2,3))))) * v.dimensions.GY.vehicle_width; %(front and back) falls nicht integriert
	cabin=(   (v.body.p_pane_r(1,1)-v.body.p_pane_f(1,1)) * v.dimensions.GZ.vehicle_height   -  ((((v.body.p_pane_f(2,1)-v.body.p_pane_f(1,1))*(v.body.p_pane_f(2,3)-v.body.p_pane_f(1,3)))/2)  +    (((v.body.p_pane_r(1,1)-v.body.p_pane_r(2,1))*(v.body.p_pane_r(2,3)-v.body.p_pane_r(1,3)))/2)   ))      * v.dimensions.GY.vehicle_width;
    subs_volume =(cabin+front_wagon+rear_wagon)/(10^9);
    v.body.vol_new=(subs_volume*(10^9));
    
if 0 %1 for Fuchs[4] 0 for Mast[5]
 subs_volume=subs_volume_2;
end
 
%Calculate frame mass and assign outputs
var1 = subs_volume*perc_alu/100;
var2 = subs_volume*(1-perc_alu/100);

%Calculate the weight of the Body in white
BIW_weight = regr_BIW(var1,var2,gross_weight);

%Check that the result is not negative
check_mass(BIW_weight);

%% 3) Weight of other frame components
%Calculate mass of other frame components besides BIW
% Front cross members, body reinforcements, sound insulation on body,
% aerodynamic screens, water draining system, rocker panels, exterior
% pillar trims, roof trim and roof rack, running boards, seals, jack pads
% for servicing, underbody protections.
%other_frame_components_weight = -18.270+4.090*subs_volume;  % [kg]
other_frame_components_weight = regr_other_frame_components(subs_volume);

%Check that the result is not negative
check_mass(other_frame_components_weight);

%% 4) Assign outputs
v.masses.frame.BIW_weight = BIW_weight;
v.masses.frame.other_frame_components_weight = other_frame_components_weight;

%check for wrong results
end

