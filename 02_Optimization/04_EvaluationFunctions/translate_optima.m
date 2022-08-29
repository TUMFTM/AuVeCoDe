function [ResultTable] = translate_optima(OptimaStruct)
	%% Description:
	% This function translates input variables values (mostly as
	% optimization result) into a table with parameters that the values 
    % represent

	% Author: Fabian Liemawan Adji
	% Date: July 2021
    
    %% Input
    % OptimaStruct: a struct contains multiples input variables to be
    % translated
    
    %% Output
    % ResultTable: table containings the input variables and which
    % parameter they represent
    % Result.xlsx: sheet Input Variable shows the ResultTable in Excel
    
    % initialize ResultTable
	ResultTable = table;
    % collect fieldnames of OptimaStruct in string
	FieldNames = string(fieldnames(OptimaStruct));
    
    %% Translation
	for i = 1:length(FieldNames)
        % use fieldnames as concept name
		ResultTable.Concept(i) = FieldNames(i);
	%%---------------------------Motor topology--------------------------------
		switch OptimaStruct.(FieldNames(i))(1)
			case 1
				ResultTable.topology(i) ="GM_X";
			case 2
				ResultTable.topology(i) ="X_GM";
			case 3
				ResultTable.topology(i) ="GM_GM";
			case 4
				ResultTable.topology(i) ="2G2M_X";
			case 5
				ResultTable.topology(i) ="X_2G2M";
			case 6
				ResultTable.topology(i) ="2G2M_GM";
			case 7
				ResultTable.topology(i) ="GM_2G2M";
			case 8
				ResultTable.topology(i) ="2G2M_2G2M";
		end

	% ---------------------------Engine Type-----------------------------------
		switch OptimaStruct.(FieldNames(i))(2)
			case 1
				ResultTable.machine_type(i) = "PSM";
			case 2
				ResultTable.machine_type(i) = "ASM";
		end

	% ---------------------------Gear type front-------------------------------
		switch OptimaStruct.(FieldNames(i))(3)
			case 1
				ResultTable.gear_type_f(i) = "Parallel";
			case 2
				ResultTable.gear_type_f(i) = "Coaxial";
			case 3
				ResultTable.gear_type_f(i) = "Coaxial-layshaft";
		end

	% ---------------------------Gear type rear-------------------------------
		switch OptimaStruct.(FieldNames(i))(4)
			case 1
				ResultTable.gear_type_r(i) = "Parallel";
			case 2
				ResultTable.gear_type_r(i) = "Coaxial";
			case 3
				ResultTable.gear_type_r(i) = "Coaxial-layshaft";
		end

	% ---------------------------Battery cell-type----------------------------
		switch OptimaStruct.(FieldNames(i))(6)
			case 1
				ResultTable.cell_type(i) = "prismatic";
			case 2
				ResultTable.cell_type(i) = "cylindrical";
			case 3
				ResultTable.cell_type(i) = "pouch";
		end


	% ---------------------------Rear axle type-------------------------------
		switch OptimaStruct.(FieldNames(i))(11)
			case 1
				ResultTable.axis_type_r(i) = "torsion_beam";
				
			case 2
				ResultTable.axis_type_r(i) = "trapezoidal_link";
				
			case 3
				ResultTable.axis_type_r(i) = "sword_arm_link";
				
			case 4
				ResultTable.axis_type_r(i) = "five_link";
        end

	% ---------------------------Gearbox optimization goal--------------------
		switch OptimaStruct.(FieldNames(i))(14)
			case 1
				ResultTable.gearbox_opt(i)="Length";
			case 2
				ResultTable.gearbox_opt(i)="Height";
		end

	%---------------------------Small Overlap Second Level Battery-----------
		switch OptimaStruct.(FieldNames(i))(17)
			case 1
				ResultTable.small_overlap(i)=1;
			case 2
				ResultTable.small_overlap(i)=0;
		end


		% Wheel radius (conversion from diameter to radius)
		ResultTable.r_tire(i) = OptimaStruct.(FieldNames(i))(5)/2;
		
		% Drive-shaft-axle angles [Front-xy Front-xz Rear-xy Rear-xz]
		ResultTable.alpha_axle(i,:)=[OptimaStruct.(FieldNames(i))(7) OptimaStruct.(FieldNames(i))(8)];
		
		% Engine-gearbox angle[Front Rear]
		ResultTable.psi_motor(i,:)=[OptimaStruct.(FieldNames(i))(9) OptimaStruct.(FieldNames(i))(10)];
		
		% Gear ratios (dependend if 2 speed or 1 speed gearbox)
		ResultTable.i_gearbox_f(i) = [OptimaStruct.(FieldNames(i))(12)];
		ResultTable.i_gearbox_r(i) = [OptimaStruct.(FieldNames(i))(13)];
		
			
		
		% Battery capacity ratio Underfloor/Total [%]
		ResultTable.trunk_ratio(i) = OptimaStruct.(FieldNames(i))(15)/100;
		
		% Interior height [mm]
		ResultTable.int_height(i) = OptimaStruct.(FieldNames(i))(16);
		
		% Distance wheelbase to bumper [mm]
		ResultTable.dist_wheelhouse2bumper_f(i)=OptimaStruct.(FieldNames(i))(18);
		ResultTable.dist_wheelhouse2bumper_r(i)=OptimaStruct.(FieldNames(i))(19);
		
		% Steering ratio front/total [0-1]
		ResultTable.steering_ratio(i) = OptimaStruct.(FieldNames(i))(20);
		
		% Torque ratio front/rear [%]
		ResultTable.torque_ratio(i) = OptimaStruct.(FieldNames(i))(21);
		
		%H30 measurement [mm]
		ResultTable.H30_f(i)= OptimaStruct.(FieldNames(i))(22);
		ResultTable.H30_r(i)= OptimaStruct.(FieldNames(i))(23);
    end
    % write results in Result.xlsx in sheet "Input Variables"
	writetable(ResultTable, "Result.xlsx", "Sheet", "Input Variables");
end

