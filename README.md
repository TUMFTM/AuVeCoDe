<h1><a ...section link code />Autonomous Vehicle Concept Development (AuVeCoDe)</h1>

A tool for designing autonomous or conventional electric vehicles. The tool was created during the Ph.D. Thesis of Adrian König [[1]](#1) at the Institute of Automotive Technology of the Technical University of Munich

![AuVeCoDe](https://user-images.githubusercontent.com/67913102/187223831-b9bfb8fd-44a5-4c15-8d63-d6598bb9c4a1.png)


<h2><a ...section link code />Requirements</h2>
  
* [MATLAB](https://de.mathworks.com/products/matlab.html) R2020b or newer

<h2><a ...section link code />Structure of the AuVeCoDe:</h2>

The structure of the AuVeCoDe is shown in the figure underneath, based on [[1]](#1) and [[2]](#2):

![overview](https://user-images.githubusercontent.com/67913102/187156739-34d68eeb-5e77-4c00-a582-72dc15f1c1ec.png)


After setting the requirements, a first longitudinal dynamic simulation (LDS) is performed. The LDS was already published by the authors ([[3]](#3)). In the next step, the package of the vehicle is calculated based on the new method developed by the first author [[4]](#4). The battery and mass calculation is performed with modified versions of the models by Nicoletti [[5]](#5). The auxiliary power user are calculated based on literature values and a HVAC simulation already published in [[2]](#2). If the vehicle data converge during the iterations the costs are calculated and the output is a vehicle concept with the option to plot its package.

<h2><a ...section link code />Manual:</h2>

This section gives an overview of the AuVeCoDe Tool and how to use it. The AuVeCoDe tool can be started by calling [```Run_AuVeCoDe.m```](/Run_AuVeCoDe.m). This will add all needed folders and subfolders and start the userform:

![userform1](https://user-images.githubusercontent.com/67913102/187167474-a8fa4f2b-5704-4c88-9a28-09073e47b29c.png)


 1. Standard selection is optimization mode. You can select "Single vehicle" to calculate just one vehicle. For optimization mode, you have to select an optimizer and the two optimization goals. You can also select the parts of the vehicle to be plotted under "Vizualization".
 2. You can select the vehicle parameters inside the different pages (1. General - 5. Optimization). The Optimization page will change depending on the "Single Vehicle" setting. You can also select the standard set "Conventional vehicle settings" or "Autonomous one-box vehicle settings". Detailled result data of the vehicle will be displayed in the "6. Results" Page.
 3. You can save the configuration or load a already saved configuration
 4. You can save (especially when optimization mode is selected) a simulation for an external simulation e.g. on a compute cloud, where Matlab Userforms are not working. You can start the simulation with either [```LoadSimulation```](/03_GUI/LoadSimulation.m) or [```LoadSimulation_nodesktop```](/03_GUI/LoadSimulation_nodesktop.m/) for MATLAB in the nodesktop mode.
 5. Click on "Run Simulation" to start the simulation. The status light will turn to yellow during calculation and becomes green after the simulation is finished.
 6. In this area either the vehicle length, weight and cost are shown (successfull simulation) or the reason of the termination of the calculation is shown.

<h3><a ...section link code />Single Vehicle Mode:</h3>

When using the single vehicle mode, the vehicle is calculated and plotted. The vehicle and Parameter struct is saved to the [```06_Results```](/06_Results/) folder and the most important date is saved in an Excel Sheet. Every data set of a single vehicle simulated on one day will be saved chronologically in the Excel Sheet while the vehicle and Parameter struct will be overwritten after every simulation.

![Single vehicle](https://user-images.githubusercontent.com/67913102/187168999-02508d3f-9c99-45bc-ba30-f7538c2e04cb.png)

1. The vehicle plot. The figure can be rotated and saved like a common MATLAB figure.
2. The most important data is shown here.
3. Detailled information about the vehicle are saved in the "6. Results" page.
4. The status light is green.

<h3><a ...section link code />Optimization Mode:</h3>

In the optimization mode, an optimizer has to be selected (e.g. the NSGA-II). Furthermore, the number of generation and the population size has to be defined. When the MATLAB Parallel Computing Toolbox is installed, the "Parallel computing" can be used to get lower computation time. Because the optimizer (NSGA-II/MOPSO) are stochastic methods the number of optimization can be increased to ensure the search for a global optimum. Instead of the standard folder [```06_Results```](/06_Results/) your can also select an own folder with "Select Optimization Folder". The results will be saved to this folder instead.

![Optimziation Selection](https://user-images.githubusercontent.com/67913102/187170911-59cc868e-2aa5-40b9-8d70-2259d246c6cd.png)

During the optimization, the following windows are shown:
![Optimization Run](https://user-images.githubusercontent.com/67913102/187175346-c72dd599-d2a3-44e0-a66a-73a7591a2ae4.png)

1. The status of the running simulation is shown in the userform
2. The current status of calculation (generation and population) is shown in the command window.
3. The optimization goal values of the last population is plotted. If the goals are contradictory a pareto front is created over time.
4. The best three or the best, middle and worst vehicle of one generation is plotted. You can change the mode in the [```plotnsga```](/02_Optimization/03_PlotFunctions/plotnsga.m) function in Line 380.

<h2><a ...section link code />Developers:</h2>

- Adrian König (Institute for Automotive Technology, Technical University of Munich): Creation of research topic and idea for the tool, Conceptualization, Code   creating, Code detailing, Code documentation, Supervison
- Lorenzo Nicoletti (Institute for Automotive Technology, Technical University of Munich): Code creating (Modeling of mass, gearbox and chassis), Code documentation
- Daniel Telschow (Institute for Automotive Technology, Technical University of Munich): Code creating, Code documentation, First implementation of NSGA-II
- Korbinian Moller (Institute for Automotive Technology, Technical University of Munich): Longitudinal dynamic simulation (validation, creating and detailing code, code documentation)
- Michael Mast (Institute for Automotive Technology, Technical University of Munich): Code creating (Trunk, silhouette and BIW design), Code documentation
- Fabian Liemawan Adji (Institute for Automotive Technology, Technical University of Munich): Code creating, Code documentation, Code detailing of NSGA-II and implementation of MOPSO
- Peter Köhler (Institute for Automotive Technology, Technical University of Munich): Code creating (gearbox calculation), Code documentation
- Felix Fahn (Institute for Automotive Technology, Technical University of Munich): Code detailing (Interior calculation and plot), Code documentation
- Dila Akguel (Institute for Automotive Technology, Technical University of Munich): Code creating (cost model), Code documentation


<h2><a ...section link code />References:</h2>
<a id="1">[1]</a> König A., "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Ph.D. Thesis, Technical University of Munich, Institute of Automotive Technology, 2022.

<a id="2">[2]</a> König, A.; Mayer, S.; Nicoletti, L.; Tumphart, S.; Lienkamp, M. The Impact of HVAC on the Development of Autonomous and Electric Vehicle Concepts. Energies 2022, 15, 441. https://doi.org/10.3390/en15020441

<a id="3">[3]</a> König, A., Nicoletti, L. et. al. „An Open-Source Modular Quasi-Static Longitudinal Simulation for Full Electric Vehicles,“ in 15th International Conference on Ecological Vehicles and Renewable Energies, Monte-Carlo, Monaco, 2020, pp. 1–9, DOI: 10.1109/EVER48776.2020.9242981.

<a id="4">[4]</a> König, A.; Telschow, D.; Nicoletti, L.; Lienkamp, M., “Package Planning of Autonomous Vehicle Concepts,” Proc. Des. Soc., Jg. 1, S. 2369–2378, 2021, doi: 10.1017/pds.2021.498.

<a id="5">[5]</a> Nicoletti. L., "Parametric Modeling of Battery Electric Vehicles in the Early Development Phase", Ph.D. Thesis, Technical University of Munich, Institute of Automotive Technology, 2022.
