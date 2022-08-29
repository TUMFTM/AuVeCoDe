function varargout = plotnsga(varargin)
%PLOTNSGA M-file for plotnsga.fig
%      PLOTNSGA, by itself, creates a new PLOTNSGA or raises the existing
%      singleton*.
%
%      H = PLOTNSGA returns the handle to a new PLOTNSGA or the handle to
%      the existing singleton*.
%
%      PLOTNSGA('Property','Value',...) creates a new PLOTNSGA using the
%      given property value pairs. Unrecognized properties are passed via
%      varargin to plotnsga_OpeningFcn.  This calling syntax produces a
%      warning when there is an existing singleton*.
%
%      PLOTNSGA('CALLBACK') and PLOTNSGA('CALLBACK',hObject,...) call the
%      local function named CALLBACK in PLOTNSGA.M with the given input
%      arguments.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES
%*************************************************************************
% Syntax:
%   plotnsga(result)
%     Plot the optimization result.
%
%   plotnsga(result, curGen)
%     Plot the optimization result with specified generation.
%
%   plotnsga('pops.txt')
%     Load from population file and plot the result.
%     Note: A global variable "oldresult" which contains the population loaded from file
%       would be created.
%*************************************************************************

% Edit the above text to modify the response to help plotnsga

% Last Modified by GUIDE v2.5 04-Apr-2019 10:21:09

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @plotnsga_OpeningFcn, ...
    'gui_OutputFcn',  @plotnsga_OutputFcn, ...
    'gui_LayoutFcn',  [], ...
    'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before plotnsga is made visible.
function plotnsga_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   unrecognized PropertyName/PropertyValue pairs from the
%            command line (see VARARGIN)

% Choose default command line output for plotnsga
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes plotnsga wait for user response (see UIRESUME)
% uiwait(handles.figure1);

%*************************************************************************
% 1. Save the result
%*************************************************************************
handles.bLoadFromFile = 0;          % Load from the population file.

% a) Case : plotnsga()
if( isempty(varargin) )
    error('PLOTNSGA:ParamError', 'Error: plotnsga should be called : plotnsga(result, curGen) or plotnsga(''pops.txt'')');
    
    % b) Case : plotnsga(result) or plotnsga('pops.txt')
elseif(length(varargin) == 1)
    % plotnsga(result)
    if( isstruct(varargin{1}) )
        handles.result  = varargin{1};
        handles.currentGen = 1;
        
        % plotnsga('pops.txt')
    elseif( ischar(varargin{1}) )
        global oldresult;
        oldresult = loadpopfile(varargin{1});
        evalin('base', 'global oldresult');
        
        handles.bLoadFromFile   = 1;
        handles.strPopFile      = varargin{1};
        handles.result          = oldresult;
        handles.currentGen      = 1;
    end
    
    % c) Case : plotnsga(result, curGen)
elseif(length(varargin) == 2)
    if( isstruct(varargin{1}) && isscalar(varargin{2}) )
        handles.result  = varargin{1};
        handles.currentGen = varargin{2};
    else
        error('PLOTNSGA:ParamError', ...
            'Error: plotnsga should be called : plotnsga(result, curGen) or plotnsga(''pops.txt'')');
    end
end



%*************************************************************************
% 2. Initialize the population ID listbox
%*************************************************************************
popsize = size(handles.result.pops, 1);
strList = repmat({''}, [1,popsize]);
for i = 1:popsize
    strList{i} = sprintf('%d', i);
end

curSel = handles.currentGen;   % the generation ID of population which would be ploted
set(handles.listPop, 'string', strList);
set(handles.listPop, 'value', curSel);


%*************************************************************************
% 3. Update data and plot population
%*************************************************************************
dispState(handles, curSel);
plotPopulation( handles, curSel );
guidata(hObject,handles);








% --- Outputs from this function are returned to the command line.
function varargout = plotnsga_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in btnStop.
function btnStop_Callback(hObject, eventdata, handles)
% hObject    handle to btnStop (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global STOP_NSGA;
STOP_NSGA = 1;


% --- Executes on button press in btnPause.
function btnPause_Callback(hObject, eventdata, handles)
% hObject    handle to btnPause (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if isempty(getappdata(0,'gadsSolverState'))
    setappdata(0,'gadsSolverState','pause');
    set(hObject,'String','Continue');
    % Wait for hObj to change its String property
    waitfor(hObject,'String');
    %     if isempty(findobj(0,'Type','uicontrol','string','Pause')) % Figure is deleted
    %         setappdata(0,'gadsSolverState','');
    %     end
else
    setappdata(0,'gadsSolverState','');
    set(hObject,'String','Pause');
end





% --- Executes on selection change in listPop.
function listPop_Callback(hObject, eventdata, handles)
% hObject    handle to listPop (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns listPop contents as cell array
%        contents{get(hObject,'Value')} returns selected item from listPop

curSel = get(hObject,'Value');
dispState(handles, curSel);
plotPopulation(handles, curSel);





% --- Executes during object creation, after setting all properties.
function listPop_CreateFcn(hObject, eventdata, handles)
% hObject    handle to listPop (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in btnPlotInNewWindow.
function btnPlotInNewWindow_Callback(hObject, eventdata, handles)
% hObject    handle to btnPlotInNewWindow (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%figure;
curSel = get(handles.listPop, 'Value');
plotPopulation( handles, curSel);




function dispState(handles, curSel)
% Function: dispState(handles, curSel)
% Description: Display the optimization state in listbox.
% Parameters: use handles.result.state structure
%
%         LSSSSWC, NWPU
%    Revision: 1.0  Data: 2011-04-20
%*************************************************************************

state = handles.result.states(curSel);

strName = fieldnames( state );
numState = length(strName);
table = cell(numState, 2);
for i = 1:numState
    table{i, 1} = strName{i};
    table{i, 2} = getfield(state, strName{i});
end
set( handles.tableState, 'Data', table);



function plotPopulation(handles, gen)
% Function: plotPopulation(handles, gen)
% Description: Plot population with the first two objective values.
% Parameters:
%   gen : the generation ID
%
%         LSSSSWC, NWPU
%    Revision: 1.3  Data: 2011-07-26
%*************************************************************************

cla

%*************************************************************************
% Initialize data
%*************************************************************************
pop     = handles.result.pops(gen, :);
obj     = vertcat(pop.obj);
numObj  = length(pop(1).obj);

%  When the result is readed from file, there is no 'opt' structure.
if(handles.bLoadFromFile == 0 && isfield(handles.result, 'opt'))
    opt = handles.result.opt;
    maxGen  = opt.maxGen;
    if( ~isempty(opt.nameObj) )
        strObj1 = opt.nameObj{1};
        strObj2 = opt.nameObj{2};
        if(numObj == 3)
            strObj3 = opt.nameObj{3};
        end
    end
else
    maxGen = size(handles.result.pops, 1);
end

% Determin if reference points exist
refPoints = [];
refPlotStyle = {'kd', ...
    'LineWidth', 1,...
    'MarkerEdgeColor', 'k',...
    'MarkerFaceColor', 'g',...
    'MarkerSize',6};
if( isfield(handles.result, 'opt') && ~isempty(handles.result.opt.refPoints) )
    refPoints = handles.result.opt.refPoints;
end


%*************************************************************************
% Plot population with different methods for every "numObj" number
%*************************************************************************
if(numObj == 2)
    
    obj_temp=obj;
    pop_reduced=pop(obj(:,1)<10e4);
    obj_reduced=obj(obj(:,1)<10e4,:);
    pop_reduced=pop_reduced(obj_reduced(:,2)<10e4);
    obj_reduced=obj_reduced(obj_reduced(:,2)<10e4,:);
    
    if isempty(obj_reduced)
        obj_reduced=obj_temp;
    end
    
    scatter(obj_reduced(:,1), obj_reduced(:,2));
    xlabel(strObj1, 'interpreter', 'none','FontSize',14);
    ylabel(strObj2, 'interpreter', 'none','FontSize',14);     
    % plot reference points
    if(~isempty(refPoints))
        hold on
        plot(refPoints(:, 1), refPoints(:, 2), refPlotStyle{:});
    end
elseif(numObj == 3)
    
    scatter3(obj(:,1),obj(:,2),obj(:,3));
    hold on
    veh_1=obj(1,:);
    veh_3=obj(end,:);
    veh_2=obj(ceil(end/2),:);
    
    plot3(veh_1(1),veh_1(2),veh_1(3),'s','MarkerEdgeColor','g','MarkerFaceColor','g','MarkerSize',10);
    plot3(veh_2(1),veh_2(2),veh_2(3),'o','MarkerEdgeColor','b','MarkerFaceColor','b','MarkerSize',10);
    plot3(veh_3(1),veh_3(2),veh_3(3),'+','MarkerEdgeColor','r','MarkerFaceColor','r','MarkerSize',10);
    hold off
    xlabel(strObj1, 'interpreter', 'none','FontSize',14);
    ylabel(strObj2, 'interpreter', 'none','FontSize',14);
    zlabel(strObj3, 'interpreter', 'none','FontSize',14);
    
    if length(obj(obj(:,1)>0))>2
        xlim([min(-obj(:,1))/2,max(-obj(:,1))*2]);
        ylim([min(obj(:,2))/2,max(obj(:,2))*2]);
        zlim([Min(obj(:,3))/2,max(obj(:,3))*2]);
    end
    % plot reference points
    if(~isempty(refPoints))
        hold on
        plot3(refPoints(:, 1), refPoints(:, 2), refPoints(:, 3), refPlotStyle{:});
    end
else
    plot(obj', 'b-');
    xlim([1,numObj]);
    set(gca, 'XGrid', 'on');
    xlabel('Objective number');
    ylabel('Objective value');
    
    % plot reference points
    if(~isempty(refPoints))
        hold on
        refPlotStyle{1} = 'gd-';
        plot(refPoints', refPlotStyle{:});
    end
end


%*************************************************************************
% Common operations
%*************************************************************************
% Title
strTitle = sprintf('Generation %d / %d', gen, maxGen);

if(handles.bLoadFromFile == 1)
    strTitle = sprintf('%s\nLoad from : %s', strTitle, handles.strPopFile);
end
title(strTitle, 'interpreter', 'none');
axis manual
if length(pop_reduced)>=3
    hold on
    %% edited by Fabian
    % plot setting to define which vehicles are going to be plot
    plot_settings = "best"; % best or default
    switch plot_settings
        % default: plot at position 1, end and end/2
        case "default" 
        veh_1=obj_reduced(1,:);
        veh_3=obj_reduced(end,:);
        veh_2=obj_reduced(ceil(end/2),:);
        % best: plot three best populations
        case "best"
        veh_1=obj_reduced(1,:);
        veh_3=obj_reduced(2,:);
        veh_2=obj_reduced(3,:);    
    end
    % end of editing
    
    hfig = plot(veh_1(1),veh_1(2),'s','MarkerEdgeColor','g','MarkerFaceColor','g','MarkerSize',10);
    hfig = plot(veh_2(1),veh_2(2),'o','MarkerEdgeColor','b','MarkerFaceColor','b','MarkerSize',10);
    hfig = plot(veh_3(1),veh_3(2),'+','MarkerEdgeColor','r','MarkerFaceColor','r','MarkerSize',10);
    
    % Save Limits
    clear Parameters
    % load Parameters Pareto for plotting
    load(fullfile(opt.AdditionalResultPath, "Parameters_Pareto.mat")); % edited by Fabian
    if gen==1
        Parameters.lim.min_x=min(obj_reduced(:,1))*0.9;
        Parameters.lim.max_x=max(obj_reduced(:,1))*1.1;
        Parameters.lim.min_y=min(obj_reduced(:,2))*0.9;
        Parameters.lim.max_y=max(obj_reduced(:,2))*1.1;
        path_Param = fullfile(opt.AdditionalResultPath,"Parameters_Pareto.mat"); % edited by Fabian
        save(path_Param,'Parameters');
    end
    
    if Parameters.settings.scale_axis==0 % skip auto-scale axis if option selected
        xlim([0,Parameters.lim.max_x]);
        ylim([0,Parameters.lim.max_y]);
    end
    
    hold off
    
    pareto_veh_plot(pop_reduced, plot_settings, opt)
end
