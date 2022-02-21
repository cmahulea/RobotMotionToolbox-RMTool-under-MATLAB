%    This is part of RMTool - Robot Motion Toolbox, for Matlab 2010b or newer.
%
%    Copyright (C) 2016 RMTool developing team. For people, details and citing
%    information, please see: http://webdiis.unizar.es/RMTool/index.html.
%
%    This program is free software: you can redistribute it and/or modify
%    it under the terms of the GNU General Public License as published by
%    the Free Software Foundation, either version 3 of the License, or
%    (at your option) any later version.
%
%    This program is distributed in the hope that it will be useful,
%    but WITHOUT ANY WARRANTY; without even the implied warranty of
%    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%    GNU General Public License for more details.
%
%    You should have received a copy of the GNU General Public License
%    along with this program.  If not, see <http://www.gnu.org/licenses/>.

%% ============================================================================
%   MOBILE ROBOT TOOLBOX
%   Graphical User Interface
%   First version released on September, 2014.
%   Last modification June, 2020.
%   More information: http://webdiis.unizar.es/RMTool
% ============================================================================

function ret = rmt(action,~)

% robot motion toolbox

% actions:

%  ini:  figure and controls


if nargin<1
    action='ini';
end


thisfile='rmt';

%add to path the subfolders with needed tools and functions
addpath(genpath('aux_toolboxes'));
addpath(genpath('environment'));

switch action
    
    %================
    % INITIALIZATION
    %================
    case 'ini'  % Initialize figure and controlsF
        
        figpri=figure( ...
            'Name','Robot Motion Toolbox', ...
            'Position',[35 50 1100 600], ...
            'NumberTitle','off', ...
            'ToolBar','auto',...
            'Visible','off',...
            'InvertHardcopy','off',...
            'MenuBar', 'none',...%figure
            'Color',[.8 .8 .8]...
            );
        ret = figpri;
        set(figpri, 'Visible','on');
        rmt('ini_UserData');
        
        %default values
        data.initial{1} = [2 2];
        data.final{1} = [5 5];
        data.orientation = 0;
        data.removeLine = 0;
        temp(1) = 0.8;
        temp(2) = 0.5;
        temp(3) = 0.2;
        temp(4) = 0.01;
        data.pi_tuning = temp;
        data.epsilonvoronoi = 0.4;
        data.obstacles=[];
        data.Nobstacles=0;
        data.formula='(F y1) & G !(y2 | y3)';
        data.Bool_formula='!Y1 & y2';
        data.rob_plot.line={'-','--',':','-.','-','--',':','-.','-','--',':','-.','-','--',':','-.'};
        data.rob_plot.line_color={'k','k','k','k','k','k','k','k','k','k','k','k','k','k','k','k'};
        data.rob_plot.line_width={2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2};
        data.rob_plot.marker={'s','o','x','p','s','o','x','p','s','o','x','p','s','o','x','p'};
        data.rob_plot.face_color={'c','m','y','r','c','m','y','r','c','m','y','r','c','m','y','r'};
        data.reg_plot.color = ['r','b','g','c','m','k','r','b','g','c','m','k','r','b','g','c','m','k','r','b','g','c','m','k','r','b','g','c','m','k','r','b','g','c','m','k','r','b','g','c','m','k','r','b','g','c','m','k','r','b','g','c','m','k','r','b','g','c','m','k','r','b','g','c','m','k','r','b','g','c','m','k','r','b','g','c','m','k','r','b','g','c','m','k','r','b','g','c','m','k','r','b','g','c','m','k','r','b','g','c','m','k','r','b','g','c','m','k','r','b','g','c','m','k','r','b','g','c','m','k','r','b','g','c','m','k','r','b','g','c','m','k','r','b','g','c','m','k','r','b','g','c','m','k','r','b','g','c','m','k','r','b','g','c','m','k','r','b','g','c','m','k'];
        data.reg_plot.color_full = {'red','blue','green','cyan','magenta','black',...
            'red','blue','green','cyan','magenta','black','red','blue','green','cyan',...
            'magenta','black','red','blue','green','cyan','magenta','black','red','blue',...
            'green','cyan','magenta','black','red','blue','green','cyan','magenta','black',...
            'red','blue','green','cyan','magenta','black','red','blue','green','cyan',...
            'magenta','black','red','blue','green','cyan','magenta','black','red',...
            'blue','green','cyan','magenta','black','red','blue','green','cyan',...
            'magenta','black','red','blue','green','cyan','magenta','black','red',...
            'blue','green','cyan','magenta','black','red','blue','green','cyan',...
            'magenta','black','red','blue','green','cyan','magenta','black','red',...
            'blue','green','cyan','magenta','black','red','blue','green','cyan',...
            'magenta','black','red','blue','green','cyan','magenta','black','red',...
            'blue','green','cyan','magenta','black','red','blue','green','cyan',...
            'magenta','black','red','blue','green','cyan','magenta','black','red',...
            'blue','green','cyan','magenta','black','red','blue','green','cyan',...
            'magenta','black','red','blue','green','cyan','magenta','black','red',...
            'blue','green','cyan','magenta','black','red','blue','green','cyan',...
            'magenta','black','red','blue','green','cyan','magenta','black'};
        data.waypointsMPC.N = 2;
        data.waypointsMPC.safe_dist = 0.5;
        data.waypointsMPC.intermPoints = 1;
        data.control.sampling_period = 0.1;
        data.control.max_ang_vel = 1;
        data.control.max_linear_vel = 1;
        data.control.max_steering = 10;
        data.control.wheel_radius = 0.05;
        data.control.wheel_base = 0.25;
        data.control.lookahead_distance = 10;
        data.control.robot = 'Car-like';
        data.control.motion = 'Pure-Pursuit';
        data.reg_plot.text_cells_h = [];
        set(gcf,'UserData',data);
        
        
        set(gcf,'UserData',data);
        
        %MOBILE ROBOT TOOLBOX
        uicontrol( ...
            'Style','text', ...
            'Units','normalized', ...
            'BackgroundColor',[0.80 0.80 0.80], ...
            'ListboxTop',0, ...
            'HorizontalAlignment','center',...
            'FontName','Helvetica',...
            'FontSize',16,...
            'FontUnits','points',...
            'FontWeight','normal',...
            'Position',[0.03    0.928    0.29    0.037], ...
            'String','Robot Motion Toolbox (RMTool)');
        
        %axes 1 --> workspace
        handle_axes = axes('position',[0.35    0.37    0.63    0.566]);
        data.frame_limits = [0 20 0 10]; %
        data.handle_env = handle_axes;
        set(data.handle_env,'xlim',[0 20],'ylim',[0 10],'XGrid','on','YGrid','on');
        
        
        %axes 2 --> orientation
        handle_axes = axes('position',[0.35    0.054    0.19    0.2477]);
        data.handle_ori = handle_axes;
        hold on;%new
        set(data.handle_ori,'xlim',[0 20],'ylim',[-180 180],'XGrid','on','YGrid','on');
        
        %axes 3 --> linear velocity
        handle_axes = axes('position',[0.573    0.054    0.19    0.2477]);
        data.handle_vel = handle_axes;
        hold on;
        set(data.handle_vel,'xlim',[0 20],'ylim',[0 10],'XGrid','on','YGrid','on');
        
        %axes 4 --> steering angle
        handle_axes = axes('position',[0.79    0.054    0.19    0.2477],'Tag','ang');
        data.handle_ang = handle_axes;
        hold on;
        set(data.handle_ang,'xlim',[0 20],'ylim',[-50 50],'XGrid','on','YGrid','on');
        
        %text --> info regions of interest
        handle_text = uicontrol('Units','normalized','Style','edit',...
            'BackgroundColor',[0.7 0.7 0.7],'Position',[0.35 0.054 0.63 0.2477],...
            'Tag','text','String','Regions of interest:',...
            'HorizontalAlignment','left','min',0,'max',2);
        hold on;
        
        data.handle_text = handle_text;
        set(data.handle_text,'Visible','on')
        
        a = uimenu('Label','File');
        uimenu(a,'Label','&Open','Callback',strcat(thisfile,'(''open'')'));
        uimenu(a,'Label','S&ave','Callback',strcat(thisfile,'(''save'')'));
        uimenu(a,'Label','E&xport as eps','Callback',strcat(thisfile,'(''export_eps'')'),...
            'Separator','on');
        
        a = uimenu('Label','Setup');
        uimenu(a,'Label','&Environment limits','Callback',strcat(thisfile,'(''environment_limits'')'));
        uimenu(a,'Label','&Robots initial and final positions','Callback',strcat(thisfile,'(''robot_initial'')'),...
            'Separator','on');
        uimenu(a,'Label','&Add a Robot','Callback','rmt_add_robot');
        uimenu(a,'Label','Re&move Robots','Callback','rmt_remove_robot');
        uimenu(a,'Label','&Select size and name for cell''s label',...
            'Callback',strcat(thisfile,'(''size_label_cells'')'), 'Separator','on');
        uimenu(a,'Label','&Parameters for MILP PN planning Boolean specifications',...
            'Callback',strcat(thisfile,'(''parameter_MILP_pn_boolean'')'), 'Separator','on');
        uimenu(a,'Label','&Parameters for MILP PN planning following runs in Buchi',...
            'Callback',strcat(thisfile,'(''parameter_MILP_pn_following_buchi'')'),'Separator','on');
        uimenu(a,'Label','P&arameters for MILP PN with Buchi',...
            'Callback',strcat(thisfile,'(''parameter_MILP_pn_with_buchi'')'));
        uimenu(a,'Label','E&psilon Voronoi','Callback',strcat(thisfile,'(''EpsilonVoronoi'')'), 'Separator','on');
        uimenu(a,'Label','P&I tuning parameters','Callback',strcat(thisfile,'(''PI_tuning'')'));
        uimenu(a,'Label','&Motion Control Parameters','Callback',strcat(thisfile,'(''motion_control_parameters'')'), 'Separator','on');
        a = uimenu(a,'Label','&MILP solver', 'Separator','on');
        data.optim.menuCplex = uimenu(a,'Label','&CPLEX','Callback',strcat(thisfile,'(''menu_cplex'')'));
        data.optim.menuGlpk = uimenu(a,'Label','&GLPK','Callback',strcat(thisfile,'(''menu_glpk'')'),'Separator','on','Checked','on');
        data.optim.menuIntlinprog = uimenu(a,'Label','&Intlinprog','Callback',strcat(thisfile,'(''menu_intlinprog'')'),'Separator','on','Checked','off');
        
        a = uimenu('Label','Environment');
        uimenu(a,'Label','&Load environment','Callback',strcat(thisfile,'(''load_env'')'));
        uimenu(a,'Label','&Save environment','Callback',strcat(thisfile,'(''save_env'')'),'Separator','on');
        uimenu(a,'Label','&Export to workspace','Callback','rmt_export_environment_to_workspace','Separator','on');
        uimenu(a,'Label','E&xport to figure window','Callback','rmt_export_environment_to_figure');
        data.menuViewLabels = uimenu(a,'Label','&View region labels','Callback','rmt_change_region_label_visibility','Separator','on','Checked','on');
        
        a = uimenu('Label','Path Planning');
        uimenu(a,'Label','&Export to workspace','Callback','rmt_export_path_to_workspace');
        uimenu(a,'Label','E&xport to figure window','Callback','rmt_export_environment_to_figure');
        
        
        a = uimenu('Label','Help');
        uimenu(a,'Label','&About RMTool','Callback',strcat(thisfile,'(''help_menu'')'));
        
        %frame path planning
        uicontrol( ...
            'Style','frame', ...
            'Units','normalized', ...
            'Position',[0.02    0.52    0.285   0.36], ...
            'BackgroundColor',[0.70 0.70 0.70]);
        
        %path planning approach
        uicontrol( ...
            'Style','text', ...
            'Units','normalized', ...
            'BackgroundColor',[0.70 0.70 0.70], ...
            'ListboxTop',0, ...
            'HorizontalAlignment','left',...
            'Position',[0.0303    0.76    0.2628    0.0952], ...
            'String','Path Planning Approach:');
        uicontrol( ...
            'Style','popupmenu', ...
            'Units','normalized', ...
            'BackgroundColor',[1 1 1], ...
            'Value', 1,...
            'ListboxTop',0, ...
            'Position',[0.0303    0.7280    0.2628    0.0952], ...
            'Tag', 'pathpoints', ...
            'CallBack',strcat(thisfile,'(''change_planning_approach'')'), ...
            'String','Cell Decomposition|Visibility Graph|Voronoi Diagram'); % |variable step saving data');
        
        %frame type of task
        uicontrol( ...
            'Style','frame', ...
            'Units','normalized', ...
            'Position',[0.02    0.22    0.285   0.28], ...
            'BackgroundColor',[0.70 0.70 0.70]);
        
        %mission type
        uicontrol( ...
            'Style','text', ...
            'Units','normalized', ...
            'BackgroundColor',[0.70 0.70 0.70], ...
            'ListboxTop',0, ...
            'HorizontalAlignment','left',...
            'Position',[0.0303    0.38    0.2628    0.0952], ...
            'String','Mission type:');
        
        
        %radiobutton reachability
        uicontrol( ...
            'Style','radiobutton', ...
            'Units','normalized', ...
            'BackgroundColor',[0.7 0.7 0.7], ...
            'ListboxTop',0, ...
            'Position',[0.033    0.40    0.099    0.0416], ...
            'Tag', 'reach', ...
            'CallBack',strcat(thisfile,'(''reachability_task'')'), ...
            'Value',1,...
            'String','Reachability');
        %radiobotton  BOOLEAN
        uicontrol( ...
            'Style','radiobutton', ...
            'Units','normalized', ...
            'BackgroundColor',[0.7 0.7 0.7], ...
            'ListboxTop',0, ...
            'Position',[0.033    0.36    0.099    0.0416], ...
            'Tag', 'boolean', ...
            'CallBack',strcat(thisfile,'(''boolean_task'')'), ...
            'Value',0,...
            'String','Boolean formula');
        %edittext  BOOLEAN
        uicontrol( ...
            'Style','edit', ...
            'Units','normalized', ...
            'BackgroundColor',[1 1 1], ...
            'ListboxTop',0, ...
            'Position',[0.05    0.32    0.22    0.045], ...
            'CallBack',strcat(thisfile,'(''boolean_formula_changed'')'), ...
            'Tag', 'booleanformula', ...
            'String',data.Bool_formula,...
            'Visible','on');
        %radiobutton LTL
        uicontrol( ...
            'Style','radiobutton', ...
            'Units','normalized', ...
            'BackgroundColor',[0.7 0.7 0.7], ...
            'ListboxTop',0, ...
            'Position',[0.033    0.28    0.099    0.0416], ...
            'Tag', 'ltl', ...
            'CallBack',strcat(thisfile,'(''ltl_task'')'), ...
            'Value',0,...
            'String','LTL formula');
        
        %edittext ltl formula
        uicontrol( ...
            'Style','edit', ...
            'Units','normalized', ...
            'BackgroundColor',[1 1 1], ...
            'ListboxTop',0, ...
            'Position',[0.05    0.24    0.22    0.045], ...
            'CallBack',strcat(thisfile,'(''ltl_formula_changed'')'), ...
            'Tag', 'ltlformula', ...
            'String',data.formula,...
            'Visible','on');
        
        %environment button
        uicontrol( ...
            'Style','push', ...
            'Units','normalized', ...
            'BackgroundColor',[0.80 0.80 0.80], ...
            'ListboxTop',0, ...
            'Position',[0.02    0.16    0.285    0.05], ...
            'CallBack',strcat(thisfile,'(''run_environment'')'), ...
            'String','Environment');
        
        %path planning button
        uicontrol( ...
            'Style','push', ...
            'Units','normalized', ...
            'BackgroundColor',[0.80 0.80 0.80], ...
            'ListboxTop',0, ...
            'Position',[0.02    0.1   0.285    0.05], ...
            'Tag','path_planning_button',...
            'CallBack',strcat(thisfile,'(''run_path_planning'')'), ...
            'String','Path Planning');
        
        %control button
        uicontrol( ...
            'Style','push', ...
            'Units','normalized', ...
            'BackgroundColor',[0.80 0.80 0.80], ...
            'ListboxTop',0, ...
            'Position',[0.02    0.04    0.285    0.05], ...
            'CallBack',strcat(thisfile,'(''run_control'')'), ...
            'String','Motion Control');
        
        
        %radiobutton triangular cell
        uicontrol( ...
            'Style','radiobutton', ...
            'Units','normalized', ...
            'BackgroundColor',[0.7 0.7 0.7], ...
            'ListboxTop',0, ...
            'Position',[0.028    0.73    0.099    0.0416], ...
            'Tag', 'triang', ...
            'CallBack',strcat(thisfile,'(''triangular'')'), ...
            'Value',1,...
            'String','Triangular cell');
        
        %radiobutton polytopal cell
        uicontrol( ...
            'Style','radiobutton', ...
            'Units','normalized', ...
            'BackgroundColor',[0.7 0.7 0.7], ...
            'ListboxTop',0, ...
            'Position',[0.16    0.73    0.0983    0.0416], ...
            'Tag', 'poly', ...
            'CallBack',strcat(thisfile,'(''polytopal'')'), ...
            'Value',0,...
            'String','Polytopal cell');
        
        %radiobutton rectangular cell
        uicontrol( ...
            'Style','radiobutton', ...
            'Units','normalized', ...
            'BackgroundColor',[0.7 0.7 0.7], ...
            'ListboxTop',0, ...
            'Position',[0.028    0.69    0.1161    0.0416], ...
            'Tag', 'rect', ...
            'CallBack',strcat(thisfile,'(''rectangle'')'), ...
            'Value',0,...
            'String','Rectangular cell');
        
        %radiobutton trapezoidal cell
        uicontrol( ...
            'Style','radiobutton', ...
            'Units','normalized', ...
            'BackgroundColor',[0.7 0.7 0.7], ...
            'ListboxTop',0, ...
            'Position',[0.16    0.69    0.1140    0.0416], ...
            'Tag', 'trapez', ...
            'CallBack',strcat(thisfile,'(''trapez'')'), ...
            'Value',0,...
            'String','Trapezoidal cell');
        
        %graph weights
        uicontrol( ...
            'Style','text', ...
            'Units','normalized', ...
            'BackgroundColor',[0.70 0.70 0.70], ...
            'ListboxTop',0, ...
            'HorizontalAlignment','left',...
            'Position',[0.0303    0.635    0.2628    0.05], ...
            'String','Graph weights:');
        uicontrol( ...
            'Style','popupmenu', ...
            'Units','normalized', ...
            'BackgroundColor',[1 1 1], ...
            'Value', 1,...
            'ListboxTop',0, ...
            'Position',[0.0303    0.56    0.2628    0.0952], ...
            'Tag', 'weights', ...
            'String','1|norm(centr(c_i)-centr(c_j))|norm(centr(c_i)-mid(c_i,c_j))|sum(c_h, c_h~=c_j, norm(mid(c_h,c_i) mid(c_i,c_j))/(number of neighbors of c1 - 1)'); % |variable step saving data');
        
        
        %intermediate trajectory points
        uicontrol( ...
            'Style','text', ...
            'Units','normalized', ...
            'BackgroundColor',[0.70 0.70 0.70], ...
            'ListboxTop',0, ...
            'HorizontalAlignment','left',...
            'Position',[0.0303    0.555    0.2628    0.05], ...
            'String','Intermediate trajectory points:');
        uicontrol( ...
            'Style','popupmenu', ...
            'Units','normalized', ...
            'BackgroundColor',[1 1 1], ...
            'Value', 1,...
            'ListboxTop',0, ...
            'Position',[0.0303    0.48    0.2628    0.0952], ...
            'Tag', 'waypoints', ...
            'String','Middle points|Norm 1|Norm 2|Norm Inf.|MPC'); % |variable step saving data');
        
        %text: Trajectory-Workspace
        uicontrol( ...
            'Style','text', ...
            'Units','normalized', ...
            'BackgroundColor',[0.80 0.80 0.80], ...
            'ListboxTop',0, ...
            'HorizontalAlignment','left',...
            'Position',[0.6    0.955    0.1349    0.0235], ...
            'String','Trajectory / Workspace');
        %text: Orientation
        data.handles_control(1) = uicontrol( ...
            'Style','text', ...
            'Units','normalized', ...
            'BackgroundColor',[0.80 0.80 0.80], ...
            'ListboxTop',0, ...
            'HorizontalAlignment','left',...
            'Position',[0.42    0.31   0.1349    0.0235], ...
            'String','Orientation [deg]');
        %text: Orientation - time
        data.handles_control(2) = uicontrol( ...
            'Style','text', ...
            'Units','normalized', ...
            'BackgroundColor',[0.80 0.80 0.80], ...
            'ListboxTop',0, ...
            'HorizontalAlignment','left',...
            'Position',[0.43    0.005   0.1349    0.0235], ...
            'String','Time [s]');
        %text: Angular/linear velocity
        data.handles_control(3) = uicontrol( ...
            'Style','text', ...
            'Units','normalized', ...
            'BackgroundColor',[0.80 0.80 0.80], ...
            'ListboxTop',0, ...
            'HorizontalAlignment','left',...
            'Position',[0.61    0.31   0.1349    0.0235], ...
            'String','Linear velocity [m/s]');
        %text: Angular/linear velocity - time
        data.handles_control(4) = uicontrol( ...
            'Style','text', ...
            'Units','normalized', ...
            'BackgroundColor',[0.80 0.80 0.80], ...
            'ListboxTop',0, ...
            'HorizontalAlignment','left',...
            'Position',[0.65    0.005   0.1349    0.0235], ...
            'String','Time [s]');
        %text: Steering Angle
        data.handles_control(5) = uicontrol( ...
            'Style','text', ...
            'Units','normalized', ...
            'BackgroundColor',[0.80 0.80 0.80], ...
            'ListboxTop',0, ...
            'HorizontalAlignment','left',...
            'Position',[0.84    0.31   0.1349    0.0235], ...
            'String','Steering angle [deg]');
        %text: Steering Angle - time
        data.handles_control(6) = uicontrol( ...
            'Style','text', ...
            'Units','normalized', ...
            'BackgroundColor',[0.80 0.80 0.80], ...
            'ListboxTop',0, ...
            'HorizontalAlignment','left',...
            'Position',[0.87    0.005   0.1349    0.0235], ...
            'String','Time [s]');
        set(gcf,'UserData',data);
        
        data.label_cells.size = 8;
        data.label_cells.name = 'p';
        
        data.optim.cplex_variable='true';
        set(data.optim.menuGlpk,'Checked','off');
        set(data.optim.menuCplex,'Checked','on');
        set(data.optim.menuIntlinprog,'Checked','off');
        
        data.optim.param.alpha = 1;
        data.optim.param.beta = 1;
        data.optim.param.gamma = 100;
        data.optim.param.kappa = 3;
        data.optim.param.intMarkings = 10;
        data.optim.param_boolean.lambda = 1;
        data.optim.param_boolean.mu = 1000;
        data.optim.param_boolean.kappa = 10;
        data.optim.param_boolean.UserCount = 8; % for replanning robots trajectories
        data.optim.options_glpk.round=1; %Replace tiny primal and dual values by exact zero
        data.optim.options_glpk.tmlim=10; %Searching time limit, in seconds
        data.optim.paramWith.interM = 10;
        
        %check if CPLEX and Intlinprog is installed
        
        try [~]=intlinprog(1, [], 1, 1, [], [], 0, 1, optimoptions(@intlinprog,'Display','off','MaxTime',1));
            data.optim.options_milp=optimoptions(@intlinprog,'Display','off','MaxTime',10);   %stop optimization if not finished in specified time (in seconds)
        catch
            message = sprintf('\nIntlinprog (from Matlab''s Optimization Toolbox, >=2014) not available.\n');
            set(data.optim.menuIntlinprog,'Checked','off','Enable','off');
            uiwait(msgbox(message,'Robot Motion Toolbox','modal'));
        end
        try [~]=cplexmilp(1, 1, 1, [], [], [], [], [], 0, 1, 'C', []);
            warning off MATLAB:lang:badlyScopedReturnValue; %false warnings by Cplex in Matlab >= 2015b
            %linux error...
            %data.optim.options_cplex=cplexoptimset('Display','off','MaxTime',10);   %stop optimization if not finished in specified time (in seconds)
        catch
            message = sprintf('\nCplex not available. If you have Cplex, please add it to Matlab''s path and rerun.\n');
            data.optim.cplex_variable='false';
            set(data.optim.menuGlpk,'Checked','on');
            set(data.optim.menuCplex,'Checked','off','Enable','off');
            uiwait(msgbox(message,'Robot Motion Toolbox','modal'));
        end
        
        %path to ltl2ba
        if ispc
            WindowsVersion = system_dependent('getos');
            if (isempty(strfind(WindowsVersion,'10'))) %different than Windows 10 (7 or before)
                runLTL2BA = ['.' filesep 'aux_toolboxes' filesep 'ltl2ba' filesep 'ltl2ba_Win7.exe'];
            else %Windows 10
                runLTL2BA = ['.' filesep 'aux_toolboxes' filesep 'ltl2ba' filesep 'ltl2ba.exe'];
            end
        elseif (isunix && ~ismac)
            runLTL2BA = ['.' filesep 'aux_toolboxes' filesep 'ltl2ba' filesep 'ltl2bal'];
        elseif ismac
            runLTL2BA = ['.' filesep 'aux_toolboxes' filesep 'ltl2ba' filesep 'ltl2ba'];
        end
        data.ltl2ba = runLTL2BA;
        set(figpri, 'Visible','on');
        set(gcf,'UserData',data);
        rmt_delete_axes(data,0);
        
    case 'triangular'
        set(findobj(gcf,'Tag','triang'),'Value',1);
        set(findobj(gcf,'Tag','poly'),'Value',0);
        set(findobj(gcf,'Tag','rect'),'Value',0);
        set(findobj(gcf,'Tag','trapez'),'Value',0);
    case 'polytopal'
        set(findobj(gcf,'Tag','triang'),'Value',0);
        set(findobj(gcf,'Tag','poly'),'Value',1);
        set(findobj(gcf,'Tag','rect'),'Value',0);
        set(findobj(gcf,'Tag','trapez'),'Value',0);
    case 'rectangle'
        set(findobj(gcf,'Tag','triang'),'Value',0);
        set(findobj(gcf,'Tag','poly'),'Value',0);
        set(findobj(gcf,'Tag','rect'),'Value',1);
        set(findobj(gcf,'Tag','trapez'),'Value',0);
    case 'trapez'
        set(findobj(gcf,'Tag','triang'),'Value',0);
        set(findobj(gcf,'Tag','poly'),'Value',0);
        set(findobj(gcf,'Tag','rect'),'Value',0);
        set(findobj(gcf,'Tag','trapez'),'Value',1);
    case 'reachability_task'
        previous_reach = ~get(findobj(gcf,'Tag','reach'),'Value') && ~get(findobj(gcf,'Tag','ltl'),'Value') ...
            && ~get(findobj(gcf,'Tag','boolean'),'Value');
        if (previous_reach)
            %if previous was reachable do nothing. keep the regions
            set(findobj(gcf,'Tag','reach'),'Value',1);
            return;
        end
        button = questdlg('What to do with Regions?','Robot Motion Toolbox',...
            'To Obstacles', 'Delete', 'Cancel','To Obstacles');
        if strcmpi(button,'Delete')
            data = get(gcf,'UserData');
            rmt_delete_axes(data,0);
            data.initial={};
            data.final={};
            data.obstacles = {};
            data.trajectory = {};
            data.Nobstacles = 0;
            if (isfield(data,'T'))
                data = rmfield(data,'T');
            end
            set(gcf,'UserData',data);
            rmt('run_environment');
        elseif strcmpi(button,'Cancel')
            set(findobj(gcf,'Tag','reach'),'Value',0);
            return;
        elseif strcmpi(button,'To Obstacles') %
            data = get(gcf,'UserData');
            colors=data.reg_plot.color_full;
            rmt_delete_axes(data,0);
            rmt_plot_environment(data.obstacles,data.frame_limits);
            data.initial = {[data.initial{1}(1) data.initial{1}(2)]};
            data.final = data.initial;
            data.orientation = 0;
            data.RO = data.RO(1);
            plot(data.initial{1}(1),data.initial{1}(2),'color',colors{1},'marker','o','Linewidth',2);
            text((data.initial{1}(1)+0.2),(data.initial{1}(2)+0.2),{num2str(1)});
            plot(data.final{1}(1),data.final{1}(2),'color','k','marker','x','Linewidth',2);
            text((data.final{1}(1)+0.2),(data.final{1}(2)+0.2),{char(64+1)});
            uiwait(msgbox('Initial and final point of the robot coincide!','Robot Motion Toolbox','modal'));
            set(gcf,'UserData',data);
            rmt('robot_initial');
        end
        set(findobj(gcf,'Tag','reach'),'Value',1);
        set(findobj(gcf,'Tag','ltl'),'Value',0);
        set(findobj(gcf,'Tag','boolean'),'Value',0);
        set(findobj(gcf,'Tag','pathpoints'),'Enable','on');
        set(findobj(gcf,'Tag','triang'),'Enable','on');
        set(findobj(gcf,'Tag','rect'),'Enable','on');
        set(findobj(gcf,'Tag','poly'),'Enable','on');
        set(findobj(gcf,'Tag','trapez'),'Enable','on');
        set(findobj(gcf,'Tag','waypoints'),'Enable','on');
    case 'ltl_task'
        previous_ltl = ~get(findobj(gcf,'Tag','reach'),'Value') && ~get(findobj(gcf,'Tag','ltl'),'Value') ...
            && ~get(findobj(gcf,'Tag','boolean'),'Value');
        if (get(findobj(gcf,'Tag','boolean'),'Value') || previous_ltl)
            %if previous was boolean or ltl do nothing. keep the regions
            set(findobj(gcf,'Tag','boolean'),'Value',0);
            set(findobj(gcf,'Tag','ltl'),'Value',1);
            return;
        end
        button = questdlg('What to do with obstacles?','Robot Motion Toolbox',...
            'Relabel as regions', 'Delete', 'Cancel','Relabel as regions');
        if strcmpi(button,'Delete')
            data = get(gcf,'UserData');
            rmt_delete_axes(data,1);
            data.initial={};
            data.final={};
            data.obstacles = {};
            data.trajectory = {};
            data.Nobstacles = 0;
            set(gcf,'UserData',data);
            rmt('run_environment');
        elseif strcmpi(button,'Relabel as regions')
            data = get(gcf,'UserData');
            if (length(data.obstacles)<1)
                return;
            end
            rmt_delete_axes(data,1);
            pause(0.001);
            rmt_generate_partitions(data.obstacles,data.initial);%generate the partitions and initialize the transition system and the PN
            set(findobj(gcf,'Tag','path_planning_button'),'Enable','on');
            set(findobj(gcf,'Tag','reach'),'Value',0);
            set(findobj(gcf,'Tag','boolean'),'Value',0);
            set(findobj(gcf,'Tag','ltl'),'Value',1);
        else
            if ((get(findobj(gcf,'Tag','reach'),'Value')==1) || (get(findobj(gcf,'Tag','boolean'),'Value')==1))
                set(findobj(gcf,'Tag','ltl'),'Value',0);
            else
                set(findobj(gcf,'Tag','ltl'),'Value',1);
            end
        end
    case 'boolean_task'
        previous_bool = ~get(findobj(gcf,'Tag','reach'),'Value') && ~get(findobj(gcf,'Tag','ltl'),'Value') ...
            && ~get(findobj(gcf,'Tag','boolean'),'Value');
        if (get(findobj(gcf,'Tag','ltl'),'Value') || previous_bool)
            %if previous was ltl or boolean do nothing. keep the regions
            set(findobj(gcf,'Tag','ltl'),'Value',0);
            set(findobj(gcf,'Tag','boolean'),'Value',1);
            return;
        end
        button = questdlg('What to do with obstacles?','Robot Motion Toolbox',...
            'Relabel as regions', 'Delete', 'Cancel','Relabel as regions');
        if strcmpi(button,'Delete')
            data = get(gcf,'UserData');
            rmt_delete_axes(data,2);
            data.initial={};
            data.final={};
            data.trajectory = {};
            data.obstacles = {};
            data.Nobstacles = 0;
            set(gcf,'UserData',data);
            rmt('run_environment');
        elseif strcmpi(button,'Relabel as regions')
            data = get(gcf,'UserData');
            if (length(data.obstacles)<1)
                return;
            end
            rmt_delete_axes(data,1);
            pause(0.001);
            rmt_generate_partitions(data.obstacles,data.initial);%generate the partitions and initialize the transition system and the PN
            set(findobj(gcf,'Tag','path_planning_button'),'Enable','on');
            set(findobj(gcf,'Tag','reach'),'Value',0);
            set(findobj(gcf,'Tag','ltl'),'Value',0);
            set(findobj(gcf,'Tag','boolean'),'Value',1);
        else
            if ((get(findobj(gcf,'Tag','reach'),'Value')==1) || (get(findobj(gcf,'Tag','ltl'),'Value')==1))
                set(findobj(gcf,'Tag','boolean'),'Value',0);
            else
                set(findobj(gcf,'Tag','boolean'),'Value',1);
            end
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %
        %               RUN ENVIRONMENT
        %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
    case 'run_environment'
        
        if get(findobj(gcf,'Tag','reach'),'Value')
            mission_task = 0;
        elseif get(findobj(gcf,'Tag','ltl'),'Value')
            mission_task = 1;
        elseif get(findobj(gcf,'Tag','boolean'),'Value')
            mission_task = 2;
        end
        %mission_task=1 - reachability tasks; 0 - ltl tasks or boolean
        
        % This is the code panel  containing:
        % NUMBER OF REGION OF INTEREST and NUMBER OF ROBOTS
        
        if ((mission_task == 1) || (mission_task == 2))
            [objects,initial_points,~,random_grid] = rmt_get_regions(mission_task);%get the regions of interests random or given by the user
            if (isempty(objects) || isempty(initial_points))
                return;
            end
            rmt_generate_partitions(objects,initial_points,random_grid);%generate the partitions and initialize the transition system and the PN
            set(findobj(gcf,'Tag','path_planning_button'),'Enable','on');
        else %mission_task == 0 ---> reachability tasks
            planning_approach = get(findobj(gcf,'Tag','pathpoints'),'Value');
            %planning_approach= 1 - cell descomposition; 2 - visibility graph; 3 - Voronoi;
            data = get(gcf,'UserData');
            switch planning_approach
                case {1,2} %cell decomposition && visibility_graph
                    %obs_no = char(inputdlg('Number of obstacles:','Robot Motion Toolbox',1,{'1'}));
                    [objects,initial_points,final_points] = rmt_get_regions(mission_task);%get the regions of interests random or given by the user
                    data.Nobstacles = length(objects);
                    data.obstacles = objects;
                    data.initial=initial_points;
                    data.final=final_points;
                    data.orientation = 0;
                    set(gcf,'UserData',data);
                    set(findobj(gcf,'Tag','path_planning_button'),'Enable','on');
                case 3 %Voronoi
                    disp('Voronoi diagram...');
                    obs_no = char(inputdlg('Number of obstacles:','Robot Motion Toolbox',1,{'1'}));
                    if isempty(obs_no)
                        return;
                    end
                    try
                        obs_no = str2double(obs_no);
                    catch
                        uiwait(errordlg(sprintf('\nNumber of obstacles should be a natural number between 1 and 5!'),'Robot Motion Toolbox','modal'));
                        rmt('run_environment');
                        return;
                    end
                    data = get(gcf,'UserData');
                    limits = data.frame_limits;
                    %we clean the workspace figure
                    cla(data.handle_env);
                    set(data.handle_env,'xlim',[limits(1) limits(2)],'ylim',[limits(3) limits(4)],'XGrid','on','YGrid','on');
                    
                    %we clean the orientation figure
                    cla(data.handle_ori);
                    set(data.handle_ori,'XGrid','on','YGrid','on','Visible','off');
                    %we clean the velocities figure
                    cla(data.handle_vel);
                    set(data.handle_vel,'XGrid','on','YGrid','on','Visible','off');
                    %we clean the steering angle figure
                    cla(data.handle_ang);
                    set(data.handle_ang,'XGrid','on','YGrid','on','Visible','off');
                    wheelbase_var = eval(get(findobj(gcf,'Tag','wheelbase'),'String'));
                    %[map, obstacles,initial_point,final_point] = rmt_build_grid_map2(data.handle_env,limits,obs_no);
                    [initial_point, final_point, X_Total_points,Y_Total_points, ...
                        All_cells_Number, Cell_start, X1] = rmt_obstacle_draw(data.handle_env,obs_no,limits,wheelbase_var*0.5,data.epsilonvoronoi);
                    set(data.handle_ori,'Visible','on','xlim',[0 20],'ylim',[-180 180],'XGrid','on','YGrid','on');
                    set(data.handle_vel,'Visible','on','xlim',[0 20],'ylim',[0 10],'XGrid','on','YGrid','on');
                    set(data.handle_ang,'Visible','on','xlim',[0 20],'ylim',[-50 50],'XGrid','on','YGrid','on');
                    %saving data
                    data.Nobstacles = obs_no;
                    data.initial{1}=initial_point;
                    data.final{1}=final_point;
                    data.X_Total_points = X_Total_points;
                    data.Y_Total_points = Y_Total_points;
                    data.All_cells_Number = All_cells_Number;
                    data.Cell_start = Cell_start;
                    data.X1 = X1;
                    for(a=2:length(X1))
                        poly_obstacles(a-1) = {X1{a}};
                    end
                    data.obstacles = poly_obstacles;
                    data.orientation= data.orientation(1);
                    set(gcf,'UserData',data);
                    set(findobj(gcf,'Tag','path_planning_button'),'Enable','on');
            end%switch
            set(gcf,'UserData',data);%to save data
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %
        %               RUN PATH PLANNING
        %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case 'run_path_planning'
        % Code relative to: "RUN PATH PLANNING".
        % Case 1: Reachability
        % Case 2: LTL Formula
        % Case 3: Bloolean formula
        if (get(findobj(gcf,'Tag','reach'),'Value') == 1)
            mission_task = 1;
        elseif (get(findobj(gcf,'Tag','ltl'),'Value') == 1)
            mission_task = 2;
        elseif (get(findobj(gcf,'Tag','boolean'),'Value') == 1)
            mission_task = 3;
        else
            error('Unknown mission type!');
        end
        
        switch mission_task
            case 1 %rechability tasks
                rmt_path_planning_reachability;
            case 2
                data=get(gcf,'UserData');
                cla(data.handle_env);
                rmt_plot_environment(data.obstacles,data.frame_limits,data.T.Vert);
                rmt_represent_atomic_props(data.T.Vert,data.propositions);    %represent all atomic props
                
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % Insert button of analysis choice between Buchi and Petri
                % Net with LTL Formula
                choiceMenuLTL = questdlg('Which model do you want to use?', ...
                    'Robot Motion Toolbox', ...
                    'Transistion System','Petri Net','Petri Net');
                %% Part about analysis with Buchi Automaton
                if strcmpi(choiceMenuLTL,'Transistion System')
                    rmt_path_planning_ltl_trsys;
                elseif strcmpi(choiceMenuLTL,'Petri Net')
                    choiceMenuLTL = questdlg('Which approach do you want to use?', ...
                        'Robot Motion Toolbox', ...
                        'Following runs in Buchi','With Buchi Included','With Buchi Included');
                    %% Part about PN model with Buchi Automaton
                    if strcmpi(choiceMenuLTL,'Following runs in Buchi')
                        rmt_path_planning_ltl_pn_following_buchi;
                    elseif strcmpi(choiceMenuLTL,'With Buchi Included')
                        rmt_path_planning_ltl_pn_with_buchi;
                    end
                end
            case 3 %Boolean formulas on Petri net models
                data=get(gcf,'UserData');
                Bool_formula = get(findobj(gcf,'Tag','booleanformula'),'String');
                [A,~,negated_trajectory_alone] = rmt_formula2constraints(Bool_formula, [],[],length(data.T.props));
                if (negated_trajectory_alone == 1) %nagation propostion on the trajectory are alone hence we will compute collision free trajectories
                    %check if the formula on trajectory and on final state are
                    %independent
                    indep = 1;
                    for i = 1 : size(A,1)
                        temp = find(A(i,:));
                        if (~isempty(find(temp<= length(data.T.props), 1)) && ~isempty(find(temp>length(data.T.props), 1)))
                            indep = 0;
                            break;
                        end
                    end
                    if (indep == 1)
                        if (length(data.RO) ~= length(unique(data.RO))) %exist robots in the same region
                            button = questdlg('Assumtions to compute collision free trajectories are satisfied. However, some regions contain more than one robot. Do you want to randomly re-distribute the robots in free regions?','Robot Motion Toolbox');
                            if strcmpi(button,'Cancel')
                                return;
                            elseif strcmpi(button,'No')
                                uiwait(msgbox('Collisions will be considered as soft constraints. The robots could collide! Use the Setup menu to select the weights of the cost function.Approach in "C. Mahulea and M. Kloetzer, Robot Planning based on Boolean Specifications using Petri Net Models, IEEE TAC, 63(7): 2218-2225, July 2018" will be used!','Robot Motion Toolbox','modal'));
                                rmt_path_planning_boolean;%%approach without collision avoidance (it is only soft constraint) TAC 2018
                                return;
                            end
                            rmt_redistribute_robots;
                        end
                        button = questdlg('Assumtions to compute collision free trajectories are satisfied. Consider collision avoidance as hard constraints (WODES 2020)? If chose no, collisions will be considered as soft constrains ("C. Mahulea and M. Kloetzer, Robot Planning based on Boolean Specifications using Petri Net Models, IEEE TAC, 63(7): 2218-2225, July 2018").','Robot Motion Toolbox');%%approach with collision avoidance WODES2020
                        if strcmpi(button,'Cancel')
                            return;
                        elseif strcmpi(button,'Yes')
                            uiwait(msgbox('Collisions will be considered as hard constraints. Use the Setup menu to select the weights of the cost function. Approach in "WODES 2020" will be used!','Robot Motion Toolbox','modal'));
                            rmt_path_planning_boolean_new;
                            return;
                        end
                    end
                end
                rmt_path_planning_boolean;%%approach without collision avoidance (it is only soft constraint) TAC 2018
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %
        %               RUN CONTROL
        %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case 'run_control'
        data = get(gcf,'UserData');
        sampling_period = data.control.sampling_period;
        max_ang_vel = data.control.max_ang_vel;
        max_linear_vel = data.control.max_linear_vel;
        max_steering = data.control.max_steering;
        wheel_radius = data.control.wheel_radius;
        wheel_base = data.control.wheel_base;
        lookahead_distance = data.control.lookahead_distance;
        
        if strcmpi(data.control.robot,'Car-like')
            robotType = 1; %car-like robot
        else
            robotType = 2; %differential robot
        end
        
        temp1 = data.initial{1};
        xini = temp1(1);
        yini = temp1(2);
        if ~isfield(data,'trajectory')
            return;
        end
        set(data.handle_text,'Visible','Off');
        for ii = 1 : length(data.trajectory)
            ref_trajectory = data.trajectory{ii};
            %threshold_goal = 0.15;%distance to consider the goal has been reached.
            threshold_goal = 0.05;%distance to consider the goal has been reached.
            input_variables = [sampling_period, xini, yini, wheel_radius,...
                wheel_base,max_linear_vel,...
                max_steering, lookahead_distance,robotType,threshold_goal,...
                max_ang_vel,data.frame_limits(1),data.frame_limits(2),...
                data.frame_limits(3) data.frame_limits(4)];
            if strcmpi(data.control.motion,'Pure-Pursuit')
                %function pure-pursuit
                %fprintf('\nPure pursuit controller is running...');
                [array_time, array_alpha, array_v, array_pos] = rmt_pure_pursuit(input_variables,ref_trajectory,data.orientation,data.obstacles,data.Nobstacles);
            else
                %PI control
                %fprintf('\nPI controller is running...');
                theta_init = data.orientation;%improvement permit to the user to select the initial orientation of the robot
                dstar = 0.01;
                threshold_goal = 0.2;%distance to consider the goal has been reached.
                input_variables = [sampling_period, xini, yini, theta_init,wheel_radius,...
                    wheel_base,max_linear_vel,max_steering, dstar,robotType,threshold_goal,...
                    max_ang_vel,lookahead_distance,data.frame_limits(1),data.frame_limits(2),...
                    data.frame_limits(3) data.frame_limits(4)];
                pi_tuning = data.pi_tuning;
                [array_time, array_alpha, array_v, array_pos] = rmt_pi_controller(input_variables,ref_trajectory,pi_tuning,data.obstacles,data.Nobstacles);
            end
            
            %new code (removing lines) - NOVEMBER 2018
            removePlots = questdlg('Remove previous trajectories?', 'Robot Motion Toolbox', 'Yes', 'No','Yes');
            if(strcmp(removePlots,'Yes'))
                if(data.removeLine > 0)
                    h = findobj('type','line');
                    aux = data.removeLine;
                    delete(h(1:aux));
                    delete(h(aux+4:end));
                    data.removeLine = 1;
                end
            else
                data.removeLine = data.removeLine + 1;
            end
            set(gcf,'UserData',data);%to save data
            
            
            %drawing the result
            color = hsv(5);
            color = color(randperm(5),:);
            cha = floor(rand(1)*5+1);
            %trajectory
            colora = rand(1,3);
            plot(data.handle_env,array_pos(1,:),array_pos(2,:),'Color', colora, 'LineWidth',2);
            set(data.handle_env,'XGrid','on','YGrid','on');
            
            %orientation
            plot(data.handle_ori,array_time,rad2deg(array_pos(3,:)),'Color', colora,'LineWidth',2);
            hold on;
            title(data.handle_ori,'Orientation [deg]');
            xmax = max(array_time);
            ymax = max(rad2deg(array_pos(3,:)));
            ymin = min(rad2deg(array_pos(3,:)));
            if(ymax == ymin)
                ymax = ymax + 0.1;
            end
            set(data.handle_ori,'Visible','on','xlim',[0 xmax],'ylim',[ymin ymax],'XGrid','on','YGrid','on');
            
            %velocities
            plot(data.handle_vel, array_time, array_v,'Color', colora,'LineWidth',2);
            hold on;
            title(data.handle_vel,'Velocities [m/s]');
            xmax = max(array_time);
            ymax = max(array_v)+1;
            ymin = min(array_v)-1;
            if(ymax == ymin)
                ymax = ymax + 0.1;
            end
            set(data.handle_vel,'Visible','on','xlim',[0 xmax],'ylim',[ymin ymax],'XGrid','on','YGrid','on');
            %set(data.handle_vel,'XGrid','on','YGrid','on');
            
            %steering angle
            plot(data.handle_ang,array_time,rad2deg(array_alpha),'Color', colora,'LineWidth',2);
            hold on;
            title(data.handle_ang,'Steering angle [deg]');
            xmax = max(array_time);
            ymax = max(rad2deg(array_alpha));
            ymin = min(rad2deg(array_alpha));
            if(ymax == ymin)
                ymax = ymax + 0.1;
            end
            set(data.handle_ang,'Visible','on','xlim',[0 xmax],'ylim',[ymin ymax],'XGrid','on','YGrid','on');
            %set(data.handle_ang,'XGrid','on','YGrid','on');
        end %%%for corresponding to each trajectory
    case 'motion_control_parameters'
        data = get(gcf,'UserData');
        data.control = rmt_control_setup(data.control);
        set(gcf,'UserData',data);
        
    case 'change_planning_approach'
        planning_approach = get(findobj(gcf,'Tag','pathpoints'),'Value'); %planning_approach= 1 - cell descomposition; 2 - visibility graph; 3 - Voronoi
        if (planning_approach == 1)
            set(findobj(gcf,'Tag','triang'),'Enable','on');
            set(findobj(gcf,'Tag','rect'),'Enable','on');
            set(findobj(gcf,'Tag','poly'),'Enable','on');
            set(findobj(gcf,'Tag','trapez'),'Enable','on');
            set(findobj(gcf,'Tag','waypoints'),'Enable','on');
        else
            set(findobj(gcf,'Tag','triang'),'Enable','off');
            set(findobj(gcf,'Tag','rect'),'Enable','off');
            set(findobj(gcf,'Tag','poly'),'Enable','off');
            set(findobj(gcf,'Tag','trapez'),'Enable','off');
            set(findobj(gcf,'Tag','waypoints'),'Enable','off');
        end
    case 'save'
        [filename, pathname] = uiputfile('*.rmt', 'Save Workspace as');
        
        if (~isequal(filename,0) && ~isequal(pathname,0))
            hgsave(fullfile(pathname, filename));
        end
    case 'open'
        [file, path1] = uigetfile({'*.rmt'}, 'Load');
        file = char(file);
        path1 = char(path1);
        file2=fullfile(path1,file);
        if ~isempty(strfind(file,'.rmt'));
            delete(gcf);
            hgload(file2);
        end
    case 'export_eps'
        [filename, pathname] = uiputfile('*.eps', 'Export figure as eps');
        
        if (~isequal(filename,0) && ~isequal(pathname,0))
            oldscreenunits = get(gcf,'Units');
            oldpaperunits = get(gcf,'PaperUnits');
            oldpaperpos = get(gcf,'PaperPosition');
            set(gcf,'Units','pixels');
            scrpos = get(gcf,'Position');
            newpos = scrpos/100;
            set(gcf,'PaperUnits','inches',...
                'PaperPosition',newpos);
            
            print('-depsc2',fullfile(pathname, filename),'-r300');
            
            set(gcf,'Units',oldscreenunits,...
                'PaperUnits',oldpaperunits,...
                'PaperPosition',oldpaperpos);
        end
    case 'environment_limits'
        data = get(gcf,'UserData');
        answer = inputdlg({...
            sprintf('x min:'),...
            sprintf('x max:'),...
            sprintf('y min:')...
            sprintf('y max: \n\t')},'Robot Motion Toolbox',...
            [1,1,1,1],{num2str(data.frame_limits(1)),num2str(data.frame_limits(2)),...
            num2str(data.frame_limits(3)),num2str(data.frame_limits(4))});
        try
            temp(1) = eval(answer{1});
            temp(2) = eval(answer{2});
            temp(3) = eval(answer{3});
            temp(4) = eval(answer{4});
        catch
            uiwait(errordlg('Error introducing data!','Robot Motion Toolbox','modal'));
            return;
        end
        if ((temp(1) >= temp(2)) || (temp(3)>= temp(4)) || (temp(1)~=round(temp(1))) ...
                || (temp(2)~=round(temp(2))) || (temp(3)~=round(temp(3))) || (temp(4)~=round(temp(4))))
            uiwait(errordlg('Error introducing data!','Robot Motion Toolbox','modal'));
            return;
        end
        data.frame_limits = temp;
        set(data.handle_env,'xlim',[temp(1) temp(2)],'ylim',[temp(3) temp(4)],'XGrid','on','YGrid','on');
        set(gcf,'UserData',data);
    case 'robot_initial'
        data = get(gcf,'UserData');
        mission_task = get(findobj(gcf,'Tag','reach'),'Value'); %mission_task=1 - reachability tasks; 0 - ltl tasks
        
        for i = 1 :length(data.initial)
            str{i} = sprintf('Robot %d located at [%s,%s]',i,...
                mat2str(data.initial{i}(1),3),mat2str(data.initial{i}(2),3));
        end
        [Selection,ok] = listdlg('PromptString','Select the robot to move:',...
            'SelectionMode','single',...
            'ListString',str);
        if (ok == 0)
            return;
        end
        if (mission_task == 1) %%consider also the final destination
            answer = inputdlg({...
                sprintf('Initial point:'),...
                sprintf('Final point:'),...
                sprintf('Initial orientation (deg):')},'Robot Motion Toolbox',...
                [1;1;1],{mat2str(data.initial{Selection},3),mat2str(data.final{Selection},3),...
                num2str(data.orientation(Selection),3)});
        else %only initial points
            answer = inputdlg({...
                sprintf('Initial point:'),...
                sprintf('Initial orientation (deg):')},'Robot Motion Toolbox',...
                [1;1],{mat2str(data.initial{Selection},3),...
                num2str(data.orientation(Selection),3)});
        end
        try
            initial = eval(answer{1});
            if (mission_task == 1)
                final = eval(answer{2});
                ini_ori = eval(answer{3});
            else
                ini_ori = eval(answer{2});
            end
        catch
            uiwait(errordlg('Error introducing data!','Robot Motion Toolbox','modal'));
            return;
        end
        if (length(initial) ~= 2 )
            uiwait(errordlg('Error introducing data!','Robot Motion Toolbox','modal'));
            return;
        end
        if (mission_task == 1)
            if (length(final) ~= 2)
                uiwait(errordlg('Error introducing data!','Robot Motion Toolbox','modal'));
                return;
            end
        end
        %%check if inside any region of interest /obstacles
        for i=1:length(data.obstacles)
%             if inpolygon(initial(1),initial(2),data.obstacles{i}(1,:),data.obstacles{i}(2,:))
%                 uiwait(errordlg('Initial point inside a region of interest/obstacle! Robot not moved!','Robot Motion Toolbox','modal'));
%                 return;
%             end
            if (mission_task == 1)
                if inpolygon(final(1),final(2),data.obstacles{i}(1,:),data.obstacles{i}(2,:))
                    uiwait(errordlg('Final point inside a region of interest/obstacle! Robot not moved!','Robot Motion Toolbox','modal'));
                    return;
                end
            end
        end
        if isfield(data,'T')
            for j=1:length(data.T.Vert)   %indices of starting cell
                if inpolygon(initial(1),initial(2),data.T.Vert{j}(1,:),data.T.Vert{j}(2,:))
                    data.RO(Selection) = j;
                    data.T.m0(data.T.RO(Selection)) = data.T.m0(data.T.RO(Selection)) - 1;
                    data.T.RO(Selection) = j;
                    data.T.m0(data.T.RO(Selection)) = data.T.m0(data.T.RO(Selection)) + 1;
                    break;
                end
            end
        end
        data.initial{Selection} = initial;
        if (mission_task == 1)
            data.final{Selection} = final;
        end
        data.orientation(Selection) = ini_ori;
        set(gcf,'UserData',data);
        mission_task = get(findobj(gcf,'Tag','reach'),'Value');
        %mission_task=1 - reachability tasks; 0 - ltl tasks
        if (mission_task == 1)  %%update the graphical representation
            cla(data.handle_env);
            for i = 1 : length(data.obstacles)
                fill(data.obstacles{i}(1,:),data.obstacles{i}(2,:),...
                    'b-','FaceAlpha',0.5); %or functia patch (similara cu fill)
            end
            plot(data.initial{1}(1),data.initial{1}(2),'or','LineWidth',3);
            plot(data.final{1}(1),data.final{1}(2),'xk','LineWidth',3);
        else
            rmt_plot_robots;
        end
        
    case 'PI_tuning'
        data = get(gcf,'UserData');
        answer = inputdlg({...
            sprintf('Kv:'),...
            sprintf('Ki:'),...
            sprintf('Kh:')...
            sprintf('dstar: \n\t')},'Robot Motion Toolbox',...
            [1,1,1,1],{num2str(data.pi_tuning(1)),num2str(data.pi_tuning(2)),...
            num2str(data.pi_tuning(3)),num2str(data.pi_tuning(4))});
        try
            temp(1) = eval(answer{1});
            temp(2) = eval(answer{2});
            temp(3) = eval(answer{3});
            temp(4) = eval(answer{4});
        catch
            uiwait(errordlg('Error introducing data!','Robot Motion Toolbox','modal'));
            return;
        end
        data.pi_tuning = temp;
        set(gcf,'UserData',data);
        
    case 'EpsilonVoronoi'
        data = get(gcf,'UserData');
        answer = inputdlg({...
            sprintf('Epsilon:')},'Robot Motion Toolbox',...
            [1],{num2str(data.epsilonvoronoi)});
        try
            temp(1) = eval(answer{1});
        catch
            uiwait(errordlg('Error introducing data!','Robot Motion Toolbox','modal'));
            return;
        end
        data.epsilonvoronoi = temp;
        set(gcf,'UserData',data);
        
    case 'ltl_formula_changed'
        data = get(gcf,'UserData');
        data.formula = get(findobj(gcf,'Tag','ltlformula'),'String');
        set(gcf,'UserData',data);
    case 'boolean_formula_changed'
        data = get(gcf,'UserData');
        data.Bool_formula = get(findobj(gcf,'Tag','booleanformula'),'String');
        set(gcf,'UserData',data);
    case 'help_menu'
        set(gcf,'Units','pixel');
        dim = get(gcf,'Position');
        set(gcf,'Units','normalized');
        figure('WindowStyle','modal','Units','pixel','Menubar','None','NumberTitle','off','Name',...
            'About RMTool','Visible','on','Position',[dim(1)+(dim(3)-700)/2 dim(2)+(dim(4)-414)/2 700 414],'Resize','off');%,...
        axes('Position',[0 0 1 1]);
        about23 = imread('rmt_data.txt','bmp');
        %load PNTuse about23; %  524 x  886
        imshow(about23);
        axis off;
    case 'close_hwait'
        data = get(gcf,'UserData');
        delete(data.hwait);
    case 'save_env'
        data = get(gcf,'UserData');
        
        [filename, pathname] = uiputfile('*.env', 'Save Environment as');
        
        if (~isequal(filename,0) && ~isequal(pathname,0))
            eval(sprintf('save %s data',fullfile(pathname, filename)));
        end
    case 'load_env'
        [file, path1] = uigetfile({'*.env'}, 'Load');
        file = char(file);
        path1 = char(path1);
        file2=fullfile(path1,file);
        data = get(gcf,'UserData');
        warning off;
        if ~isempty(strfind(file,'.env'))
            eval(sprintf('load ''%s'' ''-mat''',file2));
            if ~exist('data','var')
                h = errordlg('Error loading from file','Robot Motion Toolbox');
                uiwait(h);
                return;
            end
            data2 = get(gcf,'UserData');
            %%%%
            data2.initial = data.initial;
            data2.final = data.final;
            data2.obstacles = data.obstacles;
            data2.Nobstacles = data.Nobstacles;
            if isfield(data,'RO')
                data2.RO = data.RO;
            else
                data2.RO = length(data2.initial);
            end
            data2.T = data.T;
            if isfield(data,'Tr')
                data2.Tr = data.Tr;
            else
                data2.Tr = rmt_quotient_T_new(data.T);
            end
            data2.propositions = data.propositions;
            mission_task = get(findobj(gcf,'Tag','reach'),'Value'); %mission_task=1 - reachability tasks; 0 - ltl tasks
            if (mission_task == 0)
                limits = data2.frame_limits;
                %we clean the workspace figure
                cla(data2.handle_env);
                set(data2.handle_env,'xlim',[limits(1) limits(2)],'ylim',[limits(3) limits(4)],'XGrid','on','YGrid','on');
                %we clean the orientation figure
                cla(data2.handle_ori);
                set(data2.handle_ori,'XGrid','on','YGrid','on','Visible','off');
                %we clean the velocities figure
                cla(data2.handle_vel);
                set(data2.handle_vel,'XGrid','on','YGrid','on','Visible','off');
                %we clean the steering angle figure
                cla(data2.handle_ang);
                set(data2.handle_ang,'XGrid','on','YGrid','on','Visible','off');
                axes(data2.handle_env);
                rmt_plot_environment(data2.obstacles,data2.frame_limits,data2.T.Vert);
                rmt_represent_atomic_props(data2.T.Vert,data2.propositions);    %represent all atomic props
                rob_plot = data2.rob_plot;
                %%Execution monitoring strategy starts from here:
                robot_no = length(data2.RO);
                current_pos=cell(1,robot_no);
                for r=1:robot_no
                    current_pos{r}=data2.initial{r};%R_trajs{r}(:,1); %current_pos - initial (current) position of robots (cell, current_pos{i} is a column vector with 2 rows)
                end
                for r=1:robot_no
                    plot(current_pos{r}(1),current_pos{r}(2),'Color',rob_plot.line_color{r},'LineStyle',rob_plot.line{r},'LineWidth',rob_plot.line_width{r},'Marker',rob_plot.marker{r},'MarkerFaceColor',rob_plot.face_color{r});
                end
                set(gcf,'UserData',data2);
            end
        end
    %% Choose size and label for cells    
    case 'size_label_cells'
        data = get(gcf,'UserData');
        data.label_cells = rmt_label_cells_setup(data.label_cells);
        set(gcf,'UserData',data);
        %% Part of Menu 'Setup' CPLEX
    case 'menu_cplex'
        data = get(gcf,'UserData');
        data.optim.cplex_variable= 'true';
        set(data.optim.menuGlpk,'Checked','off');
        set(data.optim.menuIntlinprog,'Checked','off');
        set(data.optim.menuCplex,'Checked','on');
        set(gcf,'UserData',data);%to save data
        
        %% Part of Menu 'Setup' GLPK
    case 'menu_glpk'
        data = get(gcf,'UserData');
        data.optim.cplex_variable= 'false';
        set(data.optim.menuGlpk,'Checked','on');
        set(data.optim.menuIntlinprog,'Checked','off');
        set(data.optim.menuCplex,'Checked','off');
        set(gcf,'UserData',data);%to save data
        
        %% Part of Menu 'Setup' Intlinprog
    case 'menu_intlinprog'
        data = get(gcf,'UserData');
        data.optim.cplex_variable= 'false';
        set(data.optim.menuGlpk,'Checked','off');
        set(data.optim.menuIntlinprog,'Checked','on');
        set(data.optim.menuCplex,'Checked','off');
        set(gcf,'UserData',data);%to save data
    case 'parameter_MILP_pn_boolean'
        data = get(gcf,'UserData');
        data.optim.param_boolean = rmt_milp_pn_boolean_setup(data.optim.param_boolean);
        set(gcf,'UserData',data);
    case 'parameter_MILP_pn_following_buchi'
        data = get(gcf,'UserData');
        data.optim.param = rmt_milp_pn_following_setup(data.optim.param);
        set(gcf,'UserData',data);
    case 'parameter_MILP_pn_with_buchi'
        data = get(gcf,'UserData');
        data.optim.paramWith = rmt_milp_pn_with_setup(data.optim.paramWith);
        set(gcf,'UserData',data);
end    % switch
