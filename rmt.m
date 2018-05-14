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
%   First version released on September, 2014. k
%   Last modification February 14, 2018.
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

switch action
    
    %================
    % INITIALIZATION
    %================
    case 'ini'  % Initialize figure and controls
        
        figpri=figure( ...
            'Name','Robot Motion Toolbox', ...
            'Position',[35 50 1100 600], ...
            'NumberTitle','off', ...
            'MenuBar', 'none',...
            'ToolBar','auto',...
            'Visible','off',...
            'Color',[.8 .8 .8]...
            );
        ret = figpri;
        
        set(figpri, 'Visible','on');
        rmt('ini_UserData');
        
        %default values
        data.initial{1} = [2 2];
        data.final{1} = [5 5];
        data.orientation = 0;
        temp(1) = 0.8;
        temp(2) = 0.5;
        temp(3) = 0.2;
        temp(4) = 0.01;
        data.pi_tuning = temp;
        data.epsilonvoronoi = 0.4;
        data.intermediateMarkings = 10;
        data.obstacles=[];
        data.Nobstacles=0;
        data.formula='(F y1) & G !(y2 | y3)';
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
            'Position',[0.03    0.9277    0.29    0.0362], ...
            'String','MOBILE ROBOT TOOLBOX');
        
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
        handle_text = annotation(gcf,'textbox',...
            'Units','normalized', ...
            'BackgroundColor',[0.7 0.7 0.7], ...
            'Position',[0.35    0.054    0.63    0.2477],...
            'Tag','text',...
            'String','Regions of interest:',...
            'HorizontalAlignment','left','Interpreter','Tex');
        hold on;
        
        data.handle_text = handle_text;
        set(data.handle_text,'Visible','off')
        
        a = uimenu('Label','File');
        uimenu(a,'Label','&Open','Callback',strcat(thisfile,'(''open'')'));
        uimenu(a,'Label','S&ave','Callback',strcat(thisfile,'(''save'')'));
        
        
        a = uimenu('Label','Setup');
        uimenu(a,'Label','&Environment limits','Callback',strcat(thisfile,'(''environment_limits'')'));
        uimenu(a,'Label','&Robots initial and final positions','Callback',strcat(thisfile,'(''robot_initial'')'),...
            'Separator','on');
        uimenu(a,'Label','&Add a Robot','Callback',strcat(thisfile,'(''add_robot'')'));
        uimenu(a,'Label','Re&move Robots','Callback',strcat(thisfile,'(''remove_robot'')'));
        uimenu(a,'Label','&Number of intermediate markings (PN planning)','Callback',strcat(thisfile,'(''change_k'')'), 'Separator','on');
        uimenu(a,'Label','E&psilon Voronoi','Callback',strcat(thisfile,'(''EpsilonVoronoi'')'), 'Separator','on');
        uimenu(a,'Label','P&I tuning parameters','Callback',strcat(thisfile,'(''PI_tuning'')'));
        uimenu(a,'Label','&Change LTL formula','Callback',strcat(thisfile,'(''menu_change_ltl_formula'')'), 'Separator','on');
        a = uimenu(a,'Label','&MILP solver', 'Separator','on');
        uimenu(a,'Label','&CPLEX','Callback',strcat(thisfile,'(''menu_cplex'')'));
        uimenu(a,'Label','&GLPK','Callback',strcat(thisfile,'(''menu_glpk'')'),'Separator','on');
        
        a = uimenu('Label','Simulation');
        uimenu(a,'Label','&Load environment','Callback',strcat(thisfile,'(''load_env'')'));
        uimenu(a,'Label','&Save environment','Callback',strcat(thisfile,'(''save_env'')'),'Separator','on');
        b = uimenu(a,'Label','&Save path planning simulation to','Separator','on');
        uimenu(b,'Label','&Workspace','Callback',strcat(thisfile,'(''save_paths_workspace'')'));
        uimenu(b,'Label','&Figure window','Callback',strcat(thisfile,'(''save_paths_figure'')'),'Separator','on');
        
        a = uimenu('Label','Export to workspace');
        uimenu(a,'Label','&Environment','Callback',strcat(thisfile,'(''save_work_env'')'));
        uimenu(a,'Label','&Petri net','Callback',strcat(thisfile,'(''save_work_pn'')'));
        
        a = uimenu('Label','Help');
        uimenu(a,'Label','&About RMTool','Callback',strcat(thisfile,'(''help_menu'')'));
        
        %frame path planning
        uicontrol( ...
            'Style','frame', ...
            'Units','normalized', ...
            'Position',[0.02    0.59    0.3   0.29], ...
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
            'String','Cell Decomposition|Visibility Graph|Voronoi Diagram|Manual points'); % |variable step saving data');
        
        %frame type of taks
        uicontrol( ...
            'Style','frame', ...
            'Units','normalized', ...
            'Position',[0.0303    0.038    0.1349    0.172], ...
            'BackgroundColor',[0.70 0.70 0.70]);
        %mission type
        uicontrol( ...
            'Style','text', ...
            'Units','normalized', ...
            'BackgroundColor',[0.70 0.70 0.70], ...
            'ListboxTop',0, ...
            'HorizontalAlignment','left',...
            'Position',[0.06    0.11    0.1    0.0952], ...
            'String','Mission type:');
        %radiobutton reachability
        uicontrol( ...
            'Style','radiobutton', ...
            'Units','normalized', ...
            'BackgroundColor',[0.7 0.7 0.7], ...
            'ListboxTop',0, ...
            'Position',[0.033    0.145    0.099    0.0416], ...
            'Tag', 'reach', ...
            'CallBack',strcat(thisfile,'(''reachability_task'')'), ...
            'Value',1,...
            'String','Reachability');
        %radiobutton LTL
        uicontrol( ...
            'Style','radiobutton', ...
            'Units','normalized', ...
            'BackgroundColor',[0.7 0.7 0.7], ...
            'ListboxTop',0, ...
            'Position',[0.033    0.113    0.099    0.0416], ...
            'Tag', 'ltl', ...
            'CallBack',strcat(thisfile,'(''ltl_task'')'), ...
            'Value',0,...
            'String','LTL formula');
        
        %radiobutton BOOLEAN
        uicontrol( ...
            'Style','radiobutton', ...
            'Units','normalized', ...
            'BackgroundColor',[0.7 0.7 0.7], ...
            'ListboxTop',0, ...
            'Position',[0.033    0.06    0.099    0.0416], ...
            'Tag', 'boolean', ...
            'CallBack',strcat(thisfile,'(''boolean_task'')'), ...
            'Value',0,...
            'String','Boolean formula');
        %radiobotton  BOOLEAN
        uicontrol( ...
            'Style','radiobutton', ...
            'Units','normalized', ...
            'BackgroundColor',[0.7 0.7 0.7], ...
            'ListboxTop',0, ...
            'Position',[0.033    0.06    0.099    0.0416], ...
            'Tag', 'boolean', ...
            'CallBack',strcat(thisfile,'(''boolean_task'')'), ...
            'Value',0,...
            'String','Boolean formula');
        
        %edittext ltl formula
        uicontrol( ...
            'Style','edit', ...
            'Units','normalized', ...
            'BackgroundColor',[1 1 1], ...
            'ListboxTop',0, ...
            'Position',[0.05    0.093    0.11    0.029], ...
            'CallBack',strcat(thisfile,'(''ltl_formula_changed'')'), ...
            'Tag', 'ltlformula', ...
            'String','(F y1) & G !(y2 | y3)',...
            'Visible','on');
        %edittext  BOOLEAN
        uicontrol( ...
            'Style','edit', ...
            'Units','normalized', ...
            'BackgroundColor',[1 1 1], ...
            'ListboxTop',0, ...
            'Position',[0.05    0.04    0.11    0.029], ...
            'CallBack',strcat(thisfile,'(''boolean_formula_changed'')'), ...
            'Tag', 'booleanformula', ...
            'String','!Y1 & y2',...
            'Visible','on');
        
        
        %environment button
        uicontrol( ...
            'Style','push', ...
            'Units','normalized', ...
            'BackgroundColor',[0.80 0.80 0.80], ...
            'ListboxTop',0, ...
            'Position',[0.1726    0.16    0.1349    0.05], ...
            'CallBack',strcat(thisfile,'(''run_environment'')'), ...
            'String','Environment');
        
        %path planning button
        uicontrol( ...
            'Style','push', ...
            'Units','normalized', ...
            'BackgroundColor',[0.80 0.80 0.80], ...
            'ListboxTop',0, ...
            'Position',[0.1726    0.1   0.1349    0.05], ...
            'Tag','path_planning_button',...
            'CallBack',strcat(thisfile,'(''run_path_planning'')'), ...
            'String','Path Planning');
        
        %control button
        uicontrol( ...
            'Style','push', ...
            'Units','normalized', ...
            'BackgroundColor',[0.80 0.80 0.80], ...
            'ListboxTop',0, ...
            'Position',[0.1726    0.04    0.1349    0.05], ...
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
        
        %intermediate trajectory points
        uicontrol( ...
            'Style','text', ...
            'Units','normalized', ...
            'BackgroundColor',[0.70 0.70 0.70], ...
            'ListboxTop',0, ...
            'HorizontalAlignment','left',...
            'Position',[0.0303    0.635    0.2628    0.05], ...
            'String','Intermediate trajectory points:');
        uicontrol( ...
            'Style','popupmenu', ...
            'Units','normalized', ...
            'BackgroundColor',[1 1 1], ...
            'Value', 1,...
            'ListboxTop',0, ...
            'Position',[0.0303    0.56    0.2628    0.0952], ...
            'Tag', 'waypoints', ...
            'String','Middle points|Norm 1|Norm 2|Norm Inf.|MPC'); % |variable step saving data');
        
        %Max linear velocity
        uicontrol( ...
            'Style','text', ...
            'Units','normalized', ...
            'BackgroundColor',[0.80 0.80 0.80], ...
            'ListboxTop',0, ...
            'HorizontalAlignment','left',...
            'Position',[0.0303    0.56    0.1349    0.0235], ...
            'String','Max linear velocity (m/s)');
        uicontrol( ...
            'Style','edit', ...
            'Units','normalized', ...
            'BackgroundColor',[1 1 1], ...
            'ListboxTop',0, ...
            'Position',[0.0303    0.5166    0.1349    0.0434], ...
            'CallBack',strcat(thisfile,'(''max_lin_vel_changed'')'), ...
            'Tag', 'linvel', ...
            'String','1');
        
        %Max angular velocity
        uicontrol( ...
            'Style','text', ...
            'Units','normalized', ...
            'BackgroundColor',[0.80 0.80 0.80], ...
            'ListboxTop',0, ...
            'HorizontalAlignment','left',...
            'Position',[0.1726    0.56    0.1349    0.0235], ...
            'String','Max angular velocity (rad/s)');
        uicontrol( ...
            'Style','edit', ...
            'Units','normalized', ...
            'BackgroundColor',[1 1 1], ...
            'ListboxTop',0, ...
            'Position',[0.1726    0.5166    0.1349    0.0434], ...
            'CallBack',strcat(thisfile,'(''max_ang_vel_changed'')'), ...
            'Tag', 'angvel', ...
            'String','1');
        
        %Max steering angle
        uicontrol( ...
            'Style','text', ...
            'Units','normalized', ...
            'BackgroundColor',[0.80 0.80 0.80], ...
            'ListboxTop',0, ...
            'HorizontalAlignment','left',...
            'Position',[0.0303    0.48    0.1349    0.0235], ...
            'String','Max steering angle (deg)');
        uicontrol( ...
            'Style','edit', ...
            'Units','normalized', ...
            'BackgroundColor',[1 1 1], ...
            'ListboxTop',0, ...
            'Position',[0.0303    0.4366    0.1349    0.0434], ...
            'Tag', 'steering', ...
            'CallBack',strcat(thisfile,'(''max_steering_changed'')'), ...
            'String','30');
        
        %Wheel radius
        uicontrol( ...
            'Style','text', ...
            'Units','normalized', ...
            'BackgroundColor',[0.80 0.80 0.80], ...
            'ListboxTop',0, ...
            'HorizontalAlignment','left',...
            'Position',[0.1726    0.48    0.0889    0.0235], ...
            'String','Wheel radius (m)');
        uicontrol( ...
            'Style','edit', ...
            'Units','normalized', ...
            'BackgroundColor',[1 1 1], ...
            'ListboxTop',0, ...
            'Position',[0.1726    0.4366    0.1349    0.0434], ...
            'Tag', 'wheel', ...
            'CallBack',strcat(thisfile,'(''wheel_radius_changed'')'), ...
            'String','0.05');
        
        %Wheelbase
        uicontrol( ...
            'Style','text', ...
            'Units','normalized', ...
            'BackgroundColor',[0.80 0.80 0.80], ...
            'ListboxTop',0, ...
            'HorizontalAlignment','left',...
            'Position',[0.0303    0.4    0.0889    0.0235], ...
            'String','Wheel Base (m)');
        uicontrol( ...
            'Style','edit', ...
            'Units','normalized', ...
            'BackgroundColor',[1 1 1], ...
            'ListboxTop',0, ...
            'Position',[0.0303    0.3566    0.1349    0.0434], ...
            'Tag', 'wheelbase', ...
            'CallBack',strcat(thisfile,'(''wheel_base_changed'')'), ...
            'String','0.25');
        
        
        %Sampling period
        uicontrol( ...
            'Style','text', ...
            'Units','normalized', ...
            'BackgroundColor',[0.80 0.80 0.80], ...
            'ListboxTop',0, ...
            'HorizontalAlignment','left',...
            'Position',[0.1726    0.4    0.1349    0.0235], ...
            'String','Sampling period (seconds)');
        uicontrol( ...
            'Style','edit', ...
            'Units','normalized', ...
            'BackgroundColor',[1 1 1], ...
            'ListboxTop',0, ...
            'Position',[0.1726    0.3566    0.1349    0.0434], ...
            'Tag', 'sampling', ...
            'CallBack',strcat(thisfile,'(''sampling_changed'')'), ...
            'String','0.1');
        
        %lookahead distance
        uicontrol( ...
            'Style','text', ...
            'Units','normalized', ...
            'BackgroundColor',[0.80 0.80 0.80], ...
            'ListboxTop',0, ...
            'HorizontalAlignment','left',...
            'Position',[0.0303    0.32    0.1349    0.0235], ...
            'String','Lookahead Distance (int value)');
        uicontrol( ...
            'Style','edit', ...
            'Units','normalized', ...
            'BackgroundColor',[1 1 1], ...
            'ListboxTop',0, ...
            'Position',[0.0303    0.2766    0.1349    0.0434], ...
            'Tag', 'lookahead', ...
            'CallBack',strcat(thisfile,'(''lookahead_changed'')'), ...
            'String','10');
        
        %motion controller
        uicontrol( ...
            'Style','text', ...
            'Units','normalized', ...
            'BackgroundColor',[0.80 0.80 0.80], ...
            'ListboxTop',0, ...
            'HorizontalAlignment','left',...
            'Position',[0.1726    0.32    0.1349    0.0235], ...
            'String','Motion Controller:');
        uicontrol( ...
            'Style','popupmenu', ...
            'Units','normalized', ...
            'BackgroundColor',[1 1 1], ...
            'Value', 1,...
            'ListboxTop',0, ...
            'Position',[0.1726    0.27    0.1349    0.0434], ...
            'Tag', 'controller', ...
            'String','Pure-Pursuit|PI'); % |variable step saving data');
        
        
        %radiobutton car-like
        uicontrol( ...
            'Style','radiobutton', ...
            'Units','normalized', ...
            'BackgroundColor',[0.80 0.80 0.80], ...
            'ListboxTop',0, ...
            'Position',[0.0513    0.2333    0.0983    0.0416], ...
            'Tag', 'robotCar', ...
            'CallBack',strcat(thisfile,'(''robot_car'')'), ...
            'Value',1,...
            'String','Car-like');
        
        %radiobutton Differential-drive
        uicontrol( ...
            'Style','radiobutton', ...
            'Units','normalized', ...
            'BackgroundColor',[0.80 0.80 0.80], ...
            'ListboxTop',1, ...
            'Position',[0.18    0.2333    0.1203    0.0416], ...
            'Tag', 'robotDifferential', ...
            'CallBack',strcat(thisfile,'(''robot_differential'')'), ...
            'String','Differential-drive');
        
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
        %check if CPLEX is installed
        try
            [~,~,~] = cplexmilp(1,-1,-1);
        catch
            message = sprintf('Cannot find CPLEX. It is required for solving the optimization problems.');
            uiwait(msgbox(message,'Robot Motion Toolbox','modal'));
        end
        data = get(gcf,'UserData');
        data.cplex_variable='false';
        
        %check the ltl2ba if it is corrected installed
        fid = fopen('RMTconfig.txt','r');
        if (fid == -1)
            message = sprintf('The RMTconfig.txt file not found!\nSome of the functionalities will not work properly!\nPlease select the path to LTL to Buchi located in subdir .\\ltl2ba\n');
            uiwait(msgbox(message,'Robot Motion Toolbox','modal'));
            temp = computer;
            if (strcmpi(temp,'PCWIN') || strcmpi(temp,'PCWIN64'))
                instruct = 'ltl2ba.exe';
            elseif strcmpi(temp,'GLNXA64')
                instruct = 'ltl2bal';
            elseif strcmpi(temp,'MACI64')
                instruct = 'ltl2ba';
            end
            [filename, pathname] = uigetfile({instruct,'*.*'},  sprintf('Locate file %s in subdir .\\ltl2ba',instruct));
            [filename2, pathname2] = uiputfile('RMTconfig.txt', 'Save configuration file as');
            fileID = fopen(fullfile(pathname2, filename2),'w');
            fid = fopen(fullfile(pathname2, filename2),'r');
            fprintf(fileID,'%s',sprintf('ltl2buchi=%s\n',fullfile(pathname, filename)));
            fclose(fileID);
        end
        fis = textscan(fid,'%s','delimiter','=');
        runLTL2BA = '';
        for i = 1 : size(fis{1})
            if strcmpi(deblank(fis{1}{i}),'ltl2buchi')
                runLTL2BA = fis{1}{i+1};
                break;
            end
        end
        temp = computer;
        if (strcmpi(temp,'PCWIN') || strcmpi(temp,'PCWIN64'))
            [s,~]=dos([runLTL2BA ' -d -f " <> p1"']); %sintax for calling ltl2ba.exe (located in subdir .\ltl2ba); use full description result (-d)
        elseif strcmpi(temp,'GLNXA64')
            [s,~]=unix([runLTL2BA ' -d -f " <> p1"']);
        elseif strcmpi(temp,'MACI64')
            [s,~]=unix([runLTL2BA ' -d -f " <> p1"']);
        end
        if (s~=0) %error
            message = sprintf('LTL to Buchi not corrected installed.\nRemove RMTconfig.txt file and run again the toolbox\nIt will be automatically created!');
            uiwait(msgbox(message,'Robot Motion Toolbox','modal'));
            return
        end
        data.ltl2ba = runLTL2BA;
        set(figpri, 'Visible','on');
        set(gcf,'UserData',data);
        
    case 'robot_car'
        set(findobj(gcf,'Tag','robotCar'),'Value',1);
        set(findobj(gcf,'Tag','robotDifferential'),'Value',0);
        
    case 'robot_differential'
        set(findobj(gcf,'Tag','robotCar'),'Value',0);
        set(findobj(gcf,'Tag','robotDifferential'),'Value',1);
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
        button = questdlg('All figures will be deleted. Are you sure?','Robot Motion Toolbox');
        if strcmpi(button,'Yes')
            set(findobj(gcf,'Tag','reach'),'Value',1);
            set(findobj(gcf,'Tag','ltl'),'Value',0);
            set(findobj(gcf,'Tag','boolean'),'Value',0);
            set(findobj(gcf,'Tag','pathpoints'),'Enable','on');
            set(findobj(gcf,'Tag','triang'),'Enable','on');
            set(findobj(gcf,'Tag','rect'),'Enable','on');
            set(findobj(gcf,'Tag','poly'),'Enable','on');
            set(findobj(gcf,'Tag','trapez'),'Enable','on');
            set(findobj(gcf,'Tag','waypoints'),'Enable','on');
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
            %disable the text information box
            set(data.handle_text,'Visible','off','String','');
            for i = 1 : 6
                set(data.handles_control(i),'Visible','on');
            end
            data = get(gcf,'UserData');
            if (isfield(data,'T'))
                data = rmfield(data,'T');
                set(gcf,'UserData',data);
            end
        else
            set(findobj(gcf,'Tag','reach'),'Value',0);
        end
    case 'ltl_task'
        button = questdlg('All figures will be deleted. Are you sure?','Robot Motion Toolbox');
        if strcmpi(button,'Yes')
            set(findobj(gcf,'Tag','reach'),'Value',0);
            set(findobj(gcf,'Tag','ltl'),'Value',1);
            set(findobj(gcf,'Tag','boolean'),'Value',0);
            set(findobj(gcf,'Tag','pathpoints'),'Value',1,'Enable','off');
            set(findobj(gcf,'Tag','triang'),'Enable','off','Value',1);
            set(findobj(gcf,'Tag','rect'),'Enable','off','Value',0);
            set(findobj(gcf,'Tag','poly'),'Enable','off','Value',0);
            set(findobj(gcf,'Tag','trapez'),'Enable','off','Value',0);
            set(findobj(gcf,'Tag','waypoints'),'Enable','off','Value',1);
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
            %enable the text information box
            set(data.handle_text,'Visible','on','String','');
            for i = 1 : 6
                set(data.handles_control(i),'Visible','off');
            end
        else
            set(findobj(gcf,'Tag','ltl'),'Value',0);
        end
    case 'boolean_task'
        button = questdlg('All figures will be deleted. Are you sure?','Robot Motion Toolbox');
        if strcmpi(button,'Yes')
            set(findobj(gcf,'Tag','reach'),'Value',0);
            set(findobj(gcf,'Tag','ltl'),'Value',0);
            set(findobj(gcf,'Tag','boolean'),'Value',1);
            set(findobj(gcf,'Tag','pathpoints'),'Value',1,'Enable','off');
            set(findobj(gcf,'Tag','triang'),'Enable','off','Value',1);
            set(findobj(gcf,'Tag','rect'),'Enable','off','Value',0);
            set(findobj(gcf,'Tag','poly'),'Enable','off','Value',0);
            set(findobj(gcf,'Tag','trapez'),'Enable','off','Value',0);
            set(findobj(gcf,'Tag','waypoints'),'Enable','off','Value',1);
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
            %enable the text information box
            set(data.handle_text,'Visible','on','String','');
            for i = 1 : 6
                set(data.handles_control(i),'Visible','off');
            end
        else
            set(findobj(gcf,'Tag','boolean'),'Value',0);
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %
        %               RUN ENVIRONMENT
        %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
    case 'run_environment'
        
        mission_task = get(findobj(gcf,'Tag','reach'),'Value'); %mission_task=1 - reachability tasks; 0 - ltl tasks
        
        % This is the code panel  containing:
        % NUMBER OF REGION OF INTEREST and NUMBER OF ROBOTS
        
        if (mission_task == 0)
            prompt = {'Number of regions of interest:','Number of robots'};
            dlg_title = 'Robot Motion Toolbox';
            num_lines = 1;
            defaultans = {'3','2'};
            input_user = inputdlg(prompt,dlg_title,num_lines,defaultans);
            reg_no = char(input_user(1));   % Reading of region's numbers from input interface
            robot_no = char(input_user(2)); % Reading of robot's numbers from input interface
            if (isempty(reg_no) || isempty(robot_no))
                return;
            end
            try
                reg_no = str2double(reg_no);
                robot_no = str2double(robot_no);
            catch
                uiwait(errordlg(sprintf('\nNumber of regions and of robots should be a natural number!'),'Robot Motion Toolbox','modal'));
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
            
            % Menu to create the Environment
            % 1 Mode Manual
            % 2 Mode Random
            choiceMenuEnv = questdlg('How do you want to generate the environment?', ...
                'Robot Motion Toolbox', ...
                '1. Manual','2. Random','Yes');
            
            % Mode Manual
            if(strcmpi(choiceMenuEnv,'1. Manual'))
                %[objects,initial_points,final_points] = rmt_define_regions(data.handle_env,reg_no,robot_no,0,limits(1),limits(2),limits(3),limits(4));
                [objects,initial_points,final_points] = rmt_define_regions([limits(1),limits(2),limits(3),limits(4)],reg_no, data.handle_env,robot_no,0);
                
                
            else
                % Creation Enviroment with Random Function
                bound=[limits(1),limits(2),limits(3),limits(4)];
                color=data.reg_plot.color_full;
                
                %Set dimension of region of interest
                prompt = {'Set dimension of region of interest'};
                dlg_title = 'Robot Motion Toolbox';
                num_lines = 1;
                default_size = {'1'};
                size_obs = inputdlg(prompt,dlg_title,num_lines,default_size); % Reading a dimension of interest's region from input interface
                sizeObs= str2double(size_obs{1,1}); %Converiosn char in number
                
                if isempty(size_obs)
                    sizeObs = default_size;
                end
                [objects,initial_points,final_points]=rmt_random_env(data.handle_env,reg_no,sizeObs,bound,0,robot_no,limits,color);
            end
            
            [T,propositions] = rmt_tr_sys_partition(objects,limits(2),limits(4));  %create partition and represent it
            rmt_plot_environment(objects,data.frame_limits,T.Vert);
            rmt_represent_atomic_props(T.Vert,propositions);    %represent all atomic props
            rob_plot = data.rob_plot;
            
            %%Execution monitoring strategy starts from here:
            %Creation of cell's matrix with (n,m) position
            %The firtst row have the number of robot that the user enters
            current_pos=cell(1,robot_no);
            for r=1:robot_no
                current_pos{r}=initial_points{r};%R_trajs{r}(:,1); %current_pos - initial (current)
                %position of robots (cell, current_pos{i} is a column vector with 2 rows)
            end
            for r=1:robot_no
                plot(current_pos{r}(1),current_pos{r}(2),'Color',rob_plot.line_color{r},'LineStyle',rob_plot.line{r},'LineWidth',rob_plot.line_width{r},'Marker',rob_plot.marker{r},'MarkerFaceColor',rob_plot.face_color{r});
            end
            uiwait(msgbox('Partition was constructed based on defined regions of interest.','Robot Motion Toolbox','modal'));
            %Each triangular region has three vertices, shown in structure C according to this order:
            %1. Central;
            %2. Right;
            %3. Left;
            %"C" is an array of 1 x number of regions. Each column has two elements (X and Y coordinates for vertices)
            C = T.Vert;
            RO=[];
            for i = 1 : robot_no
                temp = initial_points{i};
                for j=1:length(C)   %indices of starting & final cells
                    if inpolygon(temp(1),temp(2),C{j}(1,:),C{j}(2,:))
                        RO(i)=j;
                        break;
                    end
                end
            end
            %T.RO contains the information about the position of initial marking (ex. the initial marking is present
            %in region P5 e P11),
            %instead T.M0 contains the  initial marking, in which region
            %the inserted robot is present.
            T.m0=zeros(length(T.Q),1);    %initial marking (for PN)
            T.RO = RO;
            for i=T.Q
                T.m0(i)=sum(ismember(RO,i));  %number of robots initially in state i
            end
            Tr = rmt_quotient_T(T); % quotient of partition T, with fewer states
            %(based on collapsing states with same observables in same connected component
            % with same obs)
            orient = [];
            for i = 1 : length(initial_points)
                orient(i) = 0;
            end
            data.Nobstacles = reg_no;
            data.obstacles = objects;
            data.T=T;
            data.Tr=Tr;
            data.propositions=propositions;
            data.RO = RO;
            data.initial=initial_points;
            data.final=final_points;
            data.orientation=orient;
            set(gcf,'UserData',data);
            set(findobj(gcf,'Tag','path_planning_button'),'Enable','on');
            return;
        end
        
        planning_approach = get(findobj(gcf,'Tag','pathpoints'),'Value');
        %planning_approach= 1 - cell descomposition; 2 - visibility graph; 3 - Voronoi; 4 - manual points
        data = get(gcf,'UserData');
        
        switch planning_approach
            case 1
                %cell decomposition
                %obs_no = char(inputdlg('Number of obstacles:','Robot Motion Toolbox',1,{'1'}));
                prompt = {'Number of obstacles:'};
                dlg_title = 'Obstacles';
                num_lines = 1;
                defaultans = {'3'};
                input_user = inputdlg(prompt,dlg_title,num_lines,defaultans);
                if isempty(input_user)
                    return;
                end
                obs_no = char(input_user(1));
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
                
                [objects,initial_point,final_point] = rmt_define_regions([limits(1),limits(2),limits(3),limits(4)],obs_no, data.handle_env,1,1);

                
                set(data.handle_ori,'Visible','on','xlim',[0 20],'ylim',[-180 180],'XGrid','on','YGrid','on');
                set(data.handle_vel,'Visible','on','xlim',[0 20],'ylim',[0 10],'XGrid','on','YGrid','on');
                set(data.handle_ang,'Visible','on','xlim',[0 20],'ylim',[-50 50],'XGrid','on','YGrid','on');
                
                data.Nobstacles = obs_no;
                data.obstacles = objects;
                data.initial=initial_point;
                data.final=final_point;
                data.orientation = data.orientation(1);
                set(gcf,'UserData',data);
                set(findobj(gcf,'Tag','path_planning_button'),'Enable','on');
                
            case 2 %visibility graph
                disp('Visibility graph...');
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
                %if ((obs_no <= 0) || (obs_no >= 5) || (obs_no ~= round(obs_no)))
                %    uiwait(errordlg(sprintf('\nNumber of obstacles should be a natural number between 1 and 4!'),'Robot Motion Toolbox','modal'));
                %    rmt('run_environment');
                %    return;
                %end
                
                
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
                
                [~, obstacles,initial_point,final_point] = rmt_bu2ild_grid_map2(data.handle_env,limits,obs_no);
                set(data.handle_ori,'Visible','on','xlim',[0 20],'ylim',[-180 180],'XGrid','on','YGrid','on');
                set(data.handle_vel,'Visible','on','xlim',[0 20],'ylim',[0 10],'XGrid','on','YGrid','on');
                set(data.handle_ang,'Visible','on','xlim',[0 20],'ylim',[-50 50],'XGrid','on','YGrid','on');
                
                %data.map = map;
                data.Nobstacles = obs_no;
                data.obstacles = obstacles;
                data.initial=initial_point;
                data.final=final_point;
                data.orientation = data.orientation(1);
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
                %if ((obs_no <= 0) || (obs_no >= 5) || (obs_no ~= round(obs_no)))
                %    uiwait(errordlg(sprintf('\nNumber of obstacles should be a natural number between 1 and 4!'),'Robot Motion Toolbox','modal'));
                %    rmt('run_environment');
                %    return;
                %end
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
            case 4 %manual points
                cla(data.handle_env);
                axes(data.handle_env);
                cla(data.handle_ori);
                axes(data.handle_ori);
                cla(data.handle_vel);
                axes(data.handle_vel);
                cla(data.handle_ang);
                axes(data.handle_ang);
                traj_ini = [0 0];
                data = get(gcf,'UserData');
                sampling_period = eval(get(findobj(gcf,'Tag','sampling'),'String'));
                %axes(data.handle_env);
                traj = rmt_get_waypoints(data.handle_env,data.frame_limits,sampling_period,traj_ini);
                %cla(data.handle_env);
                data.initial = [traj(1,1) traj(2,1)];
                data.final = [traj(1,end) traj(2,end)];
                data.trajectory = traj;
                plot(data.initial(1),data.initial(2),'pw','Markersize',13, 'Color', 'k');
                plot(data.trajectory(1,:),data.trajectory(2,:),'r','LineWidth',3);
                plot(data.final(1),data.final(2),'pw','Markersize',13, 'Color', 'b');
                grid on;
                set(findobj(gcf,'Tag','path_planning_button'),'Enable','off');
        end%switch
        set(gcf,'UserData',data);%to save data
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
        else %(get(findobj(gcf,'Tag','boolean'),'Value') == 1)
            mission_task = 3;
        end
        
        data = get(gcf,'UserData');
        
        switch mission_task
            case 2  
                % Insert button of analysis choice between Buchi and Petri
                % Net with LTL Formula
                choiceMenuLTL = questdlg('Which model do you want to use?', ...
                    'Robot Motion Toolbox', ...
                    '1. Transistion System Model','2. Petri Net Model','Yes');
                
                %% Part about analysis with Buchi Automaton
                if strcmpi(choiceMenuLTL,'1. Transistion System Model')
                    data.formula = get(findobj(gcf,'Tag','ltlformula'),'String');
                    if isfield(data,'Tg')
                        choice1 = questdlg('Should the product automaton of the robot team be computed again?', ...
                            'Robot Motion Toolbox', ...
                            'Yes','No','Yes');
                        if strcmpi(choice1,'Yes')
                            choice1=1;
                        else
                            choice1=0;
                        end
                    else
                        choice1 = 1;
                    end
                    probability=ones(1,length(data.propositions));
                    if ((choice1 == 1) || ~isfield(data,'Tg'))
                        tic;
                        Tg = rmt_tr_sys_obs_team(data.T,data.RO,data.propositions,probability);    %compute team (global) transition system, including probabilities of observing propositions
                        message = sprintf('Transition system of a robot has %d states\nTeam (global) transition system has %d states\nTime spent for creating it: %g secs', length(data.T.Q),length(Tg.Q), toc);
                        uiwait(msgbox(message,'Robot Motion Toolbox','modal'));
                    else
                        Tg = data.Tg;
                        message = sprintf('Transition system of a robot has %d states\nTeam (global) transition system has %d states', length(data.T.Q),length(Tg.Q));
                        uiwait(msgbox(message,'Robot Motion Toolbox','modal'));
                    end
                    data.Tg=Tg;
                    set(gcf,'UserData',data);
                    
                    if isfield(data,'B')
                        choice2 = questdlg('Should the Buchi automaton be computed again?', ...
                            'Robot Motion Toolbox', ...
                            'Yes','No','Yes');
                        if strcmpi(choice2,'Yes')
                            choice2=1;
                        else
                            choice2=0;
                        end
                    else
                        choice2 = 1;
                    end
                    
                    % Control of regions of interest in the LTL Formula
                    if ((choice2 == 1) || ~isfield(data,'B'))
                        regionFormula=strfind(data.formula, 'u');
                        if(data.Nobstacles < size(regionFormula,2))
                            uiwait(msgbox('LTL Formula is not correct. The number of proposition and region of interest is not equal. Please re-insert!','Robot Motion Toolbox','modal'));
                            prompt = {'New LTL Formula:'};
                            dlg_title = 'Robot Motion Toolbox';
                            num_lines = 1;
                            defaultans = {''};
                            %defaultans = {'(F u1) & G !(u2 | u3)'};
                            input_user = inputdlg(prompt,dlg_title,num_lines,defaultans);
                            data.formula= char(input_user(1));   % Reading of region's numbers from input interface
                            B = rmt_create_buchi(data.formula, Tg.Obs);
                            data.B=B;
                        else
                            B = rmt_create_buchi(data.formula, Tg.Obs);
                            data.B=B;
                        end
                        
                    else
                        B = data.B;
                    end
                    set(gcf,'UserData',data);
                    if ((choice1==1) || (choice2==1))
                        tic;
                        Pg = rmt_product_autom_prob_team(Tg,B);  %Pg has trans with log of probabilities plus epsilon (used for Dijkstra), and 2 additional costs: probability of transition and number of moving robots
                        message2 = sprintf('\nBuchi automaton has %d states;\nProduct automaton has %d states;\nTime spent for creating it: %g secs', size(B.S,2), size(Pg.S,1), toc);
                        message = sprintf('%s%s', message, message2);
                        uiwait(msgbox(message2,'Robot Motion Toolbox','modal'));
                    else
                        Pg = data.Pg;
                        message2 = sprintf('\nProduct automaton has %d states', size(Pg.S,1));
                        message = sprintf('%s%s', message, message2);
                        uiwait(msgbox(message2,'Robot Motion Toolbox','modal'));
                    end
                    data.Pg = Pg;
                    set(gcf,'UserData',data);
                    
                    data.hwait = waitbar(0,'Computing trajectories. Please wait...','Name','Robot Motion Toolbox',...
                        'WindowStyle','modal','CloseRequestFcn',strcat(thisfile,'(''close_hwait'')'));
                    set(gcf,'UserData',data);%to save data
                    
                    tic;
                    [run_Tg,~,~,path_Tg,~,~] = rmt_find_accepted_run_multicost(Pg,'prob','move');  %solution in Pg and projection to Tg and B
                    delete(data.hwait);
                    message2 = sprintf('\nTime for finding accepted run: %g secs', toc);
                    message = sprintf('%s%s', message, message2);
                    uiwait(msgbox(message2,'Robot Motion Toolbox','modal'));
                    [~,R_paths,R_trajs,~] = rmt_robot_trajectory_team(data.T,Tg,run_Tg,path_Tg);  %each robot starts from centroid of its initial cell; R_trajs is a cell array,
                    %we clean the workspace figure
                    data.R_trajs = R_trajs;
                    set(gcf,'UserData',data);
                    cla(data.handle_env);
                    rob_plot = data.rob_plot;
                    %%Execution monitoring strategy starts from here:
                    N_r = length(data.RO);
                    current_pos=cell(1,N_r);
                    for r=1:N_r
                        current_pos{r}=data.initial{r};%R_trajs{r}(:,1); %current_pos - initial (current) position of robots (cell, current_pos{i} is a column vector with 2 rows)
                    end
                    rmt_represent_atomic_props(data.T.Vert,data.propositions);    %represent all atomic props and store handles
                    for r=1:N_r
                        %plot(current_pos{r}(1),current_pos{r}(2),rob_marker{r});  %position where robot waits
                        plot(current_pos{r}(1),current_pos{r}(2),'Color',rob_plot.line_color{r},'LineStyle',rob_plot.line{r},'LineWidth',rob_plot.line_width{r},'Marker',rob_plot.marker{r},'MarkerFaceColor',rob_plot.face_color{r});
                    end
                    i = 1;
                    while i<=size(R_paths,2)  %go through synchronized paths and simulate robot moves and appearance/disappearance of propositions
                        %                        fprintf('\n\n Current team state (simplices) = [%s]'' ; press any key to test cases.',num2str(R_paths(:,i)'));
                        %c_obs= rmt_current_observation_team(Tg.Obs,data.propositions,R_paths(:,i));   %current observation of Tg (row index in Tg.Obs) - current simplices R_paths(:,i), and appeared/dissapeared regions are given by app_props
                        for r=1:N_r
                            %plot([current_pos{r}(1),R_trajs{r}(1,i+1)],[current_pos{r}(2),R_trajs{r}(2,i+1)],rob_marker{r},'LineWidth',2)    %plot trajectory until next cell
                            plot([current_pos{r}(1),R_trajs{r}(1,i+1)],[current_pos{r}(2),R_trajs{r}(2,i+1)],'Color',rob_plot.line_color{r},'LineStyle',rob_plot.line{r},'LineWidth',rob_plot.line_width{r},'Marker',rob_plot.marker{r},'MarkerFaceColor',rob_plot.face_color{r});
                            current_pos{r} = R_trajs{r}(:,i+1);   %update current robot position (on boundary of next cell)
                        end
                        i=i+1;
                    end
                    
                    button = questdlg('Save the details to a text file?','Robot Motion Toolbox');
                    if strcmpi(button,'Yes')
                        [filename, pathname] = uiputfile('*.txt', 'Save experiments as');
                        fileID = fopen(fullfile(pathname, filename),'w');
                        fprintf(fileID,'%s',message);
                        fclose(fileID);
                    end
                end
                %% Part of creation Petri Net with LTL Formula
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                if strcmpi(choiceMenuLTL,'2. Petri Net Model')
                    disp('START CREATION PETRI NET WITH LTL FORMULA');
                    data = get(gcf,'UserData');
                    data.formula = get(findobj(gcf,'Tag','ltlformula'),'String'); %% read LTL formula
                    tic;
                    if ~isfield(data,'Tr')
                        data.Tr = rmt_quotient_T(data.T); %quotient of partition T, with fewer states (based on collapsing states with same observables in same connected component with same obs)
                    end
                    
                    [Pre,Post] = rmt_construct_PN(data.Tr.adj);
                    m0=data.Tr.m0;
                    message = sprintf('Petri net system has %d places and %d transitions\nTime spent for creating it: %g secs', size(Pre,1),size(Pre,2),toc);
                    uiwait(msgbox(message,'Robot Motion Toolbox','modal'));
                    nplaces_orig = size(Pre,1);
                    ntrans_orig = size(Pre,2);
                    
                    %crete the observation set
                    N_r = length(data.RO); %In RO there is a region that contains a token (robot)
                    observ_set = data.Tr.OBS_set(1:size(data.Tr.OBS_set,1)-1,:); %Remove free space from initial Obs set
                    temp_cell=mat2cell( observ_set , ones(1,size(observ_set,1)) , size(observ_set,2) );  %duplicate observables of transition systems
                    temp_obs=rmt_cartesian_product(temp_cell{:});  %possible observables, on rows (more robots can have the same observables, that's why we use carth product); observables will be labeled with indices of rows (in T.obs)
                    temp_obs=unique(sort(temp_obs,2),'rows');   %sort rows and remove identical ones (they would have the same observable)
                    for i=1:size(temp_obs,1)  %modify observables (keep at most one occurence of same prop on each row, and pad with zeros until length
                        obs=unique(temp_obs(i,:));    %keep unique elements on each row, and pad wih zeros
                        if length(obs)<size(temp_obs,2) %pad with zeros until number of propositions
                            obs((end+1):size(temp_obs,2))=0;
                        end
                        temp_obs(i,:)=obs;
                    end
                    temp_obs=unique(sort(temp_obs,2),'rows');   %again remove identical rows (there may appear new repetitions, due to keeping only unique elements on each row and padding with zeros)
                    temp_obs(end+1,:)=[length(data.Tr.props)+1 , zeros(1,size(temp_obs,2)-1)]; %dummy has index "ind_dummy", and pad with zeros after it
                    
                    % Creating the automaton Buchi to be included in the global Petri Net
                    if ~isfield(data,'B')
                        % Control on the number of region of interest
                        regionFormula=strfind(data.formula, 'u');
                        if(data.Nobstacles < size(regionFormula,2))
                            uiwait(msgbox('LTL Formula is not correct. The number of proposition and region of interest is not equal. Please re-insert!','Robot Motion Toolbox','modal'));
                            prompt = {'New LTL Formula:'};
                            dlg_title = 'Robot Motion Toolbox';
                            num_lines = 1;
                            defaultans = {''};
                            %defaultans = {'(F u1) & G !(u2 | u3)'};
                            input_user = inputdlg(prompt,dlg_title,num_lines,defaultans);
                            data.formula= char(input_user(1));   % Reading of region's numbers from input interface
                            
                            
                            B = rmt_create_buchi(data.formula, temp_obs);
                            data.B=B;
                        else
                            B = rmt_create_buchi(data.formula, temp_obs);
                            data.B=B;
                        end
                        
                    else
                        choice2 = questdlg('Should the Buchi automaton be computed again?', ...
                            'Robot Motion Toolbox', ...
                            'Yes','No','Yes');
                        if strcmpi(choice2,'Yes')
                            B = rmt_create_buchi(data.formula, temp_obs);
                        else
                            B = data.B;
                        end
                    end
                    data.B=B;
                    set(gcf,'UserData',data);
                    
                    tic;
                    [Pre,Post,m0,final_places] = rmt_construct_PN_ltl(Pre,Post,m0,data.Tr.props, data.B,temp_obs);%final places - places corresponding to the final states in the Buchi automaton
                    message2 = sprintf('Petri net system including Buchi and observations has %d places and %d transitions\nTime spent for creating it: %g secs', size(Pre,1),size(Pre,2),toc);
                    message = sprintf('%s\n%s', message, message2);
                    uiwait(msgbox(message2,'Robot Motion Toolbox','modal'));
                    
                    data.Pre_full = Pre;
                    data.Post_full = Post;
                    set(gcf,'UserData',data);%to save data
                    
                    % This function return
                    % Aeq= [I(n_places) - (Post-Pre)] that correspond at the equation mi+1 = mi + (Post-Pre)*sigma
                    % beq = m0
                    % A = (m0 - Pre*sigma)
                    % b = m0
                    [A,b,Aeq,beq,cost] = rmt_construct_constraints_ltl(Pre,Post,m0, nplaces_orig, ntrans_orig, length(data.Tr.props) , 2*data.intermediateMarkings, final_places);
                    
                    data = get(gcf,'UserData');
                    % Part about analysis with Buchi Automaton
                    if strcmpi(data.cplex_variable,'true')
                        %%%%%%%%%
                        ctype='';
                        for i = 1 : size(Aeq,1)
                            ctype = sprintf('%sS',ctype);
                        end
                        for i = 1 : size(A,1)
                            ctype = sprintf('%sU',ctype);
                        end
                        vartype = '';
                        for i = 1 : 2*data.intermediateMarkings
                            for j = 1 : size(Pre,1)
                                vartype = sprintf('%sC',vartype); %put the markings as real
                            end
                            for j = 1 : size(Pre,2)
                                vartype = sprintf('%sI',vartype); %put the sigma as integer
                            end
                        end
                        
                        tic;
                        [xmin,f,exitflag] = cplexmilp(cost,A,b,Aeq,beq,[],[],[],zeros(1,size(A,2)),[],vartype);
                        time = toc;
                        if isempty(f)%no solution
                            uiwait(errordlg('Error solving the ILP. The problem may have no feasible solution. Increase k!','Robot Motion Toolbox','modal'));
                            return;
                        end
                        message2 = sprintf('\nTime of solving the MILP: %g secs\n', time);
                        message = sprintf('%s%s', message, message2);
                        uiwait(msgbox(message2,'Robot Motion Toolbox','modal'));
                        
                        message = sprintf('%s\n=======================================================',message);
                        message = sprintf('%s\nInitial solution on the reduced transition system',message);
                        message = sprintf('%s\n=======================================================\n',message);
                        
                        % After the optimization problem was solved, an
                        % initial solution was obtained on the reduced system
                        m0_old = m0;
                        m0 = m0(1:nplaces_orig);
                        m0_Buchi = m0_old(nplaces_orig+length(data.Tr.props)+1:end);
                        m0_obs = m0_old(nplaces_orig+1:nplaces_orig+length(data.Tr.props));
                        xm = xmin;
                        xmin = [];
                        ymin = [];
                        fprintf(1,'\nInitial state of Buchi = %s',mat2str(find(m0_Buchi)));
                        fprintf(1,'\n\tActive observations = %s',mat2str(find(m0_obs)));
                        for i = 1 : 2*data.intermediateMarkings
                            xmin = [xmin ; xm((i-1)*(size(Pre,1)+size(Pre,2))+1:(i-1)*(size(Pre,1)+size(Pre,2))+nplaces_orig)];%marking of places modeling the team
                            %fprintf(1,'Marking of places modeling the team at step %d: %s',)
                            xmin = [xmin ; xm((i-1)*(size(Pre,1)+size(Pre,2))+size(Pre,1)+1:(i-1)*(size(Pre,1)+size(Pre,2))+size(Pre,1)+ntrans_orig)];%firing vector of places modeling the team
                            
                            ymin = [ymin ; xm((i-1)*(size(Pre,1)+size(Pre,2))+nplaces_orig+length(data.Tr.props)+1:(i-1)*(size(Pre,1)+size(Pre,2))+size(Pre,1))];%marking of places modeling the Buchi
                            ymin = [ymin ; xm((i-1)*(size(Pre,1)+size(Pre,2))+size(Pre,1)+ntrans_orig+1:(i-1)*(size(Pre,1)+size(Pre,2))+size(Pre,1)+size(Pre,2))];%firing vector of places modeling the buchi
                            
                            if (i/2 == round(i/2))
                                trans_buchi=find([xm((i-1)*(size(Pre,1)+size(Pre,2))+size(Pre,1)+ntrans_orig+1:i*(size(Pre,1)+size(Pre,2)))]);
                                input_place = find(Pre(:,trans_buchi+ntrans_orig)) ;
                                input_place = input_place(input_place>nplaces_orig+length(data.Tr.props))-nplaces_orig-length(data.Tr.props); %take only the place of the buchi
                                output_place = find(Post(:,trans_buchi+ntrans_orig));
                                output_place = output_place(output_place>nplaces_orig+length(data.Tr.props))- nplaces_orig-length(data.Tr.props);
                                fprintf(1,'\n Transition in Buchi from state %d to state %d with observation (%s)',input_place,output_place,mat2str(find([xm((i-1)*(size(Pre,1)+size(Pre,2))+nplaces_orig+1:(i-1)*(size(Pre,1)+size(Pre,2))++length(data.Tr.props)+nplaces_orig)])));
                                fprintf(1,'\nState of Buchi in step %d = %s',i/2,mat2str(find([xm((i-1)*(size(Pre,1)+size(Pre,2))+nplaces_orig+length(data.Tr.props)+1:(i-1)*(size(Pre,1)+size(Pre,2))+size(Pre,1))])))
                                %Decomment this part
                                fprintf(1,'\n\tActive observations at step %d = %s',i/2,mat2str(find([xm((i-1)*(size(Pre,1)+size(Pre,2))+nplaces_orig+1:(i-1)*(size(Pre,1)+size(Pre,2))++length(data.Tr.props)+nplaces_orig)])));
                            end
                            
                        end
                        fprintf(1,'\n');
                        message = sprintf('%s\nInitial marking [ %s ] = %s\n',message,mat2str(find(m0>eps*10^5)),mat2str(m0(m0>eps*10^5)));
                        original_places{1} = [];
                        temp = rmt_marking2places(m0);
                        for j = 1 : length(temp)
                            original_places{1} = union(original_places{1},data.Tr.Cells{temp(j)});
                        end
                        nplaces = nplaces_orig;
                        ntrans = ntrans_orig;
                        
                        for i = 1 : 2*data.intermediateMarkings
                            m = xmin((i-1)*(nplaces+ntrans)+1 : (i-1)*(nplaces+ntrans)+nplaces);
                            fire = xmin((i-1)*(nplaces+ntrans)+nplaces+1 : (i-1)*(nplaces+ntrans)+nplaces+ntrans);
                            if (max(fire) > 0)
                                message = sprintf('%s\n============STEP %d =============\n',message,i);
                                message=sprintf('%s\nMarking [ %s ] = %s\n',message,mat2str(find(m>eps*10^5)),mat2str(m(m>eps*10^5)));
                                message = sprintf('%s\nSigma [ %s ] = %s\n',message,mat2str(find(fire>eps*10^5)),mat2str(fire(fire>eps*10^5)));
                                %                 if (reduced == 0)
                                %                     Run_cells = [Run_cells, rmt_marking2places(m)];  %visited cells (rows)
                                %                 else
                                temp = rmt_marking2places(m);
                                original_places{i+1} = [];
                                for j = 1 : length(temp)
                                    original_places{i+1} = union(original_places{i+1},data.Tr.Cells{temp(j)});
                                end
                                %                end
                            else
                                original_places{i+1} = original_places{i};
                            end
                        end
                        
                        options = cplexoptimset('cplex');
                        options.display='off';
                        options.lpmethod=1;
                        message = sprintf('%s\n=======================================================',message);
                        message = sprintf('%s\nProject the solution to the initial transition system with CPLEX',message);
                        message = sprintf('%s\n=======================================================\n',message);
                        [Pre,Post] = rmt_construct_PN(data.T.adj);
                        m0 = data.T.m0;
                        steps = 1;
                        message = sprintf('%s\nInitial marking [ %s ] = %s\n',message,mat2str(find(m0>eps*10^5)),mat2str(m0(m0>eps*10^5)));
                        Run_cells = data.RO';%rmt_marking2places(m0);
                        tic;
                        for i = 1 : 2*data.intermediateMarkings
                            m = xmin((i-1)*(nplaces+ntrans)+1 : (i-1)*(nplaces+ntrans)+nplaces);
                            fire = xmin((i-1)*(nplaces+ntrans)+nplaces+1 : (i-1)*(nplaces+ntrans)+nplaces+ntrans);
                            if (max(fire) > 0)
                                % places = find(m>eps*10^5);%final marked places
                                places = rmt_marking2places(m);%final marked places
                                markings = m(m>eps*10^5); %number of tokens in final marked places
                                temp = setdiff(1:size(Post,1),union(original_places{i},original_places{i+1}));
                                TT = data.T;
                                for j = 1 : length(temp)
                                    TT.adj(temp(j),:)=0;
                                    TT.adj(:,temp(j))=0;
                                end
                                [Pre_new,Post_new] = rmt_construct_PN(TT.adj);
                                Aeq = [eye(size(Pre_new,1)) -Post_new+Pre_new]; %state equation: m = m0 + C \sigma
                                beq = m0;
                                %add constraints on the intermediate final marking
                                % for j = 1 : length(places)
                                for j = 1 : length(markings)
                                    m_final = zeros(1,size(Pre_new,1));
                                    m_final(data.Tr.Cells{places(j)}) = 1;
                                    Aeq = [Aeq; m_final zeros(1,size(Pre_new,2))];
                                    beq = [beq; markings(j)];
                                end
                                [X,~,exitflag] = cplexlp(ones(1,size(Pre_new,2)+size(Pre_new,1)),[],[],...
                                    Aeq,beq,zeros(1,size(Pre_new,1)+size(Pre_new,2)),[],[],options);
                                if ~(exitflag > 0)
                                    uiwait(errordlg('Error solving LPP to project the solution!','Robot Motion Toolbox','modal'));
                                    return;
                                end
                                
                                [ret,Run_cells] = rmt_find_sequence(m0, round(X(size(Pre_new,1)+1:end)), Pre_new, Post_new,steps,Run_cells);
                                steps = steps + sum(round(X(size(Pre,1)+1:end)))+1;
                                fire = round(X(size(Pre_new,1)+1:end));
                                disp(ret);
                                m0 = round(X(1:size(Pre_new,1))); % final marking is the initial marking for the next step
                            end
                        end
                        
                        % Start part with GLPK
                    else
                        ctype1='';
                        ctype2='';
                        for i = 1 : size(Aeq,1)
                            ctype2 = sprintf('%sS',ctype2);
                        end
                        for i = 1 : size(A,1)
                            ctype1 = sprintf('%sU',ctype1);
                        end
                        vartype = '';
                        for i = 1 : 2*data.intermediateMarkings
                            for j = 1 : size(Pre,1)
                                vartype = sprintf('%sC',vartype); %put the markings as real
                            end
                            for j = 1 : size(Pre,2)
                                vartype = sprintf('%sI',vartype); %put the sigma as integer
                            end
                        end
                        tic;
                        
                        Atot = [Aeq; A];
                        btot= [beq; b];
                        ctype = [ctype2 ctype1];
                        [xmin,f,exitflag] = glpk(cost,Atot,btot,zeros(1,size(A,2)),[],ctype,vartype);
                        
                        time = toc;
                        if f==0%no solution
                            uiwait(errordlg('Error solving the ILP. The problem may have no feasible solution. Increase k!','Robot Motion Toolbox','modal'));
                            return;
                        end
                        message2 = sprintf('\nTime of solving the MILP: %g secs\n', time);
                        message = sprintf('%s%s', message, message2);
                        uiwait(msgbox(message2,'Robot Motion Toolbox','modal'));
                        
                        message = sprintf('%s\n=======================================================',message);
                        message = sprintf('%s\nInitial solution on the reduced transition system',message);
                        message = sprintf('%s\n=======================================================\n',message);
                        
                        % After the optimization problem was solved, an
                        % initial solution was obtained on the reduced system
                        m0_old = m0;
                        m0 = m0(1:nplaces_orig);
                        m0_Buchi = m0_old(nplaces_orig+length(data.Tr.props)+1:end);
                        m0_obs = m0_old(nplaces_orig+1:nplaces_orig+length(data.Tr.props));
                        xm = xmin;
                        xmin = [];
                        ymin = [];
                        fprintf(1,'\nInitial state of Buchi = %s',mat2str(find(m0_Buchi)));
                        fprintf(1,'\n\tActive observations = %s',mat2str(find(m0_obs)));
                        for i = 1 : 2*data.intermediateMarkings
                            xmin = [xmin ; xm((i-1)*(size(Pre,1)+size(Pre,2))+1:(i-1)*(size(Pre,1)+size(Pre,2))+nplaces_orig)];%marking of places modeling the team
                            %fprintf(1,'Marking of places modeling the team at step %d: %s',)
                            xmin = [xmin ; xm((i-1)*(size(Pre,1)+size(Pre,2))+size(Pre,1)+1:(i-1)*(size(Pre,1)+size(Pre,2))+size(Pre,1)+ntrans_orig)];%firing vector of places modeling the team
                            
                            ymin = [ymin ; xm((i-1)*(size(Pre,1)+size(Pre,2))+nplaces_orig+length(data.Tr.props)+1:(i-1)*(size(Pre,1)+size(Pre,2))+size(Pre,1))];%marking of places modeling the Buchi
                            ymin = [ymin ; xm((i-1)*(size(Pre,1)+size(Pre,2))+size(Pre,1)+ntrans_orig+1:(i-1)*(size(Pre,1)+size(Pre,2))+size(Pre,1)+size(Pre,2))];%firing vector of places modeling the buchi
                            
                            if (i/2 == round(i/2))
                                trans_buchi=find([xm((i-1)*(size(Pre,1)+size(Pre,2))+size(Pre,1)+ntrans_orig+1:i*(size(Pre,1)+size(Pre,2)))]);
                                input_place = find(Pre(:,trans_buchi+ntrans_orig)) ;
                                input_place = input_place(input_place>nplaces_orig+length(data.Tr.props))-nplaces_orig-length(data.Tr.props); %take only the place of the buchi
                                output_place = find(Post(:,trans_buchi+ntrans_orig));
                                output_place = output_place(output_place>nplaces_orig+length(data.Tr.props))- nplaces_orig-length(data.Tr.props);
                                fprintf(1,'\n Transition in Buchi from state %d to state %d with observation (%s)',input_place,output_place,mat2str(find([xm((i-1)*(size(Pre,1)+size(Pre,2))+nplaces_orig+1:(i-1)*(size(Pre,1)+size(Pre,2))++length(data.Tr.props)+nplaces_orig)])));
                                fprintf(1,'\nState of Buchi in step %d = %s',i/2,mat2str(find([xm((i-1)*(size(Pre,1)+size(Pre,2))+nplaces_orig+length(data.Tr.props)+1:(i-1)*(size(Pre,1)+size(Pre,2))+size(Pre,1))])))
                                %Decomment this part
                                fprintf(1,'\n\tActive observations at step %d = %s',i/2,mat2str(find([xm((i-1)*(size(Pre,1)+size(Pre,2))+nplaces_orig+1:(i-1)*(size(Pre,1)+size(Pre,2))++length(data.Tr.props)+nplaces_orig)])));
                            end
                            
                        end
                        fprintf(1,'\n');
                        message = sprintf('%s\nInitial marking [ %s ] = %s\n',message,mat2str(find(m0>eps*10^5)),mat2str(m0(m0>eps*10^5)));
                        original_places{1} = [];
                        temp = rmt_marking2places(m0);
                        for j = 1 : length(temp)
                            original_places{1} = union(original_places{1},data.Tr.Cells{temp(j)});
                        end
                        nplaces = nplaces_orig;
                        ntrans = ntrans_orig;
                        
                        for i = 1 : 2*data.intermediateMarkings
                            m = xmin((i-1)*(nplaces+ntrans)+1 : (i-1)*(nplaces+ntrans)+nplaces);
                            fire = xmin((i-1)*(nplaces+ntrans)+nplaces+1 : (i-1)*(nplaces+ntrans)+nplaces+ntrans);
                            if (max(fire) > 0)
                                message = sprintf('%s\n============STEP %d =============\n',message,i);
                                message=sprintf('%s\nMarking [ %s ] = %s\n',message,mat2str(find(m>eps*10^5)),mat2str(m(m>eps*10^5)));
                                message = sprintf('%s\nSigma [ %s ] = %s\n',message,mat2str(find(fire>eps*10^5)),mat2str(fire(fire>eps*10^5)));
                                %                 if (reduced == 0)
                                %                     Run_cells = [Run_cells, rmt_marking2places(m)];  %visited cells (rows)
                                %                 else
                                temp = rmt_marking2places(m);
                                original_places{i+1} = [];
                                for j = 1 : length(temp)
                                    original_places{i+1} = union(original_places{i+1},data.Tr.Cells{temp(j)});
                                end
                                %                end
                            else
                                original_places{i+1} = original_places{i};
                            end
                        end
                        
                        message = sprintf('%s\n=======================================================',message);
                        message = sprintf('%s\nProject the solution to the initial transition system with GLPK',message);
                        message = sprintf('%s\n=======================================================\n',message);
                        [Pre,Post] = rmt_construct_PN(data.T.adj);
                        m0 = data.T.m0;
                        steps = 1;
                        message = sprintf('%s\nInitial marking [ %s ] = %s\n',message,mat2str(find(m0>eps*10^5)),mat2str(m0(m0>eps*10^5)));
                        Run_cells = data.RO';%rmt_marking2places(m0);
                        tic;
                        for i = 1 : 2*data.intermediateMarkings
                            m = xmin((i-1)*(nplaces+ntrans)+1 : (i-1)*(nplaces+ntrans)+nplaces);
                            fire = xmin((i-1)*(nplaces+ntrans)+nplaces+1 : (i-1)*(nplaces+ntrans)+nplaces+ntrans);
                            if (max(fire) > 0)
                                % places = find(m>eps*10^5);%final marked places
                                places = rmt_marking2places(m);%final marked places
                                markings = m(m>eps*10^5); %number of tokens in final marked places
                                temp = setdiff(1:size(Post,1),union(original_places{i},original_places{i+1}));
                                TT = data.T;
                                for j = 1 : length(temp)
                                    TT.adj(temp(j),:)=0;
                                    TT.adj(:,temp(j))=0;
                                end
                                [Pre_new,Post_new] = rmt_construct_PN(TT.adj);
                                Aeq = [eye(size(Pre_new,1)) -Post_new+Pre_new]; %state equation: m = m0 + C \sigma
                                beq = m0;
                                %add constraints on the intermediate final marking
                                % for j = 1 : length(places)
                                for j = 1 : length(markings)
                                    m_final = zeros(1,size(Pre_new,1));
                                    m_final(data.Tr.Cells{places(j)}) = 1;
                                    Aeq = [Aeq; m_final zeros(1,size(Pre_new,2))];
                                    beq = [beq; markings(j)];
                                end
                                %[X,~,exitflag] = cplexlp(ones(1,size(Pre_new,2)+size(Pre_new,1)),[],[],...
                                %    Aeq,beq,zeros(1,size(Pre_new,1)+size(Pre_new,2)),[],[],options);
                                ctype3='';
                                for i = 1 : size(Aeq,1)
                                    ctype3 = sprintf('%sS',ctype3);
                                end
                                vartype3 = '';
                                for j = 1 : size(Aeq,2)
                                    vartype3 = sprintf('%sC',vartype3); %put the sigma as integer
                                end
                                [X,~,exitflag] = glpk(ones(1,size(Pre_new,2)+size(Pre_new,1)),Aeq,beq,zeros(1,size(Pre_new,1)+size(Pre_new,2)),[],ctype3,vartype3,1);
                                if f==0
                                    uiwait(errordlg('Error solving LPP to project the solution!','Robot Motion Toolbox','modal'));
                                    return;
                                end
                                
                                [ret,Run_cells] = rmt_find_sequence(m0, round(X(size(Pre_new,1)+1:end)), Pre_new, Post_new,steps,Run_cells);
                                steps = steps + sum(round(X(size(Pre,1)+1:end)))+1;
                                fire = round(X(size(Pre_new,1)+1:end));
                                disp(ret);
                                m0 = round(X(1:size(Pre_new,1))); % final marking is the initial marking for the next step
                            end
                        end
                    end
                    time = toc;
                    message2 = sprintf('Total time for solving %d LPPs for projecting the solution: %g secs', ...
                        2*data.intermediateMarkings, time);
                    uiwait(msgbox(message2,'Robot Motion Toolbox','modal'));
                    message = sprintf('%s\n%s',message,message2);
                    
                    cla(data.handle_env);
                    %%Execution monitoring strategy starts from here:
                    N_r = length(data.RO);
                    current_pos=cell(1,N_r);
                    for r=1:N_r
                        current_pos{r}=data.initial{r};%R_trajs{r}(:,1); %current_pos - initial (current) position of robots (cell, current_pos{i} is a column vector with 2 rows)
                    end
                    rmt_represent_atomic_props(data.T.Vert,data.propositions);    %represent all atomic props and store handles
                    
                    %independent runs of each robot (no synchronizations, because of our def. of Boolean formulae)
                    [Runs, ~] = rmt_robot_runs(data.T,Run_cells); %remove successive repetitions from individual discrete trajectories
                    
                    
                    rob_traj = rmt_rob_cont_traj(data.T,Runs,data.initial);    %continuous trajectory of each robot
                    message = sprintf('%s\nSOLUTION - runs of robots: \n',message);
                    for j = 1 : length(Runs)
                        message = sprintf('%s\nRobot %d: ',message,j);
                        temp = Runs{j};
                        for k = 1 : length(temp)-1
                            message = sprintf('%s%d,',message,temp(k));
                        end
                        message = sprintf('%s%d',message,temp(length(temp)));
                    end
                    
                    
                    for r=1:length(rob_traj)    %plot trajectories of robots
                        plot(rob_traj{r}(1,1),rob_traj{r}(2,1),data.rob_plot.line_color{r},...
                            'Marker',data.rob_plot.marker{r},'LineWidth',data.rob_plot.line_width{r});
                        plot(rob_traj{r}(1,:),rob_traj{r}(2,:),data.rob_plot.line_color{r},...
                            'LineWidth',data.rob_plot.line_width{r});
                        plot(rob_traj{r}(1,end),rob_traj{r}(2,end),data.rob_plot.line_color{r},...
                            'Marker',data.rob_plot.marker{r},'LineWidth',data.rob_plot.line_width{r});
                    end
                    button = questdlg('Save the details to a text file?','Robot Motion Toolbox');
                    if strcmpi(button,'Yes')
                        [filename, pathname] = uiputfile('*.txt', 'Save experiments as');
                        fileID = fopen(fullfile(pathname, filename),'w');
                        fprintf(fileID,'%s',message);
                        fclose(fileID);
                    end
                    
                    set(gcf,'UserData',data);%to save data
                    
                    
                    
                end
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                
            case 3 %Boolean formulas on Petri net models
                data = get(gcf,'UserData');
                % Takes the string containing the Boolean formula inserted
                data.formula = get(findobj(gcf,'Tag','booleanformula'),'String');
                button = questdlg('Solve on the full trasition system or on the reduced one?',...
                    'Robot Motion Toolbox','Full system','Reduced system','Full system');
                if strcmpi(button,'Reduced system')
                    reduced = 1;
                    tic;
                    if ~isfield(data,'Tr')
                        data.Tr = rmt_quotient_T(data.T); %quotient of partition T, with fewer states (based on collapsing states with same observables in same connected component with same obs)
                    end
                    
                    % Construction of the Petri net
                    [Pre,Post] = rmt_construct_PN(data.Tr.adj);
                    m0=data.Tr.m0;
                    message = sprintf('Petri net system has %d places and %d transitions\nTime spent for creating it: %g secs', size(Pre,1),size(Pre,2),toc);
                    uiwait(msgbox(message,'Robot Motion Toolbox','modal'));
                    tic;
                    [A,b,Aeq,beq,cost] = rmt_construct_constraints(Pre,Post,m0,data.Tr.props,data.intermediateMarkings,data.formula);
                    message2 = sprintf('\nMathematical program has %d variables and %d equality constraints and %d inequality constraints;\nTime spent for creating the problem: %g secs\n', size(A,2), size(Aeq,1), size(A,1), toc);
                    message = sprintf('%s%s', message, message2);
                    uiwait(msgbox(message2,'Robot Motion Toolbox','modal'));
                else
                    reduced = 0;
                    tic;
                    [Pre,Post] = rmt_construct_PN(data.T.adj);
                    m0=data.T.m0;
                    tic;
                    [A,b,Aeq,beq,cost] = rmt_construct_constraints(Pre,Post,m0,data.T.props,data.intermediateMarkings,data.formula);
                    message = sprintf('Petri net system has %d places and %d transitions\nTime spent for creating it: %g secs', size(Pre,1),size(Pre,2),toc);
                    message = sprintf('%s\nThe MILP has %d variables and %d equality constraints and %d inequality constraints;\nTime spent for creating the problem: %g secs\n', message, size(A,2), size(Aeq,1), size(A,1),toc);
                    uiwait(msgbox(message,'Robot Motion Toolbox','modal'));
                end
                data.Pre = Pre;
                data.Post = Post;
                set(gcf,'UserData',data);%to save data
                
                data = get(gcf,'UserData');
                if strcmp(data.cplex_variable,'true')
                    message = sprintf('%s\n\nThe MILP solution is with CPLEX\n\n', message);
                    %%%%%%%%%%%%%%%%%%%%%%%%%%
                    ctype='';
                    for i = 1 : size(Aeq,1)
                        ctype = sprintf('%sS',ctype);
                    end
                    for i = 1 : size(A,1)
                        ctype = sprintf('%sU',ctype);
                    end
                    vartype = '';
                    for i = 1 : data.intermediateMarkings
                        for j = 1 : size(Pre,1)
                            vartype = sprintf('%sC',vartype); %put the markings as real
                        end
                        for j = 1 : size(Pre,2)
                            vartype = sprintf('%sI',vartype); %put the sigma as integer
                        end
                    end
                    for i = 1 : 2 * length(data.T.props)
                        vartype = sprintf('%sB',vartype); %put the binary variable
                    end
                    vartype = sprintf('%sC',vartype); %put the infinite norm b as real 
                    tic;
                    [xmin,f,exitflag] = cplexmilp(cost,A,b,Aeq,beq,[],[],[],zeros(1,size(A,2)),[],vartype);
                    
                else
                    % Solution with GLPK
                    message = sprintf('%s\n\nThe MILP solution is with GLPK\n\n', message);
                    ctype1='';
                    ctype2='';
                    for i = 1 : size(Aeq,1)
                        ctype2 = sprintf('%sS',ctype2);
                    end
                    for i = 1 : size(A,1)
                        ctype1 = sprintf('%sU',ctype1);
                    end
                    
                    vartype = '';
                    for i = 1 : data.intermediateMarkings
                        for j = 1 : size(Pre,1)
                            vartype = sprintf('%sC',vartype); %put the markings as real
                        end
                        for j = 1 : size(Pre,2)
                            vartype = sprintf('%sI',vartype); %put the sigma as integer
                        end
                    end
                    for i = 1 : 2 * length(data.T.props)
                        vartype = sprintf('%sB',vartype); %put the binary variable
                    end
                    vartype = sprintf('%sC',vartype); %put the infinite norm b as real
                    tic;
                    Atot = [Aeq; A];
                    btot= [beq; b];
                    ctype = [ctype2 ctype1];
                    [xmin,f,exitflag] = glpk(cost,Atot,btot,zeros(1,size(A,2)),[],ctype,vartype);
                end
                time = toc;
                if f==0%no solution
                    uiwait(errordlg('Error solving the ILP. The problem may have no feasible solution. Increase k!','Robot Motion Toolbox','modal'));
                    return;
                end
                message2 = sprintf('\nTime of solving the MILP: %g secs\n', time);
                message = sprintf('%s%s', message, message2);
                uiwait(msgbox(message2,'Robot Motion Toolbox','modal'));
                
                if (reduced == 1)
                    message = sprintf('%s\n=======================================================',message);
                    message = sprintf('%s\nInitial solution on the reduced transition system',message);
                    message = sprintf('%s\n=======================================================\n',message);
                end
                
                message = sprintf('%s\nInitial marking [ %s ] = %s\n',message,mat2str(find(m0>eps*10^5)),mat2str(m0(m0>eps*10^5)));
                if (reduced == 0)
                    Run_cells = data.RO';%rmt_marking2places(m0);
                else
                    original_places{1} = [];
                    temp = rmt_marking2places(m0);
                    for j = 1 : length(temp)
                        original_places{1} = union(original_places{1},data.Tr.Cells{temp(j)});
                    end
                end
                nplaces = size(Pre,1);
                ntrans = size(Pre,2);
                
                for i = 1 : data.intermediateMarkings
                    m = xmin((i-1)*(nplaces+ntrans)+1 : (i-1)*(nplaces+ntrans)+nplaces);
                    fire = xmin((i-1)*(nplaces+ntrans)+nplaces+1 : (i-1)*(nplaces+ntrans)+nplaces+ntrans);
                    if (max(fire) > 0)
                        message = sprintf('%s\n============STEP %d =============\n',message,i);
                        message=sprintf('%s\nMarking [ %s ] = %s\n',message,mat2str(find(m>eps*10^5)),mat2str(m(m>eps*10^5)));
                        message = sprintf('%s\nSigma [ %s ] = %s\n',message,mat2str(find(fire>eps*10^5)),mat2str(fire(fire>eps*10^5)));
                        if (reduced == 0)
                            Run_cells = [Run_cells, rmt_marking2places(m)];  %visited cells (rows)
                        else
                            temp = rmt_marking2places(m);
                            original_places{i+1} = [];
                            for j = 1 : length(temp)
                                original_places{i+1} = union(original_places{i+1},data.Tr.Cells{temp(j)});
                            end
                        end
                    else
                        if (reduced == 1)
                            original_places{i+1} = original_places{i};
                        end
                    end
                end
                message = sprintf('%sBoolean variables in solutions are: %s\n',message,mat2str(xmin(data.intermediateMarkings*(size(Pre,1)+size(Pre,2))+1:end)));
                
                if (reduced == 1)
                    %                     options = cplexoptimset('cplex');
                    %                     options.display='off';
                    %                     options.lpmethod=1;
                    message = sprintf('%s\n================================================================',message);
                    message = sprintf('%s\nProject the solution to the initial transition system with GLPK',message);
                    message = sprintf('%s\n================================================================\n',message);
                    [Pre,Post] = rmt_construct_PN(data.T.adj);
                    m0 = data.T.m0;
                    steps = 1;
                    message = sprintf('%s\nInitial marking [ %s ] = %s\n',message,mat2str(find(m0>eps*10^5)),mat2str(m0(m0>eps*10^5)));
                    Run_cells = data.RO';%rmt_marking2places(m0);
                    tic;
                    for i = 1 : data.intermediateMarkings
                        m = xmin((i-1)*(nplaces+ntrans)+1 : (i-1)*(nplaces+ntrans)+nplaces);
                        fire = xmin((i-1)*(nplaces+ntrans)+nplaces+1 : (i-1)*(nplaces+ntrans)+nplaces+ntrans);
                        if (max(fire) > 0)
                            % places = find(m>eps*10^5);%final marked places
                            places = rmt_marking2places(m);%final marked places
                            markings = m(m>eps*10^5); %number of tokens in final marked places
                            temp = setdiff(1:size(Post,1),union(original_places{i},original_places{i+1}));
                            TT = data.T;
                            for j = 1 : length(temp)
                                TT.adj(temp(j),:)=0;
                                TT.adj(:,temp(j))=0;
                            end
                            [Pre_new,Post_new] = rmt_construct_PN(TT.adj);
                            Aeq = [eye(size(Pre_new,1)) -Post_new+Pre_new]; %state equation: m = m0 + C \sigma
                            beq = m0;
                            %add constraints on the intermediate final marking
                            % for j = 1 : length(places)
                            for j = 1 : length(markings)
                                m_final = zeros(1,size(Pre_new,1));
                                m_final(data.Tr.Cells{places(j)}) = 1;
                                Aeq = [Aeq; m_final zeros(1,size(Pre_new,2))];
                                beq = [beq; markings(j)];
                            end
                            %[X,~,exitflag] = cplexlp(ones(1,size(Pre_new,2)+size(Pre_new,1)),[],[],...
                            %          Aeq,beq,zeros(1,size(Pre_new,1)+size(Pre_new,2)),[],[],options);
                            ctype3='';
                            for i = 1 : size(Aeq,1)
                                ctype3 = sprintf('%sS',ctype3);
                            end
                            vartype3 = '';
                            for j = 1 : size(Aeq,2)
                                vartype3 = sprintf('%sC',vartype3); %put the sigma as integer
                            end
                            [X,~,exitflag] = glpk(ones(1,size(Pre_new,2)+size(Pre_new,1)),Aeq,beq,zeros(1,size(Pre_new,1)+size(Pre_new,2)),[],ctype3,vartype3,1);
                            if f==0
                                uiwait(errordlg('Error solving LPP to project the solution!','Robot Motion Toolbox','modal'));
                                return;
                            end
                            
                            [ret,Run_cells] = rmt_find_sequence(m0, round(X(size(Pre_new,1)+1:end)), Pre_new, Post_new,steps,Run_cells);
                            steps = steps + sum(round(X(size(Pre,1)+1:end)))+1;
                            fire = round(X(size(Pre_new,1)+1:end));
                            disp(ret);
                            m0 = round(X(1:size(Pre_new,1))); % final marking is the initial marking for the next step
                        end
                    end
                    time = toc;
                    message2 = sprintf('Total time for solving %d LPPs for projecting the solution: %g secs', ...
                        data.intermediateMarkings, time);
                    uiwait(msgbox(message2,'Robot Motion Toolbox','modal'));
                    message = sprintf('%s\n%s',message,message2);
                end
                cla(data.handle_env);
                %%Execution monitoring strategy starts from here:
                N_r = length(data.RO);
                current_pos=cell(1,N_r);
                for r=1:N_r
                    current_pos{r}=data.initial{r};%R_trajs{r}(:,1); %current_pos - initial (current) position of robots (cell, current_pos{i} is a column vector with 2 rows)
                end
                rmt_represent_atomic_props(data.T.Vert,data.propositions);    %represent all atomic props and store handles
                
                %independent runs of each robot (no synchronizations, because of our def. of Boolean formulae)
                [Runs, ~] = rmt_robot_runs(data.T,Run_cells); %remove successive repetitions from individual discrete trajectories
                
                
                rob_traj = rmt_rob_cont_traj(data.T,Runs,data.initial);    %continuous trajectory of each robot
                message = sprintf('%s\nSOLUTION - runs of robots: \n',message);
                for j = 1 : length(Runs)
                    message = sprintf('%s\nRobot %d: ',message,j);
                    temp = Runs{j};
                    for k = 1 : length(temp)-1
                        message = sprintf('%s%d,',message,temp(k));
                    end
                    message = sprintf('%s%d',message,temp(length(temp)));
                end
                for r=1:length(rob_traj)    %plot trajectories of robots
                    plot(rob_traj{r}(1,1),rob_traj{r}(2,1),data.rob_plot.line_color{r},...
                        'Marker',data.rob_plot.marker{r},'LineWidth',data.rob_plot.line_width{r});
                    plot(rob_traj{r}(1,:),rob_traj{r}(2,:),data.rob_plot.line_color{r},...
                        'LineWidth',data.rob_plot.line_width{r});
                    plot(rob_traj{r}(1,end),rob_traj{r}(2,end),data.rob_plot.line_color{r},...
                        'Marker',data.rob_plot.marker{r},'LineWidth',data.rob_plot.line_width{r});
                end
                button = questdlg('Save the details to a text file?','Robot Motion Toolbox');
                if strcmpi(button,'Yes')
                    [filename, pathname] = uiputfile('*.txt', 'Save experiments as');
                    fileID = fopen(fullfile(pathname, filename),'w');
                    fprintf(fileID,'%s',message);
                    fclose(fileID);
                end
                
                set(gcf,'UserData',data);%to save data
            case 1 %rechability tasks
                planning_approach = get(findobj(gcf,'Tag','pathpoints'),'Value'); %planning_approach= 1 - cell descomposition; 2 - visibility graph; 3 - Voronoi
                data = get(gcf,'UserData');
                
                cla(data.handle_ori);
                cla(data.handle_vel);
                cla(data.handle_ang);
                if ~isfield(data,'Nobstacles')
                    return;
                end
                for i=1:data.Nobstacles
                    aux = data.obstacles{i};
                    [as, bs] = size(data.obstacles{1});
                    if(as > bs)
                        aux = aux';
                    end
                    obstaclesCD(i) = {aux};
                end
                
                switch planning_approach
                    case 1%cell decomposition
                        if (get(findobj(gcf,'Tag','triang'),'Value') == 1)
                            [C,adj,mid_X,mid_Y,com_F]=rmt_triangular_decomposition(obstaclesCD,data.frame_limits);   %triangular decomposition
                        elseif (get(findobj(gcf,'Tag','rect'),'Value') == 1)
                            [C,adj,mid_X,mid_Y,com_F]=rmt_rectangular_decomposition(obstaclesCD,data.frame_limits);   %rectangular decomposition
                        elseif (get(findobj(gcf,'Tag','poly'),'Value') == 1)
                            [C,adj,mid_X,mid_Y,com_F]=rmt_polytopal_decomposition(obstaclesCD,data.frame_limits);   %polytopal decomposition
                        elseif (get(findobj(gcf,'Tag','trapez'),'Value') == 1)
                            [C,adj,mid_X,mid_Y,com_F]=rmt_trapezoidal_decomposition(obstaclesCD,data.frame_limits);   %trapezoidal decomposition
                        end
                        cla(data.handle_env);
                        axes(data.handle_env);
                        rmt_plot_environment(obstaclesCD,data.frame_limits,C,'c');
                        for k = 1 : length(data.initial)
                            plot(data.initial{k}(1),data.initial{k}(2),'pw',...
                                'Markersize',13, 'Color', 'k');
                        end
                        for k = 1 : length(data.final)
                            plot(data.final{k}(1),data.final{k}(2),'pw',...
                                'Markersize',13, 'Color', 'b');
                        end
                        set(data.handle_env,'xlim',[data.frame_limits(1) data.frame_limits(2)],'ylim',[data.frame_limits(3) data.frame_limits(4)],'XGrid','on','YGrid','on');
                        grid on;
                        %assign weight on the arcs if MPC is not chosen
                        if (get(findobj(gcf,'Tag','waypoints'),'Value') ~= 5)
                            prompt = 'Cost weights of each edge (ci,cj):';
                            dlg_title = 'Robot Motion Toolbox';
                            str = {'1','norm(centr(c_i)-centr(c_j))','norm(centr(c_i)-mid(c_i,c_j))',...
                                'sum(c_h, c_h~=c_j, norm(mid(c_h,c_i)?mid(c_i,c_j))/(number of neighbors of c1 - 1)'};
                            [s,~] = listdlg('PromptString',prompt,'Name',dlg_title,'SelectionMode','single','ListString',str);
                            switch s
                                case 2 %distance between the centroids
                                    for i = 1 : size(adj,1)-1
                                        adj(i,i) = 0.0001; %for selfloops add a very small cost
                                        for j = i+1 : size(adj,1)
                                            if (adj(i,j)~=0)
                                                adj(i,j) = norm(mean(C{i},2)-mean(C{j},2));
                                                adj(j,i) = adj(i,j);
                                            end
                                        end
                                    end
                                case 3 %cost(ci,cj) = norm(centr(ci)-mid(ci,cj))
                                    for i = 1 : size(adj,1)
                                        adj(i,i) = 0.0001; %for selfloops add a very small cost
                                        for j = 1 : size(adj,1)
                                            if ((j ~=i) && (adj(i,j)~=0))
                                                temp = com_F{i,j};
                                                adj(i,j) = norm(mean(C{i},2)-norm(temp(:,1)-temp(:,2)));
                                            end
                                        end
                                    end
                                case 4 %cost(ci,cj) = sum(c_h, c_h~=c_j, norm(mid(c_h,c_i)?mid(c_i,c_j))/(number of neighbors of c1 - 1)
                                    for i = 1 : size(adj,1)
                                        adj(i,i) = 0.0001; %for selfloops add a very small cost
                                        for j = 1 : size(adj,1)
                                            if ((j ~=i) && (adj(i,j)~=0))
                                                temp = setdiff(find(adj(:,i)),[i,j]);
                                                adj(i,j) = 0;
                                                for k = 1 : length(temp)
                                                    temp2 = com_F{temp(k),i};
                                                    temp3 = com_F{j,i};
                                                    adj(i,j) = adj(i,j)+...
                                                        norm( norm(temp2(:,1) - temp2(:,2))...
                                                        - norm(norm(temp3(:,1) - temp3(:,2))));
                                                end
                                                adj(i,j) = adj(i,j) / length(temp);
                                            end
                                        end
                                    end
                            end
                        end
                        temp1 = data.initial{1};
                        temp2 = data.final{1};
                        if (get(findobj(gcf,'Tag','waypoints'),'Value') == 1) %middle points
                            tic;
                            [traj, travel_dist, path_cells, cost_path] = rmt_find_trajectory(C,adj,mid_X,mid_Y,...
                                temp1(1),temp1(2),temp2(1),temp2(2),obstaclesCD);
                            plot(traj(1,:),traj(2,:),'r','LineWidth',2);
                            data.trajectory = traj;
                            message = sprintf('Travelled distance via middle points: %.2f.\n\n Time to compute trajectory %.2f segundos',...
                                travel_dist,toc);
                            uiwait(msgbox(message,'Robot Motion Toolbox','modal'));
                        elseif (get(findobj(gcf,'Tag','waypoints'),'Value') == 2)  %norm 1
                            %function 1-norm
                            safe_dist=0.5;  %should be smaller than half of shortest traversed segment
                            tic;
                            [traj, travel_dist, path_cells, cost_path] = rmt_find_trajectory(C,adj,mid_X,mid_Y,...
                                temp1(1),temp1(2),temp2(1),temp2(2),obstaclesCD);
                            start_p = temp1;
                            goal_p = temp2;
                            [traj_norm_one, dist_norm_one] = rmt_optimize_traj_norm_one(com_F,...
                                path_cells,start_p,goal_p,safe_dist);  %via LP
                            plot(traj_norm_one(1,:),traj_norm_one(2,:),'r','LineWidth',2);
                            data.trajectory = traj_norm_one;
                            message = sprintf('Travelled distance via Norm-1: %.2f.\n\nTime to compute the trajectory: %.2f.',...
                                dist_norm_one,toc);
                            uiwait(msgbox(message,'Robot Motion Toolbox','modal'));
                        elseif (get(findobj(gcf,'Tag','waypoints'),'Value') == 3)  %norm 2
                            safe_dist=0.5;  %should be smaller than half of shortest traversed segment
                            tic;
                            [traj, travel_dist, path_cells, cost_path] = rmt_find_trajectory(C,adj,mid_X,mid_Y,...
                                temp1(1),temp1(2),temp2(1),temp2(2),obstaclesCD);
                            start_p = temp1;
                            goal_p = temp2;
                            [traj_norm_2_sq, dist_norm_2_sq] = rmt_optimize_traj_norm_two_sq(com_F,path_cells,start_p,goal_p,safe_dist);  %via QP
                            data.trajectory = traj_norm_2_sq;
                            plot(traj_norm_2_sq(1,:),traj_norm_2_sq(2,:),'r','LineWidth',2);
                            message = sprintf('Travelled distance via Norm-2: %.2f.\n\nTime to compute the trajectory %.2f.',...
                                dist_norm_2_sq,toc);
                            uiwait(msgbox(message,'Robot Motion Toolbox','modal'));
                        elseif (get(findobj(gcf,'Tag','waypoints'),'Value') == 4)  %norm inf.
                            safe_dist=0.5;  %should be smaller than half of shortest traversed segment
                            tic;
                            [traj, travel_dist, path_cells, cost_path] = rmt_find_trajectory(C,adj,mid_X,mid_Y,...
                                temp1(1),temp1(2),temp2(1),temp2(2),obstaclesCD);
                            start_p = temp1;
                            goal_p = temp2;
                            [traj_norm_inf, dist_norm_inf] = rmt_optimize_traj_norm_inf(com_F,path_cells,start_p,goal_p,safe_dist);  %via LP
                            data.trajectory = traj_norm_inf;
                            plot(traj_norm_inf(1,:),traj_norm_inf(2,:),'r','LineWidth',2);
                            message = sprintf('Travelled distance via Norm-Inf.: %.2f.\n\nTime to compute the trajectory: %.2f',...
                                dist_norm_inf,toc);
                            uiwait(msgbox(message,'Robot Motion Toolbox','modal'));
                        elseif (get(findobj(gcf,'Tag','waypoints'),'Value') == 5)  %MPC 
                            data = get(gcf,'UserData');
                            prompt = {'Safety distance:','N:','Number of intermediate points on common edges:'};
                            dlg_title = 'Robot Motion Toolbox';
                            num_lines = 1;
                            defaultans = {num2str(data.waypointsMPC.safe_dist),num2str(data.waypointsMPC.N),...
                                num2str(data.waypointsMPC.intermPoints)};
                            input_user = inputdlg(prompt,dlg_title,num_lines,defaultans);
                            data.waypointsMPC.safe_dist = str2num(char(input_user(1)));   %should be smaller than half of shortest traversed segment
                            data.waypointsMPC.N = str2num(char(input_user(2))); % Receding horizon
                            data.waypointsMPC.intermPoints = str2num(char(input_user(3)));                           
                            set(gcf,'UserData',data);
                            tic;
                            [traj, travel_dist] = rmt_optimize_traj_mpc(C,adj,com_F,...
                                data.waypointsMPC.safe_dist,data.waypointsMPC.intermPoints,...
                                data.waypointsMPC.N, temp1(1),temp1(2),temp2(1),temp2(2),obstaclesCD);
                            for i = 1 : size(traj,2)-1
                                plot([traj(1,i) traj(1,i+1)],[traj(2,i) traj(2,i+1)],'r');
                            end
                            message = sprintf('Travelled distance via MPC: %.2f.\n\nTime to compute the trajectory: %.2f',...
                                travel_dist,toc);
                            uiwait(msgbox(message,'Robot Motion Toolbox','modal'));
                        end
                        set(data.handle_env,'xlim',[data.frame_limits(1) data.frame_limits(2)],'ylim',[data.frame_limits(3) data.frame_limits(4)],'XGrid','on','YGrid','on');
                    case 2%visibility graph
                        input_variables = zeros(8,1);
                        input_variables(1) = data.frame_limits(1);
                        input_variables(2) = data.frame_limits(2);
                        input_variables(3) = data.frame_limits(3);
                        input_variables(4) = data.frame_limits(4);
                        input_variables(5) = length(data.obstacles);
                        input_variables(6) = eval(get(findobj(gcf,'Tag','wheelbase'),'String'));
                        temp1 = data.initial{1};
                        temp2 = data.final{1};
                        input_variables(7) = temp1(1);
                        input_variables(8) = temp1(2);
                        input_variables(9) = temp2(1);
                        input_variables(10) = temp2(2);
                        cla(data.handle_env);%new
                        axes(data.handle_env);
                        rmt_plot_environment(data.obstacles,data.frame_limits);
                        plot(temp1(1),temp1(2),'pw','Markersize',13, 'Color', 'k');
                        plot(temp2(1),temp2(2),'pw','Markersize',13, 'Color', 'b');
                        grid on;
                        %traj = rmt_visibility_graph_new2(data.handle_env,input_variables,data.map,data.obstacles);
                        traj = rmt_vgraph2(data.handle_env,input_variables,data.obstacles);
                        %plot(traj(1,:),traj(2,:),'--r','LineWidth',2);%new
                        %set(data.handle_env,'xlim',[data.frame_limits(1) data.frame_limits(2)],'ylim',[data.frame_limits(3) data.frame_limits(4)],'XGrid','on','YGrid','on');
                        %[trajectory] = rmt_vgraph2(handle,input_variables,seq_obstacles)
                        data.trajectory = traj';
                        set(data.handle_env,'xlim',[data.frame_limits(1) data.frame_limits(2)],'ylim',[data.frame_limits(3) data.frame_limits(4)],'XGrid','on','YGrid','on');
                    case 3%voronoi
                        cla(data.handle_env);%new
                        axes(data.handle_env);
                        rmt_plot_environment(data.obstacles,data.frame_limits);
                        temp1 = data.initial{1};
                        temp2 = data.final{1};
                        plot(temp1(1),temp1(2),'pw','Markersize',13, 'Color', 'k');
                        plot(temp2(1),temp2(2),'pw','Markersize',13, 'Color', 'b');
                        grid on;
                        limits = data.frame_limits;
                        whe = eval(get(findobj(gcf,'Tag','wheelbase'),'String'));
                        [X_Total_points,Y_Total_points, ...
                            All_cells_Number, Cell_start, X1] = rmt_voronoi_epsi(data.handle_env,data.Nobstacles,limits,whe*0.5,...
                            data.epsilonvoronoi,data.obstacles);
                        data.X_Total_points = X_Total_points;
                        data.Y_Total_points = Y_Total_points;
                        data.All_cells_Number = All_cells_Number;
                        data.Cell_start = Cell_start;
                        data.X1 = X1;
                        traj = rmt_get_voronoi(data.handle_env, data.frame_limits, (data.Nobstacles+1),...
                            temp1, temp2,data.X_Total_points, data.Y_Total_points, ...
                            data.All_cells_Number, data.Cell_start, data.X1);
                        %traj = rmt_visibility_graph(data.handle_env,input_variables,data.map,data.obstacles);
                        data.trajectory = traj';
                end
        end
        set(gcf,'UserData',data);
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %
        %               RUN CONTROL
        %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case 'run_control'
        sampling_period = eval(get(findobj(gcf,'Tag','sampling'),'String'));
        max_ang_vel = eval(get(findobj(gcf,'Tag','angvel'),'String'));
        max_linear_vel = eval(get(findobj(gcf,'Tag','linvel'),'String'));
        max_steering = eval(get(findobj(gcf,'Tag','steering'),'String'));
        wheel_radius = eval(get(findobj(gcf,'Tag','wheel'),'String'));
        wheel_base = eval(get(findobj(gcf,'Tag','wheelbase'),'String'));
        lookahead_distance = eval(get(findobj(gcf,'Tag','lookahead'),'String'));
        
        if (get(findobj(gcf,'Tag','robotCar'),'Value') == 1)
            robotType = 1; %car-like robot
        elseif (get(findobj(gcf,'Tag','robotDifferential'),'Value') == 1)
            robotType = 2; %differential robot
        end
        data = get(gcf,'UserData');
        temp1 = data.initial{1};
        xini = temp1(1);
        yini = temp1(2);
        if ~isfield(data,'trajectory')
            return;
        end
        ref_trajectory = data.trajectory;
        threshold_goal = 0.15;%distance to consider the goal has been reached.
        input_variables = [sampling_period, xini, yini, wheel_radius,...
            wheel_base,max_linear_vel,...
            max_steering, lookahead_distance,robotType,threshold_goal,...
            max_ang_vel,data.frame_limits(1),data.frame_limits(2),...
            data.frame_limits(3) data.frame_limits(4)];
        if(get(findobj(gcf,'Tag','controller'),'Value') == 1)
            %function pure-pursuit
            fprintf('\nPure pursuit controller is running...');
            [array_time, array_alpha, array_v, array_pos] = rmt_pure_pursuit(input_variables,ref_trajectory,data.orientation,data.obstacles,data.Nobstacles);
        else
            %PI control
            fprintf('\nPI controller is running...');
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
        
        %drawing the result
        color = hsv(5);
        color = color(randperm(5),:);
        cha = floor(rand(1)*5+1);
        %trajectory
        plot(data.handle_env,array_pos(1,:),array_pos(2,:),'Color','k','LineWidth',2);
        set(data.handle_env,'XGrid','on','YGrid','on');
        
        %orientation
        plot(data.handle_ori,array_time,rad2deg(array_pos(3,:)),'Color','k','LineWidth',2);
        hold on;
        xmax = max(array_time);
        ymax = max(rad2deg(array_pos(3,:)));
        ymin = min(rad2deg(array_pos(3,:)));
        if(ymax == ymin)
            ymax = ymax + 0.1;
        end
        set(data.handle_ori,'Visible','on','xlim',[0 xmax],'ylim',[ymin ymax],'XGrid','on','YGrid','on');
        
        %velocities
        plot(data.handle_vel,array_time,array_v,'Color','k','LineWidth',2);
        hold on;
        xmax = max(array_time);
        ymax = max(array_v)+1;
        ymin = min(array_v)-1;
        if(ymax == ymin)
            ymax = ymax + 0.1;
        end
        set(data.handle_vel,'Visible','on','xlim',[0 xmax],'ylim',[ymin ymax],'XGrid','on','YGrid','on');
        %set(data.handle_vel,'XGrid','on','YGrid','on');
        
        %steering angle
        plot(data.handle_ang,array_time,rad2deg(array_alpha),'Color','k','LineWidth',2);
        hold on;
        xmax = max(array_time);
        ymax = max(rad2deg(array_alpha));
        ymin = min(rad2deg(array_alpha));
        if(ymax == ymin)
            ymax = ymax + 0.1;
        end
        set(data.handle_ang,'Visible','on','xlim',[0 xmax],'ylim',[ymin ymax],'XGrid','on','YGrid','on');
        %set(data.handle_ang,'XGrid','on','YGrid','on');
        
    case 'max_lin_vel_changed'
        input_val = char(get(findobj(gcf,'Tag','linvel'),'String'));
        todoOK = rmt_detect_error(input_val,0.1,10);
        if todoOK == 0
            uiwait(errordlg(sprintf('\nValid range betweeen 0.1 and 10!'),'Robot Motion Toolbox','modal'));
            set(findobj(gcf,'Tag','linvel'),'String','1');
        end
    case 'max_ang_vel_changed'
        input_val = char(get(findobj(gcf,'Tag','angvel'),'String'));
        todoOK = rmt_detect_error(input_val,0.1,5);
        if todoOK == 0
            uiwait(errordlg(sprintf('\nValid range betweeen 0.1 and 5!'),'Robot Motion Toolbox','modal'));
            set(findobj(gcf,'Tag','angvel'),'String','1');
        end
    case 'max_steering_changed'
        input_val = char(get(findobj(gcf,'Tag','steering'),'String'));
        todoOK = rmt_detect_error(input_val,10,60);
        if todoOK == 0
            uiwait(errordlg(sprintf('\nValid range betweeen 10 and 60!'),'Robot Motion Toolbox','modal'));
            set(findobj(gcf,'Tag','steering'),'String','30');
        end
    case 'wheel_radius_changed'
        input_val = char(get(findobj(gcf,'Tag','wheel'),'String'));
        todoOK = rmt_detect_error(input_val,0.01,1.5);
        if todoOK == 0
            uiwait(errordlg(sprintf('\nValid range betweeen 0.01 and 1.5!'),'Robot Motion Toolbox','modal'));
            set(findobj(gcf,'Tag','wheel'),'String','0.05');
        end
    case 'wheel_base_changed'
        input_val = char(get(findobj(gcf,'Tag','wheelbase'),'String'));
        todoOK = rmt_detect_error(input_val,0.1,2.5);
        if todoOK == 0
            uiwait(errordlg(sprintf('\nValid range betweeen 0.1 and 2.5!'),'Robot Motion Toolbox','modal'));
            set(findobj(gcf,'Tag','wheelbase'),'String','0.25');
        end
    case 'sampling_changed'
        input_val = char(get(findobj(gcf,'Tag','sampling'),'String'));
        todoOK = rmt_detect_error(input_val,0.01,2);
        if todoOK == 0
            uiwait(errordlg(sprintf('\nValid range betweeen 0.01 and 2!'),'Robot Motion Toolbox','modal'));
            set(findobj(gcf,'Tag','sampling'),'String','0.1');
        end
    case 'lookahead_changed'
        input_val = char(get(findobj(gcf,'Tag','lookahead'),'String'));
        todoOK = rmt_detect_error(input_val,1,20);
        if todoOK == 0
            uiwait(errordlg(sprintf('\nValid range betweeen 1 and 20!'),'Robot Motion Toolbox','modal'));
            set(findobj(gcf,'Tag','lookahead'),'String','10');
        end
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
        if contains(file,'.rmt')
            delete(gcf);
            hgload(file2);
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
            if inpolygon(initial(1),initial(2),data.obstacles{i}(1,:),data.obstacles{i}(2,:))
                uiwait(errordlg('Initial point inside a region of interest/obstacle! Robot not moved!','Robot Motion Toolbox','modal'));
                return;
            end
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
                    data.T.RO(Selection) = j;
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
            cla(data.handle_env);
            rmt_plot_environment(data.obstacles,data.frame_limits,data.T.Vert);
            rmt_represent_atomic_props(data.T.Vert,data.propositions);    %represent all atomic props
            for r=1:length(data.RO)
                plot(data.initial{r}(1),data.initial{r}(2),'Color',data.rob_plot.line_color{r},...
                    'LineStyle',data.rob_plot.line{r},...
                    'LineWidth',data.rob_plot.line_width{r},...
                    'Marker',data.rob_plot.marker{r},...
                    'MarkerFaceColor',data.rob_plot.face_color{r});
            end
        end
        
    case 'add_robot'
        IsReach = get(findobj(gcf,'Tag','reach'),'Value');
        if (IsReach == 1)
            uiwait(errordlg('This option is only for multi-robot systems!',...
                'Robot Motion Toolbox','modal'));
            return;
        end
        uiwait(msgbox(sprintf('\nChoose the initial point of the new robot with right click.\n'),'Robot Motion Toolbox','modal'));
        data = get(gcf,'UserData');
        point_ok = 0;
        while(point_ok == 0)
            but=1;
            while but==1
                [x,y,but]=ginput(1);
            end
            in = 0;
            for(ii=1:data.Nobstacles)
                in = in + inpolygon(x,y,data.obstacles{ii}(1,:),data.obstacles{ii}(2,:));
            end
            if(in>0)
                uiwait(msgbox(sprintf('\nInvalid point!\n'),'Robot Motion Toolbox','modal'));
            else
                point_ok = 1;
            end
        end
        plot(x,y,'or','LineWidth',3);
        data.initial{length(data.initial)+1} = [x,y];
        for j=1:length(data.T.Vert)   %indices of starting cell
            if inpolygon(x,y,data.T.Vert{j}(1,:),data.T.Vert{j}(2,:))
                data.RO = [data.RO j];
                data.T.RO = [data.T.RO j];
                break;
            end
        end
        data.T.m0(j) = data.T.m0(j) + 1;  %number of robots initially in state i
        data.orientation=[data.orientation 0];
        set(gcf,'UserData',data);
        %update the plots
        cla(data.handle_env);
        rmt_plot_environment(data.obstacles,data.frame_limits,data.T.Vert);
        rmt_represent_atomic_props(data.T.Vert,data.propositions);    %represent all atomic props
        for r=1:length(data.RO)
            plot(data.initial{r}(1),data.initial{r}(2),'Color',data.rob_plot.line_color{r},...
                'LineStyle',data.rob_plot.line{r},...
                'LineWidth',data.rob_plot.line_width{r},...
                'Marker',data.rob_plot.marker{r},...
                'MarkerFaceColor',data.rob_plot.face_color{r});
        end
    case 'remove_robot'
        IsReach = get(findobj(gcf,'Tag','reach'),'Value');
        if (IsReach == 1)
            uiwait(errordlg('This option is only for multi-robot systems!',...
                'Robot Motion Toolbox','modal'));
            return;
        end
        data = get(gcf,'UserData');
        for i = 1 :length(data.initial)
            str{i} = sprintf('Robot %d located at [%s,%s]',i,...
                mat2str(data.initial{i}(1),3),mat2str(data.initial{i}(2),3));
        end
        [Selection,ok] = listdlg('PromptString','Select the robots to remove:',...
            'SelectionMode','multiple',...
            'ListString',str);
        if (ok == 0)
            return;
        end
        if (length(Selection) == length(data.RO))
            uiwait(errordlg('Not possible to remove all robots!',...
                'Robot Motion Toolbox','modal'));
            return;
        end
        for i = length(Selection):-1:1
            data.initial(Selection(i))=[];
            data.RO(Selection(i))=[];
            data.T.RO(Selection(i)) = [];
            data.T.m0(Selection(i)) = data.T.m0(Selection(i)) - 1;
            data.orientation(Selection(i)) =[];
        end
        set(gcf,'UserData',data);
        %update the plots
        cla(data.handle_env);
        rmt_plot_environment(data.obstacles,data.frame_limits,data.T.Vert);
        rmt_represent_atomic_props(data.T.Vert,data.propositions);    %represent all atomic props
        for r=1:length(data.RO)
            plot(data.initial{r}(1),data.initial{r}(2),'Color',data.rob_plot.line_color{r},...
                'LineStyle',data.rob_plot.line{r},...
                'LineWidth',data.rob_plot.line_width{r},...
                'Marker',data.rob_plot.marker{r},...
                'MarkerFaceColor',data.rob_plot.face_color{r});
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
            [1],{num2str(data.voronoi)});
        try
            temp(1) = eval(answer{1});
        catch
            uiwait(errordlg('Error introducing data!','Robot Motion Toolbox','modal'));
            return;
        end
        data.epsilonvoronoi = temp;
        set(gcf,'UserData',data);
        
    case 'menu_change_ltl_formula'
        data = get(gcf,'UserData');
        if ~isfield(data,'propositions')
            uiwait(errordlg(sprintf('Define first the envirronment'),'Robot Motion Toolbox','modal'));
            return;
        end
        formula = rmt_read_formula(length(data.propositions));
        data.formula=formula;
        set(findobj(gcf,'Tag','ltlformula'),'String',formula);
        set(gcf,'UserData',data);
    case 'ltl_formula_changed'
        data = get(gcf,'UserData');
        data.formula = get(findobj(gcf,'Tag','ltlformula'),'String');
        set(gcf,'UserData',data);
    case 'boolean_formula_changed'
        data = get(gcf,'UserData');
        data.formula = get(findobj(gcf,'Tag','booleanformula'),'String');
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
        warning off;
        if contains(file,'.env')
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
            data2.RO = data.RO;
            data2.T = data.T;
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
    case 'save_paths_workspace'
        data = get(gcf,'UserData');
        checkLabels = {'Save initial points of the robots to variable named:' ...
            'Save regions of interest to variable named:'...
            'Save LTL formula to variable named:',...
            'Save transition system of one robot to variable named:'...
            'Save transition system of the robot team to variable named:'...
            'Save Buchi automaton to variable named:'...
            'Save product automaton of team and Buchi automaton to variable named:'...
            'Save regions of atomic propositions to variable named:'...
            'Save initial regions of robots to variable named:',...
            'Save robot trajectories to variable named:'...
            };
        varNames = {'robot_init', 'regions','formula', 'T', 'Tg', 'B', 'Pg', ...
            'propositions', 'RO', 'R_trajs'};
        items = {data.initial, data.obstacles, data.formula, data.T, data.Tg, data.B, ...
            data.Pg, data.propositions, data.RO, data.R_trajs};
        export2wsdlg(checkLabels, varNames, items, 'Save Simulation Results to Workspace');
    case 'save_paths_figure'
        [filename, pathname] = uiputfile('*.fig', 'Save experiments as');
        data = get(gcf,'UserData');
        Fig2 = figure;
        copyobj(data.handle_env, Fig2);
        temp = get(Fig2,'CurrentAxes');
        set(temp, 'Units', 'normalized', 'Position', [0.1300 0.1100 0.7750 0.8150]);
        hgsave(Fig2, fullfile(pathname, filename));
    case 'change_k'
        data = get(gcf,'UserData');
        answer = inputdlg({...
            sprintf('Maximum number of intermediate markings:')},'Robot Motion Toolbox',...
            [1],{num2str(data.intermediateMarkings)});
        try
            temp(1) = eval(answer{1});
        catch
            uiwait(errordlg('Error introducing data!','Robot Motion Toolbox','modal'));
            return;
        end
        data.intermediateMarkings = temp;
        set(gcf,'UserData',data);
    case 'ltl_pn'
        data = get(gcf,'UserData');
        data.formula = get(findobj(gcf,'Tag','ltlformula'),'String');
        tic;
        
        if ~isfield(data,'Tr')
            data.Tr = rmt_quotient_T(data.T); %quotient of partition T, with fewer states (based on collapsing states with same observables in same connected component with same obs)
        end
        
        [Pre,Post] = rmt_construct_PN(data.Tr.adj);
        m0=data.Tr.m0;
        message = sprintf('Petri net system has %d places and %d transitions\nTime spent for creating it: %g secs', size(Pre,1),size(Pre,2),toc);
        uiwait(msgbox(message,'Robot Motion Toolbox','modal'));
        nplaces_orig = size(Pre,1);
        ntrans_orig = size(Pre,2);
        
        %crete the observation set
        N_r = length(data.RO);
        observ_set = data.Tr.OBS_set(1:size(data.Tr.OBS_set,1)-1,:);
        temp_cell=mat2cell( observ_set , ones(1,size(observ_set,1)) , size(observ_set,2) );  %duplicate observables of transition systems
        temp_obs=rmt_cartesian_product(temp_cell{:});  %possible observables, on rows (more robots can have the same observables, that's why we use carth product); observables will be labeled with indices of rows (in T.obs)
        temp_obs=unique(sort(temp_obs,2),'rows');   %sort rows and remove identical ones (they would have the same observable)
        for i=1:size(temp_obs,1)  %modify observables (keep at most one occurence of same prop on each row, and pad with zeros until length
            obs=unique(temp_obs(i,:));    %keep unique elements on each row, and pad wih zeros
            if length(obs)<size(temp_obs,2) %pad with zeros until number of propositions
                obs((end+1):size(temp_obs,2))=0;
            end
            temp_obs(i,:)=obs;
        end
        temp_obs=unique(sort(temp_obs,2),'rows');   %again remove identical rows (there may appear new repetitions, due to keeping only unique elements on each row and padding with zeros)
        temp_obs(end+1,:)=[length(data.Tr.props)+1 , zeros(1,size(temp_obs,2)-1)]; %dummy has index "ind_dummy", and pad with zeros after it
        
        if ~isfield(data,'B')
            B = rmt_create_buchi(data.formula, temp_obs);
        else
            choice2 = questdlg('Should the Buchi automaton be computed again?', ...
                'Robot Motion Toolbox', ...
                'Yes','No','Yes');
            if strcmpi(choice2,'Yes')
                B = rmt_create_buchi(data.formula, temp_obs);
            else
                B = data.B;
            end
        end
        data.B=B;
        set(gcf,'UserData',data);
        
        tic;
        [Pre,Post,m0,final_places] = rmt_construct_PN_ltl(Pre,Post,m0,data.Tr.props, data.B,temp_obs);%final places - places corresponding to the final states in the Buchi automaton
        message2 = sprintf('Petri net system including Buchi and observations has %d places and %d transitions\nTime spent for creating it: %g secs', size(Pre,1),size(Pre,2),toc);
        message = sprintf('%s\n%s', message, message2);
        uiwait(msgbox(message2,'Robot Motion Toolbox','modal'));
        
        data.Pre_full = Pre;
        data.Post_full = Post;
        set(gcf,'UserData',data);%to save data
        
        [A,b,Aeq,beq,cost] = rmt_construct_constraints_ltl(Pre,Post,m0, nplaces_orig, ntrans_orig, length(data.Tr.props) , 2*data.intermediateMarkings, final_places);
        
        %%%%%%%%%
        ctype='';
        for i = 1 : size(Aeq,1)
            ctype = sprintf('%sS',ctype);
        end
        for i = 1 : size(A,1)
            ctype = sprintf('%sU',ctype);
        end
        vartype = '';
        for i = 1 : 2*data.intermediateMarkings
            for j = 1 : size(Pre,1)
                vartype = sprintf('%sC',vartype); %put the markings as real
            end
            for j = 1 : size(Pre,2)
                vartype = sprintf('%sI',vartype); %put the sigma as integer
            end
        end
        
        tic;
        [xmin,~,exitflag] = cplexmilp(cost,A,b,Aeq,beq,[],[],[],zeros(1,size(A,2)),[],vartype);
        time = toc;
        if isempty(f)%no solution
            uiwait(errordlg('Error solving the ILP. The problem may have no feasible solution. Increase k!','Robot Motion Toolbox','modal'));
            return;
        end
        message2 = sprintf('\nTime of solving the MILP: %g secs\n', time);
        message = sprintf('%s%s', message, message2);
        uiwait(msgbox(message2,'Robot Motion Toolbox','modal'));
        
        message = sprintf('%s\n=======================================================',message);
        message = sprintf('%s\nInitial solution on the reduced transition system',message);
        message = sprintf('%s\n=======================================================\n',message);
        
        m0_old = m0;
        m0 = m0(1:nplaces_orig);
        m0_Buchi = m0_old(nplaces_orig+length(data.Tr.props)+1:end);
        m0_obs = m0_old(nplaces_orig+1:nplaces_orig+length(data.Tr.props));
        xm = xmin;
        xmin = [];
        ymin = [];
        fprintf(1,'\nInitial state of Buchi = %s',mat2str(find(m0_Buchi)));
        fprintf(1,'\n\tActive observations = %s',mat2str(find(m0_obs)));
        for i = 1 : 2*data.intermediateMarkings
            xmin = [xmin ; xm((i-1)*(size(Pre,1)+size(Pre,2))+1:(i-1)*(size(Pre,1)+size(Pre,2))+nplaces_orig)];%marking of places modeling the team
            %fprintf(1,'Marking of places modeling the team at step %d: %s',)
            xmin = [xmin ; xm((i-1)*(size(Pre,1)+size(Pre,2))+size(Pre,1)+1:(i-1)*(size(Pre,1)+size(Pre,2))+size(Pre,1)+ntrans_orig)];%firing vector of places modeling the team
            
            ymin = [ymin ; xm((i-1)*(size(Pre,1)+size(Pre,2))+nplaces_orig+length(data.Tr.props)+1:(i-1)*(size(Pre,1)+size(Pre,2))+size(Pre,1))];%marking of places modeling the Buchi
            ymin = [ymin ; xm((i-1)*(size(Pre,1)+size(Pre,2))+size(Pre,1)+ntrans_orig+1:(i-1)*(size(Pre,1)+size(Pre,2))+size(Pre,1)+size(Pre,2))];%firing vector of places modeling the team
            
            if (i/2 == round(i/2))
                trans_buchi=find([xm((i-1)*(size(Pre,1)+size(Pre,2))+size(Pre,1)+ntrans_orig+1:i*(size(Pre,1)+size(Pre,2)))]);
                input_place = find(Pre(:,trans_buchi+ntrans_orig)) ;
                input_place = input_place(input_place>nplaces_orig+length(data.Tr.props))-nplaces_orig-length(data.Tr.props); %take only the place of the buchi
                output_place = find(Post(:,trans_buchi+ntrans_orig));
                output_place = output_place(output_place>nplaces_orig+length(data.Tr.props))-nplaces_orig-length(data.Tr.props);
                fprintf(1,'\n Transition in Buchi from state %d to state %d with observation (%s)',input_place,output_place,mat2str(find([xm((i-1)*(size(Pre,1)+size(Pre,2))+nplaces_orig+1:(i-1)*(size(Pre,1)+size(Pre,2))++length(data.Tr.props)+nplaces_orig)])));
                fprintf(1,'\nState of Buchi in step %d = %s',i/2,mat2str(find([xm((i-1)*(size(Pre,1)+size(Pre,2))+nplaces_orig+length(data.Tr.props)+1:(i-1)*(size(Pre,1)+size(Pre,2))+size(Pre,1))])))
                %fprintf(1,'\n\tActive observations at step %d = %s',i/2,mat2str(find([xm((i-1)*(size(Pre,1)+size(Pre,2))+nplaces_orig+1:(i-1)*(size(Pre,1)+size(Pre,2))++length(data.Tr.props)+nplaces_orig)])));
            end
            
        end
        fprintf(1,'\n');
        message = sprintf('%s\nInitial marking [ %s ] = %s\n',message,mat2str(find(m0>eps*10^5)),mat2str(m0(m0>eps*10^5)));
        original_places{1} = [];
        temp = rmt_marking2places(m0);
        for j = 1 : length(temp)
            original_places{1} = union(original_places{1},data.Tr.Cells{temp(j)});
        end
        nplaces = nplaces_orig;
        ntrans = ntrans_orig;
        
        for i = 1 : 2*data.intermediateMarkings
            m = xmin((i-1)*(nplaces+ntrans)+1 : (i-1)*(nplaces+ntrans)+nplaces);
            fire = xmin((i-1)*(nplaces+ntrans)+nplaces+1 : (i-1)*(nplaces+ntrans)+nplaces+ntrans);
            if (max(fire) > 0)
                message = sprintf('%s\n============STEP %d =============\n',message,i);
                message=sprintf('%s\nMarking [ %s ] = %s\n',message,mat2str(find(m>eps*10^5)),mat2str(m(m>eps*10^5)));
                message = sprintf('%s\nSigma [ %s ] = %s\n',message,mat2str(find(fire>eps*10^5)),mat2str(fire(fire>eps*10^5)));
                %                 if (reduced == 0)
                %                     Run_cells = [Run_cells, rmt_marking2places(m)];  %visited cells (rows)
                %                 else
                temp = rmt_marking2places(m);
                original_places{i+1} = [];
                for j = 1 : length(temp)
                    original_places{i+1} = union(original_places{i+1},data.Tr.Cells{temp(j)});
                end
                %                end
            else
                original_places{i+1} = original_places{i};
            end
        end
        
        options = cplexoptimset('cplex');
        options.display='off';
        options.lpmethod=1;
        message = sprintf('%s\n=======================================================',message);
        message = sprintf('%s\nProject the solution to the initial transition system',message);
        message = sprintf('%s\n=======================================================\n',message);
        [Pre,Post] = rmt_construct_PN(data.T.adj);
        m0 = data.T.m0;
        steps = 1;
        message = sprintf('%s\nInitial marking [ %s ] = %s\n',message,mat2str(find(m0>eps*10^5)),mat2str(m0(m0>eps*10^5)));
        Run_cells = data.RO';%rmt_marking2places(m0);
        tic;
        for i = 1 : 2*data.intermediateMarkings
            m = xmin((i-1)*(nplaces+ntrans)+1 : (i-1)*(nplaces+ntrans)+nplaces);
            fire = xmin((i-1)*(nplaces+ntrans)+nplaces+1 : (i-1)*(nplaces+ntrans)+nplaces+ntrans);
            if (max(fire) > 0)
                % places = find(m>eps*10^5);%final marked places
                places = rmt_marking2places(m);%final marked places
                markings = m(m>eps*10^5); %number of tokens in final marked places
                temp = setdiff(1:size(Post,1),union(original_places{i},original_places{i+1}));
                TT = data.T;
                for j = 1 : length(temp)
                    TT.adj(temp(j),:)=0;
                    TT.adj(:,temp(j))=0;
                end
                [Pre_new,Post_new] = rmt_construct_PN(TT.adj);
                Aeq = [eye(size(Pre_new,1)) -Post_new+Pre_new]; %state equation: m = m0 + C \sigma
                beq = m0;
                %add constraints on the intermediate final marking
                % for j = 1 : length(places)
                for j = 1 : length(markings)
                    m_final = zeros(1,size(Pre_new,1));
                    m_final(data.Tr.Cells{places(j)}) = 1;
                    Aeq = [Aeq; m_final zeros(1,size(Pre_new,2))];
                    beq = [beq; markings(j)];
                end
                [X,~,exitflag] = cplexlp(ones(1,size(Pre_new,2)+size(Pre_new,1)),[],[],...
                    Aeq,beq,zeros(1,size(Pre_new,1)+size(Pre_new,2)),[],[],options);
                if ~(exitflag > 0)
                    uiwait(errordlg('Error solving LPP to project the solution!','Robot Motion Toolbox','modal'));
                    return;
                end
                
                [ret,Run_cells] = rmt_find_sequence(m0, round(X(size(Pre_new,1)+1:end)), Pre_new, Post_new,steps,Run_cells);
                steps = steps + sum(round(X(size(Pre,1)+1:end)))+1;
                fire = round(X(size(Pre_new,1)+1:end));
                disp(ret);
                m0 = round(X(1:size(Pre_new,1))); % final marking is the initial marking for the next step
            end
        end
        time = toc;
        message2 = sprintf('Total time for solving %d LPPs for projecting the solution: %g secs', ...
            2*data.intermediateMarkings, time);
        uiwait(msgbox(message2,'Robot Motion Toolbox','modal'));
        message = sprintf('%s\n%s',message,message2);
        
        cla(data.handle_env);
        %%Execution monitoring strategy starts from here:
        N_r = length(data.RO);
        current_pos=cell(1,N_r);
        for r=1:N_r
            current_pos{r}=data.initial{r};%R_trajs{r}(:,1); %current_pos - initial (current) position of robots (cell, current_pos{i} is a column vector with 2 rows)
        end
        rmt_represent_atomic_props(data.T.Vert,data.propositions);    %represent all atomic props and store handles
        
        %independent runs of each robot (no synchronizations, because of our def. of Boolean formulae)
        [Runs, ~] = rmt_robot_runs(data.T,Run_cells); %remove successive repetitions from individual discrete trajectories
        
        
        rob_traj = rmt_rob_cont_traj(data.T,Runs,data.initial);    %continuous trajectory of each robot
        message = sprintf('%s\nSOLUTION - runs of robots: \n',message);
        for j = 1 : length(Runs)
            message = sprintf('%s\nRobot %d: ',message,j);
            temp = Runs{j};
            for k = 1 : length(temp)-1
                message = sprintf('%s%d,',message,temp(k));
            end
            message = sprintf('%s%d',message,temp(length(temp)));
        end
        
        
        for r=1:length(rob_traj)    %plot trajectories of robots
            plot(rob_traj{r}(1,1),rob_traj{r}(2,1),data.rob_plot.line_color{r},...
                'Marker',data.rob_plot.marker{r},'LineWidth',data.rob_plot.line_width{r});
            plot(rob_traj{r}(1,:),rob_traj{r}(2,:),data.rob_plot.line_color{r},...
                'LineWidth',data.rob_plot.line_width{r});
            plot(rob_traj{r}(1,end),rob_traj{r}(2,end),data.rob_plot.line_color{r},...
                'Marker',data.rob_plot.marker{r},'LineWidth',data.rob_plot.line_width{r});
        end
        button = questdlg('Save the details to a text file?','Robot Motion Toolbox');
        if strcmpi(button,'Yes')
            [filename, pathname] = uiputfile('*.txt', 'Save experiments as');
            fileID = fopen(fullfile(pathname, filename),'w');
            fprintf(fileID,'%s',message);
            fclose(fileID);
        end
        
        set(gcf,'UserData',data);%to save data
        
        
    case 'save_work_env'
        data = get(gcf,'UserData');
        if ~isfield(data,'T')
            uiwait(errordlg(sprintf('\nCreate a partition first'),'Robot Motion Toolbox','modal'));
            return;
        end
        checkLabels = {'Save set of discrete states to variable named:' ...
            'Save regions of interest to variable named:'...
            'Save adjacency matrix to variable named:'...
            };
        varNames = {'C', 'O','Adj'};
        items = {data.T.Q, data.T.props, data.T.adj};
        export2wsdlg(checkLabels, varNames, items, 'Save Environment Information to Workspace');
    case 'save_work_pn'
        data = get(gcf,'UserData');
        if ~isfield(data,'T')
            uiwait(errordlg(sprintf('\nCreate a partition first'),'Robot Motion Toolbox','modal'));
            return;
        end
        checkLabels = {'Save Pre matrix to variable named:' ...
            'Save Post matrix to variable named:'...
            'Save m0 to variable named:'...
            };
        varNames = {'Pre', 'Post','m0'};
        [Pre,Post] = rmt_construct_PN(data.T.adj);
        items = {Pre, Post, data.T.m0};
        export2wsdlg(checkLabels, varNames, items, 'Save Petri Net model to Workspace');
        
        
        %% Part of Menu 'Setup' CPLEX
    case 'menu_cplex'
        data = get(gcf,'UserData');
        data.cplex_variable= 'true';
        set(gcf,'UserData',data);%to save data
        
        %% Part of Menu 'Setup' GLPK
    case 'menu_glpk'
        data = get(gcf,'UserData');
        data.cplex_variable= 'false';
        set(gcf,'UserData',data);%to save data
        
        
end    % switch
