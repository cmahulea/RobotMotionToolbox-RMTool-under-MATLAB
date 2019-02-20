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
%   First version released on February, 2019.
%   Last modification February 21, 2019.
%   More information: http://webdiis.unizar.es/RMTool
% ============================================================================

function rmt_export_environment_to_workspace

data = get(gcf,'UserData');
checkLabels = {};
index = 0;

if isfield(data,'obstacles')
    if ~isempty(data.obstacles)
        index = length(checkLabels)+1;
        checkLabels{index} = 'Save the vertices of obstacles/regions of interest to variable named:';
        varNames{index} = 'Vertices';
        items{index} = data.obstacles;
    end
end

if isfield(data,'frame_limits')
    index = length(checkLabels)+1;
    checkLabels{index} = 'Save the environment limits to variable named:';
    varNames{index} = 'Env_limits';
    items{index} = data.frame_limits;
end

if isfield(data,'initial')
    index = length(checkLabels)+1;
    checkLabels{index} = 'Save initial positions of the robots to variable named:';
    varNames{index} = 'R_init';
    items{index} = data.initial;
end

if isfield(data,'RO')
    if ~isempty(data.RO)
        index = index + 1;
        checkLabels{index} = 'Save initial regions of robots to variable named:';
        varNames{index} = 'RO';
        items{index} = data.RO;
    end
end

if isfield(data,'final')
    index = length(checkLabels)+1;
    checkLabels{index} = 'Save goal positions of the robots to variable named:';
    varNames{index} = 'R_goal';
    items{index} = data.final;
end

if isfield(data,'trajectory')
    if ~isempty(data.trajectory)
        index = index + 1;
        checkLabels{index} = 'Save robot trajectories to variable named:';
        varNames{index} = 'R_trajs';
        items{index} = data.trajectory;
    end
end

if isfield(data,'T')
    index = length(checkLabels)+1;
    checkLabels{index} = 'Save set of discrete states to variable named:';
    varNames{index} = 'C';
    items{index} = data.T.Q;
    index = index + 1;
    checkLabels{index} = 'Save the discrete sets of regions of interest to variable named:';
    varNames{index} = 'O';
    items{index} = data.T.props;
    index = index + 1;
    checkLabels{index} = 'Save adjacency matrix to variable named:';
    varNames{index} = 'Adj';
    items{index} = data.T.adj;
    %%construct PN model
    [Pre,Post] = rmt_construct_PN(data.T.adj);
    index = index + 1;
    checkLabels{index} = 'Save Pre matrix to variable named:';
    varNames{index} = 'Pre';
    items{index} = Pre;
    index = index + 1;
    checkLabels{index} = 'Save Post matrix to variable named:';
    varNames{index} = 'Post';
    items{index} = Post;
    index = index + 1;
    checkLabels{index} = 'Save initial marking m0 to variable named:';
    varNames{index} = 'm0';
    items{index} = m0;
end

if (index == 0)
    uiwait(errordlg(sprintf('\nCreate a partition first'),'Robot Motion Toolbox','modal'));
    return;
else
    export2wsdlg(checkLabels, varNames, items, 'Save Environment Information to Workspace');
    return;
end
