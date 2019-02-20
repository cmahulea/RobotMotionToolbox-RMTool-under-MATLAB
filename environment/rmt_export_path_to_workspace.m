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

function rmt_export_path_to_workspace

data = get(gcf,'UserData');
index = 0;

if isfield(data,'initial')
    index = index + 1;
    checkLabels{index} = 'Save initial points of the robots to variable named:';
    varNames{index} = 'R_init';
    items{index} = data.initial;
end

if isfield(data,'obstacles')
    if ~isempty(data.obstacles)
        index = index + 1;
        checkLabels{index} = 'Save obstacles / regions of interest to variable named:';
        varNames{index} = 'Regions';
        items{index} = data.obstacles;
    end
end

if isfield(data,'formula')
    if ~isempty(data.formula)
        index = index + 1;
        checkLabels{index} = 'Save LTL formula to variable named:';
        varNames{index} = 'Formula';
        items{index} = data.formula;
    end
end

if isfield(data,'Bool_formula')
    if ~isempty(data.Bool_formula)
        index = index + 1;
        checkLabels{index} = 'Save Boolean formula to variable named:';
        varNames{index} = 'Bool_Formula';
        items{index} = data.Bool_formula;
    end
end

if isfield(data,'T')
    if ~isempty(data.T)
        index = index + 1;
        checkLabels{index} = 'Save transition system of one robot to variable named:';
        varNames{index} = 'T';
        items{index} = data.T;
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
        items{index} = data.T.m0;
    end
end

if isfield(data,'Tg')
    if ~isempty(data.Tg)
        index = index + 1;
        checkLabels{index} = 'Save transition system of the robot team to variable named:';
        varNames{index} = 'Tg';
        items{index} = data.Tg;
    end
end

if isfield(data,'B')
    if ~isempty(data.B)
        index = index + 1;
        checkLabels{index} = 'Save Buchi automaton to variable named:';
        varNames{index} = 'B';
        items{index} = data.B;
    end
end

if isfield(data,'Pg')
    if ~isempty(data.Pg)
        index = index + 1;
        checkLabels{index} = 'Save product automaton of team and Buchi automaton to variable named:';
        varNames{index} = 'Pg';
        items{index} = data.Pg;
    end
end

if isfield(data,'propositions')
    if ~isempty(data.propositions)
        index = index + 1;
        checkLabels{index} = 'Save regions of atomic propositions to variable named:';
        varNames{index} = 'propositions';
        items{index} = data.propositions;
    end
end

if isfield(data,'RO')
    if ~isempty(data.RO)
        index = index + 1;
        checkLabels{index} = 'Save initial regions of robots to variable named:';
        varNames{index} = 'RO';
        items{index} = data.RO;
    end
end

if isfield(data,'trajectory')
    if ~isempty(data.trajectory)
        index = index + 1;
        checkLabels{index} = 'Save robot trajectories to variable named:';
        varNames{index} = 'R_trajs';
        items{index} = data.trajectory;
    end
end

if (index == 0)
    uiwait(errordlg(sprintf('\nCreate a partition first'),'Robot Motion Toolbox','modal'));
    return;
else
    export2wsdlg(checkLabels, varNames, items, 'Save Simulation Results to Workspace');
    return
end