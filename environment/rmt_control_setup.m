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
%   First version released on October, 2018.
%   Last modification October 28, 2018.
%   More information: http://webdiis.unizar.es/RMTool
% ============================================================================

function control=rmt_control_setup(control)

if strcmpi(control.robot,'Car-like')
    robot = 1;
else
    robot = 2;
end
if strcmpi(control.motion,'Pure-Pursuit')
    motion = 1;
else
    motion = 2;
end
answer = inputdlg({...
    sprintf('Sampling period (seconds):'),...
    sprintf('Max angular velocity (rad/s):'),...
    sprintf('Max linear velocity (m/s)'),...
    sprintf('Max steering angle (deg)'),...
    sprintf('Wheel radius'),...
    sprintf('Wheel base (m)'),...
    sprintf('Lookahead distance (int value)'),...
    sprintf('Robot type (1 - Car-like; 2 - Differential-drive)'),...
    sprintf('Motion Controller (1 - Pure-Pursuit; 2 - PI)')},...
    'Robot Motion Toolbox',...
    [1;1;1;1;1;1;1;1;1],{num2str(control.sampling_period,3),mat2str(control.max_ang_vel,3),...
    num2str(control.max_linear_vel,3),mat2str(control.max_steering,3),...
    mat2str(control.wheel_radius,3),mat2str(control.wheel_base,3),...
    mat2str(control.lookahead_distance,3),mat2str(robot,3),mat2str(motion)});
if (isempty(answer))
    return;
end
%Sampling period
input_val = answer{1};
todoOK = rmt_detect_error(input_val,0.01,2);
if (todoOK == 0)
    uiwait(errordlg(sprintf('\nValid range for sampling period is betweeen 0.01 and 2!'),'Robot Motion Toolbox','modal'));
    error('Valid range for sampling period is betweeen 0.01 and 2!');
else
    control.sampling_period = eval(input_val);
end
%Max angular velocity
input_val = char(answer{2});
todoOK = rmt_detect_error(input_val,0.1,5);
if (todoOK == 0)
    uiwait(errordlg(sprintf('\nValid range for Max angular velocity is betweeen 0.1 and 5!'),'Robot Motion Toolbox','modal'));
    error('Valid range for Max angular velocity is betweeen 0.1 and 5!');
else
    control.max_ang_vel = eval(input_val);
end
%Max linear velocity
input_val = char(answer{3});
todoOK = rmt_detect_error(input_val,0.1,10);
if (todoOK == 0)
    uiwait(errordlg(sprintf('\nValid range for Max linear velocity is betweeen 0.1 and 10!'),'Robot Motion Toolbox','modal'));
    error('Valid range for Max linear velocity is betweeen 0.1 and 10!');
else
    control.max_linear_vel = eval(input_val);
end
%Max steering angle
input_val = char(answer{4});
todoOK = rmt_detect_error(input_val,10,60);
if (todoOK == 0)
    uiwait(errordlg(sprintf('\nValid range for Max steering angle is betweeen 10 and 60!'),'Robot Motion Toolbox','modal'));
    error('Valid range for Max steering angle is betweeen 10 and 60!');
else
    control.max_steering = eval(input_val);
end
%Wheel radius
input_val = char(answer{5});
todoOK = rmt_detect_error(input_val,0.01,1.5);
if (todoOK == 0)
    uiwait(errordlg(sprintf('\nValid range for wheel radius is betweeen 0.01 and 1.5!'),'Robot Motion Toolbox','modal'));
    error('Valid range for wheel radius is betweeen 0.01 and 1.5!');
else
    control.wheel_radius = eval(input_val);
end
%Wheel base
input_val = char(answer{6});
todoOK = rmt_detect_error(input_val,0.1,2.5);
if (todoOK == 0)
    uiwait(errordlg(sprintf('\nValid range for wheel base is betweeen 0.1 and 2.5!'),'Robot Motion Toolbox','modal'));
    error('Valid range for wheel base is betweeen 0.1 and 2.5!');
else
    control.wheel_base = eval(input_val);
end
%Lookahead distance
input_val = char(answer{7});
todoOK = rmt_detect_error(input_val,1,20);
if (todoOK == 0)
    uiwait(errordlg(sprintf('\nValid range for lookahead distance is betweeen 1 and 20!'),'Robot Motion Toolbox','modal'));
    error('Valid range for lookahead distance is betweeen 1 and 20!');
else
    control.lookahead_distance = eval(input_val);
end
%Robot type
input_val = char(answer{8});
todoOK = rmt_detect_error(input_val,1,2);
if (todoOK == 0)
    uiwait(errordlg(sprintf('\nValid range for robot type is betweeen 1 and 2!'),'Robot Motion Toolbox','modal'));
    error('Valid range for robot type is betweeen 1 and 2!');
else
    temp = round(eval(input_val));
    if (temp == 1)
        control.robot='Car-like';
    else
        control.robot='Differential-drive';
    end
end

%Motion controller
input_val = char(answer{9});
todoOK = rmt_detect_error(input_val,1,2);
if (todoOK == 0)
    uiwait(errordlg(sprintf('\nValid range for motion controller is betweeen 1 and 2!'),'Robot Motion Toolbox','modal'));
    error('Valid range for motion controller is betweeen 1 and 2!');
else
    temp = round(eval(input_val));
    if (temp == 1)
        control.motion='Pure-Pursuit';
    else
        control.motion='PI';
    end
end
return;
