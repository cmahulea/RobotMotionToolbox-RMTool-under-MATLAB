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
%   Last modification December 29, 2015.
%   More information: http://webdiis.unizar.es/RMTool
% ============================================================================


%**********************************************************************
% PI controller 
% function [array_time, array_alpha, array_v, array_pos] = rmt_pure_pursuit(input_variables, ref_trajectory)
%**********************************************************************
function [array_time, array_alpha, ...
    array_v, array_pos] = rmt_pi_controller(input_variables,...
    ref_trajectory,pi_tuning,obstacles, Nobstacles)   

% Input variables
T = input_variables(1);%sampling period
traj_ini = [input_variables(2) input_variables(3) input_variables(4)];%starting point (x,y)
r = input_variables(5);%wheel radius
lv = input_variables(6);%wheel base (distance between rear wheel centres)
max_v = input_variables(7);%linear velocity
alpha_constraint = deg2rad(input_variables(8));%constraint on the steering angle
robot_model = input_variables(10);%car-like model or differential-drive model
threshold_goal = input_variables(11);%threshold to determine whether the goals has been reached or not.
max_ang_vel = input_variables(12);%max angular velocity
s = input_variables(13);%lookahead distance
frame_limits(1) = input_variables(14);
frame_limits(2) = input_variables(15);
frame_limits(3) = input_variables(16);
frame_limits(4) = input_variables(17);


%output variables
array_time = [0];
array_v = [0];
array_alpha = [0];
array_pos = [];

[xref,yref,timeref] = rmt_re_sample(ref_trajectory, T);

xref_new = [xref(1)];
yref_new = [yref(1)];
timeref_new = [timeref(1)];
cc = 2;
for(ii = 2:length(xref))
    if(mod(ii,2))
        xref_new(cc) = xref(ii);
        yref_new(cc) = yref(ii);
        timeref_new(cc) = timeref(ii);
        cc = cc + 1;
    end
end


%save datos.mat xref yref;
%[x1, y1] = control_pi(xref,yref);
%[x2, y2] = control_pi(xref_new,yref_new);

if(xref(2) == xref(3))
    xref = xref_new;
    yref = yref_new;
end

x(1) = traj_ini(1);
y(1) = traj_ini(2);
theta(1) = traj_ini(3);
ex = [];

% ******************************************************
% TUNING PARAMETERS OF THE PI CONTROLLER
% ******************************************************
%Kv = 0.8;%0.8;% Proportional gain (Kv * e)
%Ki = 0.5;% Integral gain (Ki * int(e))
%Kh = 0.2;% Gain multiplying the orientation error (Kh (theta_star - theta))
%dstar = 0.01;

Kv = pi_tuning(1);
Ki = pi_tuning(2);
Kh = pi_tuning(3);
dstar = pi_tuning(4);

%alpha_constraint = deg2rad(25);
%T = 0.1;
%lv = 0.25;
% ******************************************************

sum_ex = 0;

i = 1;
k = length(xref);
goal_ok = 0;
%Terminating conditions: we have chosen when all the reference points 
% are checked plus a manually fixed iterations or when the goal is obtained. 
max_iterations = k + 50;
%threshold_goal = 0.15;
while((i<=max_iterations)&&(goal_ok==0))    
    pos = [x(i) y(i) theta(i)];%actual position        
    if(pos(1) < frame_limits(1))        
        uiwait(errordlg(sprintf('\nOut of limits'),'Robot Motion Toolbox','modal')); 
        goal_ok = 1;
    end
    if(pos(2) < frame_limits(3))        
        uiwait(errordlg(sprintf('\nOut of limits'),'Robot Motion Toolbox','modal')); 
        goal_ok = 1;
    end
    if(pos(1) > frame_limits(2))        
        uiwait(errordlg(sprintf('\nOut of limits'),'Robot Motion Toolbox','modal')); 
        goal_ok = 1;
    end
    if(pos(2) > frame_limits(4))        
        uiwait(errordlg(sprintf('\nOut of limits'),'Robot Motion Toolbox','modal')); 
        goal_ok = 1;
    end
    
    for(ji=1:Nobstacles)
        [aa bb] = size(obstacles{ji});
        if(aa >= bb)
            if(isPointInPolygon([pos(1) pos(2)], obstacles{ji}))
                uiwait(errordlg(sprintf('\nThe robot hits one obstacle'),'Robot Motion Toolbox','modal')); 
                goal_ok = 1;
            end
        else
            if(isPointInPolygon([pos(1) pos(2)], obstacles{ji}'))
                uiwait(errordlg(sprintf('\nThe robot hits one obstacle'),'Robot Motion Toolbox','modal')); 
                goal_ok = 1;
            end
        end
    end
    
    if(goal_ok == 0)
        if(i>k)
                xrefi = xref(k);
                yrefi = yref(k);            
        else
                xrefi = xref(i);
                yrefi = yref(i);            
        end
        e_goal = sqrt((x(i)-xrefi)^2+(y(i)-yrefi)^2);    
        if(e_goal<threshold_goal)
            vstar = 0;
            alpha = 0;
        else
            if(i>k)
                xreff = xref(k);
                yreff = yref(k);
            else
                xreff = xref(i);
                yreff = yref(i);
            end
            ex(i) = sqrt((x(i)-xreff)^2+(y(i)-yreff)^2) - dstar;
            sum_ex = sum_ex+ex(i);
            vstar = Kv*ex(i)+Ki*sum_ex;            
            if(vstar>max_v || vstar< -max_v)
                vstar = max_v;
            end
            thetastar(i) = atan2((yreff-y(i)),(xreff-x(i)));
            etheta = function_angdiff(thetastar(i),theta(i));
            alpha = Kh * etheta;
            if(alpha>alpha_constraint || alpha< -alpha_constraint)
                alpha = alpha_constraint;
            end
        end
    else
            vstar = 0;
            alpha = 0;
    end%goal

%    x(i+1) = x(i) + T*(vstar*cos(theta(i)));
%    y(i+1) = y(i) + T*(vstar*sin(theta(i)));
%    theta(i+1) = theta(i) + T*(vstar*(tan(alpha)/lv));    
    
%     if(robot_model ==1)%car-like
%         x(i+1) = x(i) + T*(vstar*cos(theta(i)));
%         y(i+1) = y(i) + T*(vstar*sin(theta(i)));
%         theta(i+1) = theta(i) + T*(vstar*(tan(alpha)/lv));    
%     else%differential_drive
%         x(i+1) = x(i) + T*(vstar*cos(theta(i)));
%         y(i+1) = y(i) + T*(vstar*sin(theta(i)));
%         theta(i+1) = theta(i) + T*(vstar*(tan(alpha)/lv));    
%     end
    
     if(robot_model == 2)%Differential-drive robot        
        position0 = [x(i) y(i) theta(i)];
        v = vstar;
        v_l = (2*v-v*tan(-alpha))/2;        
        v_r = 2*v - v_l;       
        [position1,omega] = rmt_km_diff_drive_robot(position0,T,v_r,v_l,lv);        
        x(i+1) = position1(1);
        y(i+1) = position1(2); 
        %omega = position1(3);
        if(omega>max_ang_vel)
            omega = max_ang_vel;
        elseif(omega<-max_ang_vel)
            omega = -max_ang_vel;
        else
            omega = omega;
        end
        %position1(3) = position0(3) + T * ((v_l-v_r)/lv);
        theta(i+1) = theta(i) + omega;        
    else%robot_model == 1 (default model)
        %Car-like kinematic model
        position0 = [x(i) y(i) theta(i)];
        v = vstar;
        [position1,omega] = rmt_km_car_like_robot(position0,T,v,lv,alpha);
        x(i+1) = position1(1);
        y(i+1) = position1(2);
        %omega = position1(3);
        if(omega>max_ang_vel)
            omega = max_ang_vel;
        elseif(omega<-max_ang_vel)
            omega = -max_ang_vel;
        else
            omega = omega;
        end
        theta(i+1) = theta(i) + omega;        
     end

   
    array_time(i) =  i*T;%time
    array_alpha(i) = alpha;%steering angle
    array_v(i) = vstar;%linear velocity    
    array_pos(1,i) = pos(1);
    array_pos(2,i) = pos(2);
    array_pos(3,i) = pos(3);
    i = i+1;%counter
end%while

%fprintf('\n End of PI.\n');

 [w1,l1] = size(y);
 [w2,l2] = size(yref);
 diffy = l1 - l2;
 gg = round(l1/diffy);
 removalList = 1:gg:l1;
 h = y;
h(removalList) = [];

[w3,l3] = size(h);

while(l3 ~= l2)
    if(l3 > l2)
        temp = yref(end);
        yref = [yref temp];
        [w2,l2] = size(yref);       
    else
        temp = h(end);
        h = [h temp];
        [w3,l3] = size(h);       
    end    
end

    size(h);
    size(yref);
    RMSE1 = sqrt(mean((h - yref).^2));   
    
    
    
    
    [w1,l1] = size(x);
 [w2,l2] = size(xref);
 diffx = l1 - l2;
 gg = round(l1/diffx);
 removalList = 1:gg:l1;
 h = x;
h(removalList) = [];

[w3,l3] = size(h);

while(l3 ~= l2)
    if(l3 > l2)
        temp = xref(end);
        xref = [xref temp];
        [w2,l2] = size(xref);       
    else
        temp = h(end);
        h = [h temp];
        [w3,l3] = size(h);       
    end    
end

    size(h);
    size(xref);
    RMSE2 = sqrt(mean((h - xref).^2));    
resSS = sqrt(RMSE1^2 + RMSE2^2);
    
end%function

function d = function_angdiff(th1, th2)
   d = th1 - th2;
   d = mod(d+pi, 2*pi) - pi;
end

function [x,y] = control_pi(xref,yref)

dstar = 0.01;
x(1) = xref(1);
y(1) = yref(1);
theta(1) = 0;
ex = [];
v = 1;
vstar = 1;
Kv = 0.8;
Ki = 0.5;
Kh = 0.2;
sum_ex = 0;
alpha_constraint = deg2rad(25);
T = 0.1;
lv = 0.25;
for(i=1:length(xref))
    e_goal = sqrt((x(i)-xref(end))^2+(y(i)-yref(end))^2);    
    if(e_goal<0.1)
        vstar = 0;
        alpha = 0;
    else
        ex(i) = sqrt((x(i)-xref(i))^2+(y(i)-yref(i))^2) - dstar;
        sum_ex = sum_ex+ex(i);
        vstar = Kv*ex(i)+Ki*sum_ex;
        if(vstar>v || vstar< -v)
            vstar = v;
        end
        thetastar(i) = atan2((yref(i)-y(i)),(xref(i)-x(i)));
        etheta = function_angdiff(thetastar(i),theta(i));
        alpha = Kh * etheta;
        if(alpha>alpha_constraint || alpha< -alpha_constraint)
            alpha = alpha_constraint;
        end
    end
    x(i+1) = x(i) + T*(vstar*cos(theta(i)));
    y(i+1) = y(i) + T*(vstar*sin(theta(i)));
    theta(i+1) = theta(i) + T*(vstar*(tan(alpha)/lv));    
end
end

