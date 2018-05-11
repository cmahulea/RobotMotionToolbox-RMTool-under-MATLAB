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
%   Last modification February 19, 2018.
%   More information: http://webdiis.unizar.es/RMTool
% ============================================================================

% Function to create Random Enviroment
function [objects,initial_point,final_point] = rmt_random_env(handle_axes,obs_no,obs_size,bound,dest_no,rob_no,limits,colors)
axes(handle_axes);
axis(limits);
hold on
grid on

center=zeros(2,obs_no);
objects={};
objects_bound={};

initial_point={};
initial_bound={};

final_point={};
final_bound={};

%% Obstacles

for i=1:obs_no 
    ax=limits(1)+obs_size;
    bx=limits(2)-obs_size;
    ay=limits(3)+obs_size;
    by=limits(4)-obs_size;    
    cx=ax+(bx-ax)*rand(1);
    cy=ay+(by-ay)*rand(1);
    center(:,i)=[cx;cy];
    
    pts=2+randi(20);
    apx=cx-obs_size;
    bpx=cx+obs_size;
    apy=cy-obs_size;
    bpy=cy+obs_size;
    
    apxb=apx-bound;
    bpxb=bpx+bound;
    apyb=apy-bound;
    bpyb=bpy+bound;
    
    objects_bound{i}=[apxb bpxb bpxb apxb; apyb apyb bpyb bpyb];

    for j=1:pts
        px=apx+(bpx-apx)*rand(1);
        py=apy+(bpy-apy)*rand(1);
        objects{i}(:,j)=[px;py];
    end
    
    %creating convex obstacles & drawing them
    k=convhull(objects{i}(1,:),objects{i}(2,:));
    objects{i}=objects{i}(:,k(1:length(k)-1));
    pause(0.3)
    fill(objects{i}(1,:),objects{i}(2,:),'b-','FaceAlpha',0.5); %or functia patch (similara cu fill)
end


%% Initial and final points

for k=1:rob_no %Initial points
    not=1;
    while not>=1
        not=0;
        sx=limits(1)+(limits(2)-limits(1))*rand(1);
        sy=limits(3)+(limits(4)-limits(3))*rand(1);
        initial_point{k}=[sx,sy];
        
        %Objetos
        for o=1:length(objects)
            if inpolygon(sx,sy,objects_bound{o}(1,:),objects_bound{o}(2,:))
                not=not+1;
            end  
        end   
        
        %Puntos iniciales
        if k>1
            for t=1:(k-1)
                if inpolygon(sx,sy,initial_bound{t}(1,:),initial_bound{t}(2,:))
                    not=not+1;
                end      
            end
        end 
        
    end
    
    initial_bound{k}=[sx-bound sx+bound sx+bound sx-bound;...
                      sy-bound sy-bound sy+bound sy+bound];
end

for k=1:dest_no %Final points
    not=1;
    while not>=1
        not=0;
        dx=limits(1)+(limits(2)-limits(1))*rand(1);
        dy=limits(3)+(limits(4)-limits(3))*rand(1);
        final_point{k}=[dx,dy];
        
        %Objetos
        for o=1:length(objects)
            if inpolygon(dx,dy,objects_bound{o}(1,:),objects_bound{o}(2,:))
                not=not+1;
            end
        end   


        %Puntos iniciales
        for t=1:length(initial_point)
            if inpolygon(dx,dy,initial_bound{t}(1,:),initial_bound{t}(2,:))
                not=not+1;
            end
        end
        
        %Puntos finales
        if k>1
            for t=1:(k-1)
                if inpolygon(dx,dy,final_bound{t}(1,:),final_bound{t}(2,:))
                    not=not+1;
                end
            end
        end
        
    end  
    final_bound{k}=[dx-bound dx+bound dx+bound dx-bound;...
                    dy-bound dy-bound dy+bound dy+bound];
end
   

%% Plotting points

for i=1:length(initial_point)   
    plot(initial_point{i}(1),initial_point{i}(2),'color',colors{i},'marker','o','Linewidth',2);
    text((initial_point{i}(1)+0.2),(initial_point{i}(2)+0.2),{num2str(i)});
    hold on;
end

pause(0.1)
for i=1:length(final_point)
    plot(final_point{i}(1),final_point{i}(2),'color','k','marker','x','Linewidth',2);
    text((final_point{i}(1)+0.2),(final_point{i}(2)+0.2),{char(64+i)});
    hold on;
end
pause(0.1)

end




