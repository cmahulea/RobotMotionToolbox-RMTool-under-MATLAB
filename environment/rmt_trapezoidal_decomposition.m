%    This is part of RMTool - Robot Motion Toolbox, for Matlab 2010b or newer.
%
%    Copyright (C) 2016 RMTool developing team, with support of Mihai Obada. For people, details and citing 
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

function [C,adj,varargout]=rmt_trapezoidal_decomposition(objects,env_bounds)
%doesn't work if there are two vertices with the same x coordinate (error1) or if obstacles are overlapping (error2)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%PI: descompunerea unei suprafete cu obstacole in 
%suprafete de tip trapez, crearea matricei de adiacenta,
%afisarea traseului printre obiecte
%uses "curveintersect" function, Copyright (c) 2009, Sebastian H?lz, All rights reserved.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%adapt obstacles to be used by trapez decomposition (create necessary matrices, etc.)
n=length(objects);

x_max = env_bounds(2);
y_max = env_bounds(4);

noise_x=min(x_max/50,y_max/50);
while 1 %alter x_values of vertices having the same x-coordinate
    vertices=[];
    for i=1:n,
        vertices=[vertices; objects{i}' , i*ones(size(objects{i},2),1)];
    end
    if length(unique(vertices(:,1)))==size(vertices,1)  %no identical repeated x-values
        break;
    end
    [ignore, ind] = unique(vertices(:,1),'first');
    ind=setdiff( 1:size(vertices(:,1)), ind);   %indices of points that should have x value altered (is identical with others)
    for i=1:length(ind)
        obj_nr=vertices(ind(i),3);
        [ignore, loc]=ismember(vertices(ind(i),1:2), objects{obj_nr}', 'rows');
        objects{obj_nr}(1,loc)=objects{obj_nr}(1,loc)+noise_x*(rand+0.5);   %alter x-value
    end
end
% for i=1:n
%     k=convhull(objects{i}(1,:),objects{i}(2,:));
%     objects{i}=objects{i}(:,k(1:length(k)-1));
% end

TT=[];
for i=1:n,
    R{i}=[[objects{i}'];[objects{i}(:,1)]'];
    TT=[TT;R{i} i*ones(size(R{i},1),1)];
    minim(i)=min(R{i}(:,1));
    maxim(i)=max(R{i}(:,1));
end
[TTs,In]=sort(TT(:,1));
TTd=TT(In,:);
[aa,bb]=size(TTd);
TTd=unique(TTd,'rows');
tti=unique(TTd(:,1),'rows');
[a,b]=size(TTd);
[atti,btti]=size(tti);
if a~=(atti)
    disp('error1')
else
    XX=[];
    YY=[];
    for ana=1:(n-1)
        for anail=(ana+1):n
         x1=R{ana}(:,1);
         y1=R{ana}(:,2);   
         x2=R{anail}(:,1);
         y2=R{anail}(:,2);
         [x,y]=curveintersect(x1,y1,x2,y2);
         sz=size(x,1);
         if sz~=0
            YY=[YY;y anail*ones(sz,1)];
         end  
        end
    end
    if size(YY,1)~=0
        disp('error2')
    end
end

%% Impartirea in trapeze-part1(liniile verticale)
DD=[0 0 0;0 y_max 0];   
for i=1:a
x1=[TTd(i,1);TTd(i,1)];
y1=[0;y_max];
XX=[];
YY=[];

    for j=1:n
        x2=R{j}(:,1);
        y2=R{j}(:,2);
        [x,y]=curveintersect(x1,y1,x2,y2);
        sz=size(x,1);
        if sz~=0
        YY=[YY;y j*ones(sz,1)];
        end
    end
    
    YY=unique(YY,'rows');
    YY=sortrows(YY);
    dim=size(YY,1);
    
    if dim==1
        a=TTd(i,3);
%         line([TTd(i,1);TTd(i,1)],[0;y_max]);
            if TTd(i,1)==minim(a)
                DD=[DD;[TTd(i,1) 0 0];[TTd(i,1) y_max 0]];                
                DD=[DD;[TTd(i,1) 0 0];[TTd(i,1) TTd(i,2) TTd(i,3)]];
                DD=[DD;[TTd(i,1) TTd(i,2) TTd(i,3)];[TTd(i,1) y_max 0]];
            end
            if TTd(i,1)==maxim(a)
                DD=[DD;[TTd(i,1) 0 0];[TTd(i,1) TTd(i,2) TTd(i,3)]];                
                DD=[DD;[TTd(i,1) TTd(i,2) TTd(i,3)];[TTd(i,1) y_max 0]];
                DD=[DD;[TTd(i,1) 0 0];[TTd(i,1) y_max 0]];                
            end
    end
    
    if dim==2
        if TTd(i,2)==YY(1)
%             line([TTd(i,1);TTd(i,1)],[0;TTd(i,2)]);
            DD=[DD;[TTd(i,1) 0 0];[TTd(i,1) TTd(i,2) TTd(i,3)]];            
        else
%             line([TTd(i,1);TTd(i,1)],[y_max;TTd(i,2)]);            
            DD=[DD;[TTd(i,1) TTd(i,2) TTd(i,3)];[TTd(i,1) y_max 0]];
        end
    end

    if dim>2
        poz=find(YY==TTd(i,2),1);
        if isempty(poz)
            [ignore, poz]=min(abs(YY(:)-TTd(i,2)));
        end
        if poz==1
            if mod(dim,2)==1
%                 line([TTd(i,1);TTd(i,1)],[0;YY(poz+1)]);
                a=TTd(i,3);
                if TTd(i,1)==maxim(a)                    
                    DD=[DD;[TTd(i,1) 0 0];[TTd(i,1) YY(poz,1) YY(poz,2)]];
                    DD=[DD;[TTd(i,1) YY(poz,1) YY(poz,2)];[TTd(i,1) YY(poz+1,1) YY(poz+1,2)]];                
                    DD=[DD;[TTd(i,1) 0 0];[TTd(i,1) YY(poz+1,1) YY(poz+1,2)]];                                
                end
                if TTd(i,1)==minim(a)
                    DD=[DD;[TTd(i,1) 0 0];[TTd(i,1) YY(poz+1,1) YY(poz+1,2)]];                                
                    DD=[DD;[TTd(i,1) 0 0];[TTd(i,1) YY(poz,1) YY(poz,2)]];
                    DD=[DD;[TTd(i,1) YY(poz,1) YY(poz,2)];[TTd(i,1) YY(poz+1,1) YY(poz+1,2)]];                
                end
            else
%                 line([TTd(i,1);TTd(i,1)],[0;YY(poz)]);
                DD=[DD;[TTd(i,1) 0 0];[TTd(i,1) YY(poz,1) YY(poz,2)]];
            end
        end
        if poz==dim
            if mod(dim,2)==1
%                 line([TTd(i,1);TTd(i,1)],[y_max;YY(poz-1)]);
                a=TTd(i,3);
                if TTd(i,1)==maxim(a)                               
                    DD=[DD;[TTd(i,1) YY(poz-1,1) YY(poz-1,2)];[TTd(i,1) YY(poz,1) YY(poz,2)]];
                    DD=[DD;[TTd(i,1) YY(poz,1) YY(poz,2)];[TTd(i,1) y_max 0]];                           
                    DD=[DD;[TTd(i,1) YY(poz-1,1) YY(poz-1,2)];[TTd(i,1) y_max 0]];                     
                end
                if TTd(i,1)==minim(a)
                    DD=[DD;[TTd(i,1) YY(poz-1,1) YY(poz-1,2)];[TTd(i,1) y_max 0]];                                
                    DD=[DD;[TTd(i,1) YY(poz-1,1) YY(poz-1,2)];[TTd(i,1) YY(poz,1) YY(poz,2)]];
                    DD=[DD;[TTd(i,1) YY(poz,1) YY(poz,2)];[TTd(i,1) y_max 0]];                      
                end           
            else
%                 line([TTd(i,1);TTd(i,1)],[y_max;YY(poz)]);                
                DD=[DD;[TTd(i,1) YY(poz,1) YY(poz,2)];[TTd(i,1) y_max 0]];                
            end
        end

        if ((poz~=1)&&(poz~=dim))
            if mod(poz,2)==0
%                 line([TTd(i,1);TTd(i,1)],[YY(poz);YY(poz+1)]);
                DD=[DD;[TTd(i,1) YY(poz,1) YY(poz,2)];[TTd(i,1) YY(poz+1,1) YY(poz+1,2)]];                
            else
                a=TTd(i,3);
                if (TTd(i,1)==maxim(a))||(TTd(i,1)==minim(a))
%                     line([TTd(i,1);TTd(i,1)],[YY(poz-1);YY(poz+1)]);                    
                    if TTd(i,1)==minim(a)
                        DD=[DD;[TTd(i,1) YY(poz-1,1) YY(poz-1,2)];[TTd(i,1) TTd(i,2) TTd(i,3)]];
                        DD=[DD;[TTd(i,1) TTd(i,2) TTd(i,3)];[TTd(i,1) YY(poz+1,1) YY(poz+1,2)]];                        
                        DD=[DD;[TTd(i,1) YY(poz-1,1) YY(poz-1,2)];[TTd(i,1) YY(poz+1,1) YY(poz+1,2)]];
                    end
                    if TTd(i,1)==maxim(a)
                        DD=[DD;[TTd(i,1) YY(poz-1,1) YY(poz-1,2)];[TTd(i,1) YY(poz+1,1) YY(poz+1,2)]];                        
                        DD=[DD;[TTd(i,1) YY(poz-1,1) YY(poz-1,2)];[TTd(i,1) TTd(i,2) TTd(i,3)]];                        
                        DD=[DD;[TTd(i,1) TTd(i,2) TTd(i,3)];[TTd(i,1) YY(poz+1,1) YY(poz+1,2)]];                        
                    end                                      
                else
%                     line([TTd(i,1);TTd(i,1)],[YY(poz-1);YY(poz)]);
                    DD=[DD;[TTd(i,1) YY(poz-1,1) YY(poz-1,2)];[TTd(i,1) YY(poz,1) YY(poz,2)]];                    
                end
            end        
        end
    end
end

%% Impartirea in trapeze-part2(suprafetele de tip trapez)
DD=[DD;[x_max 0 0];[x_max y_max 0]];
[noi,noc]=size(DD);

k=1;

Trace{k}=DD(1:4,1:2);
DD(1,:)=[-1 -1 0];
DD(2,:)=[-1 -1 0];
DD(3,:)=[-1 -1 0];
DD(4,:)=[-1 -1 0];
% text(sum(Trace{k}(:,1))/4,sum(Trace{k}(:,2))/4,int2str(k));    
m=convhull([Trace{k}(1,1);Trace{k}(2,1);Trace{k}(3,1);Trace{k}(4,1);Trace{k}(1,1)],[Trace{k}(1,2);Trace{k}(2,2);Trace{k}(3,2);Trace{k}(4,2);Trace{k}(1,2)]);
Trace{k}=Trace{k}(m(1:(end-1)),:);
% ty=patch([Trace{k}(1,1);Trace{k}(2,1);Trace{k}(3,1);Trace{k}(4,1);Trace{k}(1,1)],[Trace{k}(1,2);Trace{k}(2,2);Trace{k}(3,2);Trace{k}(4,2);Trace{k}(1,2)],'b');
% set(ty,'FaceAlpha',0.15);
[aio,bio]=sort(Trace{k}(:,1));
Trace{k}=Trace{k}(bio,:);
aio=[];bio=[];
[aio,bio]=sort(Trace{k}(1:2,2));
Trace{k}(1:2,2)=Trace{k}(bio,2);
if Trace{k}(3,2)<Trace{k}(4,2)
    aux=Trace{k}(3,2);
    Trace{k}(3,2)=Trace{k}(4,2);
    Trace{k}(4,2)=aux;
end
Trace{k}(5,:)=Trace{k}(1,:);
k=k+1;

Trace{k}=DD(noi-3:noi,1:2);
DD(noi-3,:)=[-1 -1 0];
DD(noi-2,:)=[-1 -1 0];
DD(noi-1,:)=[-1 -1 0];
DD(noi-0,:)=[-1 -1 0];
% text(sum(Trace{k}(:,1))/4,sum(Trace{k}(:,2))/4,int2str(k));    
m=convhull([Trace{k}(1,1);Trace{k}(2,1);Trace{k}(3,1);Trace{k}(4,1);Trace{k}(1,1)],[Trace{k}(1,2);Trace{k}(2,2);Trace{k}(3,2);Trace{k}(4,2);Trace{k}(1,2)]);
Trace{k}=Trace{k}(m(1:(end-1)),:);
% ty=patch([Trace{k}(1,1);Trace{k}(2,1);Trace{k}(3,1);Trace{k}(4,1);Trace{k}(1,1)],[Trace{k}(1,2);Trace{k}(2,2);Trace{k}(3,2);Trace{k}(4,2);Trace{k}(1,2)],'b');
% set(ty,'FaceAlpha',0.15);
[aio,bio]=sort(Trace{k}(:,1));
Trace{k}=Trace{k}(bio,:);
aio=[];bio=[];
[aio,bio]=sort(Trace{k}(1:2,2));
Trace{k}(1:2,2)=Trace{k}(bio,2);
if Trace{k}(3,2)<Trace{k}(4,2)
    aux=Trace{k}(3,2);
    Trace{k}(3,2)=Trace{k}(4,2);
    Trace{k}(4,2)=aux;
end
Trace{k}(5,:)=Trace{k}(1,:);
k=k+1;

ANA=DD;
ANAIL=DD;

poz=find(DD(5:noi-4,2)==0);
sz=size(poz,1);
for i=1:sz-1
    if DD(4+poz(i),1)~=DD(4+poz(i+1),1)
    Trace{k}=[DD(4+poz(i),1:2);DD(4+poz(i)+1,1:2);DD(4+poz(i+1),1:2);DD(4+poz(i+1)+1,1:2)];
%     text(sum(Trace{k}(:,1))/4,sum(Trace{k}(:,2))/4,int2str(k));        
    m=convhull([Trace{k}(1,1);Trace{k}(2,1);Trace{k}(3,1);Trace{k}(4,1);Trace{k}(1,1)],[Trace{k}(1,2);Trace{k}(2,2);Trace{k}(3,2);Trace{k}(4,2);Trace{k}(1,2)]);
    Trace{k}=Trace{k}(m(1:(end-1)),:);
%     ty=patch([Trace{k}(1,1);Trace{k}(2,1);Trace{k}(3,1);Trace{k}(4,1);Trace{k}(1,1)],[Trace{k}(1,2);Trace{k}(2,2);Trace{k}(3,2);Trace{k}(4,2);Trace{k}(1,2)],'b');
%     set(ty,'FaceAlpha',0.15);
    [aio,bio]=sort(Trace{k}(:,1));
    Trace{k}=Trace{k}(bio,:);
    [aio,bio]=sort(Trace{k}(1:2,2));
    Trace{k}(1:2,2)=Trace{k}(bio,2);  
    if Trace{k}(3,2)<Trace{k}(4,2)
        aux=Trace{k}(3,2);
        Trace{k}(3,2)=Trace{k}(4,2);
        Trace{k}(4,2)=aux;
    end
    Trace{k}(5,:)=Trace{k}(1,:);
    k=k+1;
    i=i+1;
    end
    ANA(4+poz(i),3)=0;
    ANA(4+poz(i)+1,3)=0;
end
ANA(4+poz(i),3)=0;
ANA(4+poz(i)+1,3)=0;

poz=find(DD(5:noi-4,2)==y_max);
sz=size(poz,1);
for i=1:sz-1
    if DD(4+poz(i),1)~=DD(4+poz(i+1),1)
        if ((DD(4+poz(i)-1,2)~=0)&&(DD(4+poz(i+1)-1,2)~=0))
    Trace{k}=[DD(4+poz(i)-1,1:2);DD(4+poz(i),1:2);DD(4+poz(i+1)-1,1:2);DD(4+poz(i+1),1:2)];
%     text(sum(Trace{k}(:,1))/4,sum(Trace{k}(:,2))/4,int2str(k));    
    m=convhull([Trace{k}(1,1);Trace{k}(2,1);Trace{k}(3,1);Trace{k}(4,1);Trace{k}(1,1)],[Trace{k}(1,2);Trace{k}(2,2);Trace{k}(3,2);Trace{k}(4,2);Trace{k}(1,2)]);
    Trace{k}=Trace{k}(m(1:(end-1)),:);
%     ty=patch([Trace{k}(1,1);Trace{k}(2,1);Trace{k}(3,1);Trace{k}(4,1);Trace{k}(1,1)],[Trace{k}(1,2);Trace{k}(2,2);Trace{k}(3,2);Trace{k}(4,2);Trace{k}(1,2)],'b');
%     set(ty,'FaceAlpha',0.15);
    [aio,bio]=sort(Trace{k}(:,1));
    Trace{k}=Trace{k}(bio,:);
    [aio,bio]=sort(Trace{k}(1:2,2));
    Trace{k}(1:2,2)=Trace{k}(bio,2);  
    if Trace{k}(3,2)<Trace{k}(4,2)
        aux=Trace{k}(3,2);
        Trace{k}(3,2)=Trace{k}(4,2);
        Trace{k}(4,2)=aux;
    end
    Trace{k}(5,:)=Trace{k}(1,:);
    k=k+1;
    i=i+1;
       end
    end
    ANAIL(4+poz(i),3)=0;
    ANAIL(4+poz(i)-1,3)=0;
end
ANAIL(4+poz(i),3)=0;
ANAIL(4+poz(i)-1,3)=0;

for i=1:sz
  if ((ANA(i,3)~=0)&&(ANAIL(i,3)==0)||(ANA(i,3)==0)&&(ANAIL(i,3)~=0))
      DD(i,3)=0;
  end
end

DD=DD(5:noi-4,:);
DD=unique(DD,'rows');
m=find(DD(:,3));
DD=DD(m,:);
sz=size(DD,1);
for i=1:sz-3
    for j=i+1:sz-2
        if (DD(i,1)==DD(j,1))
            if(DD(i,3)~=DD(j,3))
            for m=j+1:sz-1
                for n=m+1:sz
                    if (DD(m,1)==DD(n,1))
                        if(DD(i,3)==DD(m,3))&&(DD(j,3)==DD(n,3))
                            if (i<j)&&(j<m)&&(m<n)
                            Trace{k}=[DD(i,1:2);DD(j,1:2);DD(m,1:2);DD(n,1:2)];
%                             text(sum(Trace{k}(:,1))/4,sum(Trace{k}(:,2))/4,int2str(k));    
                            o=convhull([Trace{k}(1,1);Trace{k}(2,1);Trace{k}(3,1);Trace{k}(4,1);Trace{k}(1,1)],[Trace{k}(1,2);Trace{k}(2,2);Trace{k}(3,2);Trace{k}(4,2);Trace{k}(1,2)]);
                            Trace{k}=Trace{k}(o(1:(end-1)),:);
%                             ty=patch([Trace{k}(1,1);Trace{k}(2,1);Trace{k}(3,1);Trace{k}(4,1);Trace{k}(1,1)],[Trace{k}(1,2);Trace{k}(2,2);Trace{k}(3,2);Trace{k}(4,2);Trace{k}(1,2)],'b');
%                             set(ty,'FaceAlpha',0.15);
                            [aio,bio]=sort(Trace{k}(:,1));
                            Trace{k}=Trace{k}(bio,:);
                            [aio,bio]=sort(Trace{k}(1:2,2));
                            Trace{k}(1:2,2)=Trace{k}(bio,2);
                           if Trace{k}(3,2)<Trace{k}(4,2)
                                aux=Trace{k}(3,2);
                                Trace{k}(3,2)=Trace{k}(4,2);
                                Trace{k}(4,2)=aux;
                            end
                            Trace{k}(5,:)=Trace{k}(1,:);
                            k=k+1;
                            n=sz+1;
                            m=sz;
                            j=sz-1;
                            end
                        end
                    end
                end
            end
            end
        end
     end
end

%% Matricea de adiacenta
k=k-1;
adj=sparse(eye(k)); %self transitions in every cell

if nargout>2    %if desired, compute middle points between adjacent cells (useful for an angular path finding); avoid reutrn a cell, because of memory usage:
    middle_X=sparse(k,k);   %element (i,j) is zero if i=j or if cells i and j are not adjacent, and otherwise it contains X-coordinate for middle segment between i and j
    middle_Y=sparse(k,k);
    if nargout>4    %return also common line segments shared by adjacent cells
        com_F=cell(k,k);    %com_F{i,j} is a matrix with vertices of line segment shared by adjacent cells i,j; vertices are on columns
    end
end

for i=1:(k-1)
    for j=(i+1):k
        if (Trace{i}(4,1)==Trace{j}(1,1)) || (Trace{i}(1,1)==Trace{j}(4,1))
%                 xci=sum(Trace{i}(1:4,1))/4;
%                 yci=sum(Trace{i}(1:4,2))/4;
%                 xcj=sum(Trace{j}(1:4,1))/4;
%                 ycj=sum(Trace{j}(1:4,2))/4;
%                 dist=sqrt((xci-xcj)^2+(yci-ycj)^2);
%                 MAT(i,j)=dist;
%                 MAT(j,i)=dist;            
            adj(i,j)=1;
            adj(j,i)=1;
            
            if nargout>2
                middle_temp=zeros(1,2);
                %middle of segment between cells i and j:
                if (Trace{i}(4,1)==Trace{j}(1,1))
                    middle_temp(1)=Trace{i}(4);
                    temp_y=sort([Trace{i}(3:4,2);Trace{j}(1:2,2)]);
                    middle_temp(2)=mean(temp_y(2:3));
                else %(Trace{i}(1,1)==Trace{j}(4,1))
                    middle_temp(1)=Trace{i}(1);
                    temp_y=sort([Trace{i}(1:2,2);Trace{j}(3:4,2)]);
                    middle_temp(2)=mean(temp_y(2:3));
                end
                middle_X(i,j)=middle_temp(1);
                middle_X(j,i)=middle_temp(1);
                middle_Y(i,j)=middle_temp(2);
                middle_Y(j,i)=middle_temp(2);
                if nargout>4    %common line segments shared by adjacent cells
                    com_F{i,j}=[middle_temp(1), middle_temp(1) ; temp_y(2:3)'];  %com_F{i,j} has form [x_i' , x_i'' ; y_i' , y_i''], x_i'<=x_i''
                    com_F{j,i}=com_F{i,j};
                end
            end
        end
    end
end

%return: C- containing cel vertices (cell with each element a matrix with 2 rows and 4 columns)
%    and adj - adjacency matrix, with adj(i,j)=1 if cell i is adjacent to j (adj(i,i)=1 and adj is symmetric)
C=cell(1,k);
for i=1:k
    C{i}=[Trace{i}(1:4,:)]';
end
% adj=sparse(adj);    %return sparse adjacency matrix

%arrange vertices of each cell from decomposition to be convex hull
for i=1:length(C)
    ch_in=convhull(C{i}(1,:),C{i}(2,:));
    C{i}=C{i}(:,ch_in(1:length(ch_in)-1));
end

if nargout>2
    %return middle of adjacent segments:
    varargout(1)={middle_X};
    varargout(2)={middle_Y};
    %common line segments shared by adjacent cells
    if nargout>4
        varargout(3)={com_F};
    end
end


%%%%%%%%%%%%%%%%function curveintersect:
function [x,y]=curveintersect(varargin)
% Curve Intersections.
% [X,Y]=CURVEINTERSECT(H1,H2) or [X,Y]=CURVEINTERSECT([H1 H2]) finds the
% intersection points of the two curves on the X-Y plane identified
% by the line or lineseries object handles H1 and H2.
%
% [X,Y]=CURVEINTERSECT(X1,Y1,X2,Y2) finds the intersection points of the
% two curves described by the vector data pairs (X1,Y1) and (X2,Y2).
%
% X and Y are empty if no intersection exists.
%
% Example
% -------
% x1=rand(10,1); y1=rand(10,1); x2=rand(10,1); y2=rand(10,1);
% [x,y]=curveintersect(x1,y1,x2,y2); 
% plot(x1,y1,'k',x2,y2,'b',x,y,'ro')
%
% Original Version (-> curveintersect_local)
% ---------------------------------------
% D.C. Hanselman, University of Maine, Orono, ME 04469
% Mastering MATLAB 7
% 2005-01-06
%
% Improved Version (-> this function)
% -----------------------------------
% S. H?lz, TU Berlin, Germany
% v 1.0: October 2005
% v 1.1: April   2006     Fixed some minor bugs in function 'mminvinterp'

x=[]; y=[];
[x1,y1,x2,y2]=local_parseinputs(varargin{:});
ind_x1=sign(diff(x1)); ind_x2=sign(diff(x2));

ind1=1;
while ind1<length(x1)
    ind_max = ind1+min(find(ind_x1(ind1:end)~=ind_x1(ind1)))-1;
    if isempty(ind_max) | ind_max==ind1; ind_max=length(x1); end
    ind1=ind1:ind_max;
    
    ind2=1;
    while ind2<length(x2)
        ind_max = ind2+min(find(ind_x2(ind2:end)~=ind_x2(ind2)))-1;
        if isempty(ind_max) | ind_max==ind2; ind_max=length(x2); end
        ind2=ind2:ind_max;
        
        % Fallunterscheidung
        if ind_x1(ind1(1))==0 & ind_x2(ind2(1))~=0 
            x_loc=x1(ind1(1));
            y_loc=interp1(x2(ind2),y2(ind2),x_loc);
            if ~(y_loc>=min(y1(ind1)) && y_loc<=max(y1(ind1))); y_loc=[]; x_loc=[]; end
            
        elseif ind_x2(ind2(1))==0 & ind_x1(ind1(1))~=0
            x_loc=x2(ind2(1));
            y_loc=interp1(x1(ind1),y1(ind1),x_loc);
            if ~(y_loc>=min(y2(ind2)) && y_loc<=max(y2(ind2))); y_loc=[]; x_loc=[]; end

        elseif ind_x2(ind2(1))~=0 & ind_x1(ind1(1))~=0
            [x_loc,y_loc]=curveintersect_local(x1(ind1),y1(ind1),x2(ind2),y2(ind2));
            
        elseif ind_x2(ind2(1))==0 & ind_x1(ind1(1))==0
            [x_loc,y_loc]=deal([]);
            
        end
        x=[x; x_loc(:)];
        y=[y; y_loc(:)];
        ind2=ind2(end);
    end
    ind1=ind1(end);
end
    


% ----------------------------------------------
function [x,y]=curveintersect_local(x1,y1,x2,y2)

if ~isequal(x1,x2)
   xx=unique([x1 x2]); % get unique data points
   xx=xx(xx>=max(min(x1),min(x2)) & xx<=min(max(x1),max(x2)));
   if numel(xx)<2
      x=[];
      y=[];
      return
   end
   yy=interp1(x1,y1,xx)-interp1(x2,y2,xx);
else
   xx=x1;
   yy=y1-y2;
end
x=mminvinterp(xx,yy,0); % find zero crossings of difference
if ~isempty(x)
   y=interp1(x1,y1,x);
else
   x=[];
   y=[];
end

%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
function [xo,yo]=mminvinterp(x,y,yo)
%MMINVINTERP 1-D Inverse Interpolation. From the text "Mastering MATLAB 7"
% [Xo, Yo]=MMINVINTERP(X,Y,Yo) linearly interpolates the vector Y to find
% the scalar value Yo and returns all corresponding values Xo interpolated
% from the X vector. Xo is empty if no crossings are found. For
% convenience, the output Yo is simply the scalar input Yo replicated so
% that size(Xo)=size(Yo).
% If Y maps uniquely into X, use INTERP1(Y,X,Yo) instead.
%
% See also INTERP1.

if nargin~=3
   error('Three Input Arguments Required.')
end
n = numel(y);
if ~isequal(n,numel(x))
   error('X and Y Must have the Same Number of Elements.')
end
if ~isscalar(yo)
   error('Yo Must be a Scalar.')
end

x=x(:); % stretch input vectors into column vectors
y=y(:);

if yo<min(y) || yo>max(y) % quick exit if no values exist
   xo = [];
   yo = [];
else                      % find the desired points
   
   below = y<yo;          % True where below yo 
   above = y>yo;          % True where above yo
   on    = y==yo;         % True where on yo
   
   kth = (below(1:n-1)&above(2:n)) | (above(1:n-1)&below(2:n));     % point k
   kp1 = [false; kth];                                              % point k+1
   
   xo = [];                                                         % distance between x(k+1) and x(k) 
   if any(kth);                                                     
       alpha = (yo - y(kth))./(y(kp1)-y(kth));
       xo = alpha.*(x(kp1)-x(kth)) + x(kth);
   end         
   xo = sort([xo; x(on)]);                                          % add points, which are directly on line

   yo = repmat(yo,size(xo));                                        % duplicate yo to match xo points found
end 
%--------------------------------------------------------------------------
function [x1,y1,x2,y2]=local_parseinputs(varargin)

if nargin==1 % [X,Y]=CURVEINTERSECT([H1 H2])
   arg=varargin{1};
   if numel(arg)==2 && ...
      all(ishandle(arg)) && all(strcmp(get(arg,'type'),'line'))
      data=get(arg,{'XData','YData'});
      [x1,x2,y1,y2]=deal(data{:});
   else
      error('Input Must Contain Two Handles to Line Objects.')
   end
elseif nargin==2 % [X,Y]=CURVEINTERSECT(H1,H2)
   arg1=varargin{1};
   arg2=varargin{2};
   if numel(arg1)==1 && ishandle(arg1) && strcmp(get(arg1,'type'),'line')...
   && numel(arg2)==1 && ishandle(arg2) && strcmp(get(arg2,'type'),'line')
      
      data=get([arg1;arg2],{'XData','YData'});
      [x1,x2,y1,y2]=deal(data{:});
   else
      error('Input Must Contain Two Handles to Line Objects.')
   end
elseif nargin==4
   [x1,y1,x2,y2]=deal(varargin{:});
   if ~isequal(numel(x1),numel(y1))
      error('X1 and Y1 Must Contain the Same Number of Elements.')
   elseif ~isequal(numel(x2),numel(y2))
      error('X2 and Y2 Must Contain the Same Number of Elements.')
   end
   x1=reshape(x1,1,[]); % make data into rows
   x2=reshape(x2,1,[]);
   y1=reshape(y1,1,[]);
   y2=reshape(y2,1,[]);
       
else
   error('Incorrect Number of Input Arguments.')
end
if min(x1)>max(x2) | min(x2)>max(x1) | min(y2)>max(y1) | min(y1)>max(y2) % Polygons can not have intersections
    x1=[]; y1=[]; x2=[]; y2=[]; return
end
if numel(x1)<2 || numel(x2)<2 || numel(y1)<2 || numel(y2)<2
   error('At Least Two Data Points are Required for Each Curve.')
end
