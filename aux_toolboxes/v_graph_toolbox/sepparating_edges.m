%    This is part of RMTool - Robot Motion Toolbox, for visibility graph functionality.
%
%    Copyright (C) 2016 RMTool developing team, with support of Silvia Magdici. For people, details and citing 
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

function edges = sepparating_edges( reg1, reg2 )
edges = {};
    indexEdges = 0;
    len1=  length(reg1);
    len2=  length(reg2);
    for i=1:len1
        point1_temp = reg1(:,i)';
        for j=1:len2
            point2_temp = reg2(:,j)';
            [Aeq Beq lb ub] = makeLine(point1_temp, point2_temp);
            flag_reg1=0;
            flag_reg2=0;
            for k1=1:len1 %verific pt toate punctele din reg1
                if (k1 ~= i)
                    if (Aeq*reg1(:,k1)+Beq > 0)
                        flag_reg1 = flag_reg1 + 1;
                    else
                        flag_reg1 = flag_reg1 - 1;
                    end
                end
            end
            for k2=1:len2 %verific pt toate punctele din reg2
                if(k2 ~= j)
                    if (Aeq*reg2(:,k2)+Beq > 0)
                        flag_reg2 = flag_reg2 + 1;
                    else
                        flag_reg2 = flag_reg2 - 1;
                    end
                end
            end
           
            if((flag_reg1==(len1-1) && flag_reg2==-(len2-1) )||(flag_reg1==-(len1-1) && flag_reg2==(len2-1) ) )
                 fprintf('EVRIKA !!!\n'); 
                 point1_temp
                 point2_temp
                 flag_reg1
                 flag_reg2
                 
                 edgesTemp{1}(:,1)=point1_temp';
                 edgesTemp{1}(:,2)=point2_temp';
                 
                 flag_exist_edge=0;
                 % adaug edge in vector numai daca nu exista deja
				 if(indexEdges == 0)
					indexEdges = indexEdges+1;
					edges{indexEdges}=edgesTemp{1};
				 end
				 if (indexEdges > 0)
					 for t=1:indexEdges
						 if (edges{t} == edgesTemp{1})
							 flag_exist_edge=1;
							 fprintf('edge deja exista!\n');
						 end
					 end
					 if (flag_exist_edge==0)
							fprintf('adaug edge\n');
							indexEdges = indexEdges +1;
							edges{indexEdges}=edgesTemp{1};
					 end
				 end
                        
            end
        end
        
    end

end

