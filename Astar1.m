

%************************************************************************A*算法*******************************************************************
% F = G + H
% F：总代价
% G：实际代价（距离）
% H：估计代价
% 采用曼哈顿距离计算估计代价
% Manhattan distance: m*|x1-x2| + n*|y1-y2| (m、n为距离值系数)
%这里的地图是随意构建的地图，可以根据系统设计地图进行修改
%*************************************************************************************************************************************************

clc

%------------------------------构建地图------------------------------%

m = 30;
n = 30; %地图尺寸横纵30个节点

%定义起点和终点，这里可以随意定义
start = [8 23]; %起始点
target = [29 4]; %目标点

%地图外边框
for i = 1:m+2
    if ( (i == 1) || (i == m+2) ) %边框节点，-inf表示负无穷在这里为不可到达点
        for j = 1:n+2
           map(i, j) = -inf;
        end
    else 
        for j = 1:n+2
            if ((j == 1) || (j == n+2))
                map(i, j) = -inf;
            else 
                map(i, j) = inf;
            end
        end
    end
end


%障碍点
for j = 2:10
    map(5, j) = -inf;
end

for j = 2:15
    map(24, j) = -inf;
end

for j = 9:24
    map(10, j) = -inf;
end

for j = 20:31
    map(15, j) = -inf;
end

for j = 5:20
    map(20, j) = -inf;
end

for j = 18:27
    map(28, j) = -inf;
end

for i = 2:6
    map(i, 18) = -inf;
end

for i = 17:20
    map(i, 5) = -inf;
end

for i = 23:25
    map(i, 20) = -inf;
end

for i = 13:17
    map(i, 13) = -inf;
end

for j = 18:27
    map(29, j) = -inf;
end

for j = 1:26
    map(22, j) = -inf;
end

for i = 14:17
    for j = 15:18
        map(i, j) = -inf;
    end
end

for i = 1:7
    map(i, 25) = -inf;
end

for i = 16:20
    for j = 22:25
        map(i, j) = -inf;
    end
end

for i = 26:28
    for j = 7:11
        map(i, j) = -inf;
    end
end

for i = 30:31
    for j = 7:11
        map(i, j) = -inf;
    end
end

for i = 12:14
    for j = 5:10
        map(i, j) = -inf;
    end
end

for i = 26:31
    map(i, 5) = -inf;
end

map(25,9) = -inf;

%地图显示
for i=1:m+2
    for j=1:m+2
        if(map(i,j) == -inf)
            plot(i,j, 'k*')
            hold on
        end
    end
end


%标记起点
plot(start(1),start(2),'rp');
hold on

%标记终点
plot(target(1), target(2), 'rp');
hold on

%地图显示结束

%------------------------------构建地图结束------------------------------%


%------------------------------A*算法寻路------------------------------%
%map为32x32矩阵，即地图
F = map; %代价函数
G = map; %真实距离
H = map; %估计代价

openlist = map;
closelist = map;

%父节点的x、y分别对应到地图坐标
father_point_x = map;
father_point_y = map;

%当结点加入openlist中，openlist对应点初值变成0
%当结点加入closelist中，closelist对应点初值变成1
%初始时刻起点加入openlist中，而不在closelist中
openlist(start(1),start(2))=0; %起点加入到openlist中
F(start(1),start(2)) = 0;
G(start(1),start(2)) = 0; 

%遍历地图，搜索最佳路径

while(1)
%-----step1:确定当前中心节点-----%

%此时只有起点F值为0，其他点都为负无穷-inf

    temp_F = inf; %存放F值
    for a = 1:m+2
        for b = 1:n+2
            if( (openlist(a, b) == 0) && (closelist(a, b) ~= 1) ) %在openlis中但是不在closelist中，初始时刻只有起点在openlist中
                if( temp_F > F(a, b) )
                    temp_F = F(a, b);
                    center_point = [a, b];
                end
            end
        end
    end
    
%-----已找到当前扫描的中心节点-----%

    closelist(center_point(1), center_point(2)) = 1; %将中心点加入到closelist中
    
%-----step2:扫描当前中心节点的8个邻接节点-----%
%构建一个以centrepoint为中心的3*3坐标系

    for i = 1:3   %这里主要是为了构建一个小坐标系，centerpoint的为（2,2），方便计算G
        for j = 1:3
            
            temp_G = G(center_point(1)-2+i, center_point(2)-2+j); %存放G值
            
            %某一点在closelist中或就是小坐标中心点
            if( (i==2 && j==2) || (closelist(center_point(1)-2+i, center_point(2)-2+j) == 1) ) 
                continue;
                
            %某一点不可到达则加入到closlist中    
            elseif ( map(center_point(1)-2+i, center_point(2)-2+j) == -inf ) 
                    closelist(center_point(1)-2+i, center_point(2)-2+j) = 1;
                    
            %某一点可以到达并且不在openlist中则加入到openlist中，计算其F值        
            elseif ( temp_G == inf )
                    openlist(center_point(1)-2+i, center_point(2)-2+j) = 0;
                    
            %开始计算这一点的F值
                    %这一点在中心点对角线上：
                    if( (i==1&&j==1)||(i==3&&j==1)||(i==1&&j==3)||(i==3&&j==3) )
                        G(center_point(1)-2+i, center_point(2)-2+j) = G(center_point(1), center_point(2)) + 14;
                    %这一点在中心点十字线上：   
                    else 
                        G(center_point(1)-2+i, center_point(2)-2+j) = G(center_point(1), center_point(2)) + 10;
                    end
                    
                    %曼哈顿距离计算H，为当前点的坐标减目标点绝对值*10
                    H(center_point(1)-2+i, center_point(2)-2+j) = 10*abs((center_point(1)-2+i-target(1))) + 10*abs((center_point(2)-2+j-target(2)));                   
                    F(center_point(1)-2+i, center_point(2)-2+j) =  G(center_point(1)-2+i, center_point(2)-2+j) + H(center_point(1)-2+i, center_point(2)-2+j);
                    
                    %每个扫描点的父节点的横纵坐标
                    father_point_x(center_point(1)-2+i, center_point(2)-2+j) = center_point(1);
                    father_point_y(center_point(1)-2+i, center_point(2)-2+j) = center_point(2);
            
            %某一点已经在openlist中，需要比较F值，决定是否刷新这一点的父节点，由于H值都一样，所以只需比较G值即可
            else 
                if( (i==1&&j==1)||(i==3&&j==1)||(i==1&&j==3)||(i==3&&j==3) )
                        temp_G = G(center_point(1), center_point(2)) + 14;
                    else 
                        temp_G = G(center_point(1), center_point(2)) + 10;
                end
                
                %如果G值变小，则将父节点刷新为新的中心点
                if ( temp_G < G(center_point(1)-2+i, center_point(2)-2+j))
                    
                    G(center_point(1)-2+i, center_point(2)-2+j) = temp_G;
                    H(center_point(1)-2+i, center_point(2)-2+j) = 10*abs((center_point(1)-2+i-target(1))) + 10*abs((center_point(2)-2+j-target(2)));
                    F(center_point(1)-2+i, center_point(2)-2+j) =  G(center_point(1)-2+i, center_point(2)-2+j) + H(center_point(1)-2+i, center_point(2)-2+j);
                    
                    father_point_x(center_point(1)-2+i, center_point(2)-2+j) = center_point(1);
                    father_point_y(center_point(1)-2+i, center_point(2)-2+j) = center_point(2);
                end
            end
%----------------------------------------------------------------------跳出if循环-------------------------------------------------------------------------%
            %扫描到目标点
            if ( (openlist(target(1), target(2))==0) || (temp_F == inf) )
                father_point_x(target(1), target(2)) = center_point(1);
                father_point_y(target(1), target(2)) = center_point(2);
                break;
            end
        end
%----------------------------------------------------------------------跳出if循环-------------------------------------------------------------------------%
       %扫描到目标点
       if ( (openlist(target(1), target(2))==0) || (temp_F == inf) )
            father_point_x(target(1), target(2)) = center_point(1);
            father_point_y(target(1), target(2)) = center_point(2);
            break;
        end
    end
%----------------------------------------------------------------------跳出while循环-------------------------------------------------------------------------%
    %扫描到目标点
    if ( (openlist(target(1), target(2))==0) || (temp_F == inf) )
        father_point_x(target(1), target(2)) = center_point(1);
        father_point_y(target(1), target(2)) = center_point(2);
        break;
    end
end

%-----扫描当前中心节点的8个邻接节点结束-----%


%-----step3:连接父节点确定路径-----%

fatherpoint = [];
line = 1;

%标记全部父节点
while(1)
    if (temp_F == inf) %没有路径
        break;
    end
    
%     plot(target(1), target(2), 'b.');
    
    %2列n行矩阵
    fatherpoint(line,:) = target;
    line = line+1;
    
%   逐一回溯父节点
%     pause(1);
    
    temp = target(1);
    target(1) = father_point_x(target(1), target(2));
    target(2) = father_point_y(temp, target(2));
    
    %当前target点的父节点是起点
    if((father_point_x(target(1), target(2))==start(1)) && (father_point_y(target(1), target(2))==start(2)))
        fatherpoint(line,:) = target;
        fatherpoint(line+1,:) = start;
%         plot(target(1), target(2), 'b.');
%         plot(start(1), start(2), 'b.');
        break;
        
    end
    
end

%绘制路径
 plot(fatherpoint(:,1),fatherpoint(:,2));
figure;

%-----路径确定，规划结束-----%

%%********************************************************************A*算法结束************************************************************************





%-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------%
    








%**************************************************************构建实际地图进行路径规划*************************************************************
% F = G + H
% F：总代价
% G：实际代价（距离）
% H：估计代价
% 采用曼哈顿距离计算估计代价
% Manhattan distance: m*|x1-x2| + n*|y1-y2| (m、n为距离值系数)
%这里的地图是实际场地的地图
%实际为3x3
%*************************************************************************************************************************************************

clc

%------------------------------构建地图------------------------------%

m = 30;
n = 30; %地图尺寸横纵30个节点

%定义起点和终点，这里可以随意定义
start = [2 2]; %起始点
target = [21 31]; %目标点

%地图外边框
for i = 1:m+2
    if ( (i == 1) || (i == m+2) ) %边框节点，-inf表示负无穷在这里为不可到达点
        for j = 1:n+2
           map(i, j) = -inf;
        end
    else 
        for j = 1:n+2
            if ((j == 1) || (j == n+2))
                map(i, j) = -inf;
            else 
                map(i, j) = inf;
            end
        end
    end
end


%障碍点
for  i = 3:10
    for j = 3:11
        map(i,j)=-inf;
    end
end

for  i = 3:10
    for j = 13:21
        map(i,j)=-inf;
    end
end

for  i = 3:10
    for j = 23:30
        map(i,j)=-inf;
    end
end





for  i = 12:20
    for j = 3:11
        map(i,j)=-inf;
    end
end

for  i = 12:20
    for j = 13:21
        map(i,j)=-inf;
    end
end

for  i = 12:20
    for j = 23:30
        map(i,j)=-inf;
    end
end





for  i = 22:30
    for j = 3:11
        map(i,j)=-inf;
    end
end

for  i = 22:30
    for j = 13:21
        map(i,j)=-inf;
    end
end

for  i = 22:30
    for j = 23:30
        map(i,j)=-inf;
    end
end




%地图显示
for i=1:m+2
    for j=1:m+2
        if(map(i,j) == -inf)
            plot(i,j, 'k*')
            hold on
        end
    end
end


%标记起点
plot(start(1),start(2),'rp');
hold on

%标记终点
plot(target(1), target(2), 'rp');
hold on

%地图显示结束

%------------------------------构建地图结束------------------------------%


%------------------------------A*算法寻路------------------------------%
%map为32x32矩阵，即地图
F = map; %代价函数
G = map; %真实距离
H = map; %估计代价

openlist = map;
closelist = map;

%父节点的x、y分别对应到地图坐标
father_point_x = map;
father_point_y = map;

%当结点加入openlist中，openlist对应点初值变成0
%当结点加入closelist中，closelist对应点初值变成1
%初始时刻起点加入openlist中，而不在closelist中
openlist(start(1),start(2))=0; %起点加入到openlist中
F(start(1),start(2)) = 0;
G(start(1),start(2)) = 0; 

%遍历地图，搜索最佳路径

while(1)
%-----step1:确定当前中心节点-----%

%此时只有起点F值为0，其他点都为负无穷-inf

    temp_F = inf; %存放F值
    for a = 1:m+2
        for b = 1:n+2
            if( (openlist(a, b) == 0) && (closelist(a, b) ~= 1) ) %在openlis中但是不在closelist中，初始时刻只有起点在openlist中
                if( temp_F > F(a, b) )
                    temp_F = F(a, b);
                    center_point = [a, b];
                end
            end
        end
    end
    
%-----已找到当前扫描的中心节点-----%

    closelist(center_point(1), center_point(2)) = 1; %将中心点加入到closelist中
    
%-----step2:扫描当前中心节点的8个邻接节点-----%
%构建一个以centrepoint为中心的3*3坐标系

    for i = 1:3   %这里主要是为了构建一个小坐标系，centerpoint的为（2,2），方便计算G
        for j = 1:3
            
            temp_G = G(center_point(1)-2+i, center_point(2)-2+j); %存放G值
            
            %某一点在closelist中或就是小坐标中心点
            if( (i==2 && j==2) || (closelist(center_point(1)-2+i, center_point(2)-2+j) == 1) ) 
                continue;
                
            %某一点不可到达则加入到closlist中    
            elseif ( map(center_point(1)-2+i, center_point(2)-2+j) == -inf ) 
                    closelist(center_point(1)-2+i, center_point(2)-2+j) = 1;
                    
            %某一点可以到达并且不在openlist中则加入到openlist中，计算其F值        
            elseif ( temp_G == inf )
                    openlist(center_point(1)-2+i, center_point(2)-2+j) = 0;
                    
            %开始计算这一点的F值
                    %这一点在中心点对角线上：
                    if( (i==1&&j==1)||(i==3&&j==1)||(i==1&&j==3)||(i==3&&j==3) )
                        G(center_point(1)-2+i, center_point(2)-2+j) = G(center_point(1), center_point(2)) + 14;
                    %这一点在中心点十字线上：   
                    else 
                        G(center_point(1)-2+i, center_point(2)-2+j) = G(center_point(1), center_point(2)) + 10;
                    end
                    
                    %曼哈顿距离计算H，为当前点的坐标减目标点绝对值*10
                    H(center_point(1)-2+i, center_point(2)-2+j) = 10*abs((center_point(1)-2+i-target(1))) + 10*abs((center_point(2)-2+j-target(2)));                   
                    F(center_point(1)-2+i, center_point(2)-2+j) =  G(center_point(1)-2+i, center_point(2)-2+j) + H(center_point(1)-2+i, center_point(2)-2+j);
                    
                    %每个扫描点的父节点的横纵坐标
                    father_point_x(center_point(1)-2+i, center_point(2)-2+j) = center_point(1);
                    father_point_y(center_point(1)-2+i, center_point(2)-2+j) = center_point(2);
            
            %某一点已经在openlist中，需要比较F值，决定是否刷新这一点的父节点，由于H值都一样，所以只需比较G值即可
            else 
                if( (i==1&&j==1)||(i==3&&j==1)||(i==1&&j==3)||(i==3&&j==3) )
                        temp_G = G(center_point(1), center_point(2)) + 14;
                    else 
                        temp_G = G(center_point(1), center_point(2)) + 10;
                end
                
                %如果G值变小，则将父节点刷新为新的中心点
                if ( temp_G < G(center_point(1)-2+i, center_point(2)-2+j))
                    
                    G(center_point(1)-2+i, center_point(2)-2+j) = temp_G;
                    H(center_point(1)-2+i, center_point(2)-2+j) = 10*abs((center_point(1)-2+i-target(1))) + 10*abs((center_point(2)-2+j-target(2)));
                    F(center_point(1)-2+i, center_point(2)-2+j) =  G(center_point(1)-2+i, center_point(2)-2+j) + H(center_point(1)-2+i, center_point(2)-2+j);
                    
                    father_point_x(center_point(1)-2+i, center_point(2)-2+j) = center_point(1);
                    father_point_y(center_point(1)-2+i, center_point(2)-2+j) = center_point(2);
                end
            end
%----------------------------------------------------------------------跳出if循环-------------------------------------------------------------------------%
            %扫描到目标点
            if ( (openlist(target(1), target(2))==0) || (temp_F == inf) )
                father_point_x(target(1), target(2)) = center_point(1);
                father_point_y(target(1), target(2)) = center_point(2);
                break;
            end
        end
%----------------------------------------------------------------------跳出if循环-------------------------------------------------------------------------%
       %扫描到目标点
       if ( (openlist(target(1), target(2))==0) || (temp_F == inf) )
            father_point_x(target(1), target(2)) = center_point(1);
            father_point_y(target(1), target(2)) = center_point(2);
            break;
        end
    end
%----------------------------------------------------------------------跳出while循环-------------------------------------------------------------------------%
    %扫描到目标点
    if ( (openlist(target(1), target(2))==0) || (temp_F == inf) )
        father_point_x(target(1), target(2)) = center_point(1);
        father_point_y(target(1), target(2)) = center_point(2);
        break;
    end
end
%-----扫描当前中心节点的8个邻接节点结束-----%

%-----step3:连接父节点确定路径-----%

fatherpoint = [];
line = 1;

%标记全部父节点
while(1)
    if (temp_F == inf) %没有路径
        break;
    end
    
    plot(target(1), target(2), 'b.');
    
    %2列n行矩阵
    fatherpoint(line,:) = target;
    line = line+1;
    
%   逐一回溯父节点
%     pause(1);
    
    temp = target(1);
    target(1) = father_point_x(target(1), target(2));
    target(2) = father_point_y(temp, target(2));
    
    %当前target点的父节点是起点
    if((father_point_x(target(1), target(2))==start(1)) && (father_point_y(target(1), target(2))==start(2)))
        fatherpoint(line,:) = target;
        fatherpoint(line+1,:) = start;
%         plot(target(1), target(2), 'b.');
%         plot(start(1), start(2), 'b.');
        break;
        
    end
    
end

%绘制路径
 plot(fatherpoint(:,1),fatherpoint(:,2));

% figure;

%-----路径确定，规划结束-----%


%%********************************************************************实际地图路径规划结束************************************************************************







                        
                    
               
                

         
                
                        
                
                
    


































