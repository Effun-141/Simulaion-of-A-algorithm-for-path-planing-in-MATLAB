

%************************************************************************A*�㷨*******************************************************************
% F = G + H
% F���ܴ���
% G��ʵ�ʴ��ۣ����룩
% H�����ƴ���
% ���������پ��������ƴ���
% Manhattan distance: m*|x1-x2| + n*|y1-y2| (m��nΪ����ֵϵ��)
%����ĵ�ͼ�����⹹���ĵ�ͼ�����Ը���ϵͳ��Ƶ�ͼ�����޸�
%*************************************************************************************************************************************************

clc

%------------------------------������ͼ------------------------------%

m = 30;
n = 30; %��ͼ�ߴ����30���ڵ�

%���������յ㣬����������ⶨ��
start = [8 23]; %��ʼ��
target = [29 4]; %Ŀ���

%��ͼ��߿�
for i = 1:m+2
    if ( (i == 1) || (i == m+2) ) %�߿�ڵ㣬-inf��ʾ������������Ϊ���ɵ����
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


%�ϰ���
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

%��ͼ��ʾ
for i=1:m+2
    for j=1:m+2
        if(map(i,j) == -inf)
            plot(i,j, 'k*')
            hold on
        end
    end
end


%������
plot(start(1),start(2),'rp');
hold on

%����յ�
plot(target(1), target(2), 'rp');
hold on

%��ͼ��ʾ����

%------------------------------������ͼ����------------------------------%


%------------------------------A*�㷨Ѱ·------------------------------%
%mapΪ32x32���󣬼���ͼ
F = map; %���ۺ���
G = map; %��ʵ����
H = map; %���ƴ���

openlist = map;
closelist = map;

%���ڵ��x��y�ֱ��Ӧ����ͼ����
father_point_x = map;
father_point_y = map;

%��������openlist�У�openlist��Ӧ���ֵ���0
%��������closelist�У�closelist��Ӧ���ֵ���1
%��ʼʱ��������openlist�У�������closelist��
openlist(start(1),start(2))=0; %�����뵽openlist��
F(start(1),start(2)) = 0;
G(start(1),start(2)) = 0; 

%������ͼ���������·��

while(1)
%-----step1:ȷ����ǰ���Ľڵ�-----%

%��ʱֻ�����FֵΪ0�������㶼Ϊ������-inf

    temp_F = inf; %���Fֵ
    for a = 1:m+2
        for b = 1:n+2
            if( (openlist(a, b) == 0) && (closelist(a, b) ~= 1) ) %��openlis�е��ǲ���closelist�У���ʼʱ��ֻ�������openlist��
                if( temp_F > F(a, b) )
                    temp_F = F(a, b);
                    center_point = [a, b];
                end
            end
        end
    end
    
%-----���ҵ���ǰɨ������Ľڵ�-----%

    closelist(center_point(1), center_point(2)) = 1; %�����ĵ���뵽closelist��
    
%-----step2:ɨ�赱ǰ���Ľڵ��8���ڽӽڵ�-----%
%����һ����centrepointΪ���ĵ�3*3����ϵ

    for i = 1:3   %������Ҫ��Ϊ�˹���һ��С����ϵ��centerpoint��Ϊ��2,2�����������G
        for j = 1:3
            
            temp_G = G(center_point(1)-2+i, center_point(2)-2+j); %���Gֵ
            
            %ĳһ����closelist�л����С�������ĵ�
            if( (i==2 && j==2) || (closelist(center_point(1)-2+i, center_point(2)-2+j) == 1) ) 
                continue;
                
            %ĳһ�㲻�ɵ�������뵽closlist��    
            elseif ( map(center_point(1)-2+i, center_point(2)-2+j) == -inf ) 
                    closelist(center_point(1)-2+i, center_point(2)-2+j) = 1;
                    
            %ĳһ����Ե��ﲢ�Ҳ���openlist������뵽openlist�У�������Fֵ        
            elseif ( temp_G == inf )
                    openlist(center_point(1)-2+i, center_point(2)-2+j) = 0;
                    
            %��ʼ������һ���Fֵ
                    %��һ�������ĵ�Խ����ϣ�
                    if( (i==1&&j==1)||(i==3&&j==1)||(i==1&&j==3)||(i==3&&j==3) )
                        G(center_point(1)-2+i, center_point(2)-2+j) = G(center_point(1), center_point(2)) + 14;
                    %��һ�������ĵ�ʮ�����ϣ�   
                    else 
                        G(center_point(1)-2+i, center_point(2)-2+j) = G(center_point(1), center_point(2)) + 10;
                    end
                    
                    %�����پ������H��Ϊ��ǰ��������Ŀ������ֵ*10
                    H(center_point(1)-2+i, center_point(2)-2+j) = 10*abs((center_point(1)-2+i-target(1))) + 10*abs((center_point(2)-2+j-target(2)));                   
                    F(center_point(1)-2+i, center_point(2)-2+j) =  G(center_point(1)-2+i, center_point(2)-2+j) + H(center_point(1)-2+i, center_point(2)-2+j);
                    
                    %ÿ��ɨ���ĸ��ڵ�ĺ�������
                    father_point_x(center_point(1)-2+i, center_point(2)-2+j) = center_point(1);
                    father_point_y(center_point(1)-2+i, center_point(2)-2+j) = center_point(2);
            
            %ĳһ���Ѿ���openlist�У���Ҫ�Ƚ�Fֵ�������Ƿ�ˢ����һ��ĸ��ڵ㣬����Hֵ��һ��������ֻ��Ƚ�Gֵ����
            else 
                if( (i==1&&j==1)||(i==3&&j==1)||(i==1&&j==3)||(i==3&&j==3) )
                        temp_G = G(center_point(1), center_point(2)) + 14;
                    else 
                        temp_G = G(center_point(1), center_point(2)) + 10;
                end
                
                %���Gֵ��С���򽫸��ڵ�ˢ��Ϊ�µ����ĵ�
                if ( temp_G < G(center_point(1)-2+i, center_point(2)-2+j))
                    
                    G(center_point(1)-2+i, center_point(2)-2+j) = temp_G;
                    H(center_point(1)-2+i, center_point(2)-2+j) = 10*abs((center_point(1)-2+i-target(1))) + 10*abs((center_point(2)-2+j-target(2)));
                    F(center_point(1)-2+i, center_point(2)-2+j) =  G(center_point(1)-2+i, center_point(2)-2+j) + H(center_point(1)-2+i, center_point(2)-2+j);
                    
                    father_point_x(center_point(1)-2+i, center_point(2)-2+j) = center_point(1);
                    father_point_y(center_point(1)-2+i, center_point(2)-2+j) = center_point(2);
                end
            end
%----------------------------------------------------------------------����ifѭ��-------------------------------------------------------------------------%
            %ɨ�赽Ŀ���
            if ( (openlist(target(1), target(2))==0) || (temp_F == inf) )
                father_point_x(target(1), target(2)) = center_point(1);
                father_point_y(target(1), target(2)) = center_point(2);
                break;
            end
        end
%----------------------------------------------------------------------����ifѭ��-------------------------------------------------------------------------%
       %ɨ�赽Ŀ���
       if ( (openlist(target(1), target(2))==0) || (temp_F == inf) )
            father_point_x(target(1), target(2)) = center_point(1);
            father_point_y(target(1), target(2)) = center_point(2);
            break;
        end
    end
%----------------------------------------------------------------------����whileѭ��-------------------------------------------------------------------------%
    %ɨ�赽Ŀ���
    if ( (openlist(target(1), target(2))==0) || (temp_F == inf) )
        father_point_x(target(1), target(2)) = center_point(1);
        father_point_y(target(1), target(2)) = center_point(2);
        break;
    end
end

%-----ɨ�赱ǰ���Ľڵ��8���ڽӽڵ����-----%


%-----step3:���Ӹ��ڵ�ȷ��·��-----%

fatherpoint = [];
line = 1;

%���ȫ�����ڵ�
while(1)
    if (temp_F == inf) %û��·��
        break;
    end
    
%     plot(target(1), target(2), 'b.');
    
    %2��n�о���
    fatherpoint(line,:) = target;
    line = line+1;
    
%   ��һ���ݸ��ڵ�
%     pause(1);
    
    temp = target(1);
    target(1) = father_point_x(target(1), target(2));
    target(2) = father_point_y(temp, target(2));
    
    %��ǰtarget��ĸ��ڵ������
    if((father_point_x(target(1), target(2))==start(1)) && (father_point_y(target(1), target(2))==start(2)))
        fatherpoint(line,:) = target;
        fatherpoint(line+1,:) = start;
%         plot(target(1), target(2), 'b.');
%         plot(start(1), start(2), 'b.');
        break;
        
    end
    
end

%����·��
 plot(fatherpoint(:,1),fatherpoint(:,2));
figure;

%-----·��ȷ�����滮����-----%

%%********************************************************************A*�㷨����************************************************************************





%-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------%
    








%**************************************************************����ʵ�ʵ�ͼ����·���滮*************************************************************
% F = G + H
% F���ܴ���
% G��ʵ�ʴ��ۣ����룩
% H�����ƴ���
% ���������پ��������ƴ���
% Manhattan distance: m*|x1-x2| + n*|y1-y2| (m��nΪ����ֵϵ��)
%����ĵ�ͼ��ʵ�ʳ��صĵ�ͼ
%ʵ��Ϊ3x3
%*************************************************************************************************************************************************

clc

%------------------------------������ͼ------------------------------%

m = 30;
n = 30; %��ͼ�ߴ����30���ڵ�

%���������յ㣬����������ⶨ��
start = [2 2]; %��ʼ��
target = [21 31]; %Ŀ���

%��ͼ��߿�
for i = 1:m+2
    if ( (i == 1) || (i == m+2) ) %�߿�ڵ㣬-inf��ʾ������������Ϊ���ɵ����
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


%�ϰ���
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




%��ͼ��ʾ
for i=1:m+2
    for j=1:m+2
        if(map(i,j) == -inf)
            plot(i,j, 'k*')
            hold on
        end
    end
end


%������
plot(start(1),start(2),'rp');
hold on

%����յ�
plot(target(1), target(2), 'rp');
hold on

%��ͼ��ʾ����

%------------------------------������ͼ����------------------------------%


%------------------------------A*�㷨Ѱ·------------------------------%
%mapΪ32x32���󣬼���ͼ
F = map; %���ۺ���
G = map; %��ʵ����
H = map; %���ƴ���

openlist = map;
closelist = map;

%���ڵ��x��y�ֱ��Ӧ����ͼ����
father_point_x = map;
father_point_y = map;

%��������openlist�У�openlist��Ӧ���ֵ���0
%��������closelist�У�closelist��Ӧ���ֵ���1
%��ʼʱ��������openlist�У�������closelist��
openlist(start(1),start(2))=0; %�����뵽openlist��
F(start(1),start(2)) = 0;
G(start(1),start(2)) = 0; 

%������ͼ���������·��

while(1)
%-----step1:ȷ����ǰ���Ľڵ�-----%

%��ʱֻ�����FֵΪ0�������㶼Ϊ������-inf

    temp_F = inf; %���Fֵ
    for a = 1:m+2
        for b = 1:n+2
            if( (openlist(a, b) == 0) && (closelist(a, b) ~= 1) ) %��openlis�е��ǲ���closelist�У���ʼʱ��ֻ�������openlist��
                if( temp_F > F(a, b) )
                    temp_F = F(a, b);
                    center_point = [a, b];
                end
            end
        end
    end
    
%-----���ҵ���ǰɨ������Ľڵ�-----%

    closelist(center_point(1), center_point(2)) = 1; %�����ĵ���뵽closelist��
    
%-----step2:ɨ�赱ǰ���Ľڵ��8���ڽӽڵ�-----%
%����һ����centrepointΪ���ĵ�3*3����ϵ

    for i = 1:3   %������Ҫ��Ϊ�˹���һ��С����ϵ��centerpoint��Ϊ��2,2�����������G
        for j = 1:3
            
            temp_G = G(center_point(1)-2+i, center_point(2)-2+j); %���Gֵ
            
            %ĳһ����closelist�л����С�������ĵ�
            if( (i==2 && j==2) || (closelist(center_point(1)-2+i, center_point(2)-2+j) == 1) ) 
                continue;
                
            %ĳһ�㲻�ɵ�������뵽closlist��    
            elseif ( map(center_point(1)-2+i, center_point(2)-2+j) == -inf ) 
                    closelist(center_point(1)-2+i, center_point(2)-2+j) = 1;
                    
            %ĳһ����Ե��ﲢ�Ҳ���openlist������뵽openlist�У�������Fֵ        
            elseif ( temp_G == inf )
                    openlist(center_point(1)-2+i, center_point(2)-2+j) = 0;
                    
            %��ʼ������һ���Fֵ
                    %��һ�������ĵ�Խ����ϣ�
                    if( (i==1&&j==1)||(i==3&&j==1)||(i==1&&j==3)||(i==3&&j==3) )
                        G(center_point(1)-2+i, center_point(2)-2+j) = G(center_point(1), center_point(2)) + 14;
                    %��һ�������ĵ�ʮ�����ϣ�   
                    else 
                        G(center_point(1)-2+i, center_point(2)-2+j) = G(center_point(1), center_point(2)) + 10;
                    end
                    
                    %�����پ������H��Ϊ��ǰ��������Ŀ������ֵ*10
                    H(center_point(1)-2+i, center_point(2)-2+j) = 10*abs((center_point(1)-2+i-target(1))) + 10*abs((center_point(2)-2+j-target(2)));                   
                    F(center_point(1)-2+i, center_point(2)-2+j) =  G(center_point(1)-2+i, center_point(2)-2+j) + H(center_point(1)-2+i, center_point(2)-2+j);
                    
                    %ÿ��ɨ���ĸ��ڵ�ĺ�������
                    father_point_x(center_point(1)-2+i, center_point(2)-2+j) = center_point(1);
                    father_point_y(center_point(1)-2+i, center_point(2)-2+j) = center_point(2);
            
            %ĳһ���Ѿ���openlist�У���Ҫ�Ƚ�Fֵ�������Ƿ�ˢ����һ��ĸ��ڵ㣬����Hֵ��һ��������ֻ��Ƚ�Gֵ����
            else 
                if( (i==1&&j==1)||(i==3&&j==1)||(i==1&&j==3)||(i==3&&j==3) )
                        temp_G = G(center_point(1), center_point(2)) + 14;
                    else 
                        temp_G = G(center_point(1), center_point(2)) + 10;
                end
                
                %���Gֵ��С���򽫸��ڵ�ˢ��Ϊ�µ����ĵ�
                if ( temp_G < G(center_point(1)-2+i, center_point(2)-2+j))
                    
                    G(center_point(1)-2+i, center_point(2)-2+j) = temp_G;
                    H(center_point(1)-2+i, center_point(2)-2+j) = 10*abs((center_point(1)-2+i-target(1))) + 10*abs((center_point(2)-2+j-target(2)));
                    F(center_point(1)-2+i, center_point(2)-2+j) =  G(center_point(1)-2+i, center_point(2)-2+j) + H(center_point(1)-2+i, center_point(2)-2+j);
                    
                    father_point_x(center_point(1)-2+i, center_point(2)-2+j) = center_point(1);
                    father_point_y(center_point(1)-2+i, center_point(2)-2+j) = center_point(2);
                end
            end
%----------------------------------------------------------------------����ifѭ��-------------------------------------------------------------------------%
            %ɨ�赽Ŀ���
            if ( (openlist(target(1), target(2))==0) || (temp_F == inf) )
                father_point_x(target(1), target(2)) = center_point(1);
                father_point_y(target(1), target(2)) = center_point(2);
                break;
            end
        end
%----------------------------------------------------------------------����ifѭ��-------------------------------------------------------------------------%
       %ɨ�赽Ŀ���
       if ( (openlist(target(1), target(2))==0) || (temp_F == inf) )
            father_point_x(target(1), target(2)) = center_point(1);
            father_point_y(target(1), target(2)) = center_point(2);
            break;
        end
    end
%----------------------------------------------------------------------����whileѭ��-------------------------------------------------------------------------%
    %ɨ�赽Ŀ���
    if ( (openlist(target(1), target(2))==0) || (temp_F == inf) )
        father_point_x(target(1), target(2)) = center_point(1);
        father_point_y(target(1), target(2)) = center_point(2);
        break;
    end
end
%-----ɨ�赱ǰ���Ľڵ��8���ڽӽڵ����-----%

%-----step3:���Ӹ��ڵ�ȷ��·��-----%

fatherpoint = [];
line = 1;

%���ȫ�����ڵ�
while(1)
    if (temp_F == inf) %û��·��
        break;
    end
    
    plot(target(1), target(2), 'b.');
    
    %2��n�о���
    fatherpoint(line,:) = target;
    line = line+1;
    
%   ��һ���ݸ��ڵ�
%     pause(1);
    
    temp = target(1);
    target(1) = father_point_x(target(1), target(2));
    target(2) = father_point_y(temp, target(2));
    
    %��ǰtarget��ĸ��ڵ������
    if((father_point_x(target(1), target(2))==start(1)) && (father_point_y(target(1), target(2))==start(2)))
        fatherpoint(line,:) = target;
        fatherpoint(line+1,:) = start;
%         plot(target(1), target(2), 'b.');
%         plot(start(1), start(2), 'b.');
        break;
        
    end
    
end

%����·��
 plot(fatherpoint(:,1),fatherpoint(:,2));

% figure;

%-----·��ȷ�����滮����-----%


%%********************************************************************ʵ�ʵ�ͼ·���滮����************************************************************************







                        
                    
               
                

         
                
                        
                
                
    


































