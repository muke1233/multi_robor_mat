clc;
clear
close all

tic

%% ����ͼ
% դ���ͼ������ ��������
m = 150;
n = 150;
% ��ͼm��n��?
start = [10, 20];        % ��ʼ�ڵ�
target = [130, 130];       % ��ֹ�ڵ�
obs = [6, 1; 6, 2; 6, 3; 6, 4; 6, 5; 6, 6; 6, 7; 5, 5; 4, 5; 3, 5];   % 障碍物区�?
obs_ex = [];
obs_ex1 = [];
%obs = TrunToGridMap(m, n);

for i = 1 : size(obs, 1) - 1
    obs_ex =  obs(i, :);
    x=obs_ex(1);
    y=obs_ex(2);
    for j=0:2
        for k=0:2
            obs_ex = [x-1+j,y-1+k];
            inflag= ismember(obs_ex, obs_ex1);
             % in_flag��ʾ��obs_ex�Ƿ���obs_ex1��?      
            % obs_ex1_idx��ʾ��obs_ex��obs_ex1�е�����
            if inflag==0
                obs_ex1(end+1,:) = obs_ex;
            end
        end
    end
end



% ������
for i = 0 : 5 : m
    plot([0, n], [i, i], 'k', 'handlevisibility', 'off');
    hold on;
end

for j = 0 : 5 : n
    plot([j, j], [0, m], 'k', 'handlevisibility', 'off');
end

axis equal;
xlim([0, n]);
ylim([0, m]);

% �����ϰ��� ��ֹ����ɫ��
% fill([start_node(1)-1, start_node(1), start_node(1), start_node(1)-1],...
%     [start_node(2)-1, start_node(2)-1, start_node(2), start_node(2)], 'g');

scatter(start(1), start(2), 700, 'pg', 'filled');

% fill([target_node(1)-1, target_node(1), target_node(1), target_node(1)-1],...
%     [target_node(2)-1, target_node(2)-1, target_node(2), target_node(2)], 'r');

scatter(target(1), target(2), 700, 'pr', 'filled');

for i = 1 : size(obs, 1) - 1
    temp = obs(i, :);
    fill([temp(1)-1, temp(1), temp(1), temp(1)-1],...
        [temp(2)-1, temp(2)-1, temp(2), temp(2)], 'k', 'handlevisibility', 'off');
end

temp = obs(size(obs, 1), :);
fill([temp(1)-1, temp(1), temp(1), temp(1)-1],...
    [temp(2)-1, temp(2)-1, temp(2), temp(2)], 'k');




% water = 25 * ones(1, n);
% waterx = 1 : n;
% plot(waterx, water, 'c', 'linewidth', 1.5);



%% Ԥ����?
% ��ʼ�� closelist
closelist = start;
closelist_path = {start, start};      % ·��������������?
closelist_cost = 0;
children = child_nodes_cal(start, m, n, obs, closelist);

% ��ʼ��openlist
openlist = children;


for i = 1 : size(openlist, 1)   % iΪ��i���ڵ�?    
    openlist_path{i, 1} = openlist(i, :);   % openlist_path�ĵ�i�е�1��Ϊ��i���ڵ�child  
    openlist_path{i, 2} = [start; openlist(i, :)]; % openlist_path�ĵ�i�е�2��Ϊһ��������
    % �ֱ�����ʼ�ڵ�͵�ǰchild_node
end

for i = 1 : size(openlist, 1)
    g = norm(start - openlist(i, 1:2));%����
    h = abs(target(1) - openlist(i, 1)) + abs(target(2) - openlist(i, 2));%�����پ���
%     h = sqrt((target_node(1) - openlist(i, 1))^2 + (target_node(2) - openlist(i, 2))^2);
    f = g + h;
    openlist_cost(i, :) = [g, h, f];
end

%%���常�ڵ�
% ��openlist��ʼ�����ƶ�������С�Ľڵ�?
[~, min_idx] = min(openlist_cost(:, 3));    % ��fֵ��С��min_idxΪf��С����һ��?��
parent = openlist(min_idx, :);% ��min_idx���е��ӽڵ�child_nodeΪ�µĸ��ڵ�

%% ����ѭ��
flag = 1;
while flag
    % �ҳ����ڵ�ĺ���closelist���ӽڵ�
    children = child_nodes_cal(parent ,m, n, obs, closelist);
    
    % �ж���Щ�ӽڵ��Ƿ���openlist�Э����ڣ�����£�û�ڣ���׷�ӵ�openlist��
    for i = 1 : size(children, 1)
        child = children(i, :);
        [in_flag, openlist_idx] = ismember(child, openlist, 'rows');
        % in_flag��ʾ��child_node�Ƿ���openlist��?      
        % openlist_idx��ʾ��child_node��openlist�е�����
        
        g = openlist_cost(min_idx, 1) + norm(parent - child);
        % ԭ����g�����µ�g
        h = abs(child(1) - target(1)) + abs(child(2) - target(2));
%         h = sqrt((target_node(1) - openlist(i, 1))^2 + (target_node(2) - openlist(i, 2))^2);
        f = g + h;
        
        if in_flag  % ���ڣ���Ƚϸ���g, f
            if g < openlist_cost(openlist_idx, 1)   
                % openlist_cost(openlist_idx,1)ָ����openlist_cost��idx��һ�У���child_node���ڵ�һ�У��ĵ�һ�����꣬��g  
                openlist_cost(openlist_idx, 1) = g;
                openlist_cost(openlist_idx, 3) = f;
                openlist_path{openlist_idx, 2} = [openlist_path{min_idx, 2}; child];
                % openlist_path�ĵ�i�е�2��Ϊһ����������
                %�ֱ�����ʼ�ڵ�͵�ǰchild���˴������µ���ʼ�ڵ�Ϊopenlist_path(min_idx,2)�� 
                %��openlist_path(min_idx,2)ָ��min_idx������Ӧ�� child��openlist_path�ж�Ӧ��·����
                %�൱�ڰ��µ�child���ӵ���·���ϣ��ӳ���·��
                  
            end
         else
            openlist(end+1, :) = child;
            openlist_cost(end+1, :) = [g, h, f];
            openlist_path{end+1, 1} = child;
            openlist_path{end, 2} = [openlist_path{min_idx, 2}; child];
         end
     end
    
    
    % ��openlist�Ƴ�������С�Ľڵ㵽closelist
    closelist(end+1, :) = openlist(min_idx, :);
    closelist_cost(end+1, :) = openlist_cost(min_idx, 3);
    closelist_path(end+1, :) = openlist_path(min_idx, :);
    % ͬ���أ�openlist�����˸ýڵ�
    openlist(min_idx, :) = [];
    openlist_cost(min_idx, :) = [];
    openlist_path(min_idx, :) = [];
    
    
    % ������������openlist�����ƶ�������С�Ľڵ㣬��Ϊ�µĸ��ڵ�  
    [~, min_idx] = min(openlist_cost(:, 3));
    parent = openlist(min_idx, :);
    
    % �ж��Ƿ��������յ�    
    if parent == target
        closelist(end+1, :) = openlist(min_idx, :);
        closelist_cost(end+1, 1) = openlist_cost(min_idx, 1);
        closelist_path(end+1, :) = openlist_path(min_idx, :);
        flag = 0;
    end   

end
%toc

%% ��ͼ
path_opt = closelist_path{end, 2};
% closelist_path�еڶ��д��·��������path_opt��ŵ���·����(x,y)ֵ
path_opt(:, 1) = path_opt(:, 1) - 0.5;
path_opt(:, 2) = path_opt(:, 2) - 0.5;
% scatter(path_opt(:, 1), path_opt(:, 2), 60, 'g');
plot(path_opt(:, 1), path_opt(:, 2), 'm', 'linewidth', 1.5);

title(['Total length of path: ' num2str(closelist_cost(end, 1))]);
% legend('Start node', 'Target node', 'Obstacle', 'Wateface', 'Path');
legend('Start node', 'Target node', 'Obstacle', 'Path', 'location', 'northwest');
set(gca, 'fontsize', 15, 'fontname', 'times new roman');

save cxpath.mat path_opt

