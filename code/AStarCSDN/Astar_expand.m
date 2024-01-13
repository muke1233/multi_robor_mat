clc;
clear
close all

tic

%% 画地图
% 栅格地图的行数 列数定义
m = 150;
n = 150;
% 地图m行n列?
start = [10, 20];        % 起始节点
target = [130, 130];       % 终止节点
obs = [6, 1; 6, 2; 6, 3; 6, 4; 6, 5; 6, 6; 6, 7; 5, 5; 4, 5; 3, 5];   % 闅滅鐗╁尯鍩?
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
             % in_flag表示该obs_ex是否在obs_ex1中?      
            % obs_ex1_idx表示该obs_ex在obs_ex1中的行数
            if inflag==0
                obs_ex1(end+1,:) = obs_ex;
            end
        end
    end
end



% 画格子
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

% 绘制障碍物 起止点颜色块
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



%% 预处理?
% 初始化 closelist
closelist = start;
closelist_path = {start, start};      % 路径，从自身到自身?
closelist_cost = 0;
children = child_nodes_cal(start, m, n, obs, closelist);

% 初始化openlist
openlist = children;


for i = 1 : size(openlist, 1)   % i为第i个节点?    
    openlist_path{i, 1} = openlist(i, :);   % openlist_path的第i行第1列为第i个节点child  
    openlist_path{i, 2} = [start; openlist(i, :)]; % openlist_path的第i行第2列为一个列向量
    % 分别是起始节点和当前child_node
end

for i = 1 : size(openlist, 1)
    g = norm(start - openlist(i, 1:2));%范数
    h = abs(target(1) - openlist(i, 1)) + abs(target(2) - openlist(i, 2));%曼哈顿距离
%     h = sqrt((target_node(1) - openlist(i, 1))^2 + (target_node(2) - openlist(i, 2))^2);
    f = g + h;
    openlist_cost(i, :) = [g, h, f];
end

%%定义父节点
% 从openlist开始搜索移动代价最小的节点?
[~, min_idx] = min(openlist_cost(:, 3));    % 看f值最小，min_idx为f最小的那一行?
parent = openlist(min_idx, :);% 以min_idx该行的子节点child_node为新的父节点

%% 进入循环
flag = 1;
while flag
    % 找出父节点的忽略closelist的子节点
    children = child_nodes_cal(parent ,m, n, obs, closelist);
    
    % 判断这些子节点是否在openlist中若在，则更新；没在，则追加到openlist中
    for i = 1 : size(children, 1)
        child = children(i, :);
        [in_flag, openlist_idx] = ismember(child, openlist, 'rows');
        % in_flag表示该child_node是否在openlist中?      
        % openlist_idx表示该child_node在openlist中的行数
        
        g = openlist_cost(min_idx, 1) + norm(parent - child);
        % 原来的g加上新的g
        h = abs(child(1) - target(1)) + abs(child(2) - target(2));
%         h = sqrt((target_node(1) - openlist(i, 1))^2 + (target_node(2) - openlist(i, 2))^2);
        f = g + h;
        
        if in_flag  % 若在，则比较更新g, f
            if g < openlist_cost(openlist_idx, 1)   
                % openlist_cost(openlist_idx,1)指的是openlist_cost中idx这一行（即child_node所在的一行）的第一个坐标，即g  
                openlist_cost(openlist_idx, 1) = g;
                openlist_cost(openlist_idx, 3) = f;
                openlist_path{openlist_idx, 2} = [openlist_path{min_idx, 2}; child];
                % openlist_path的第i行第2列为一个列向量，
                %分别是起始节点和当前child，此处定义新的起始节点为openlist_path(min_idx,2)， 
                %而openlist_path(min_idx,2)指第min_idx行所对应的 child在openlist_path中对应的路径，
                %相当于把新的child附加到了路径上，延长了路径
                  
            end
         else
            openlist(end+1, :) = child;
            openlist_cost(end+1, :) = [g, h, f];
            openlist_path{end+1, 1} = child;
            openlist_path{end, 2} = [openlist_path{min_idx, 2}; child];
         end
     end
    
    
    % 从openlist移除代价最小的节点到closelist
    closelist(end+1, :) = openlist(min_idx, :);
    closelist_cost(end+1, :) = openlist_cost(min_idx, 3);
    closelist_path(end+1, :) = openlist_path(min_idx, :);
    % 同样地，openlist中少了该节点
    openlist(min_idx, :) = [];
    openlist_cost(min_idx, :) = [];
    openlist_path(min_idx, :) = [];
    
    
    % 重新搜索：从openlist搜索移动代价最小的节点，作为新的父节点  
    [~, min_idx] = min(openlist_cost(:, 3));
    parent = openlist(min_idx, :);
    
    % 判断是否搜索到终点    
    if parent == target
        closelist(end+1, :) = openlist(min_idx, :);
        closelist_cost(end+1, 1) = openlist_cost(min_idx, 1);
        closelist_path(end+1, :) = openlist_path(min_idx, :);
        flag = 0;
    end   

end
%toc

%% 画图
path_opt = closelist_path{end, 2};
% closelist_path中第二列存放路径，所以path_opt存放的是路径的(x,y)值
path_opt(:, 1) = path_opt(:, 1) - 0.5;
path_opt(:, 2) = path_opt(:, 2) - 0.5;
% scatter(path_opt(:, 1), path_opt(:, 2), 60, 'g');
plot(path_opt(:, 1), path_opt(:, 2), 'm', 'linewidth', 1.5);

title(['Total length of path: ' num2str(closelist_cost(end, 1))]);
% legend('Start node', 'Target node', 'Obstacle', 'Wateface', 'Path');
legend('Start node', 'Target node', 'Obstacle', 'Path', 'location', 'northwest');
set(gca, 'fontsize', 15, 'fontname', 'times new roman');

save cxpath.mat path_opt

