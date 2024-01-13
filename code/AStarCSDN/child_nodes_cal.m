function child_nodes = child_nodes_cal(parent_node, m, n, obs, closelist)
    child_nodes = [];
    field = [1, 1;
        n, 1;
        n, m;
        1, m];
    
    % 左上子节点
    child_node = [parent_node(1) - 1, parent_node(2) + 1];
    if inpolygon(child_node(1), child_node(2), field(:, 1), field(:, 2))
        % [in, on] = inpolygon, 返回in，以指明xq和yq所指定的查询点是在xv和
        % yv定义的多边行区域的边缘内部还是在边缘上，in为内部，on为边缘上
        if ~ismember(child_node, obs, 'rows')
            child_nodes = [child_nodes; child_node];
        end
    end
    
    
    % 上子节点
    child_node = [parent_node(1), parent_node(2) + 1];
    if inpolygon(child_node(1), child_node(2), field(:, 1), field(:, 2))
        if ~ismember(child_node, obs, 'rows')
            child_nodes = [child_nodes; child_node];
        end
    end
    
    
    % 右上子节点
    child_node = [parent_node(1) + 1, parent_node(2) + 1];
    if inpolygon(child_node(1), child_node(2), field(:, 1), field(:, 2))
        if ~ismember(child_node, obs, 'rows')
            child_nodes = [child_nodes; child_node];
        end
    end
    
    %左子节点
    child_node = [parent_node(1) - 1, parent_node(2)];
    if inpolygon(child_node(1), child_node(2), field(:, 1), field(:, 2))
        if ~ismember(child_node, obs, 'rows')
            child_nodes = [child_nodes; child_node];
        end
    end
    
    % 右子节点
    child_node = [parent_node(1) + 1, parent_node(2)];
    if inpolygon(child_node(1), child_node(2), field(:, 1), field(:, 2))
        if ~ismember(child_node, obs, 'rows')
            child_nodes = [child_nodes; child_node];
        end
    end
    
    
    % 左下子节点
    child_node = [parent_node(1) - 1, parent_node(2) - 1];
    if inpolygon(child_node(1), child_node(2), field(:, 1), field(:, 2))
        if ~ismember(child_node, obs, 'rows')
            child_nodes = [child_nodes; child_node];
        end
    end
    
    
    % 下子节点
    child_node = [parent_node(1), parent_node(2) - 1];
    if inpolygon(child_node(1), child_node(2), field(:, 1), field(:, 2))
        if ~ismember(child_node, obs, 'rows')
            child_nodes = [child_nodes; child_node];
        end
    end
    
    
    % 右下子节点
    child_node = [parent_node(1) + 1, parent_node(2) - 1];
    if inpolygon(child_node(1), child_node(2), field(:, 1), field(:, 2))
        if ~ismember(child_node, obs, 'rows')
            child_nodes = [child_nodes; child_node];
        end
    end
    
    
    %% 排除已经存在于closelist的节点?    
    delete_idx = [];
    for i = 1 : size(child_nodes, 1)
        if ismember(child_nodes(i, :), closelist, 'rows')
            delete_idx(end+1, :) = i;
        end
    end
    
    child_nodes(delete_idx, :) = [];
    
    
end