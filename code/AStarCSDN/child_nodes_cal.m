function child_nodes = child_nodes_cal(parent_node, m, n, obs, closelist)
    child_nodes = [];
    field = [1, 1;
        n, 1;
        n, m;
        1, m];
    
    % �����ӽڵ�
    child_node = [parent_node(1) - 1, parent_node(2) + 1];
    if inpolygon(child_node(1), child_node(2), field(:, 1), field(:, 2))
        % [in, on] = inpolygon, ����in����ָ��xq��yq��ָ���Ĳ�ѯ������xv��
        % yv����Ķ��������ı�Ե�ڲ������ڱ�Ե�ϣ�inΪ�ڲ���onΪ��Ե��
        if ~ismember(child_node, obs, 'rows')
            child_nodes = [child_nodes; child_node];
        end
    end
    
    
    % ���ӽڵ�
    child_node = [parent_node(1), parent_node(2) + 1];
    if inpolygon(child_node(1), child_node(2), field(:, 1), field(:, 2))
        if ~ismember(child_node, obs, 'rows')
            child_nodes = [child_nodes; child_node];
        end
    end
    
    
    % �����ӽڵ�
    child_node = [parent_node(1) + 1, parent_node(2) + 1];
    if inpolygon(child_node(1), child_node(2), field(:, 1), field(:, 2))
        if ~ismember(child_node, obs, 'rows')
            child_nodes = [child_nodes; child_node];
        end
    end
    
    %���ӽڵ�
    child_node = [parent_node(1) - 1, parent_node(2)];
    if inpolygon(child_node(1), child_node(2), field(:, 1), field(:, 2))
        if ~ismember(child_node, obs, 'rows')
            child_nodes = [child_nodes; child_node];
        end
    end
    
    % ���ӽڵ�
    child_node = [parent_node(1) + 1, parent_node(2)];
    if inpolygon(child_node(1), child_node(2), field(:, 1), field(:, 2))
        if ~ismember(child_node, obs, 'rows')
            child_nodes = [child_nodes; child_node];
        end
    end
    
    
    % �����ӽڵ�
    child_node = [parent_node(1) - 1, parent_node(2) - 1];
    if inpolygon(child_node(1), child_node(2), field(:, 1), field(:, 2))
        if ~ismember(child_node, obs, 'rows')
            child_nodes = [child_nodes; child_node];
        end
    end
    
    
    % ���ӽڵ�
    child_node = [parent_node(1), parent_node(2) - 1];
    if inpolygon(child_node(1), child_node(2), field(:, 1), field(:, 2))
        if ~ismember(child_node, obs, 'rows')
            child_nodes = [child_nodes; child_node];
        end
    end
    
    
    % �����ӽڵ�
    child_node = [parent_node(1) + 1, parent_node(2) - 1];
    if inpolygon(child_node(1), child_node(2), field(:, 1), field(:, 2))
        if ~ismember(child_node, obs, 'rows')
            child_nodes = [child_nodes; child_node];
        end
    end
    
    
    %% �ų��Ѿ�������closelist�Ľڵ�?    
    delete_idx = [];
    for i = 1 : size(child_nodes, 1)
        if ismember(child_nodes(i, :), closelist, 'rows')
            delete_idx(end+1, :) = i;
        end
    end
    
    child_nodes(delete_idx, :) = [];
    
    
end