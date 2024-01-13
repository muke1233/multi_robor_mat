%% 首先在参考轨迹上搜索离当前车辆位置最近的点
function  [lookaheadPoint,idx_target] = findLookaheadPoint(pos, v, RefPos, Kv, Ld0)

% 找到距离当前位置最近的一个参考轨迹点的序号
sizeOfRefPos = size(RefPos,1);
for i = 1:sizeOfRefPos
    dist(i,1) = norm(RefPos(i,:) - pos);   
end
[~,idx] = min(dist); 


% 从该点开始向轨迹前方搜索，找到与预瞄距离最相近的一个轨迹点
L_steps = 0;           % 参考轨迹上几个相邻点的累计距离
Ld = Kv*v + Ld0;       % Ld0是预瞄距离的下限值；
while L_steps < Ld && idx < sizeOfRefPos
    L_steps = L_steps + norm(RefPos(idx + 1,:) - RefPos(idx,:));
    idx = idx+1;
end
idx_target = idx;
lookaheadPoint = RefPos(idx,:);
end