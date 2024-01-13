%% �����ڲο��켣�������뵱ǰ����λ������ĵ�
function  [lookaheadPoint,idx_target] = findLookaheadPoint(pos, v, RefPos, Kv, Ld0)

% �ҵ����뵱ǰλ�������һ���ο��켣������
sizeOfRefPos = size(RefPos,1);
for i = 1:sizeOfRefPos
    dist(i,1) = norm(RefPos(i,:) - pos);   
end
[~,idx] = min(dist); 


% �Ӹõ㿪ʼ��켣ǰ���������ҵ���Ԥ������������һ���켣��
L_steps = 0;           % �ο��켣�ϼ������ڵ���ۼƾ���
Ld = Kv*v + Ld0;       % Ld0��Ԥ����������ֵ��
while L_steps < Ld && idx < sizeOfRefPos
    L_steps = L_steps + norm(RefPos(idx + 1,:) - RefPos(idx,:));
    idx = idx+1;
end
idx_target = idx;
lookaheadPoint = RefPos(idx,:);
end