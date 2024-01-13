%% 获得控制量：前轮转向
function [delta,latError] = pure_pursuit_control(lookaheadPoint,idx,pos, heading, v,RefPos,refHeading, Kv, Ld0, L)
sizeOfRefPos = size(RefPos,1);
if idx < sizeOfRefPos
    Point_temp = lookaheadPoint;
else
    Point_temp = RefPos(end,1:2);
end
alpha = atan2(Point_temp(1,2) - pos(2), Point_temp(1,1) - pos(1))  - heading;
Ld = Kv*v + Ld0;

% 求位置、航向角的误差
x_error  = pos(1) - RefPos(idx,1);
y_error = pos(2) - RefPos(idx,2);
heading_r = refHeading(idx);
% 根据百度Apolo，计算横向误差
latError = y_error*cos(heading_r) - x_error*sin(heading_r);

% 前轮转角
delta = atan2(2*L*sin(alpha), Ld);

end