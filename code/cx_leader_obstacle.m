% 纯跟踪（Pure Pursuit）法
% 作者：Ally
% 日期：20210429
clc
clear
close all
load  path.mat

fol_num=4;        
N=5;             % 4follower and 1 leader
% countmax=2000;
% dt=0.1;
gama=1.5;%机器人之间的影响因子，过大容易造成过冲而抖动
beta=15;%障碍物影响因子
K0=1;
KN=0.2;
% goal=[25 25];
m_count = 0;
is_arrive = 0;
% x最高速度m/s],y最高旋转速度[rad/s],x最高加速度[m/ss],y最高加速度[rad/ss]]
Kinematic=[0.7;0.7;0.4;0.4];
attmse(:,1) = [0;0;0;0;0;0];
error_distance = [0;0;0;0];
color='ybgcrkr'; %%%定义颜色标记
type=[2,1,0.5,0.5,2,2];%%%定义线的类型
start_time = clock;
%% 1-4行为follower 最后一行为leader
A=[0 0 0 0 1;     % a(ij)%%只考虑前面机器人的影响
   1 0 0 0 1;
   0 0 0 0 1;
   0 0 1 0 1;
   0 0 0 0 0];
 %% 初始化 位置pose、速度V、加速度控制量control
init_f=[-1.5 0 pi/4;%%%[x y th] %%队形切换 启动
          -3 0 pi/4; 
          0 -1.5 pi/4;
          0 -3 pi/4;
           0 0 pi/4];  
pose_x=init_f(:,1);
pose_y=init_f(:,2);
pose_th=init_f(:,3);

% %%障碍物坐标[x y]
ob_temp=[1 19.5;10 21.5;10 1.5;15 1.5;15 22;25 1];

%% follower相对leader的位置
delta_x=[-1.5 -3 0 0 0];   % 相对间隔误差   
delta_y=[0 0 -1.5 -3 0];  %领航者与自己无误差
V_x(:,1)=[0;0;0;0;0];
V_y(:,1)=[0;0;0;0;0]; %%%leader在y方向的初始速度为1m/s
k=0;
d_max=2;
detect_R=1;
ideal_posex=init_f(:,1);
ideal_posey=init_f(:,2);

%% 相关参数定义
RefPos = path;
targetSpeed = 1;      % m/s
Kv = 0.1;              % 前视距离系数
Kp = 0.8;              % 速度P控制器系数
Ld0 = 1;               % Ld0是预瞄距离的下限值
dt = 0.1;              % 时间间隔，单位：s
L = 2.0;               % 车辆轴距，单位：m


% 计算参考航向角
diff_x = diff(RefPos(:,1)) ;
diff_x(end+1) = diff_x(end);
diff_y = diff(RefPos(:,2)) ;
diff_y(end+1) = diff_y(end);
refHeading = atan2(diff_y , diff_x);                   % 航向角

%% 主程序

% 车辆初始状态定义
% posR = RefPos(2,:)+1;
pos = [pose_x(5) pose_y(5)];
v = 0;
heading = pose_th(5);
 
% 将初始状态纳入实际状态数组中
pos_actual = pos;
heading_actual = heading;
v_actual = v;
idx = 1;
latError_PP = [];


% 画图
figure
plot(RefPos(:,1), RefPos(:,2), 'b');
xlabel('纵向坐标 / m');
ylabel('横向坐标 / m');
hold on 

% 循环遍历轨迹点
while idx < size(RefPos,1)-1
    k=k+1;
    % 寻找预瞄距离范围内最近路径点
    [lookaheadPoint,idx] = findLookaheadPoint(pos, v, RefPos,Kv, Ld0);
   
    % 计算控制量
    [delta,latError]  = pure_pursuit_control(lookaheadPoint,idx,pos, heading, v, RefPos,refHeading, Kv, Ld0,L);
    
    % 如果误差过大，退出循迹
    if abs(latError) > 3
        disp('误差过大，退出程序!\n')
        break
    end
    
    % 计算加速度
    a = Kp* (targetSpeed-v)/dt;
    
    % 更新状态量
    [pos, heading, v] = updateState(a,pos, heading, v,delta,L, dt);
    
%     pos_new(1) = pos_old(1) + v_old*cos(heading_old)*dt;
%     pos_new(2) =  pos_old(2) + v_old*sin(heading_old)*dt;
%     heading_new=  heading_old + v_old*dt*tan(delta)/wheelbase;
%     v_new =  v_old + a*dt;
    
    V_x(N,k+1)=v*cos(heading); %%设置x,y方向的速度
    V_y(N,k+1)=v*sin(heading);  
    
     ob_pose=ob_temp;
     repulsion=compute_repulsion([pose_x(N,k),pose_y(N,k)],ob_pose,detect_R);        
     %%%%%
     V_x(N,k+1)=V_x(N,k+1)+beta*repulsion(1);
     V_y(N,k+1)=V_y(N,k+1)+beta*repulsion(2);
     
      %%%出现局部极小的情况施加随机扰动 
      if(abs(V_x(N,k+1))<=0.1&&abs(V_y(N,k+1))<=0.1)
%         V_x(N,k+1)=beta*(1+rand(1))*repulsion(1);
%         V_y(N,k+1)=beta*(1+rand(1))*repulsion(2);
          V_x(N,k+1)=-1+2*rand(1);
          V_y(N,k+1)=-1+2*rand(1);
      end
    
    
     %%跟随者运动
     for i=1:fol_num  %fol_num=4      
         sum_delta_x=0;
         sum_delta_y=0;
         for j=1:N %%考虑邻居对它的影响
             sum_delta_x=sum_delta_x+A(i,j)*((pose_x(j,k)-pose_x(i,k))-(delta_x(j)-delta_x(i)));
             sum_delta_y=sum_delta_y+A(i,j)*((pose_y(j,k)-pose_y(i,k))-(delta_y(j)-delta_y(i)));   
         end
%             distance=[];
         error_distance(i,k+1)=sqrt(sum_delta_x^2+ sum_delta_y^2);
         th=atan2(sum_delta_y, sum_delta_x);
%             if error_distance(i,k+1)>d_max
%                 error_distance(i,k+1)=d_max;
%             end
          V_x(i,k+1)=gama*error_distance(i,k+1)*cos(th);
          V_y(i,k+1)=gama*error_distance(i,k+1)*sin(th);
          
           if(rem(k,5)==1&&k>1)
                ideal_posex(i,(k-1)/5+1)=V_x(i,k+1)*dt*5+pose_x(i,k);
                ideal_posey(i,(k-1)/5+1)=V_y(i,k+1)*dt*5+pose_y(i,k);
            end
           %%%考虑冲突避免加上斥力
            kk=0;
            for j=1:N
                if j~=i
                    kk=kk+1;
                    obs_pose(kk,1)=pose_x(j,k);
                    obs_pose(kk,2)=pose_y(j,k);
                end
            end
            ob_pose=[obs_pose;ob_temp];
            repulsion=compute_repulsion([pose_x(i,k),pose_y(i,k)],ob_pose,detect_R);        
            %%%%%
            V_x(i,k+1)=K0*V_x(N,k)+V_x(i,k+1)+beta*repulsion(1);
            V_y(i,k+1)=K0*V_y(N,k)+V_y(i,k+1)+beta*repulsion(2);
            %%%跟随着出现局部极小的情况施加随机扰动 
            if(error_distance(i,k+1)>0.5&&abs(V_x(i,k+1))<=0.1&&abs(V_y(i,k+1))<=0.1&&distance>1)
                V_x(i,k+1)=-1+2*rand(1);
                V_y(i,k+1)=-1+2*rand(1);
                disp(['distance is',num2str(error_distance(i,k+1))]);%打印distance
                disp(['rand V_x is',num2str(V_x(i,k+1))]);
                disp(['rand V_y is',num2str(V_y(i,k+1))]);
            end
     end
     
     for i=1:N
%         out=confine([V_x(i,k) V_y(i,k)],[V_x(i,k+1) V_y(i,k+1)],Kinematic,0.1);
% %             out=[V_x(i,k+1) V_y(i,k+1)];
%         V_x(i,k+1)=out(1);
%         V_y(i,k+1)=out(2);
        pose_x(i,k+1)=pose_x(i,k)+dt*V_x(i,k+1);
        pose_y(i,k+1)=pose_y(i,k)+dt*V_y(i,k+1);
        pose_th(i,k+1)=atan2(V_y(i,k+1),V_x(i,k+1));
     end
    
%     pose_x(N,k+1)=pose_x(N,k)+dt*V_x(N,k+1);
%     pose_y(N,k+1)=pose_y(N,k)+dt*V_y(N,k+1);
%     pose_th(N,k+1)=atan2(V_y(N,k+1),V_x(N,k+1));
    
%     % 保存每一步的实际量
%     pos_actual(end+1,:) = pos;
%     heading_actual(end+1,:) = heading;
%     v_actual(end+1,:) = v;
%     latError_PP(end+1,:) = [idx,latError];
    
    ArrowLength=0.7;% 
    hold off;
    plot(RefPos(:,1), RefPos(:,2), 'b');
    xlabel('纵向坐标 / m');
    ylabel('横向坐标 / m');
    hold on 
   
   
    for j=1:N
%   绘制箭头起始位置-----》目标位置
       quiver(pose_x(j,k+1),pose_y(j,k+1),ArrowLength*cos(pose_th(j,k+1)),ArrowLength*sin(pose_th(j,k+1)),'.','color',color(1,j),'LineWidth',1.3);hold on;
       %绘制原型机器人模型位置 半径 颜色
       draw_circle(pose_x(j,k+1),pose_y(j,k+1),0.1,j);hold on;
    end
     %绘制障碍物  
     obn = size(ob_temp);
     for i =1:obn
         %方形 位置 半径
        draw_square(ob_temp(i,1),ob_temp(i,2),0.2);hold on;
     end
     area=[-15 50 -15 50];
     axis(area);
     grid on;
     drawnow; 
end



%% 画图
    figure                               
    for i=1:N
        plot(pose_x(i,:),pose_y(i,:),color(1,i),'LineWidth',2);
        hold on
    end
    for i=1:N
        plot(pose_x(i,1),pose_y(i,1),'p','color',color(1,i),'LineWidth',2);
        hold on
        draw_circle(pose_x(i,180),pose_y(i,180),0.2,i);hold on;
        draw_circle(pose_x(i,490),pose_y(i,490),0.2,i);hold on;
        draw_circle(pose_x(i,650),pose_y(i,650),0.2,i);hold on;
    end
    for i=1:N
        plot(pose_x(i,k),pose_y(i,k),'h','color',color(1,i),'LineWidth',2);
        hold on
    end
    for i =1:obn
        draw_square(ob_temp(i,1),ob_temp(i,2),0.2);hold on;
    end
    %plot(ob_temp(:,1),ob_temp(:,2),'xk','LineWidth',2);hold on;
    grid on;
    xlabel('x');
    ylabel('y');
    legend('follower1','follower2','follower3','follower4','leader','Location','Northeast');
    xlabel('x Position(m)');
    ylabel('y Position(m)');
    title('基于拓扑图+路径跟踪算法+人工势场的机器人编队避障算法');
%     %% 画误差图
%     cost_time = 3600*(end_time(4)-start_time(4)) + 60 * (end_time(5)-start_time(5)) + (end_time(6) - start_time(6));
%     kx=cost_time/k;
%     cx=0:kx:cost_time;
%     figure                                %   生成三维平面图  连续
%     error=sqrt(error_x.^2+error_y.^2);
%     for i=1:4
%         plot(cx(1:k-1),error(i,1:k-1),color(1,i),'LineWidth',1.5);
%         hold on;
%     end
%     legend('follower1','follower2','follower3','follower4');
%     xlabel('时间(s)');
%     ylabel('位置误差(m)');
%     title('五机器人编队避障各机器人仿真误差曲线');

% % 画图
% figure
% plot(RefPos(:,1), RefPos(:,2), 'b');
% xlabel('纵向坐标 / m');
% ylabel('横向坐标 / m');
% hold on 

% ArrowLength=0.7;% 
% for i = 1:size(pos_actual,1)
% %   绘制箭头起始位置-----》目标位置
%    quiver(pos_actual(i,1),pos_actual(i,2),ArrowLength*cos( heading_actual(i)),ArrowLength*sin(heading_actual(i)),'.','color',color(1,5),'LineWidth',1.3);hold on;
%    %绘制原型机器人模型位置 半径 颜色
%    draw_circle(pos_actual(i,1),pos_actual(i,2),0.1,5);hold on;
%    pause(0.01)
% end

% for i = 1:size(pos_actual,1)
%     scatter(pos_actual(i,1), pos_actual(i,2),150, '.r');
%     pause(0.01)
% end



% for i = 1:size(pos_actual,1)
%     scatter(pos_actual(i,1), pos_actual(i,2),150, '.r');
%     pause(0.01)
% end
%legend('规划车辆轨迹', '实际行驶轨迹')

% 保存
path_PP = pos_actual;
save path_PP.mat path_PP
save latError_PP.mat latError_PP


