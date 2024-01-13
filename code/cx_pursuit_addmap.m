% �����٣�Pure Pursuit����
% ���ߣ�Ally
% ���ڣ�20210429
clc
clear
close all
load  cxpath.mat

%% ����ͼ
% դ���ͼ������ ��������
m = 150;
n = 150;
% ��ͼm��n��?
start = [10, 20];        % ��ʼ�ڵ�
target = [130, 80];       % ��ֹ�ڵ�
% obs = [6, 1; 6, 2; 6, 3; 6, 4; 6, 5; 6, 6; 6, 7; 5, 5; 4, 5; 3, 5];   % 障碍物区�?

obs = TrunToGridMap(m, n);


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

%scatter(start(1), start(2), 700, 'pg', 'filled');

% fill([target_node(1)-1, target_node(1), target_node(1), target_node(1)-1],...
%     [target_node(2)-1, target_node(2)-1, target_node(2), target_node(2)], 'r');

%scatter(target(1), target(2), 700, 'pr', 'filled');

for i = 1 : size(obs, 1) - 1
    temp = obs(i, :);
    fill([temp(1)-1, temp(1), temp(1), temp(1)-1],...
        [temp(2)-1, temp(2)-1, temp(2), temp(2)], 'k', 'handlevisibility', 'off');
end

fol_num=4;        
N=5;             % 4follower and 1 leader
% countmax=2000;
% dt=0.1;
gama=1.5;%������֮���Ӱ�����ӣ�����������ɹ��������
beta=15;%�ϰ���Ӱ������
K0=1;
KN=0.2;
% goal=[25 25];
m_count = 0;
is_arrive = 0;
% x����ٶ�m/s],y�����ת�ٶ�[rad/s],x��߼��ٶ�[m/ss],y��߼��ٶ�[rad/ss]]
Kinematic=[0.7;0.7;0.4;0.4];
attmse(:,1) = [0;0;0;0;0;0];
error_distance = [0;0;0;0];
color='ybgcrkr'; %%%������ɫ���
type=[2,1,0.5,0.5,2,2];%%%�����ߵ�����
start_time = clock;
%% 1-4��Ϊfollower ���һ��Ϊleader
A=[0 0 0 0 1;     % a(ij)%%ֻ����ǰ������˵�Ӱ��
   1 0 0 0 1;
   0 0 0 0 1;
   0 0 1 0 1;
   0 0 0 0 0];
 %% ��ʼ�� λ��pose���ٶ�V�����ٶȿ�����control
init_f=[-1.5 0 pi/4;%%%[x y th] %%�����л� ����
          -3 0 pi/4; 
          0 -1.5 pi/4;
          0 -3 pi/4;
           10 20 pi/4];  
pose_x=init_f(:,1);
pose_y=init_f(:,2);
pose_th=init_f(:,3);

% %%�ϰ�������[x y]
%ob_temp=[7 19.5;10 21.5;13 22];
ob_temp=[];

%% follower���leader��λ��
delta_x=[-1.5 -3 0 0 0];   % ��Լ�����   
delta_y=[0 0 -1.5 -3 0];  %�캽�����Լ������
V_x(:,1)=[0;0;0;0;0];
V_y(:,1)=[0;0;0;0;0]; %%%leader��y����ĳ�ʼ�ٶ�Ϊ1m/s
k=0;
d_max=2;
detect_R=1.5;
ideal_posex=init_f(:,1);
ideal_posey=init_f(:,2);

%% ��ز�������
RefPos = path_opt;
targetSpeed = 0.7;      % m/s
Kv = 0.1;              % ǰ�Ӿ���ϵ��
Kp = 0.8;              % �ٶ�P������ϵ��
Ld0 = 2;               % Ld0��Ԥ����������ֵ
dt = 0.1;              % ʱ��������λ��s
L = 2.9;               % ������࣬��λ��m


% ����ο������
diff_x = diff(RefPos(:,1)) ;
diff_x(end+1) = diff_x(end);
diff_y = diff(RefPos(:,2)) ;
diff_y(end+1) = diff_y(end);
refHeading = atan2(diff_y , diff_x);                   % �����

%% ������

% ������ʼ״̬����
% posR = RefPos(2,:)+1;
pos = [pose_x(5) pose_y(5)];
v = 0;
heading = pose_th(5);
 
% ����ʼ״̬����ʵ��״̬������
pos_actual = pos;
heading_actual = heading;
v_actual = v;
idx = 1;
latError_PP = [];


% ��ͼ
%figure
plot(RefPos(:,1), RefPos(:,2), 'b');
xlabel('�������� / m');
ylabel('�������� / m');
hold on 

% ѭ�������켣��
while idx < size(RefPos,1)-1
    k=k+1;
    % Ѱ��Ԥ����뷶Χ�����·����
    [lookaheadPoint,idx] = findLookaheadPoint(pos, v, RefPos,Kv, Ld0);
   
    % ���������
    [delta,latError]  = pure_pursuit_control(lookaheadPoint,idx,pos, heading, v, RefPos,refHeading, Kv, Ld0,L);
    
    % ����������˳�ѭ��
    if abs(latError) > 5
        disp('�������˳�����!\n')
        break
    end
    
    % ������ٶ�
    a = Kp* (targetSpeed-v)/dt;
    
    % ����״̬��
    [pos, heading, v] = updateState(a,pos, heading, v,delta,L, dt);
    
%     pos_new(1) = pos_old(1) + v_old*cos(heading_old)*dt;
%     pos_new(2) =  pos_old(2) + v_old*sin(heading_old)*dt;
%     heading_new=  heading_old + v_old*dt*tan(delta)/wheelbase;
%     v_new =  v_old + a*dt;
    
    V_x(N,k+1)=v*cos(heading); %%����x,y������ٶ�
    V_y(N,k+1)=v*sin(heading);   
    
    
     ob_pose=ob_temp;
     repulsion=compute_repulsion([pose_x(N,k),pose_y(N,k)],ob_pose,detect_R);        
     %%%%%
     V_x(N,k+1)=V_x(N,k+1)+beta*repulsion(1);
     V_y(N,k+1)=V_y(N,k+1)+beta*repulsion(2);
     
      %%%���־ֲ���С�����ʩ������Ŷ� 
      if(abs(V_x(N,k+1))<=0.1&&abs(V_y(N,k+1))<=0.1)
%         V_x(N,k+1)=beta*(1+rand(1))*repulsion(1);
%         V_y(N,k+1)=beta*(1+rand(1))*repulsion(2);
          V_x(N,k+1)=-1+2*rand(1);
          V_y(N,k+1)=-1+2*rand(1);
      end
    
     pose_x(N,k+1)=pose_x(N,k)+dt*V_x(N,k+1);
     pose_y(N,k+1)=pose_y(N,k)+dt*V_y(N,k+1);
     pose_th(N,k+1)=atan2(V_y(N,k+1),V_x(N,k+1));
     
%     % ����ÿһ����ʵ����
%     pos_actual(end+1,:) = pos;
%     heading_actual(end+1,:) = heading;
%     v_actual(end+1,:) = v;
%     latError_PP(end+1,:) = [idx,latError];
    
    

    ArrowLength=0.7;% 
%      hold off;
%   ���Ƽ�ͷ��ʼλ��-----��Ŀ��λ��
    quiver(pose_x(N,k+1),pose_y(N,k+1),ArrowLength*cos(pose_th(N,k+1)),ArrowLength*sin(pose_th(N,k+1)),'.','color',color(1,5),'LineWidth',1.3);hold on;
    %����ԭ�ͻ�����ģ��λ�� �뾶 ��ɫ
    draw_circle(pose_x(N,k+1),pose_y(N,k+1),0.1,N);hold on;
    
     %�����ϰ���  
     obn = size(ob_temp);
     for i =1:obn
         %���� λ�� �뾶
        draw_square(ob_temp(i,1),ob_temp(i,2),0.2);hold on;
     end
        
%     %   ���Ƽ�ͷ��ʼλ��-----��Ŀ��λ��
%    quiver(pos(1),pos(2),ArrowLength*cos( heading),ArrowLength*sin(heading),'.','color',color(1,5),'LineWidth',1.3);hold on;
%    %����ԭ�ͻ�����ģ��λ�� �뾶 ��ɫ
%    draw_circle(pos(1),pos(2),0.1,5);hold on;

     area=[0 150 0 150];
     axis(area);
     grid on;
     drawnow; 
end

% % ��ͼ
% figure
% plot(RefPos(:,1), RefPos(:,2), 'b');
% xlabel('�������� / m');
% ylabel('�������� / m');
% hold on 

% ArrowLength=0.7;% 
% for i = 1:size(pos_actual,1)
% %   ���Ƽ�ͷ��ʼλ��-----��Ŀ��λ��
%    quiver(pos_actual(i,1),pos_actual(i,2),ArrowLength*cos( heading_actual(i)),ArrowLength*sin(heading_actual(i)),'.','color',color(1,5),'LineWidth',1.3);hold on;
%    %����ԭ�ͻ�����ģ��λ�� �뾶 ��ɫ
%    draw_circle(pos_actual(i,1),pos_actual(i,2),0.1,5);hold on;
%    pause(0.01)
% end

% for i = 1:size(pos_actual,1)
%     scatter(pos_actual(i,1), pos_actual(i,2),150, '.r');
%     pause(0.01)
% end
legend('�滮�����켣', 'ʵ����ʻ�켣')

% ����
path_PP = pos_actual;
save path_PP.mat path_PP
save latError_PP.mat latError_PP


