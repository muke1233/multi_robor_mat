function obs = TrunToGridMap(a, b)

    I=imread('2.jpg');   %����ͼƬ
    I = rgb2gray(I);     %��ͼƬתΪ�Ҷ�ͼ
    
    
    
    I = imrotate(I, -90);%��ת90��
   % BW = imrotate(BW, -90);%��ת90��

%     a=500;
%     b=500;
    l=1;    %����߳�
    B = imresize(I,[a/l b/l]);%  ����ͼ�� 
    J=floor(B/255);  % �����������ȡ�� 
   
    

    %B = ~J
   % BW = imdilate(J, strel('square', 3));
    
    axes('GridLineStyle', '-');
    axis equal;

%     set(gca,'ydir','reverse');     %y坐标调换�?%     set(gca,'xdir','reverse')     %x坐标调换
    hold on
    grid on
    axis([0,a,0,b]);
    set(gca,'xtick',0:10:a,'ytick',0:10:b);
    % set(gca,'xtick',longitude,'ytick',latitude)

    obs = [];

    %�ϰ������Ϊ��ɫ
    for i=1:a/l-1
        for j=1:b/l-1
            if(J(i,j)==0)
                y=[i,i,i+1,i+1]*l;
                x=[j,j+1,j+1,j]*l;
%                 h=fill(x,y,'k');
                obs(end+1, :) = [i, j];
%                 hold on
            end
        end
    end

end