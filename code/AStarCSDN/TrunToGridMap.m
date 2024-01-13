function obs = TrunToGridMap(a, b)

    I=imread('2.jpg');   %读入图片
    I = rgb2gray(I);     %将图片转为灰度图
    
    
    
    I = imrotate(I, -90);%旋转90度
   % BW = imrotate(BW, -90);%旋转90度

%     a=500;
%     b=500;
    l=1;    %网格边长
    B = imresize(I,[a/l b/l]);%  缩放图像 
    J=floor(B/255);  % 朝负无穷大方向取整 
   
    

    %B = ~J
   % BW = imdilate(J, strel('square', 3));
    
    axes('GridLineStyle', '-');
    axis equal;

%     set(gca,'ydir','reverse');     %y鍧愭爣璋冩崲锛?%     set(gca,'xdir','reverse')     %x鍧愭爣璋冩崲
    hold on
    grid on
    axis([0,a,0,b]);
    set(gca,'xtick',0:10:a,'ytick',0:10:b);
    % set(gca,'xtick',longitude,'ytick',latitude)

    obs = [];

    %障碍物填充为黑色
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