function [pos_new, heading_new, v_new] = updateState(a,pos_old, heading_old, v_old,delta,wheelbase, dt)
pos_new(1) = pos_old(1) + v_old*cos(heading_old)*dt;
pos_new(2) =  pos_old(2) + v_old*sin(heading_old)*dt;
heading_new=  heading_old + v_old*dt*tan(delta)/wheelbase;
v_new =  v_old + a*dt;
end