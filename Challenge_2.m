clear all
[X,Y] = meshgrid([-2:0.1:5],[0:0.1:7]);

origin = [2 0];
boxes(1,:) = [-1.33 4.5];
boxes(2,:) = [2 3];
boxes(3,:) = [2 6];
BoB = [1 6];
s = 0.5;

Z = 1.5*log(sqrt((X-BoB(:,1)).^2+(Y-BoB(:,2)).^2))-s*log(sqrt((X-origin(:,1)).^2+(Y-origin(:,2)).^2))-s*log(sqrt((X-boxes(1,1)).^2+(Y-boxes(1,2)).^2))-s*log(sqrt((X-boxes(2,1)).^2+(Y-boxes(2,2)).^2))-s*log(sqrt((X-boxes(3,1)).^2+(Y-boxes(3,2)).^2));
[U,V] = gradient(Z);
mag = sqrt(U.^2 + V.^2);
norm_U = -U./mag;
norm_V = -V./mag;
% figure
% contour(X,Y,Z)
% hold on
% quiver(X,Y,norm_U,norm_V)
% hold off
%% Actually Running
pub = rospublisher('/raw_vel');
sub_bump = rossubscriber('/bump');
msg = rosmessage(pub);

d = 0.24; 
v_l = 0.1;
v_r = 0.1;
v = 0.1;
omega_l = -0.1;
omega_r = 0.1;
omega = (omega_r-omega_l)/d;

theta(1) = 0;
pt1 = [0 2];
dist = sqrt((origin(:,1)-pt1(:,1))^2+(origin(:,2)-pt1(:,2))^2);
dir = (pt1-origin)./dist;
theta(2) = pi-asin(dir(:,2));
d_theta = theta(2) - theta(1);
t_forward = (dist*.3048)/v;
t_rotate = d_theta/omega;
msg.Data = [omega_l,omega_r];
send(pub, msg);
pause(t_rotate);
msg.Data = [v_l,v_r];
send(pub, msg); 
delta = v*t_forward .* dir .* 3.2808;
pause(t_forward);
position = origin + delta;
pt2 = [0 4];
dist = sqrt((pt1(:,1)-pt2(:,1))^2+(pt1(:,2)-pt2(:,2))^2);
dir = (pt2-pt1)./dist;
theta(3) = pi/2;
d_theta = theta(3) - theta(2);
t_forward = (dist*.3048)/v;
t_rotate = abs(d_theta)/omega;
msg.Data = [omega_r,omega_l];
send(pub, msg);
pause(t_rotate);
msg.Data = [v_l,v_r];
send(pub, msg);
delta = v*t_forward.* dir .* 3.2808;
pause(t_forward); 
position = position + delta;
counter = 3;
while abs(round(position(1),1)-1) > 0.1 && abs(round(position(2),1)-6) > 0.1
    counter = counter +1;
    x = 10*round(position(1),1)+21;
    y = 10*round(position(2),1)+1;
    heading = [norm_U(y,x) norm_V(y,x)];
    theta(counter) = acos(heading(1));
    d_theta = theta(counter)-theta(counter-1);
    t_rotate = abs(d_theta)/omega;
    if d_theta > 0
        msg.Data = [omega_l,omega_r];
    end
    if d_theta < 0
        msg.Data = [omega_r,omega_l];
    end
    send(pub,msg);
    pause(t_rotate);
    msg.Data = [v_l, v_r];
    send(pub,msg);
    pause(1);
    bumpMessage = receive(sub_bump);
    if any(bumpMessage.Data)
        msg.Data = [0.0, 0.0];
        send(pub, msg);
        pause(0.1);
    end
    delta = v*1.* heading*3.2808;
    position = position + delta;
end
msg.Data = [0,0];
send(pub, msg);