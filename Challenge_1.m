clear all
origin = [0 0];
BoB = [1 6]*0.3048;

dist = sqrt((origin(:,1)-BoB(:,1))^2+(origin(:,2)-BoB(:,2))^2);
dir = -(origin-BoB)/dist;
theta = acos(dir(:,1));
v_l = 0.2;
v_r = 0.2;
v = 0.2;
t_forward = dist/v;

d = 0.24;
omega_l = -0.1;
omega_r = 0.1;
omega = (omega_r-omega_l)/d;
t_rotate = theta/omega;

% pub = rospublisher('/raw_vel');
% sub_bump = rossubscriber('/bump');
% msg = rosmessage(pub);

% msg.Data = [omega_l,omega_r];
% send(pub, msg);
% pause(t_rotate);
% msg.Data = [v_l,v_r];
% send(pub, msg);
% pause(t_forward);
% bumpMessage = receive(sub_bump);
% if any(bumpMessage.Data)
%     msg.Data = [0.0, 0.0];
%     send(pub, msg);
%     pause(0.1);
% end
% msg.Data = [0,0];
% send(pub,msg);

[X,Y] = meshgrid([-2:0.5:5],[0:0.5:7]);
Z = log(sqrt((X-BoB(:,1)).^2+(Y-BoB(:,2)).^2));
figure
surf(X,Y,Z)
figure
contour(X,Y,Z)

[U,V] = gradient(Z);
hold on
quiver(X,Y,U,V)
hold off