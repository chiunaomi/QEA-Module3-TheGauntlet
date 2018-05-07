% load qea_gauntlet_1.mat

% sub = rossubscriber('/stable_scan');
% for i = 1:4
%     scan_message = receive(sub);
%     r(:,i) = scan_message.Ranges(1:end-1);
%     theta(:,i) = [0:359]';
% end

origin = [0 0];
BoB = [1 6];
s = 0.75;

clean_index = 0;
for j = 1:4
    for i = 1:size(r,1)
        if r(i,:) ~= 0
            clean_index = clean_index+1;
            pos(1,clean_index) = r(i,j)*3.2808*cosd(theta(i,j));
            pos(2,clean_index) = r(i,j)*3.2808*sind(theta(i,j));
        end
    end
end
% for i = 1:size(r_2,1)
%     if r_2(i,:) ~= 0
%         clean_index = clean_index+1;
%         pos(1,clean_index) = r_2(i)*cosd(theta_2(i))+0.6;
%         pos(2,clean_index) = r_2(i)*sind(theta_2(i));
%     end
% end
% figure
% plot(pos(1,:),pos(2,:),'o');
within_range_index = 0;
for i = 1:size(pos,2)
    if pos(1,i) > -4 && pos(1,i) < 5 && pos(2,i) > -2 && pos(2,i) < 7
        within_range_index = within_range_index + 1;
        cart(1,within_range_index) = pos(1,i);
        cart(2,within_range_index) = pos(2,i);
    end
end
figure
plot(cart(1,:),cart(2,:),'ro');

[X,Y] = meshgrid([-3:0.1:5],[-2:0.1:8]);
Z =(200*log(sqrt((X-BoB(1)).^2+(Y-BoB(2)).^2)))-s*log(sqrt((X-origin(1)).^2+(Y-origin(2)).^2));
for i = 1:size(cart,2)
    f = log(sqrt((X-cart(1,i)).^2+(Y-cart(2,i)).^2));
    Z = Z-(s*f);
end
[U,V] = gradient(Z);
mag = sqrt(U.^2 + V.^2);
norm_U = -U./mag;
norm_V = -V./mag;
% norm_U(10:30,11:31) = -sqrt(2);
% norm_V(10:30,11:31) = sqrt(2);
% norm_U(21,31) = -1
% norm_V(21,31) = 1
figure
contour(X,Y,Z)
hold on
streamslice(X,Y,norm_U,norm_V)
quiver(X(21,31),Y(21,31),norm_U(21,31),norm_V(21,31),'Color',[1,0,0]);
hold off
%% Run
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

position = origin;
counter = 1;
theta(1) = 0;
while abs(round(position(1),1)-1) > 0.1 && abs(round(position(2),1)-6) > 0.1
    counter = counter+1;
    x = 10*round(position(1),1)+31;
    y = 10*round(position(2),1)+21;
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
    pause(0.5);
    bumpMessage = receive(sub_bump);
    if any(bumpMessage.Data)
        msg.Data = [0.0, 0.0];
        send(pub, msg);
        pause(0.1);
    end
    delta = v*0.5.* heading*3.2808;
    position = position + delta;
end
msg.Data = [0,0];
send(pub, msg);