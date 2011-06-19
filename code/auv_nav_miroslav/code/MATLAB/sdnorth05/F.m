function [x_k, v_k] = F(x_k_1, v_k_1, T)
% non-linear state transition
% inputs can have more than one vector
% state vector has noisy variables included 
x    = x_k_1(1,:);
y    = x_k_1(2,:);
z    = x_k_1(3,:);
a    = x_k_1(4,:);
u    = x_k_1(5,:);
v    = x_k_1(6,:);
w    = x_k_1(7,:);
yaw  = x_k_1(8,:);
pch  = x_k_1(9,:);
r    = x_k_1(10,:);
q    = x_k_1(11,:);
%-----noise variable included---
nu   = v_k_1(1,:);
nv   = v_k_1(2,:);
nw   = v_k_1(3,:);
nr   = v_k_1(4,:);
nq   = v_k_1(5,:);

x_k(1,:) = x + (u*T + nu*(T^2/2)).*cos(yaw).*cos(pch)...
             - (v*T + nv*(T^2/2)).*sin(yaw).*cos(pch);
x_k(2,:) = y + (u*T + nu*(T^2/2)).*sin(yaw).*cos(pch)...
             + (v*T + nv*(T^2/2)).*cos(yaw).*cos(pch);
x_k(3,:) = z + (w*T + nw*(T^2/2)).*cos(pch);
x_k(4,:) = a;
x_k(5,:) = u + nu*T;
x_k(6,:) = v + nv*T;
x_k(7,:) = w + nw*T;
x_k(8,:) = normalize(yaw + r*T + nr*(T^2/2));
x_k(9,:) = normalize(pch + q*T + nq*(T^2/2));
x_k(10,:) = normalize(r + nr*T);
x_k(11,:) = normalize(q + nq*T);

v_k(1,:) = v_k_1(1,1);
v_k(2,:) = v_k_1(2,1);
v_k(3,:) = v_k_1(3,1);
v_k(4,:) = v_k_1(4,1);
v_k(5,:) = v_k_1(5,1);