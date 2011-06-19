function [XA,PA]=UnscentedKalman(XA, PA, z, T, conf, lambda)
%global PARAM DATA whichDevice whichValues
Pv = PA(12:16, 12:16);
Pn = PA(17:27, 17:27);
%calculate sigma points
L = size(XA, 1); % state length
[sigmaVec, Wm, Wc] = unscentedTransform(XA, PA, lambda);
chi_x = sigmaVec(1:11,:);
chi_v = sigmaVec(12:16,:);
chi_n = sigmaVec(17:27,:);
%time update
chi_x = F(chi_x, chi_v, T);
x_pred = (Wm * chi_x')'; x_pred_normal  = x_pred(1:7,:); 
                         x_pred_angular = x_pred(8:11,:);
                         x_pred_angular = normalize(x_pred_angular);
                         x_pred = [x_pred_normal; x_pred_angular];

P_pred = 0; % start the sum
for i = 1:2*L+1,
    T = chi_x(:,i)-x_pred;
    T_normal = T(1:7,:); 
    T_angular  = T(8:11,:);
    T_angular  = normalize(T_angular);
    T = [T_normal; T_angular];
    P_pred = P_pred + Wc(i)*T*T';
end
if(length(z)==1 && isnan(z)),
    % no need for observation
    XA = [x_pred; sigmaVec(12:27,1)];
    PA = [P_pred       zeros(11,5) zeros(11,11); ...
          zeros(5,11)  Pv          zeros(5 ,11); ...
          zeros(11,11) zeros(11,5) Pn         ];
else

    chi_z = H(chi_x,chi_n, conf);
    z_pred = (Wm * chi_z')';

    % measurement update equations
    Pyy = 0;
    Pxy = 0;
    for i = 1 : 2*L+1,
        T = chi_z(:,i)-z_pred;
        %T((conf>=8) & (conf<=11), :) = normalize(T((conf>=8) & (conf<=11), :));
        
        A = chi_x(:,i)-x_pred;
        %A(8:11, :) = normalize(A(8:11, :));
        
        Pyy = Pyy + Wc(i)*T*T';
        Pxy = Pxy + Wc(i)*A*T';
    end
    K = Pxy * Pyy^(-1);
    INN = z'-z_pred;
    INN((conf>=8) & (conf<=11)) = normalize(INN((conf>=8) & (conf<=11)));
    x = x_pred + K * INN;
    P = P_pred - K*Pyy*K';
    XA = [x; sigmaVec(12:27,1)];
    PA = [P            zeros(11,5) zeros(11,11); ...
          zeros(5,11)  Pv          zeros(5 ,11); ...
          zeros(11,11) zeros(11,5) Pn         ];
end
% mesLength   = size(z_k, 1);
% % store input state
% x    = x_k_1_k_1(1);
% y    = x_k_1_k_1(2);
% z    = x_k_1_k_1(3);
% yaw  = normalize(x_k_1_k_1(4));
% pitch= normalize(x_k_1_k_1(5));
% u    = x_k_1_k_1(6);
% v    = x_k_1_k_1(7);
% w    = x_k_1_k_1(8);
% r    = x_k_1_k_1(9);
% q    = x_k_1_k_1(10);
% % represent the input density with sample points
% 
% %[chi_k_1, W] = unscentedTransform(x_k_1_k_1, P_k_1_k_1, k);
% %--PREDICTION---------------------------------------------------
% x_k_k_1 = f(chi_k_1, t)* W';
%     if ~isreal(x_k_k_1),
%         disp('complex number!');
%     end
% %Jacobian of partial derivates of the funcion respect to noise s (noise propagated through linear and angular velocities)            
% W_noise=[ 1/2*t^2*cos(yaw)*cos(pitch),...
%        -1/2*t^2*sin(yaw)*cos(pitch),...
%            0,...
%               -(1/2)*(u*t)*t^2*sin(yaw)*cos(pitch)-(1/2)*(v*t)*t^2*cos(yaw)*cos(pitch),...
%                  -(1/2)*(u*t)*t^2*cos(yaw)*sin(pitch)+(1/2)*(v*t)*t^2*sin(yaw)*sin(pitch);
%                  
%     1/2*t^2*sin(yaw)*cos(pitch),...
%        1/2*t^2*cos(yaw)*cos(pitch),...
%           0,...
%              (1/2)*(u*t)*t^2*cos(yaw)*cos(pitch)-(1/2)*(v*t)*t^2*sin(yaw)*cos(pitch),...
%                 -(1/2)*(u*t)*t^2*sin(yaw)*sin(pitch)+(1/2)*(v*t)*t^2*cos(yaw)*sin(pitch);
%     0,...
%        0,...
%           (1/2)*t^2*sin(pitch),...
%              0,...
%                 (1/2)*(w*t)*t^2*cos(pitch);
%     0, 0, 0, (1/2)*t^2,...
%                 0;
%     0, 0, 0, 0, (1/2)*t^2;            
%     t, 0, 0, 0, 0;
%     0, t, 0, 0, 0;
%     0, 0, t, 0, 0;
%     0, 0, 0, t, 0;
%     0, 0, 0, 0, t];
% %Model Uncertainty  % system noise                   
% Q_k_1=diag([PARAM.SDuR^2 PARAM.SDvR^2 PARAM.SDwR^2 PARAM.SDvyawR^2 PARAM.SDvpchR^2]);
% P_k_k_1 = W_noise*Q_k_1*W_noise';
% for i = 1 : 2*stateLength+1,
%     P_k_k_1 = P_k_k_1 + W(i)*[f(chi_k_1(:,i), t)-x_k_k_1]*[f(chi_k_1(:,i), t)-x_k_k_1]';
% end
% %--PREDICTED DENSITY SAMPLE POINTS-----------------------------------------
% chi_k_k_1 = f(chi_k_1, t);
% % x_k_k_1, chi_k_k_1, P_k_k_1 available...
% if nargin==6, %in case there are 6 arguments
%     
%     [files_ang1,dummy]=find(H(:,4)); %all the measurements upon the angle yaw
%     [files_ang2,dummy]=find(H(:,5)); % pitch
%     files_ang = [files_ang1 files_ang2]; % indexes where the angle was involved in calculus
%     H_chi = H * chi_k_k_1;
%     H_chi(files_ang)=normalize(H_chi(files_ang)); %wrap those calculations that included angle
%     % predicted measurement
%     z_k_k_1 = (H_chi) * W';
%     %
%     z_dif = z_k-z_k_k_1;
%     z_dif(files_ang) = normalize(z_dif(files_ang));
%     %
%     Pxz = 0; Pzz = 0;
%     for i = 1 : 2*stateLength+1,
%         chi_x_dif = chi_k_k_1(:,i)-x_k_k_1;
%         %chi_x_dif(files_ang) = normalize(chi_x_dif(files_ang));
%         chi_z_dif = H_chi(:,i)-z_k_k_1; %chi_z_dif = H*chi_k_k_1(:,i)-z_k_k_1;
%         chi_z_dif(files_ang) = normalize(chi_z_dif(files_ang));
%         
%         Pxz = Pxz + W(i) * (chi_x_dif) * (chi_z_dif)';
%         Pzz = Pzz + W(i) * (chi_z_dif) * (chi_z_dif)';
%     end
%     S_k = R + Pzz;
%     K_k = Pxz * S_k^(-1);
%     % update step
%     x_k_k = x_k_k_1 + K_k * (z_dif);
%     P_k_k = P_k_k_1 - K_k*S_k*K_k';
% end
% 
%     x_k_k(4) = normalize(x_k_k(4));
%     x_k_k(5) = normalize(x_k_k(5));
%     
%     if ~isreal(x_k_k),
%         disp('complex number!');
%     end
% 
