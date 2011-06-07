function [z] = H(x, n, conf)
% x is state
% n is measurement noise
% measurement function H is flexible depending on device
% each time different values are measured therefore - H changes
index = 1;
for q = 1 : length(conf),
    switch(conf(q))
        case 0
            
        case 1%north
            z(index,:) = x(1,:) + n(1,:); index = index + 1;
        case 2%east
            z(index,:) = x(2,:) + n(2,:); index = index + 1;
        case 3%depth
            z(index,:) = x(3,:) + n(3,:); index = index + 1;            
        case 4
            z(index,:) = x(4,:) + n(4,:); index = index + 1;
        case 5
            z(index,:) = x(5,:) + n(5,:); index = index + 1;
        case 6
            z(index,:) = x(6,:) + n(6,:); index = index + 1; 
        case 7%
            z(index,:) = x(7,:) + n(7,:); index = index + 1;
        case 8%
            z(index,:) = (x(8,:) + n(8,:)); index = index + 1;            
        case 9
            z(index,:) = (x(9,:) + n(9,:)); index = index + 1;
        case 10
            z(index,:) = (x(10,:) + n(10,:)); index = index + 1;
        case 11
            z(index,:) = (x(11,:) + n(11,:)); index = index + 1;             
    end 
% % % %             if strcmp(dev,'GPS'),
% % % %     if config(1),  end % z = x
% % % %     if config(2), z(index,:) = x(2,:) + n(2,:); index = index + 1; end % z = y
% % % % elseif strcmp(dev,'MTI'),
% % % %     if config(1), z(index,:)  = x(4,:)  + n(4,:); index = index + 1; end % z = yaw
% % % %     if config(2), z(index,:)  = x(5,:)  + n(5,:); index = index + 1; end % z = pitch
% % % %     if config(3), z(index,:)  = x(9,:)  + n(9,:); index = index + 1; end % z = pitch rate
% % % %     if config(4), z(index,:) = x(10,:) + n(10,:); index = index + 1; end % z = pitch rate
% % % % else strcmp(dev,'DVL'),
% % % %     if config(1), z(index:index+2,:) = x(6:8,:) + n(6:8,:); index = index + 3; end % z = [u; v; w]
% % % %     if config(2), z(index,:)   = x(4,:) + n(4,:); index = index + 1; end % z = yaw
% % % %     if config(3), z(index,:)   = x(5,:) + n(5,:); index = index + 1; end % z = pitch
% % % %     if config(4), z(index,:)   = x(3,:) + n(3,:); index = index + 1; end % z = depth
end
