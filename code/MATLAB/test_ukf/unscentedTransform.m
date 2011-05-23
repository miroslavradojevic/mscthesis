function [sigmaVec, Wm, Wc] = unscentedTransform(x_mean, P_x)
% function presumes that there is a random variable mean and covariance
% at the input. it calculates sample points and their weights before 
% undergoing non-linear transform
% unscented transform concept tries to represent the density by a set of
% sample points called sigma vectors and their weights (W) which are named
% Wm (weights when calculating mean) and Wc (weights when calculating cov) 
% INPUT:
% x_mean = mean of the random variable that is undergoing a non-linear
% transform -> size(x_mean) = L x 1
% P_x    = covariance of the random variable that is undergoing a
% non-linear transform -> size(P_x) = L x L
% alpha, beta, k, lambda = patameters used
% OUTPUT:
% sigmaVec  = samples -> size(sPt) =       L x 2*L+1
% Wm and Wc = sample weights -> size(Wm) = 1 x 2*L+1
%%% SOURCE %%%
% http://cslu.cse.ogi.edu/nsel/ukf/node6.html#sec:ukf

L = size(x_mean, 1);

alpha = 1e-3; % determines the spread of sigma points around mean
k = 0; % secondary scaling parameter
beta = 2;% incorporate prior knowledge  
lambda = alpha^2 * (L+k) - L;
lambda = -0.5; %1e-3;
sigmaVec(:,1) = x_mean;
Wm(1)   = lambda/(L+lambda)
Wc(1)   = lambda/(L+lambda);%+(1-alpha^2+beta)

TEMP = sqrtm((L+lambda)*P_x); % matrix square root 

for i = 2 : 2*L+1,
    if i <=L+1,
        sigmaVec(:,i) = x_mean + TEMP(i-1,:)';
    else 
        sigmaVec(:,i) = x_mean - TEMP(i-1-L,:)';
    end
    Wm(i)   = 1/(2*(L+lambda));
    Wc(i)   = Wm(i);
end
% to avoid complex numbers in state vector
sigmaVec = real(sigmaVec);