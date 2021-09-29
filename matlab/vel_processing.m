% This script processes the velocity profile generated in oFX and attempts
% to fit it to a lognormal distribution for application of the XZERO alg.

%% Read and process data:

clc; clear all; close all;

data = readtable('velocity.txt');
v = table2array(data);

%data processing for lognfit
for i = 1:1:length(v)
    checkinf = isinf(v(i)); checknum = isnumeric(v(i));
    if checkinf == 1 || checknum == 0 % fill gaps in data
        v(i) = (v(i-1)+v(i+1)/2);
    end
    if v(i) == 0 % replace zeros for lognfit
        v(i) = 0.001; %near-zero small value
    end
end

vsmooth = smoothdata(medfilt1(v,3)); %use smoothing and median filters

figure(1) % shows processed data, smoothed data, and local maxima
subplot(1,2,1)
plot(v, 'g')
hold on
xaxis = 0:length(vsmooth);
plot(vsmooth, 'r')
LMax = islocalmax(vsmooth, 'MinProminence', 0.3); % using prominence to filter values
LMin = islocalmin(vsmooth, 'MinProminence', 0.3);
plot(xaxis(LMax),vsmooth(LMax),'ro',xaxis(LMin),vsmooth(LMin),'ro')
hold off

%% Split the curves:
% Define start and end points of log curves, done with local minima:
t0 = 1; tf = length(vsmooth); % first and last indices
bounds = t0;
for i = 1:length(LMin)
    if LMin(i) == 1
        bounds = [bounds i];
    end
end
bounds = [bounds tf]; % vector of bounding points
N = length(bounds) - 1; % number of curves
curves = cell(1,N); % cell array of curves, needs to be resized

subplot(1,2,2)
for i = 1:N % perhaps redundant, but LMin might be defined another way in future
    curve_val = vsmooth(bounds(i):bounds(i+1));
    curves{1,i} = curve_val;
    plot(curves{i})
    hold on
end
hold off
legend('curve1', 'curve2', 'curve3', 'curve4');


%% Fit Data
% We now need tmax, tinf1, tinf2...
parmhat = cell(1,N); v_prof = cell(1,N); 
mean = cell(1,N); stdv = cell(1,N); % sigma, mu

figure(2) %using the formula for lognormal distribution, lognfit to get mean and stdv
for i = 1:N
    subplot(N,1,i)
    parmhat{1,i} = lognfit(curves{i}); mean = parmhat{i}(1); stdv = parmhat{i}(2);
    for t = 1:length(curves{i})+1
        if t == 1
            v_prof{i}(t) = 0;
        else
            
            v_prof{i}(t) = 10/(stdv*(t-1)*sqrt(2*pi())) * exp(((log(t-1) - mean)^2) / (2*stdv^2));
        end
    end
    plot(v_prof{i})
end

%% Matlab fit using lognpdf
figure(3) % Using MATLAB lognpdf to fit
% Fitted distribution
xt = 0.1:0.1:10;
for i = 1:N
    plot(xt,10*lognpdf(xt,parmhat{i}(1),parmhat{i}(2)))
    hold on
end
hold off
ylabel('weighted value'); xlabel('step');
legend('curve1', 'curve2', 'curve3', 'curve4')
%}

%% Mathematical estimation of parameters
syms T Tm T0 Tinf mu sigma D v_sym(T) k(T)
k = (log(T) - mu)/sigma;
diff(k,T)
dkdt = 1/(sigma*(T));
v_sym = D/(sigma*(T)*sqrt(2*pi()))*exp((-k^2)/2);

% First we will try to find the three important time indices:

%first derivative, to find Tmax, change these to be equivalent
dvdt_sym = vpa(simplify(diff(v_sym,T)),4)
dvdt = vpa((v_sym)/(sigma*(T))*(sigma+k),4)
Tm = T0 + exp(mu-sigma^2); % tmax = t0 + exp(mu-sigma^2)

% second derivative, to find Tinf, change these to be equivalent
ddvddt_sym = vpa(simplify(diff(diff(v_sym, T), T)),4)
ddvddt = vpa((v_sym)/(sigma^2*(T^2))*(k^2 + 3*k*sigma + 2*sigma^2 - 1),4)
% the roots of (k^2 + 3*k*sigma + 2*sigma^2 - 1) are the non-trivial zeros
Tinf1 = T0 + exp(mu - sigma*((3*sigma + sqrt(sigma^2 + 4))/2)); % == t0 + alpha1*exp(mu-sigma^2) < 1
Tinf2 = T0 + exp(mu - sigma*((3*sigma - sqrt(sigma^2 + 4))/2)); % == t0 alpha2*exp(mu-sigma^2) < 1
%where
% a1 = sigma*((sigma + sqrt(sigma^2 + 4))/2) > 0 ---> alpha1 = exp(-a1) < 1
% a2 = sigma*((sigma - sqrt(sigma^2 + 4))/2) < 0 ---> alpha1 = exp(-a2) > 1
% and
% Tinf1 < Tm < Tinf2

% These three time indices should be calculated from the sampled profiles
% to estimate tm and using the max and min of acc to find tinf1 and tinf2

% Now we need to estimate the parameters sigma, mu, t0, and D. Estimated
% parameters will have a "hat":

% Estimate sigma_hat from the following nonlinear relationship
% F(sigma) = (tf - t0)/2*sinh(3*sigma) * (exp(-sigma^2) - exp(-3*sigma)) == 0

% Estimate mu_hat using sigma_hat:
% Tinf2 - Tinf1 = exp(mu - sigma^2)*(exp(-a2) - exp(-a1)) == exp(mu - sigma^2)*(alpha2 - alpha1)
% such that
% mu = sigma^2 + log((Tinf2 - Tinf1)/(alpha2 - alpha1))

% Now T0_hat can be estimated using the previous relation: Tm = T0 + exp(mu-sigma^2) and estimated values
% T0 = Tm - exp(mu-sigma^2)

% Now estimate D_hat using the equation for v_sym, at vmax:
% v(Tm) = D/(sigma*(Tm - T0)*sqrt(2*pi())) * exp((-1/(2*sigma^2)) * (log(Tm - T0) - mu)^2)
% vm = D / (sigma*sqrt(2*pi())*exp(mu-sigma^2)) * exp((-1/(2*sigma^2)) * (mu - sigma^2 - mu)^2)
% vm = D / (sigma*sqrt(2*pi())) * exp(-mu + (sigma^2)/2)
% D = vm*sigma*(sqrt(2*pi())) * exp(mu - (sigma^2)/2)
