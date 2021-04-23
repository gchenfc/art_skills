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


%% Fit Data/Extract Parameters
% We now need tmax, tinf1, tinf2...
parmhat = cell(1,N); logcurves = cell(1,N); 
mean = cell(1,N); stdv = cell(1,N); % sigma, mu

figure(2) %using the formula for lognormal distribution, lognfit to get mean and stdv
for i = 1:N
    subplot(N,1,i)
    parmhat{1,i} = lognfit(curves{i}); mean = parmhat{i}(1); stdv = parmhat{i}(2);
    for t = 1:length(curves{i})+1
        if t == 1
            logcurves{i}(t) = 0;
        else
            logcurves{i}(t) = 10/(stdv*length(curves{i})*2*sqrt(2*pi()))*exp(((log(t - 1) - mean)^2) / (2*stdv^2));
        end
    end
    plot(logcurves{i})
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