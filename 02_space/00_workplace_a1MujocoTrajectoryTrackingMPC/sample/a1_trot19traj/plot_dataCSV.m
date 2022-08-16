clc
clear all
close all

data = readtable('data.csv');
% time = data.t;
% speed = data.NlopyStepSolveTime;
x = data.WorldPosX;
y = data.WorldPosY;
theta = data.yaw;
xref = data.referenceX;
yref = data.referenceY;
thetaref = data.referenceTheta;
figure(1)
plot(x, y,"LineWidth",3,"LineStyle","-"); hold on
plot(xref, yref,"LineWidth",3)
legend("actual", "reference")


t = [];
for i = 1 : length(theta)
    t(i) = 0.002*i*4.5;
end

figure(2)
plot(t, theta, "LineWidth", 3); hold on
plot(t, thetaref, "LineWidth", 3, "LineStyle", ":")
xlabel("time")
ylabel("yaw angle")
legend("actual", "reference")