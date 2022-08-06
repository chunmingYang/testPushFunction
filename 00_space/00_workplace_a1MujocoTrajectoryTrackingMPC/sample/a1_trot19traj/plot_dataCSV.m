clc
clear all
close all

data = readtable('data.csv');
% time = data.t;
% speed = data.NlopyStepSolveTime;
x = data.WorldPosX;
y = data.WorldPosY;
xref = data.referenceX;
yref = data.referenceY;
plot(x, y,"LineWidth",3,"LineStyle","-"); hold on
plot(xref, yref,"LineWidth",3)
legend("actual", "reference")
