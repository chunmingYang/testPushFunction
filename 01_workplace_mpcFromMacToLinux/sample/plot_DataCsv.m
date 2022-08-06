clc
clear all
close all

data = readtable('data.csv');
% speed_1 = data.NlopyStepSolveTime;
% t = [];
% for i = 1 : 200
%     t(i) = i * 0.01;
% end
% 
% plot(t, speed_1)
% xlabel("Simulation Time (s)")
% ylabel("Nlopt Step Computation Time (ms)")

x_pos = data.x;
y_pos = data.y;

x_ref = data.xref;
y_ref = data.yref;
plot(x_pos, y_pos, "LineWidth",3); hold on
plot(x_ref, y_ref, "LineWidth",3,"LineStyle",":")
legend("actual", "ref")