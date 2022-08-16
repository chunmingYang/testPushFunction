clc
clear all
close all

ts = 0.002;
tstart = 0;
tend = 60;
iter = tend/ts;
xhis = [];
yhis = [];

r = 1;
w = pi/(tend - tstart);

for i = 1 : iter
    theta = w*(ts*i);
    xhis(i) = r - cos(theta);
    yhis(i) = sin(theta);
end

plot(xhis, yhis)
