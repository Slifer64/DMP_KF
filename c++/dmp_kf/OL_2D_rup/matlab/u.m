clc;
close all;
clear;

f = 0:0.01:5;

f_thres = 1.0;
f_a = 5;

a = 1 ./ ( 1 + exp(f_a*(f_thres - f)) );

figure;
plot(f,a, 'LineWidth',1.2);
title('Force sigmoid function');


v = 0:0.005:0.4;

v_thres = 0.06;
v_a = 100;

a = 1 ./ ( 1 + exp(v_a*(v_thres - v)) );

figure;
plot(v,a, 'LineWidth',1.2);
title('Velocity sigmoid function');