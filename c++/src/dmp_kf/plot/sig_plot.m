clc;
close all;
clear;

a_m = 1.5;
c_m = 5.0;

f_norm = 0:0.1:10;

h = 1 ./ (1 + exp(a_m*(f_norm-c_m)));

figure;
plot(f_norm, h);