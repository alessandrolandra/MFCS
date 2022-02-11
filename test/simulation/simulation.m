clear all, close all
%% Moth simulation from Yale University
T = readtable('./Data_Second_Flight_Test.csv');
Inputs = transpose(table2array(T(:,2)));
% Take only a portion of the profile
samples = 2000;
Inputs = Inputs(1:samples);
Time = 0:1:(samples-1);
plot(Time,Inputs);
