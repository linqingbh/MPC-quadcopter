clear all
clc
disp('Beginning Predictive Control simulation...')

global data;

%TrueTime initialization:

addpath(genpath(pwd))
setenv('TTKERNEL', '../truetime-2.0/kernel')

addpath([getenv('TTKERNEL')])

init_truetime

%Simulation parameters:
fs=10;
data.h=1/fs;
tstop=60;
max_step_size=1e-6;
min_step_size=0.1e-6;
warning('off');


sim('model');

figure
hold on
grid on
ylabel('angle \theta');
xlabel('time t');
plot(data.t_vector,data.y_vector);
plot(data.t_vector,data.r_vector);
plot(data.t_vector,data.ymax*ones(1,length(data.t_vector)));
plot(data.t_vector,data.ymin*ones(1,length(data.t_vector)));
legend('Output','reference','ymax','ymin');







 






