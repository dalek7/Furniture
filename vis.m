clc; clear; close all;

%A = load('_outFurniture/Data_ 1_ 2_22_30_14.txt');
A = load('_outFurniture/Data_ 1_ 2_22_30_22.txt');
A = load('data_example/Data_20210317_152649.txt');

% A(:,1) is for nanoseconds System.nanoTime()

t = A(:,1) / 1000 / 1000 / 1000;
t = t - t(1);

dt_sec = mean(diff(t));


figure(1);
subplot(6,1,1);
plot(t,A(:,2)); 
title('Acc');
subplot(6,1,2);
plot(t,A(:,3)); 
subplot(6,1,3);
plot(t,A(:,4)); 

subplot(6,1,4);
plot(t,A(:,5));
title('Gyro');

subplot(6,1,5);
plot(t,A(:,6)); 

subplot(6,1,6);
plot(t,A(:,7)); 

%%
figure(2);
subplot(3,1,1);
plot(t,A(:,2)); 
grid;
title('Acc');
ylabel('g')

subplot(3,1,2);
plot(t,A(:,3)); 
grid;
ylabel('g')

subplot(3,1,3);
plot(t,A(:,4)); 
grid;
ylabel('g')

%%
figure(3);
subplot(3,1,1);
plot(t,A(:,5)); 
title('Gyro');
grid;
% ylabel('Angular velocity (deg/sec)')
ylabel('deg/sec')

subplot(3,1,2);
plot(t,A(:,6)); 
ylabel('deg/sec')
grid;

subplot(3,1,3);
plot(t,A(:,7)); 
ylabel('deg/sec')
grid;
xlabel('Time (sec)');

% figure(4);
% plot(t,A(:,2)-mean(A(:,2))); 
% hold on;
% plot(t,A(:,3)-mean(A(:,3))); 
% plot(t,A(:,4)-mean(A(:,4))); 
