%%
% author: Liu Xiao
% ID: 21S053284
clear, clc

%% Q1
theta1 = 5;
theta2 = 15;
theta3 = 40;
acc = 80;
t_d12 = 1;
t_d23 = 1; 

%step1 
t_b1 = t_d12 - sqrt(t_d12*t_d12 - 2*(theta2-theta1)/acc);
v_12 = (theta2-theta1)/(t_d12-0.5*t_b1);

%step2
t_b3 = t_d23 - sqrt(t_d23*t_d23 - 2*(theta3-theta2)/acc);
v_23 = (theta3-theta2)/(t_d23-0.5*t_b3);

%step3
t_b2 = (v_23-v_12)/acc;

%function theta
f1 = @(t_) theta1 + 0.5*acc*t_.^2;
f2 = @(t_) f1(t_b1) + v_12 * (t_ - t_b1);
f3 = @(t_) f2(t_d12-0.5*t_b2) + v_12*(t_-(t_d12 - 0.5*t_b2)) + 1/2 * acc * (t_-(t_d12 - 0.5*t_b2)).^2;
f4 = @(t_) f3(t_d12+0.5*t_b2) + v_23 * (t_ - (t_d12+0.5*t_b2));
f5 = @(t_) f4(t_d12+t_d23-t_b3) + v_23*(t_ - (t_d12+t_d23-t_b3)) - 1/2*acc*((t_ - (t_d12+t_d23-t_b3))).^2;

%v
df1 = @(t_) acc*t_;
df2 = @(t_) v_12 * ones(size(t_, 2));
df3 = @(t_) v_12 + acc * (t_-(t_d12 - 0.5*t_b2));
df4 = @(t_) v_23* ones(size(t_, 2));
df5 = @(t_) v_23 - acc*((t_ - (t_d12+t_d23-t_b3)));

%acc
ddf1 = @(t_) acc * ones(size(t_, 2));
ddf2 = @(t_) zeros(size(t_, 2));
ddf3 = @(t_) acc * ones(size(t_, 2));
ddf4 = @(t_) zeros(size(t_, 2));
ddf5 = @(t_) -acc * ones(size(t_, 2));

tt1 = 0:1e-2:t_b1;
tt2 = t_b1:1e-2:t_d12-0.5*t_b2;
tt3 = t_d12-0.5*t_b2:1e-2:t_d12+0.5*t_b2;
tt4 = t_d12+0.5*t_b2:1e-2:t_d12+t_d23-t_b3;
tt5 = t_d12+t_d23-t_b3:1e-2:t_d12+t_d23;

figure(1);
%draw theta
subplot(131);
plot(tt1, f1(tt1), 'linewidth', 2);
hold on;
plot(tt2, f2(tt2), 'linewidth', 2);
plot(tt3, f3(tt3), 'linewidth', 2);
plot(tt4, f4(tt4), 'linewidth', 2);
plot(tt5, f5(tt5), 'linewidth', 2);
grid on;
title('position');
xlabel('t/s');
ylabel('deg');

%draw v
subplot(132);
plot(tt1, df1(tt1), 'linewidth', 2);
hold on;
plot(tt2, df2(tt2), 'linewidth', 2);
plot(tt3, df3(tt3), 'linewidth', 2);
plot(tt4, df4(tt4), 'linewidth', 2);
plot(tt5, df5(tt5), 'linewidth', 2);
grid on;
title('velocity');
xlabel('t/s');
ylabel('deg');

%draw acc
subplot(133);
plot(tt1, ddf1(tt1), 'linewidth', 2);
hold on;
plot(tt2, ddf2(tt2), 'linewidth', 2);
plot(tt3, ddf3(tt3), 'linewidth', 2);
plot(tt4, ddf4(tt4), 'linewidth', 2);
plot(tt5, ddf5(tt5), 'linewidth', 2);
grid on;
title('acceleration');
xlabel('t/s');
ylabel('deg');



%% Q2
theta0 = 5;
theta1 = 15;
theta2 = -10;
td = 2;

%step1
a0 = 5;
a1 = 0;
h = (theta1 - theta0);
a2 = 3*h / td^2;
a3 = -2*h / td^3;

cubic1 = [a3 a2 a1 a0];

a0 = 15;
a1 = 0;
h = (theta2 - theta1);
a2 = 3*h / td^2;
a3 = -2*h / td^3;

cubic2 = [a3 a2 a1 a0];

figure(2);
subplot(131);
x = 0:1e-2:td;
plot(x, polyval(cubic1, x), 'linewidth', 2);
hold on;
x = td:1e-2:2*td;
plot(x, polyval(cubic2, x-td), 'linewidth', 2);
grid on;
title('postion');
xlabel('t/s');
ylabel('deg');
hold off;

subplot(132);
x = 0:1e-2:td;
dcubic1 = polyder(cubic1);
dcubic2 = polyder(cubic2);
plot(x, polyval(dcubic1, x), 'linewidth', 2);
hold on;
x = td:1e-2:2*td;
plot(x, polyval(dcubic2, x-td), 'linewidth', 2);
grid on;
title('velocity');
xlabel('t/s');
ylabel('deg');
hold off;

subplot(133);
x = 0:1e-2:td;
ddcubic1 = polyder(dcubic1);
ddcubic2 = polyder(dcubic2);
plot(x, polyval(ddcubic1, x), 'linewidth', 2);
hold on;
x = td:1e-2:2*td;
plot(x, polyval(ddcubic2, x-td), 'linewidth', 2);
grid on;
title('acceleration');
xlabel('t/s');
ylabel('deg');
hold off;