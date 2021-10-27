theta1 = 5;
theta2 = 15;
theta3 = 40;
acc = 80;
t_d12 = 1;
t_d23 = 1; 

%step1 
t_b1 = t_d12 - sqrt(t_d12*t_d12 - 2*(theta2-theta1)/acc)；
v_12 = (theta2-theta1)/(t_d12-0.5*t_b1);

%step2
t_b3 = t_d23 - sqrt(t_d23*t_d23 - 2*(theta3-theta2)/acc)；
v_23 = (theta3-theta2)/(t_d23-0.5*t_b3);

t_b2 = (v_23-v_12)/acc;

%draw acc

%draw v

%draw theta
