function [x_tip, y_tip, theta_tip] = forward_kinematics_method_one(K, theta0, L)

k1 = K(1);
k2 = K(2);
k3 = K(3);
k4 = K(4);
k5 = K(5);
k6 = K(6);

s1 = L(1);
s2 = L(2);
s3 = L(3);
s4 = L(4);
s5 = L(5);
s6 = L(6);

l = 1;

theta1 = k1*s1 + theta0;
theta2 = k2*s2 + theta1;
theta3 = k3*s3 + theta2;
theta4 = k4*s4 + theta3;
theta5 = k5*s5 + theta4;
theta6 = k6*s6 + theta5;

x_1 = l/k1*sin(k1*s1 + theta0) - l/k1*sin(theta0);
x_2 = l/k2*sin(k2*s2 + theta1) + x_1 - l/k2*sin(theta1);
x_3 = l/k3*sin(k3*s3 + theta2) + x_2 - l/k3*sin(theta2);
x_4 = l/k4*sin(k4*s4 + theta3) + x_3 - l/k4*sin(theta3);
x_5 = l/k5*sin(k5*s5 + theta4) + x_4 - l/k5*sin(theta4);
x_6 = l/k6*sin(k6*s6 + theta5) + x_5 - l/k6*sin(theta5);

y_1 = -l/k1*cos(k1*s1 + theta0) + l/k1*cos(theta0);
y_2 = -l/k2*cos(k2*s2 + theta1) + y_1 + l/k2*cos(theta1);
y_3 = -l/k3*cos(k3*s3 + theta2) + y_2 + l/k3*cos(theta2);
y_4 = -l/k4*cos(k4*s4 + theta3) + y_3 + l/k4*cos(theta3);
y_5 = -l/k5*cos(k5*s5 + theta4) + y_4 + l/k5*cos(theta4);
y_6 = -l/k6*cos(k6*s6 + theta5) + y_5 + l/k6*cos(theta5);

x_tip = x_6;
y_tip = y_6;
theta_tip = theta6;

