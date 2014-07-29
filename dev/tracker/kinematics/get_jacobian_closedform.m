function [J] = get_jacobian_closedform(K, L, theta0)

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


%assumption: start at theta0 = pi/2

%x_6 wrt k
J(1,1) = sin(theta0)/k1^2 - sin(theta0 + k1*s1)/k1^2 + (s1*cos(theta0 + k1*s1 + k2*s2 + k3*s3 + k4*s4 + k5*s5))/k5 - (s1*cos(theta0 + k1*s1 + k2*s2 + k3*s3 + k4*s4 + k5*s5))/k6 + (s1*cos(theta0 + k1*s1))/k1 - (s1*cos(theta0 + k1*s1))/k2 + (s1*cos(theta0 + k1*s1 + k2*s2 + k3*s3 + k4*s4 + k5*s5 + k6*s6))/k6 + (s1*cos(theta0 + k1*s1 + k2*s2))/k2 - (s1*cos(theta0 + k1*s1 + k2*s2))/k3 + (s1*cos(theta0 + k1*s1 + k2*s2 + k3*s3))/k3 - (s1*cos(theta0 + k1*s1 + k2*s2 + k3*s3))/k4 + (s1*cos(theta0 + k1*s1 + k2*s2 + k3*s3 + k4*s4))/k4 - (s1*cos(theta0 + k1*s1 + k2*s2 + k3*s3 + k4*s4))/k5;
J(1,2) = sin(theta0 + k1*s1)/k2^2 - sin(theta0 + k1*s1 + k2*s2)/k2^2 + (s2*cos(theta0 + k1*s1 + k2*s2 + k3*s3 + k4*s4 + k5*s5))/k5 - (s2*cos(theta0 + k1*s1 + k2*s2 + k3*s3 + k4*s4 + k5*s5))/k6 + (s2*cos(theta0 + k1*s1 + k2*s2 + k3*s3 + k4*s4 + k5*s5 + k6*s6))/k6 + (s2*cos(theta0 + k1*s1 + k2*s2))/k2 - (s2*cos(theta0 + k1*s1 + k2*s2))/k3 + (s2*cos(theta0 + k1*s1 + k2*s2 + k3*s3))/k3 - (s2*cos(theta0 + k1*s1 + k2*s2 + k3*s3))/k4 + (s2*cos(theta0 + k1*s1 + k2*s2 + k3*s3 + k4*s4))/k4 - (s2*cos(theta0 + k1*s1 + k2*s2 + k3*s3 + k4*s4))/k5;
J(1,3) = sin(theta0 + k1*s1 + k2*s2)/k3^2 - sin(theta0 + k1*s1 + k2*s2 + k3*s3)/k3^2 + (s3*cos(theta0 + k1*s1 + k2*s2 + k3*s3 + k4*s4 + k5*s5))/k5 - (s3*cos(theta0 + k1*s1 + k2*s2 + k3*s3 + k4*s4 + k5*s5))/k6 + (s3*cos(theta0 + k1*s1 + k2*s2 + k3*s3 + k4*s4 + k5*s5 + k6*s6))/k6 + (s3*cos(theta0 + k1*s1 + k2*s2 + k3*s3))/k3 - (s3*cos(theta0 + k1*s1 + k2*s2 + k3*s3))/k4 + (s3*cos(theta0 + k1*s1 + k2*s2 + k3*s3 + k4*s4))/k4 - (s3*cos(theta0 + k1*s1 + k2*s2 + k3*s3 + k4*s4))/k5;
J(1,4) = sin(theta0 + k1*s1 + k2*s2 + k3*s3)/k4^2 - sin(theta0 + k1*s1 + k2*s2 + k3*s3 + k4*s4)/k4^2 + (s4*cos(theta0 + k1*s1 + k2*s2 + k3*s3 + k4*s4 + k5*s5))/k5 - (s4*cos(theta0 + k1*s1 + k2*s2 + k3*s3 + k4*s4 + k5*s5))/k6 + (s4*cos(theta0 + k1*s1 + k2*s2 + k3*s3 + k4*s4 + k5*s5 + k6*s6))/k6 + (s4*cos(theta0 + k1*s1 + k2*s2 + k3*s3 + k4*s4))/k4 - (s4*cos(theta0 + k1*s1 + k2*s2 + k3*s3 + k4*s4))/k5;
J(1,5) = sin(theta0 + k1*s1 + k2*s2 + k3*s3 + k4*s4)/k5^2 - sin(theta0 + k1*s1 + k2*s2 + k3*s3 + k4*s4 + k5*s5)/k5^2 + (s5*cos(theta0 + k1*s1 + k2*s2 + k3*s3 + k4*s4 + k5*s5))/k5 - (s5*cos(theta0 + k1*s1 + k2*s2 + k3*s3 + k4*s4 + k5*s5))/k6 + (s5*cos(theta0 + k1*s1 + k2*s2 + k3*s3 + k4*s4 + k5*s5 + k6*s6))/k6;
J(1,6) = sin(theta0 + k1*s1 + k2*s2 + k3*s3 + k4*s4 + k5*s5)/k6^2 - sin(theta0 + k1*s1 + k2*s2 + k3*s3 + k4*s4 + k5*s5 + k6*s6)/k6^2 + (s6*cos(theta0 + k1*s1 + k2*s2 + k3*s3 + k4*s4 + k5*s5 + k6*s6))/k6;

%y_6 wrt k
J(2,1) = cos(theta0 + k1*s1)/k1^2 - cos(theta0)/k1^2 + (s1*sin(theta0 + k1*s1 + k2*s2 + k3*s3 + k4*s4 + k5*s5))/k5 - (s1*sin(theta0 + k1*s1 + k2*s2 + k3*s3 + k4*s4 + k5*s5))/k6 + (s1*sin(theta0 + k1*s1))/k1 - (s1*sin(theta0 + k1*s1))/k2 + (s1*sin(theta0 + k1*s1 + k2*s2 + k3*s3 + k4*s4 + k5*s5 + k6*s6))/k6 + (s1*sin(theta0 + k1*s1 + k2*s2))/k2 - (s1*sin(theta0 + k1*s1 + k2*s2))/k3 + (s1*sin(theta0 + k1*s1 + k2*s2 + k3*s3))/k3 - (s1*sin(theta0 + k1*s1 + k2*s2 + k3*s3))/k4 + (s1*sin(theta0 + k1*s1 + k2*s2 + k3*s3 + k4*s4))/k4 - (s1*sin(theta0 + k1*s1 + k2*s2 + k3*s3 + k4*s4))/k5;
J(2,2) = cos(theta0 + k1*s1 + k2*s2)/k2^2 - cos(theta0 + k1*s1)/k2^2 + (s2*sin(theta0 + k1*s1 + k2*s2 + k3*s3 + k4*s4 + k5*s5))/k5 - (s2*sin(theta0 + k1*s1 + k2*s2 + k3*s3 + k4*s4 + k5*s5))/k6 + (s2*sin(theta0 + k1*s1 + k2*s2 + k3*s3 + k4*s4 + k5*s5 + k6*s6))/k6 + (s2*sin(theta0 + k1*s1 + k2*s2))/k2 - (s2*sin(theta0 + k1*s1 + k2*s2))/k3 + (s2*sin(theta0 + k1*s1 + k2*s2 + k3*s3))/k3 - (s2*sin(theta0 + k1*s1 + k2*s2 + k3*s3))/k4 + (s2*sin(theta0 + k1*s1 + k2*s2 + k3*s3 + k4*s4))/k4 - (s2*sin(theta0 + k1*s1 + k2*s2 + k3*s3 + k4*s4))/k5;
J(2,3) = cos(theta0 + k1*s1 + k2*s2 + k3*s3)/k3^2 - cos(theta0 + k1*s1 + k2*s2)/k3^2 + (s3*sin(theta0 + k1*s1 + k2*s2 + k3*s3 + k4*s4 + k5*s5))/k5 - (s3*sin(theta0 + k1*s1 + k2*s2 + k3*s3 + k4*s4 + k5*s5))/k6 + (s3*sin(theta0 + k1*s1 + k2*s2 + k3*s3 + k4*s4 + k5*s5 + k6*s6))/k6 + (s3*sin(theta0 + k1*s1 + k2*s2 + k3*s3))/k3 - (s3*sin(theta0 + k1*s1 + k2*s2 + k3*s3))/k4 + (s3*sin(theta0 + k1*s1 + k2*s2 + k3*s3 + k4*s4))/k4 - (s3*sin(theta0 + k1*s1 + k2*s2 + k3*s3 + k4*s4))/k5;
J(2,4) = cos(theta0 + k1*s1 + k2*s2 + k3*s3 + k4*s4)/k4^2 - cos(theta0 + k1*s1 + k2*s2 + k3*s3)/k4^2 + (s4*sin(theta0 + k1*s1 + k2*s2 + k3*s3 + k4*s4 + k5*s5))/k5 - (s4*sin(theta0 + k1*s1 + k2*s2 + k3*s3 + k4*s4 + k5*s5))/k6 + (s4*sin(theta0 + k1*s1 + k2*s2 + k3*s3 + k4*s4 + k5*s5 + k6*s6))/k6 + (s4*sin(theta0 + k1*s1 + k2*s2 + k3*s3 + k4*s4))/k4 - (s4*sin(theta0 + k1*s1 + k2*s2 + k3*s3 + k4*s4))/k5;
J(2,5) = cos(theta0 + k1*s1 + k2*s2 + k3*s3 + k4*s4 + k5*s5)/k5^2 - cos(theta0 + k1*s1 + k2*s2 + k3*s3 + k4*s4)/k5^2 + (s5*sin(theta0 + k1*s1 + k2*s2 + k3*s3 + k4*s4 + k5*s5))/k5 - (s5*sin(theta0 + k1*s1 + k2*s2 + k3*s3 + k4*s4 + k5*s5))/k6 + (s5*sin(theta0 + k1*s1 + k2*s2 + k3*s3 + k4*s4 + k5*s5 + k6*s6))/k6;
J(2,6) = cos(theta0 + k1*s1 + k2*s2 + k3*s3 + k4*s4 + k5*s5 + k6*s6)/k6^2 - cos(theta0 + k1*s1 + k2*s2 + k3*s3 + k4*s4 + k5*s5)/k6^2 + (s6*sin(theta0 + k1*s1 + k2*s2 + k3*s3 + k4*s4 + k5*s5 + k6*s6))/k6;

%theta6 wrt k
J(3,1) = s1;
J(3,2) = s2;
J(3,3) = s3;
J(3,4) = s4;
J(3,5) = s5;
J(3,6) = s6;