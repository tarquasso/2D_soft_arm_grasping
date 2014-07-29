function k_next = inverse_kinematics(x_tip_desired, y_tip_desired, theta_tip_desired, length_array, k_current, theta0)

%% Compute the elements of the Jacobian Matrix (\partial Cartesian Tip) / (\partial k) for the current arm pose. 
JFull = get_jacobian_closedform(k_current, length_array, theta0);
%J = JFull(:,(2:6));
J = JFull;

%% Use foward kinematics to get the current tip position
[x_tip, y_tip, theta_tip] = forward_kinematics_method_two(k_current, theta0, length_array);

%% Cartesian error between current and desired tip positions
error = [(x_tip_desired - x_tip); (y_tip_desired - y_tip); (theta_tip_desired - theta_tip)];

%% Compute the appropriate curvature updates
alpha = 2.0*dot(error, J*transpose(J)*error)/dot(J*transpose(J)*error, J*transpose(J)*error)-25;
delta_k = alpha*transpose(J)*error;

%% output the new reference curvatures to the controller 
%k_next = k_current(2:6) + delta_k;
k_next = k_current + delta_k;

end