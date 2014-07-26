function h = PlotChain(k, ARM, GRIPPER) 

theta0 = ARM.theta0;
N = ARM.N + 1;
L = [ARM.L; GRIPPER.L];
k = [k; GRIPPER.k];

M = 20;
total = 1;
x = zeros(1, N*M);
y = zeros(1, N*M);
theta = zeros(1, N*M);

for i=1:N
    for j=1:M
        [x(total), y(total), theta(total)] = recursive_forward_kinematics(k, theta0, L, N, i, L(i)*(j/M));
        total = total + 1;
    end
end

hold on
axis([-0.30 0.30 -0.10 0.50])
axis square


h = plot(x(1:end-20),y(1:end-20), 'r', x(end-20:end),y(end-20:end), 'k', 'LineWidth', 2);

end
