function h = PlotChain(k, theta0, L, N) 

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
axis([-0.25 0.25 -0.10 0.40])
axis square


h = plot(x,y, 'r', 'LineWidth', 2);


end
