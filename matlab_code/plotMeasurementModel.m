%% Measurement model

step = 0.01;
x = 0:step:2.5;

peak1 = normpdf(x, 0.30, 0.1);
peak2 = normpdf(x, 1.00, 0.15);
peak3 = normpdf(x, 1.70, 0.2);
uniformD = ones(1, length(x));
startD = .65*exp(-3*x+2.5);
% startD(round(0.35/step):end) = 0;

summedD = peak1 + peak2 + peak3 + startD + uniformD;
% summedD = peak1 + startD + uniformD;figure('Position', [1, 1, 1920/2*.5, (1080-100)*.5]);
summedD(end-10:end) = 9;
summedD(1:round(0.05/step)) = 0;

summedD = summedD/200 + 0.975;

figure('Position', [1, 1, 1920/2*1.1, (1080/2-200)*1.1]);
plot(x, summedD)
title('Example probability distribution function with laser scan angle $\psi = 90^\circ$ and crosstrack error = 0 m','Interpreter','latex')
xlabel('range (m)')
ylabel('Probability of hit')