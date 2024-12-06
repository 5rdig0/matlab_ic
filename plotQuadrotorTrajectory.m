% This script plots the closed-loop responses of the nonlinear MPC
% controller used in the quadrotor path following example.

% Copyright 2019 The MathWorks, Inc.

% Plot the closed-loop response.
time = 0:Ts:Duration;
yreftot = QuadrotorReferenceTrajectory(time)';
global env;
% Plot the states.

%Modificaçoes

%Tamanho do envelope
envelopex_sup = yreftot(:,1) + env;
envelopex_inf = yreftot(:,1) - env;
envelopey_sup = yreftot(:,2) + env;
envelopey_inf = yreftot(:,2) - env;
envelopez_sup = yreftot(:,3) + env;
envelopez_inf = yreftot(:,3) - env;
envelopexyz = [envelopex_sup, envelopex_inf, envelopey_sup, envelopey_inf, envelopez_sup, envelopez_inf];

f = figure('Name','Estado X');
f.Position(3:4) = [1000 800];
hold on
plot(time,xHistory(:,1))
plot(time,yreftot(:,1) + env)
plot(time,yreftot(:,1) - env)
plot(time,yreftot(:,1))
grid on
xlabel('tempo')
ylabel('x')
legend('atual', 'envelope superior', 'envelope inferior ','referência','Location','southwest')
title('posição X do quadrotor')

f = figure('Name','Estado Y');
f.Position(3:4) = [1000 800];
hold on
plot(time,xHistory(:,2))
plot(time,yreftot(:,2) + env)
plot(time,yreftot(:,2) - env)
plot(time,yreftot(:,2))
grid on
xlabel('tempo')
ylabel('y')
legend('atual',  'envelope superior', 'envelope inferior ','referência','Location','southwest')
title('posição Y do quadrotor')

f = figure('Name','Estado Y');
f.Position(3:4) = [1000 800];
hold on
plot(time,xHistory(:,3))
plot(time,yreftot(:,3) + env)
plot(time,yreftot(:,3) - env)
plot(time,yreftot(:,3))
grid on
xlabel('tempo')
ylabel('z')
legend('atual',  'envelope superior', 'envelope inferior ','referência','Location','southeast')
title('posição Z do quadrotor')

%subplot(2,3,4)
%hold on
%plot(time,xHistory(:,4))
%plot(time,yreftot(:,4))
%rid on
%xlabel('time')
%ylabel('phi')
%legend('actual','reference','Location','southeast')
%title('Quadrotor phi angle')

%subplot(2,3,5)
%hold on
%plot(time,xHistory(:,5))
%plot(time,yreftot(:,5))
%grid on
%label('time')
%ylabel('theta')
%legend('actual','reference','Location','southeast')
%title('Quadrotor theta angle')

%subplot(2,3,6)
%hold on
%plot(time,xHistory(:,6))
%plot(time,yreftot(:,6))
%grid on
%xlabel('time')
%ylabel('psi')
%legend('actual','reference','Location','southeast')
%title('Quadrotor psi angle')

% Plot the manipulated variables.
figure('Name','Entradas do controle')

subplot(2,2,1)
hold on
stairs(time,uHistory(:,1))
ylim([-0.5,12.5])
plot(time,nloptions.MVTarget(2)*ones(1,length(time)))
grid on
xlabel('tempo')
legend('atual','referência')
title('entrada 1')

subplot(2,2,2)
hold on
stairs(time,uHistory(:,2))
ylim([-0.5,12.5])
plot(time,nloptions.MVTarget(2)*ones(1,length(time)))
grid on
xlabel('tempo')
title('entrada 2')
legend('atual','referência')

subplot(2,2,3)
hold on
stairs(time,uHistory(:,3))
ylim([-0.5,12.5])
plot(time,nloptions.MVTarget(2)*ones(1,length(time)))
grid on
xlabel('tempo')
title('entrada 3')
legend('atual','referência')

subplot(2,2,4)
hold on
stairs(time,uHistory(:,4))
ylim([-0.5,12.5])
plot(time,nloptions.MVTarget(2)*ones(1,length(time)))
grid on
xlabel('tempo')
title('entrada 4')
legend('atual','referência')
