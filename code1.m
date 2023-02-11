clear all;
clc;
close all;

rng(1);

x = [0; 0];
X = x;
dt = 0.1; % Tempo de amostragem do sistema real

A = [1 dt; 0 1]; % Matriz de transição de estados

B = [dt^2/2; dt]; % Matriz de controle

a = 0.1; % Aceleração
u = a;

sigma_x_s = 0.01; % desvio padrão de posição [m]
sigma_x_v = 0.01; % desvio padrão de velocidade [m/s]
Q = [sigma_x_s  0; sigma_x_v 0];

C = [1 0]; % Matriz do sensor

sigma_y_s = 0.5; % Erro da medição
R = sigma_y_s^2; % Matriz de covariância do sensor

% Primeira leitura do sensor
y = C*x+sqrt(R)*randn;
Y = y;

t = 0;
tmax = 20;

xk = [x(1); 0];
Xk = xk;
P = Q;

z = C*xk;
Z = z;
e = abs(x(1,1)-z);
ecum = e;

while t <= tmax
    t = [t, t(end)+dt]; % Evolução do tempo da simulação
    
    x = A*x+B*u+sqrt(Q)*randn(size(x)); % Evolução do sistema simulado
    X = [X, x];
    
    y = C*x+sqrt(R)*randn; % Leitura de um sensor
    Y = [Y, y];
    
    xk = A*xk+B*u;
    P = A*P*A'+Q;
    
    z = C*xk;
    Z = [Z, z];
    e = [e, abs(x(1,end)-z)]; % Erro entre o estado real e a esperança
    E = e(end-1:end);
    
    ecum = [ecum, sum(e)];
    Ecum = ecum(end-1:end);
    
    K = P*C'*(C*P*C'+R)^-1; % Ganho de Kalman
    
    xk = xk+K*(y-z); % Correção da esperança do sistema
    Xk = [Xk, xk];
    
    P = (eye(size(Q))-K*C)*P; % Correçaõ da covariância do modelo
    
    subplot(3,1,1);
    plot(t,Y,'g','linewidth',2);
    hold on;
    plot(t,X(1,:),'k','linewidth',2);
    plot(t,Z,'r','linewidth',2);
    plot(t(end),x(1),'ok','linewidth',2,'markersize',10);
    plot(t(end),xk(1),'or','linewidth',2,'markersize',10);
    hold off; grid on; ylabel('Altitutde [m]');
    axis([t(1) t(end) min([X(1,1) Y Z]) max([X(1,1) Y Z])]);
    %     drawnow;
    % , 'Location', 'Best'
    %     legend({'Leitura do sensor','Sistema Real','Esperança do sistema'});
    %     legend boxoff;
    
    subplot(3,1,2);
    plot(t,e,'g','linewidth',2);
    hold on;
    plot(t(end),E(2),'og','linewidth',2,'markersize',10);
    axis([t(1) t(end) min(e) max(e)]);
    hold off; grid on; xlabel('Tempo [s]'); ylabel('Erro tratado [m]');
    
    subplot(3,1,3);
    plot(t,ecum,'c','linewidth',2);
    hold on;
    plot(t(end),Ecum(2),'oc','linewidth',2,'markersize',10);
    axis([t(1) t(end) min(ecum) max(ecum)]);
    hold off; grid on; xlabel('Tempo [s]'); ylabel('Erro acumulado [m]');
    drawnow;
end
subplot(3,1,1);
legend({'Leitura do sensor','Sistema Real','Esperança do sistema'}, 'Location', 'Best');
legend boxoff;
title('Movimento Uniformemente Variado (MUV)');