clc;
clear;
close all;

%% Dados da estufa
K = 60;
tau = 40.5;
theta = 1.5;

%% =========================================================
% b) IMC com lambda = 4*theta
%% =========================================================

lambda2 = 4 * theta;

Kp2 = (1/K) * ((2*tau + theta) / (2*(lambda2 + theta)));
Ti2 = tau + theta/2;
Td2 = (tau*theta) / (2*tau + theta);

fprintf('====================================\n');
fprintf('b) IMC COM lambda = 4*theta\n');
fprintf('====================================\n');

fprintf('lambda = %.2f s\n', lambda2);
fprintf('Kp = %.4f\n', Kp2);
fprintf('Ti = %.4f s\n', Ti2);
fprintf('Td = %.4f s\n\n', Td2);

%% =========================================================
% Parametros de lambda = 1.5*theta
% Necessarios para comparar os dois na letra C
%% =========================================================

lambda1 = 1.5 * theta;

Kp1 = (1/K) * ((2*tau + theta) / (2*(lambda1 + theta)));
Ti1 = tau + theta/2;
Td1 = (tau*theta) / (2*tau + theta);

%% =========================================================
% c) Resposta a um degrau de valor 20
%% =========================================================

degrau = 20;

dt = 0.001;
tempo_final = 150;

t = 0:dt:tempo_final;

% Referencia
r = degrau * ones(size(t));

% Saidas
y1 = zeros(size(t));
y2 = zeros(size(t));

% Sinais de controle
u1 = zeros(size(t));
u2 = zeros(size(t));

% Atraso em numero de amostras
N_atraso = round(theta/dt);

%% Variaveis PID para lambda = 1.5*theta

integral1 = 0;
erro_anterior1 = degrau;

%% Variaveis PID para lambda = 4*theta

integral2 = 0;
erro_anterior2 = degrau;

%% Simulacao

for i = 2:length(t)

    %% -------------------------------------
    % lambda = 1.5*theta
    %% -------------------------------------

    erro1 = r(i) - y1(i-1);

    integral1 = integral1 + erro1*dt;

    derivada1 = (erro1 - erro_anterior1)/dt;

    u1(i) = Kp1 * ...
        (erro1 + integral1/Ti1 + Td1*derivada1);

    % Aplicacao do atraso
    if i > N_atraso
        u_atrasado1 = u1(i-N_atraso);
    else
        u_atrasado1 = 0;
    end

    % Planta
    dy1 = (-y1(i-1) + K*u_atrasado1)/tau;

    y1(i) = y1(i-1) + dy1*dt;

    erro_anterior1 = erro1;


    %% -------------------------------------
    % lambda = 4*theta
    %% -------------------------------------

    erro2 = r(i) - y2(i-1);

    integral2 = integral2 + erro2*dt;

    derivada2 = (erro2 - erro_anterior2)/dt;

    u2(i) = Kp2 * ...
        (erro2 + integral2/Ti2 + Td2*derivada2);

    % Aplicacao do atraso
    if i > N_atraso
        u_atrasado2 = u2(i-N_atraso);
    else
        u_atrasado2 = 0;
    end

    % Planta
    dy2 = (-y2(i-1) + K*u_atrasado2)/tau;

    y2(i) = y2(i-1) + dy2*dt;

    erro_anterior2 = erro2;

end

%% =========================================================
% Grafico
%% =========================================================

figure;

plot(t, y1, 'LineWidth', 2);
hold on;

plot(t, y2, 'LineWidth', 2);

yline(degrau, '--', 'Referencia = 20');

grid on;

xlabel('Tempo [s]');
ylabel('Saida');

title('Resposta ao Degrau - PID IMC');

legend('lambda = 1.5 theta', ...
       'lambda = 4 theta', ...
       'Referencia = 20', ...
       'Location', 'southeast');