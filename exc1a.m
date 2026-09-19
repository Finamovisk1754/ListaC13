clc;
clear;
close all;

%% Dados do experimento
delta_u = 0.20;      % Degrau PWM = 20%

T_inicial = 22;      % Temperatura inicial [°C]
T_final = 34;        % Temperatura final [°C]

t1 = 15;             % 28,3% da resposta [s]
t2 = 42;             % 63,2% da resposta [s]

%% Variacao de temperatura
delta_T = T_final - T_inicial;

%% Ganho do processo
K = delta_T / delta_u;

%% Metodo de Smith
tau = 1.5 * (t2 - t1);
L = t2 - tau;

%% Mostrar os parametros
fprintf('===== PARAMETROS IDENTIFICADOS =====\n');
fprintf('Ganho K = %.2f °C/duty\n', K);
fprintf('Constante de tempo tau = %.2f s\n', tau);
fprintf('Atraso L = %.2f s\n', L);

fprintf('\nModelo identificado:\n');
fprintf('          %.2f e^(-%.2fs)\n', K, L);
fprintf('G(s) = -------------------\n');
fprintf('          %.2fs + 1\n', tau);

%% Tempo de simulacao
t = 0:0.1:200;

%% Inicializar temperatura
T = zeros(size(t));

%% Resposta do sistema
for i = 1:length(t)

    if t(i) < L
        % Antes do atraso, a temperatura ainda nao responde
        T(i) = T_inicial;

    else
        % Modelo de primeira ordem com atraso
        T(i) = T_inicial + ...
            K * delta_u * ...
            (1 - exp(-(t(i) - L) / tau));
    end

end

%% Pontos de Smith
T_283 = T_inicial + 0.283 * delta_T;
T_632 = T_inicial + 0.632 * delta_T;

%% Grafico
figure;

plot(t, T, 'LineWidth', 2);
hold on;
grid on;

% Temperatura final
yline(T_final, '--', 'Temperatura final = 34 °C');

% Tempos utilizados pelo metodo de Smith
xline(t1, '--', 't1 = 15 s');
xline(t2, '--', 't2 = 42 s');

% Pontos de 28,3% e 63,2%
plot(t1, T_283, 'o', 'MarkerSize', 8, 'LineWidth', 2);
plot(t2, T_632, 'o', 'MarkerSize', 8, 'LineWidth', 2);

xlabel('Tempo [s]');
ylabel('Temperatura [°C]');

title('Resposta da Estufa - Metodo de Smith');

legend('Temperatura', ...
       'Temperatura final', ...
       't1', ...
       't2', ...
       '28,3%', ...
       '63,2%', ...
       'Location', 'southeast');

ylim([20 36]);