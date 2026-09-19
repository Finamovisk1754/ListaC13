clc;
clear;
close all;

%% =========================================================
%  DADOS DO SISTEMA
%  G(s) = K * e^(-theta*s) / (tau*s + 1)
%% =========================================================

K = 1.5;        % Ganho do processo
tau = 6;        % Constante de tempo [s]
theta = 1;      % Tempo morto [s]

degrau = 15;    % Valor do degrau

%% =========================================================
%  a) CHR SEM SOBREVALOR (0%)
%% =========================================================

Kp_sem = 0.6 * (tau / (K * theta));
Ti_sem = tau;
Td_sem = 0.5 * theta;

%% =========================================================
%  b) CHR COM 20% DE SOBREVALOR
%% =========================================================

Kp_20 = 0.95 * (tau / (K * theta));
Ti_20 = 1.4 * tau;
Td_20 = 0.47 * theta;

%% =========================================================
%  MOSTRAR OS RESULTADOS
%% =========================================================

fprintf('============================================\n');
fprintf('       PARAMETROS DO SISTEMA\n');
fprintf('============================================\n');

fprintf('K     = %.2f\n', K);
fprintf('tau   = %.2f s\n', tau);
fprintf('theta = %.2f s\n\n', theta);


fprintf('============================================\n');
fprintf('       CHR SEM SOBREVALOR (0%%)\n');
fprintf('============================================\n');

fprintf('Kp = 0.6 * (tau / (K * theta))\n');
fprintf('Kp = 0.6 * (%.2f / (%.2f * %.2f))\n', ...
        tau, K, theta);
fprintf('Kp = %.2f\n\n', Kp_sem);

fprintf('Ti = tau\n');
fprintf('Ti = %.2f s\n\n', Ti_sem);

fprintf('Td = 0.5 * theta\n');
fprintf('Td = 0.5 * %.2f\n', theta);
fprintf('Td = %.2f s\n\n', Td_sem);


fprintf('============================================\n');
fprintf('       CHR COM 20%% DE SOBREVALOR\n');
fprintf('============================================\n');

fprintf('Kp = 0.95 * (tau / (K * theta))\n');
fprintf('Kp = 0.95 * (%.2f / (%.2f * %.2f))\n', ...
        tau, K, theta);
fprintf('Kp = %.2f\n\n', Kp_20);

fprintf('Ti = 1.4 * tau\n');
fprintf('Ti = 1.4 * %.2f\n', tau);
fprintf('Ti = %.2f s\n\n', Ti_20);

fprintf('Td = 0.47 * theta\n');
fprintf('Td = 0.47 * %.2f\n', theta);
fprintf('Td = %.2f s\n\n', Td_20);


%% =========================================================
%  COMPARACAO DOS Kp
%% =========================================================

diferenca_Kp = Kp_20 - Kp_sem;

aumento_percentual = ...
    ((Kp_20 - Kp_sem) / Kp_sem) * 100;

fprintf('============================================\n');
fprintf('       COMPARACAO DOS GANHOS\n');
fprintf('============================================\n');

fprintf('Kp sem sobrevalor = %.2f\n', Kp_sem);
fprintf('Kp com 20%%        = %.2f\n', Kp_20);

fprintf('Diferenca          = %.2f\n', diferenca_Kp);
fprintf('Aumento percentual = %.2f %%\n\n', ...
        aumento_percentual);


%% =========================================================
%  c) SIMULACAO DA RESPOSTA AO DEGRAU
%  SEM CONTROL SYSTEM TOOLBOX
%% =========================================================

dt = 0.001;            % Passo de simulacao
tempo_final = 40;      % Tempo total [s]

t = 0:dt:tempo_final;

% Referencia = degrau de valor 15
r = degrau * ones(size(t));

% Saidas
y_sem = zeros(size(t));
y_20 = zeros(size(t));

% Sinais de controle
u_sem = zeros(size(t));
u_20 = zeros(size(t));

% Converter atraso em numero de amostras
N_atraso = round(theta / dt);

%% Variaveis do PID sem sobrevalor

integral_sem = 0;
erro_anterior_sem = degrau;

%% Variaveis do PID com 20%

integral_20 = 0;
erro_anterior_20 = degrau;


%% =========================================================
%  LOOP DE SIMULACAO
%% =========================================================

for i = 2:length(t)

    %% -----------------------------------------------------
    % CHR SEM SOBREVALOR
    %% -----------------------------------------------------

    erro_sem = r(i) - y_sem(i-1);

    % Integral
    integral_sem = integral_sem + erro_sem * dt;

    % Derivada
    derivada_sem = ...
        (erro_sem - erro_anterior_sem) / dt;

    % PID
    u_sem(i) = Kp_sem * ...
        (erro_sem + ...
        (integral_sem / Ti_sem) + ...
        Td_sem * derivada_sem);

    % Aplicacao do atraso
    if i > N_atraso
        u_atrasado_sem = u_sem(i - N_atraso);
    else
        u_atrasado_sem = 0;
    end

    % Equacao da planta:
    % tau*dy/dt + y = K*u

    dy_sem = ...
        (-y_sem(i-1) + K*u_atrasado_sem) / tau;

    y_sem(i) = ...
        y_sem(i-1) + dy_sem * dt;

    erro_anterior_sem = erro_sem;


    %% -----------------------------------------------------
    % CHR COM 20% DE SOBREVALOR
    %% -----------------------------------------------------

    erro_20 = r(i) - y_20(i-1);

    integral_20 = integral_20 + erro_20 * dt;

    derivada_20 = ...
        (erro_20 - erro_anterior_20) / dt;

    u_20(i) = Kp_20 * ...
        (erro_20 + ...
        (integral_20 / Ti_20) + ...
        Td_20 * derivada_20);

    if i > N_atraso
        u_atrasado_20 = u_20(i - N_atraso);
    else
        u_atrasado_20 = 0;
    end

    dy_20 = ...
        (-y_20(i-1) + K*u_atrasado_20) / tau;

    y_20(i) = ...
        y_20(i-1) + dy_20 * dt;

    erro_anterior_20 = erro_20;

end


%% =========================================================
%  CALCULO DO SOBREVALOR REAL DA SIMULACAO
%% =========================================================

valor_max_sem = max(y_sem);
valor_max_20 = max(y_20);

sobrevalor_sem = ...
    ((valor_max_sem - degrau) / degrau) * 100;

sobrevalor_20 = ...
    ((valor_max_20 - degrau) / degrau) * 100;

% Evita mostrar sobrevalor negativo
sobrevalor_sem = max(0, sobrevalor_sem);
sobrevalor_20 = max(0, sobrevalor_20);

fprintf('============================================\n');
fprintf('       RESULTADOS DA SIMULACAO\n');
fprintf('============================================\n');

fprintf('Degrau aplicado = %.2f\n\n', degrau);

fprintf('CHR sem sobrevalor:\n');
fprintf('Valor maximo = %.2f\n', valor_max_sem);
fprintf('Sobrevalor = %.2f %%\n\n', sobrevalor_sem);

fprintf('CHR com 20%%:\n');
fprintf('Valor maximo = %.2f\n', valor_max_20);
fprintf('Sobrevalor = %.2f %%\n', sobrevalor_20);


%% =========================================================
%  GRAFICO
%% =========================================================

figure;

plot(t, y_sem, 'LineWidth', 2);
hold on;

plot(t, y_20, 'LineWidth', 2);

yline(degrau, '--', 'Referencia = 15');

grid on;

xlabel('Tempo [s]');
ylabel('Saida');

title('Resposta ao Degrau - Controladores PID CHR');

legend('CHR sem sobrevalor', ...
       'CHR com 20% de sobrevalor', ...
       'Referencia', ...
       'Location', 'southeast');