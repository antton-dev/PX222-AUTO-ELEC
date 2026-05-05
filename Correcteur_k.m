% --- Paramètres du système ---
Rs = 14; 
Rext = 12; 
Rtot = Rs + Rext;
L = 1.2; 
L_V20 = 1.2 * 0.8;  % moins 20 pourcent
L_V20_2 = 1.2 * 1.2 ; % plus 20 pourcent

tau = L / Rtot;
E0 = 60; 
Ka = 2 * E0;
B = 1;

s = tf('s');
H_charge = (1/Rtot) / (1 + tau*s);
H = Ka * H_charge * B;

% --- Optimisation du PI ---
k = 1; % Gain fixé pour éviter la saturation
wi_tests = [21.67, 30, 36.95, 40]; % Inclut la compensation de pôle

figure('Name', 'Réponse PI et Commande');
for wi = wi_tests
    R = k * (1 + wi/s);
    FTBO = R * H;
    FTBF = feedback(FTBO, 1);
    
    % Analyse des performances
    info = stepinfo(FTBF);
    [y, t] = step(FTBF, 0.4);
    
    subplot(2,1,1); plot(t, y, 'LineWidth', 1.5); hold on;
    fprintf('wi = %.2f | Dépassement = %.2f%% | Tr(5%%) = %.3f s\n', ...
            wi, info.Overshoot, info.SettlingTime);
end

% --- Mise en page graphique ---
subplot(2,1,1);
yline(1.05, 'r--', 'Limite 5%'); grid on;
title('Réponse à un échelon de courant (1A)');
ylabel('Courant (A)');
legend(string(wi_tests) + " rad/s");

% Visualisation de la commande pour wi choisi
R_opt = k * (1 + 36.95/s);
Ucom = feedback(R_opt, H); % Transfert Consigne -> Commande
[u, t_u] = step(Ucom, 0.4);
subplot(2,1,2); plot(t_u, u, 'b', 'LineWidth', 1.5); hold on;
yline(1, 'r--', 'Saturation PWM (1V)'); grid on;
title('Signal de commande Ucom(t)');
ylabel('Tension (V)'); xlabel('Temps (s)');