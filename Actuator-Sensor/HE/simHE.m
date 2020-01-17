%% HE
clc; clear; yalmip('clear');
close all;

%% Load polytope and observer matrices
load polyObs

% Ingreso las opciones de la ODE 'RelTol', 1e-6, 'AbsTol', 1e-6
options = odeset ('RelTol', 1e-6, 'AbsTol', 1e-6, ...
	'NormControl', 'on', 'InitialStep', 1.0e-4, 'MaxStep', 1.0);

%% Simulation parameters
Time = 720.1;                        % Simulation end time [min] (12 h)
Ts = 0.05;                              % Sample time [min] (3 seg)
Nsim = Time/Ts;                    % Simulation steps
t = 0:Ts:Time-Ts;                   % Simulation time
Fact_1 = 5; Fact_2 = 0.43;   % Actuator fault magnitude [5%, 5%]
Fsen_1 = 2.5; Fsen_2 = -3.5;	% Sensor fault magnitude [0.5% 0.5%]

% %% Polytope model and observers
% Theta_1s_min = 495;     % Temperatura mínima de salida de fluido 1 (K)
% Theta_1s_mid = 497.32;     % Temperatura media de salida de fluido 1 (K)
% Theta_1s_max = 500;     % Temperatura máxima de salida de fluido 1 (K)
% 
% Theta_2s_min = 680;     % Temperatura mínima de salida de fluido 2 (K)
% Theta_2s_mid = 695.915;     % Temperatura media de salida de fluido 2 (K)
% Theta_2s_max = 710;     % Temperatura máxima de salida de fluido 2 (K)
% 
% run HE_polytope;
% M = 9;
% N = 2;
% 
% % Observers start point
% Theta_1s = Theta_1s_mid;     % Temperatura de salida de fluido 1 (K)
% Theta_2s = Theta_2s_min;     % Temperatura de salida de fluido 2 (K)
% run HE_linear;
% x0_obs = [Theta_1s; Theta_2s; Theta_p];
% 
% % System start point
% Theta_1s = Theta_1s_min;     % Temperatura de salida de fluido 1 (K)
% Theta_2s = Theta_2s_mid;     % Temperatura de salida de fluido 2 (K)
% run HE_linear;
% x0 = [Theta_1s; Theta_2s; Theta_p];
% 
% % Reduced-order unknown input observer
% run HE_DLPV_RUIO;
% 
% % Unknown input output observer
% run HE_DLPV_UIOO;
% 
% % Save observers' data
% save polyObs.mat

%% Noise
sig = 3e-3*([1 1 1])';    % Ouput noise sigma

rng default;                        % Random seed start
v = sig*randn(1, Nsim);    % Measurement noise v~N(0, sig)

%% Error detection threshold
Tau = 10;    % period
mag_1 = 1.3e-1;     % Value Q1
mag_2 = 5e-2;     % Value Q2
mag_3 = 6e-2;     % Value O1 %3e-4
mag_4 = 1.3e-1;     % Value O2 %1e-1

threshold = zeros(4, Nsim);

for k = 1:Nsim
    threshold(1, k) = mag_1 + 1000*exp(-(k-1)/Tau);  % Q1
    threshold(2, k) = mag_2 + 900*exp(-(k-1)/Tau);  % Q2
    threshold(3, k) = mag_3 + 100*exp(-(k-1)/Tau);  % O1
    threshold(4, k) = mag_4 + 1000*exp(-(k-1)/Tau);  % O2
end

%% Parámetros del PID
ek = [0; 0]; ek_1 = [0; 0];
Kr = [-1; 5e-2]; 
Ki = [-2; 2e-1];

%% Simulation Setup
U = zeros(nu, Nsim);                   % Control Input
Ufail = zeros(nu, Nsim);               % Fault control Input
Ufails = zeros(nu, Nsim);             % Fails
X = zeros(nx, Nsim+1);               % States
Y = zeros(ny, Nsim);                    % Measure outputs
Yfail = zeros(ny, Nsim);                % Fault measure outputs
mu_out = zeros(M, Nsim);           % Membership
mu_in = zeros(M, Nsim);              % Membership
Xsp = zeros(nx, Nsim);                 % Set-point
delay_1 = 0; delay_2 = 0;

% RUIO 1
Phi_1 = zeros(N, Nsim+1);           % Observer states
X_UIO1 = zeros(nx, Nsim);           % Estimated states
Error_1 = zeros(1, Nsim);             % Error
Fact1 = zeros(1, Nsim);                % Estimated control Input
FQ1 = zeros(1, Nsim);                  % Fault detect Q1

% RUIO 2
Phi_2 = zeros(N, Nsim+1);           % Observer states
X_UIO2 = zeros(nx, Nsim);           % Estimated states
Error_2 = zeros(1, Nsim);             % Error
Fact2 = zeros(1, Nsim);                % Estimated control Input
FQ2 = zeros(1, Nsim);                  % Fault detect Q2

% UIOO 1
Z1 = zeros(nx, Nsim+1);              % Observer states
Yfail1 = zeros(N, Nsim);                % Monitorated outputs
X_UIOO1 = zeros(nx, Nsim);         % Estimated states
Y_UIOO1 = zeros(nx, Nsim);         % Estimated outputs
res1 = zeros(nx, Nsim);                % Residue
Error1 = zeros(1, Nsim);               % Error
Fsen1 = zeros(1, Nsim);               % Estimated sensor fault
FO1 = zeros(1, Nsim);                  % Fault detect S1

% UIOO 2
Z2 = zeros(nx, Nsim+1);              % Observer states
Yfail2 = zeros(N, Nsim);                % Monitorated outputs
X_UIOO2 = zeros(nx, Nsim);         % Estimated states
Y_UIOO2 = zeros(nx, Nsim);         % Estimated outputs
res2 = zeros(nx, Nsim);                % Residue
Error2 = zeros(1, Nsim);               % Error
Fsen2 = zeros(1, Nsim);               % Estimated sensor fault
FO2 = zeros(1, Nsim);                  % Fault detect S2

% Initial states and inputs
X(:, 1) = x0;
Xsp(:, 1) = x0;
U(:, 1) = U_lin;
X_UIO1(:, 1) = x0_obs;
X_UIO1(:, 1) = x0_obs;
X_UIOO1(:, 1) = x0_obs;
X_UIOO1(:, 1) = x0_obs;

%% Simulation
for FTC = 0 % 0 - FTC is off; 1 - FTC is on

    for k = 1:Nsim
        tk = k*Ts; % Simulation time
        
        %% Actuator fault income
        Ufail(:, k) = U(:, k);      
        Ufails(:, k) = [0; 0];

        if tk > 30 && tk < 130
            Ufails(:, k) = [Fact_1; 0];
            Ufail(:, k) = U(:, k) + Ufails(:, k);
        end
        
        if tk > 580 && tk < 680
            Ufails(:, k) = [0; -Fact_2+Fact_2*(exp(-(tk-580)/10))];
            Ufail(:, k) = U(:, k) + Ufails(:, k);
        end
        
        %% Process simulation with ODE
        [tsim, x] = ode45(@(x, u) HE(X(:, k), Ufail(:, k)) , [0 Ts], X(:, k), options);
        X(:, k+1) = x(end, :)';
        Y(:, k) = C*X(:, k)+v(:, k);
        
        %% Sensor fault income
        Yfail(:, k) = Y(:, k);

        if tk > 220 && tk < 320
            Yfail(:, k) = Y(:, k) + [0; Fsen_2-Fsen_2*(exp(-(tk-220)/5)); 0];
        end

        if tk >400 && tk < 500
            Yfail(:, k) = Y(:, k) + [Fsen_1-Fsen_1*(exp(-(tk-400)/6)); 0; 0];
        end
        
        %% Setpoint
        Xsp(1, k) = Theta_1s_mid;
        Xsp(2, k) = Theta_2s_mid;

        if tk < 160
            Xsp(1, k) = Theta_1s_min;
        elseif tk >= 160 && tk < 200
            Xsp(1, k) = Theta_1s_min+((Theta_1s_mid-Theta_1s_min)*(tk-160)/40);
        elseif tk >= 200 && tk < 340
            Xsp(1, k) = Theta_1s_mid;
         elseif tk >= 340 && tk < 380
            Xsp(1, k) = Theta_1s_mid+((Theta_1s_max-2-Theta_1s_mid)*(tk-340)/40);
        elseif tk >= 380 && tk < 520
            Xsp(1, k) = Theta_1s_max-2;
        elseif tk >= 520 && tk < 560
            Xsp(1, k) = Theta_1s_max-2;
            Xsp(2, k) = Theta_2s_mid-((Theta_2s_mid-Theta_2s_min)*(tk-520)/40);
        elseif tk >= 560 && tk < 740
            Xsp(1, k) = Theta_1s_max-2;
            Xsp(2, k) = Theta_2s_min;
        end        
        
        %% membership
        mu_out(:, k) = membership(Yfail(:, k), Theta_1s_min, Theta_1s_mid, Theta_1s_max, Theta_2s_min, Theta_2s_mid, Theta_2s_max);
        mu_in(:, k) = membership(Yfail(:, k), Theta_1s_min, Theta_1s_mid, Theta_1s_max, Theta_2s_min, Theta_2s_mid, Theta_2s_max);
      
        %% LPV-RUIO 1
        Phi_1(:, k+1) = zeros(N, 1);
        X_UIO1(:, k) = zeros(nx, 1);
        Fact1(k) = zeros(1, 1);
        for i = 1:M
            Phi_1(:, k+1) = Phi_1(:, k+1) + mu_out(i, k)*(RUIO(1).O(i).K*Phi_1(:, k) + RUIO(1).O(i).L_ast*Yfail(:, k) + RUIO(1).O(i).B_bar_1*U(:, k) + RUIO(1).O(i).delta_bar_1);
            X_UIO1(:, k) = X_UIO1(:, k) + mu_out(i, k)*(RUIO(1).O(i).T*[Phi_1(:, k); RUIO(1).O(i).U_1*Yfail(:, k)-RUIO(1).O(i).U_1*RUIO(1).O(i).C_tilde_1*Phi_1(:, k)]);
            
            Fact1(k) = Fact1(k) + mu_out(i, k)*(RUIO(1).O(i).U_1*(X(:, k+1) - RUIO(1).O(i).C_tilde_1*Phi_1(:, k+1)) + RUIO(1).O(i).A_bar_22*RUIO(1).O(i).U_1*(RUIO(1).O(i).C_tilde_1*Phi_1(:, k) - Yfail(:, k)) - RUIO(1).O(i).A_bar_21*Phi_1(:, k) - RUIO(1).O(i).B_bar_2*U(:, k) -  RUIO(1).O(i).delta_bar_2);
        end

        % Error norm 1
        Error_1(k) = sqrt((X_UIO1(1, k)-Yfail(1, k))^2 + (X_UIO1(2, k)-Yfail(2, k))^2 + (X_UIO1(3, k)-Yfail(3, k))^2);
        
        if Error_1(k) > threshold(1, k)
            FQ1(k) = true;
        else
            FQ1(k) = false;
        end
                  
        %% LPV-RUIO 2
        Phi_2(:, k+1) = zeros(N, 1);
        X_UIO2(:, k) = zeros(nx, 1);
        Fact2(k) = zeros(1, 1);
        for i = 1:M
            Phi_2(:, k+1) = Phi_2(:, k+1) + mu_out(i, k)*(RUIO(2).O(i).K*Phi_2(:, k) + RUIO(2).O(i).L_ast*Yfail(:, k) + RUIO(2).O(i).B_bar_1*U(:, k) + RUIO(2).O(i).delta_bar_1);
            X_UIO2(:, k) = X_UIO2(:, k) + mu_out(i, k)*(RUIO(2).O(i).T*[Phi_2(:, k); RUIO(2).O(i).U_1*Yfail(:, k)-RUIO(2).O(i).U_1*RUIO(2).O(i).C_tilde_1*Phi_2(:, k)]);
            
            Fact2(k) = Fact2(k) + mu_out(i, k)*(RUIO(2).O(i).U_1*(X(:, k+1) - RUIO(2).O(i).C_tilde_1*Phi_2(:, k+1)) + RUIO(2).O(i).A_bar_22*RUIO(2).O(i).U_1*(RUIO(2).O(i).C_tilde_1*Phi_2(:, k) - Yfail(:, k)) - RUIO(2).O(i).A_bar_21*Phi_2(:, k) - RUIO(2).O(i).B_bar_2*U(:, k) -  RUIO(2).O(i).delta_bar_2);
        end
                         
        % Error norm 2
        Error_2(k) = sqrt((X_UIO2(1, k)-Yfail(1, k))^2 + (X_UIO2(2, k)-Yfail(2, k))^2 + (X_UIO2(3, k)-Yfail(3, k))^2);

        if Error_2(k) > threshold(2, k)
            FQ2(k) = true;
        else
            FQ2(k) = false;
        end
        
        %% DLPV-UIOO 1
        Yfail1(:, k) = UIOO(1).T2*Yfail(:, k);
        Z1(:, k+1) = zeros(nx, 1);      
        for i = 1:M
            Z1(:, k+1) = Z1(:, k+1) + mu_in(i, k)*(UIOO(1).O(i).N*Z1(:, k) + UIOO(1).O(i).L*Yfail1(:, k) + UIOO(1).O(i).G*U(:, k) + UIOO(1).O(i).Tg);
        end

        X_UIOO1(:, k) = Z1(:, k) - UIOO(1).E*Yfail1(:, k);
        Y_UIOO1(:, k) = C*X_UIOO1(:, k);
        
        % Residue 1
        res1(:, k) = Yfail(:, k) - Y_UIOO1(:, k);
        
        % Error norm 1
        Error1(k) = sqrt(res1(1, k)^2);
        
        if Error1(k) > threshold(3, k)
            FO1(k) = true;
        else
            FO1(k) = false;
        end        

        %% DLPV-UIOO 2
        Yfail2(:, k) = UIOO(2).T2*Yfail(:, k);
        Z2(:, k+1) = zeros(nx, 1);      
        for i = 1:M
            Z2(:, k+1) = Z2(:, k+1) + mu_in(i, k)*(UIOO(2).O(i).N*Z2(:, k) + UIOO(2).O(i).L*Yfail2(:, k) + UIOO(2).O(i).G*U(:, k) + UIOO(2).O(i).Tg);
        end

        X_UIOO2(:, k) = Z2(:, k) - UIOO(2).E*Yfail2(:, k);
        Y_UIOO2(:, k) = C*X_UIOO2(:, k);
        
        % Residue 2
        res2(:, k) = Yfail(:, k) - Y_UIOO2(:, k);

        % Error norm 2
        Error2(k) = sqrt(res2(2, k)^2);
        
        if Error2(k) > threshold(4, k)
            FO2(k) = true;
        else
            FO2(k) = false;
        end
        
        %% Actuator fault estimation
        % Actuator fault 1
        if ~FQ1(k) && FQ2(k) && FO1(k) && ~FO2(k)
            if delay_1
                Fact1(k) = Fact1(k);
            else
                delay_1 = 1;
                Fact1(k) = 0;
            end
        else
            delay_1 = 0;
            Fact1(k) = 0;
        end
            
        % Actuator fault 2
        if FQ1(k) && ~FQ2(k) && ~FO1(k) && FO2(k)
            if delay_2
                Fact2(k) = Fact2(k);
            else
                delay_2 = 1;
                Fact2(k) = 0;
            end
        else
            delay_2 = 0;
            Fact2(k) = 0;
        end

        %% Sensor fault estimation
        % Sensor fault 1
        if FQ1(k) && FQ2(k) && FO1(k) && ~FO2(k)
            Fsen1(k) = res2(1, k);
        else
            Fsen1(k) = zeros(size(res2(1, k)));
        end
        
        % Sensor fault 2
        if FQ1(k) && FQ2(k) && ~FO1(k) && FO2(k)
            Fsen2(k) = res1(2, k);
        else
            Fsen2(k) = zeros(size(res1(2, k)));
        end
        
        %% PID
        u0(1) = U(1, k);
        u0(2) = U(2, k);
        ek_1 = ek;
        ek(1) = (Xsp(1, k) - Yfail(1, k));
        ek(2) = (Xsp(2, k) - Yfail(2, k));
        % Controller
        u0(1) = u0(1) + Kr(1)*(ek(1) - ek_1(1)) + Ts*Ki(1)*ek(1);
        u0(2) = u0(2) + Kr(2)*(ek(2) - ek_1(2)) + Ts*Ki(2)*ek(2);
        U(:, k+1) = [u0(1); u0(2)];

    end
end

save runHE

run enPlotHE