%% HE
clc; clear; yalmip('clear');
close all;

%% Load polytope and observer matrices
load polyObs

%% Simulation parameters
Time = 720.1;                        % Simulation end time [min] (12 h)
Ts = 0.05;                              % Sample time [min] (3 seg)
Nsim = Time/Ts;                    % Simulation steps
t = 0:Ts:Time-Ts;                   % Simulation time
Fact_1 = 5; Fact_2 = 0.43;   % Actuator fault magnitude [5%, 5%]
Fsen_1 = 2.5; Fsen_2 = 3.5;	% Sensor fault magnitude [0.5% 0.5%]

% %% Polytope model and observers
% Theta_1s_min = 495;     % Temperatura mínima de salida de fluido 1 (K)
% Theta_1s_mid = 497.32;     % Temperatura media de salida de fluido 1 (K)
% Theta_1s_max = 500;     % Temperatura máxima de salida de fluido 1 (K)
% 
% Theta_2s_min = 662;     % Temperatura mínima de salida de fluido 2 (K)
% Theta_2s_mid = 695.915;     % Temperatura media de salida de fluido 2 (K)
% Theta_2s_max = 725;     % Temperatura máxima de salida de fluido 2 (K)
% 
% run HE_polytope;
% M = 9;
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
% run RUIO;
% N = 2;
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
mag_1 = 4e-2;     % Value Q1
mag_2 = 5e-2;     % Value Q2
mag_3 = 4e-3;     % Value O1 %3e-4
mag_4 = 1e-1;     % Value O2

threshold = zeros(4, Nsim);

for k = 1:Nsim
    threshold(1, k) = mag_1 + 1000*exp(-(k-1)/Tau);  % Q1
    threshold(2, k) = mag_2 + 900*exp(-(k-1)/Tau);  % Q2
    threshold(3, k) = mag_3 + 100*exp(-(k-1)/Tau);  % O1
    threshold(4, k) = mag_4 + 1000*exp(-(k-1)/Tau);  % O2
end

%% Parámetros del PID
ek = [0; 0]; ek_1 = [0; 0];
% Kr = [0.5; 0.5]; 
% Ki = [1.25; 1.25];
Kr = [-1; 5e-2]; 
Ki = [-2; 2e-1];

% Ingreso las opciones de la ODE 'RelTol', 1e-6, 'AbsTol', 1e-6
options = odeset ('RelTol', 1e-6, 'AbsTol', 1e-6, ...
	'NormControl', 'on', 'InitialStep', 1.0e-4, 'MaxStep', 1.0);

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

% RUIO 2
Phi_2 = zeros(N, Nsim+1);           % Observer states
X_UIO2 = zeros(nx, Nsim);           % Estimated states
Error_2 = zeros(1, Nsim);             % Error
Fact2 = zeros(1, Nsim);                % Estimated control Input

% UIOO 1
Z1 = zeros(nx, Nsim+1);              % Observer states
Yest1 = zeros(p, Nsim);                % Monitorated outputs
X_UIOO1 = zeros(nx, Nsim);         % Estimated states
res1 = zeros(nx, Nsim);                % Residue
Error1 = zeros(1, Nsim);               % Error
Fsen1 = zeros(1, Nsim);               % Estimated sensor fault

% UIOO 2
Z2 = zeros(nx, Nsim+1);              % Observer states
Yest2 = zeros(p, Nsim);                % Monitorated outputs
X_UIOO2 = zeros(nx, Nsim);         % Estimated states
res2 = zeros(nx, Nsim);                % Residue
Error2 = zeros(1, Nsim);               % Error
Fsen2 = zeros(1, Nsim);               % Estimated sensor fault

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
            Ufails(:, k) = [Fact_1-Fact_1*(exp(-(tk-30)/10)); 0];
            Ufail(:, k) = U(:, k) + Ufails(:, k);
        end
        
        if tk > 220 && tk < 320
            Ufails(:, k) = [0; -Fact_2+Fact_2*(exp(-(tk-220)/10))];
            Ufail(:, k) = U(:, k) + Ufails(:, k);
        end
        
        %% Process simulation with ODE
        [tsim, x] = ode45(@(x, u) HE(X(:, k), Ufail(:, k)) , [0 Ts], X(:, k), options);
        X(:, k+1) = x(end, :)';
        Y(:, k) = C*X(:, k)+v(:, k);
        
        %% Sensor fault income
        Yfail(:, k) = Y(:, k);

        if tk >400 && tk < 500
            Yfail(:, k) = Y(:, k) + [0; Fsen_2-Fsen_2*(exp(-(tk-400)/5)); 0];
        end

        if tk > 580 && tk < 680
            Yfail(:, k) = Y(:, k) + [-Fsen_1+Fsen_1*(exp(-(tk-580)/6)); 0; 0];
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
            Xsp(1, k) = Theta_1s_mid+((Theta_1s_max-Theta_1s_mid)*(tk-340)/40);
        elseif tk >= 380 && tk < 520
            Xsp(1, k) = Theta_1s_max;
        elseif tk >= 520 && tk < 560
            Xsp(1, k) = Theta_1s_max-((Theta_1s_max-Theta_1s_mid)*(tk-520)/40);
        end        
        
        %% membership
        mu_out(:, k) = membership(Yfail(:, k), Theta_1s_min, Theta_1s_mid, Theta_1s_max, Theta_2s_min, Theta_2s_mid, Theta_2s_max);
        mu_in(:, k) = membership(Yfail(:, k), Theta_1s_min, Theta_1s_mid, Theta_1s_max, Theta_2s_min, Theta_2s_mid, Theta_2s_max);
      
        %% LPV-RUIO 1
        Phi_1(:, k+1) = mu_out(1, k)*(H1_K1*Phi_1(:, k) + H1_L1_ast*Yfail(:, k) + H1_B1_bar_1*U(:, k) + H1_delta1_bar_1) ...
                              + mu_out(2, k)*(H1_K2*Phi_1(:, k) + H1_L2_ast*Yfail(:, k) + H1_B2_bar_1*U(:, k) + H1_delta2_bar_1) ...
                              + mu_out(3, k)*(H1_K3*Phi_1(:, k) + H1_L3_ast*Yfail(:, k) + H1_B3_bar_1*U(:, k) + H1_delta3_bar_1) ...
                              + mu_out(4, k)*(H1_K4*Phi_1(:, k) + H1_L4_ast*Yfail(:, k) + H1_B4_bar_1*U(:, k) + H1_delta4_bar_1) ...
                              + mu_out(5, k)*(H1_K5*Phi_1(:, k) + H1_L5_ast*Yfail(:, k) + H1_B5_bar_1*U(:, k) + H1_delta5_bar_1) ...
                              + mu_out(6, k)*(H1_K6*Phi_1(:, k) + H1_L6_ast*Yfail(:, k) + H1_B6_bar_1*U(:, k) + H1_delta6_bar_1) ...
                              + mu_out(7, k)*(H1_K7*Phi_1(:, k) + H1_L7_ast*Yfail(:, k) + H1_B7_bar_1*U(:, k) + H1_delta7_bar_1) ...
                              + mu_out(8, k)*(H1_K8*Phi_1(:, k) + H1_L8_ast*Yfail(:, k) + H1_B8_bar_1*U(:, k) + H1_delta8_bar_1) ...
                              + mu_out(9, k)*(H1_K9*Phi_1(:, k) + H1_L9_ast*Yfail(:, k) + H1_B9_bar_1*U(:, k) + H1_delta9_bar_1);

        X_UIO1(:, k) = mu_out(1, k)*(H1_T1*[Phi_1(:, k); H1_U1_1*Yfail(:, k)-H1_U1_1*H1_C1_tilde_1*Phi_1(:, k)]) ...
                            + mu_out(2, k)*(H1_T2*[Phi_1(:, k); H1_U2_1*Yfail(:, k)-H1_U2_1*H1_C2_tilde_1*Phi_1(:, k)]) ...
                            + mu_out(3, k)*(H1_T3*[Phi_1(:, k); H1_U3_1*Yfail(:, k)-H1_U3_1*H1_C3_tilde_1*Phi_1(:, k)]) ...
                            + mu_out(4, k)*(H1_T4*[Phi_1(:, k); H1_U4_1*Yfail(:, k)-H1_U4_1*H1_C4_tilde_1*Phi_1(:, k)]) ...
                            + mu_out(5, k)*(H1_T5*[Phi_1(:, k); H1_U5_1*Yfail(:, k)-H1_U5_1*H1_C5_tilde_1*Phi_1(:, k)]) ...
                            + mu_out(6, k)*(H1_T6*[Phi_1(:, k); H1_U6_1*Yfail(:, k)-H1_U6_1*H1_C6_tilde_1*Phi_1(:, k)]) ...
                            + mu_out(7, k)*(H1_T7*[Phi_1(:, k); H1_U7_1*Yfail(:, k)-H1_U7_1*H1_C7_tilde_1*Phi_1(:, k)]) ...
                            + mu_out(8, k)*(H1_T8*[Phi_1(:, k); H1_U8_1*Yfail(:, k)-H1_U8_1*H1_C8_tilde_1*Phi_1(:, k)]) ...
                            + mu_out(9, k)*(H1_T9*[Phi_1(:, k); H1_U9_1*Yfail(:, k)-H1_U9_1*H1_C9_tilde_1*Phi_1(:, k)]);

        % Error norm 1
        Error_1(k) = sqrt((X_UIO1(1, k)-Yfail(1, k))^2 + (X_UIO1(2, k)-Yfail(2, k))^2 + (X_UIO1(3, k)-Yfail(3, k))^2);
        
        % Fault estimation 1
        Fact1(k) = mu_out(1, k)*(H1_U1_1*(X(:, k+1) - H1_C1_tilde_1*Phi_1(:, k+1)) + H1_A1_bar_22*H1_U1_1*(H1_C1_tilde_1*Phi_1(:, k) - Yfail(:, k)) - H1_A1_bar_21*Phi_1(:, k) - H1_B1_bar_2*U(:, k) -  H1_delta1_bar_2) ...
                      + mu_out(2, k)*(H1_U2_1*(X(:, k+1) - H1_C2_tilde_1*Phi_1(:, k+1)) + H1_A2_bar_22*H1_U2_1*(H1_C2_tilde_1*Phi_1(:, k) - Yfail(:, k)) - H1_A2_bar_21*Phi_1(:, k) - H1_B2_bar_2*U(:, k) -  H1_delta2_bar_2) ...
                      + mu_out(3, k)*(H1_U3_1*(X(:, k+1) - H1_C3_tilde_1*Phi_1(:, k+1)) + H1_A3_bar_22*H1_U3_1*(H1_C3_tilde_1*Phi_1(:, k) - Yfail(:, k)) - H1_A3_bar_21*Phi_1(:, k) - H1_B3_bar_2*U(:, k) -  H1_delta3_bar_2) ...
                      + mu_out(4, k)*(H1_U4_1*(X(:, k+1) - H1_C4_tilde_1*Phi_1(:, k+1)) + H1_A4_bar_22*H1_U4_1*(H1_C4_tilde_1*Phi_1(:, k) - Yfail(:, k)) - H1_A4_bar_21*Phi_1(:, k) - H1_B4_bar_2*U(:, k) -  H1_delta4_bar_2) ...
                      + mu_out(5, k)*(H1_U5_1*(X(:, k+1) - H1_C5_tilde_1*Phi_1(:, k+1)) + H1_A5_bar_22*H1_U5_1*(H1_C5_tilde_1*Phi_1(:, k) - Yfail(:, k)) - H1_A5_bar_21*Phi_1(:, k) - H1_B5_bar_2*U(:, k) -  H1_delta5_bar_2) ...
                      + mu_out(6, k)*(H1_U6_1*(X(:, k+1) - H1_C6_tilde_1*Phi_1(:, k+1)) + H1_A6_bar_22*H1_U6_1*(H1_C6_tilde_1*Phi_1(:, k) - Yfail(:, k)) - H1_A6_bar_21*Phi_1(:, k) - H1_B6_bar_2*U(:, k) -  H1_delta6_bar_2) ...                  
                      + mu_out(7, k)*(H1_U7_1*(X(:, k+1) - H1_C7_tilde_1*Phi_1(:, k+1)) + H1_A7_bar_22*H1_U7_1*(H1_C7_tilde_1*Phi_1(:, k) - Yfail(:, k)) - H1_A7_bar_21*Phi_1(:, k) - H1_B7_bar_2*U(:, k) -  H1_delta7_bar_2) ...
                      + mu_out(8, k)*(H1_U8_1*(X(:, k+1) - H1_C8_tilde_1*Phi_1(:, k+1)) + H1_A8_bar_22*H1_U8_1*(H1_C8_tilde_1*Phi_1(:, k) - Yfail(:, k)) - H1_A8_bar_21*Phi_1(:, k) - H1_B8_bar_2*U(:, k) -  H1_delta8_bar_2) ...
                      + mu_out(9, k)*(H1_U9_1*(X(:, k+1) - H1_C9_tilde_1*Phi_1(:, k+1)) + H1_A9_bar_22*H1_U9_1*(H1_C9_tilde_1*Phi_1(:, k) - Yfail(:, k)) - H1_A9_bar_21*Phi_1(:, k) - H1_B9_bar_2*U(:, k) -  H1_delta9_bar_2);

        if Error_1(k) > threshold(1, k)
            FQ1 = true;
        else
            FQ1 = false;
        end
                  
        %% LPV-RUIO 2
        Phi_2(:, k+1) = mu_out(1, k)*(H2_K1*Phi_2(:, k) + H2_L1_ast*Yfail(:, k) + H2_B1_bar_1*U(:, k) + H2_delta1_bar_1) ...
                              + mu_out(2, k)*(H2_K2*Phi_2(:, k) + H2_L2_ast*Yfail(:, k) + H2_B2_bar_1*U(:, k) + H2_delta2_bar_1) ...
                              + mu_out(3, k)*(H2_K3*Phi_2(:, k) + H2_L3_ast*Yfail(:, k) + H2_B3_bar_1*U(:, k) + H2_delta3_bar_1) ...
                              + mu_out(4, k)*(H2_K4*Phi_2(:, k) + H2_L4_ast*Yfail(:, k) + H2_B4_bar_1*U(:, k) + H2_delta4_bar_1) ...
                              + mu_out(5, k)*(H2_K5*Phi_2(:, k) + H2_L5_ast*Yfail(:, k) + H2_B5_bar_1*U(:, k) + H2_delta5_bar_1) ...
                              + mu_out(6, k)*(H2_K6*Phi_2(:, k) + H2_L6_ast*Yfail(:, k) + H2_B6_bar_1*U(:, k) + H2_delta6_bar_1) ...
                              + mu_out(7, k)*(H2_K7*Phi_2(:, k) + H2_L7_ast*Yfail(:, k) + H2_B7_bar_1*U(:, k) + H2_delta7_bar_1) ...
                              + mu_out(8, k)*(H2_K8*Phi_2(:, k) + H2_L8_ast*Yfail(:, k) + H2_B8_bar_1*U(:, k) + H2_delta8_bar_1) ...
                              + mu_out(9, k)*(H2_K9*Phi_2(:, k) + H2_L9_ast*Yfail(:, k) + H2_B9_bar_1*U(:, k) + H2_delta9_bar_1);

        X_UIO2(:, k) = mu_out(1, k)*(H2_T1*[Phi_2(:, k); H2_U1_1*Yfail(:, k)-H2_U1_1*H2_C1_tilde_1*Phi_2(:, k)]) ...
                            + mu_out(2, k)*(H2_T2*[Phi_2(:, k); H2_U2_1*Yfail(:, k)-H2_U2_1*H2_C2_tilde_1*Phi_2(:, k)]) ...
                            + mu_out(3, k)*(H2_T3*[Phi_2(:, k); H2_U3_1*Yfail(:, k)-H2_U3_1*H2_C3_tilde_1*Phi_2(:, k)]) ...
                            + mu_out(4, k)*(H2_T4*[Phi_2(:, k); H2_U4_1*Yfail(:, k)-H2_U4_1*H2_C4_tilde_1*Phi_2(:, k)]) ...
                            + mu_out(5, k)*(H2_T5*[Phi_2(:, k); H2_U5_1*Yfail(:, k)-H2_U5_1*H2_C5_tilde_1*Phi_2(:, k)]) ...
                            + mu_out(6, k)*(H2_T6*[Phi_2(:, k); H2_U6_1*Yfail(:, k)-H2_U6_1*H2_C6_tilde_1*Phi_2(:, k)]) ...
                            + mu_out(7, k)*(H2_T7*[Phi_2(:, k); H2_U7_1*Yfail(:, k)-H2_U7_1*H2_C7_tilde_1*Phi_2(:, k)]) ...
                            + mu_out(8, k)*(H2_T8*[Phi_2(:, k); H2_U8_1*Yfail(:, k)-H2_U8_1*H2_C8_tilde_1*Phi_2(:, k)]) ...
                            + mu_out(9, k)*(H2_T9*[Phi_2(:, k); H2_U9_1*Yfail(:, k)-H2_U9_1*H2_C9_tilde_1*Phi_2(:, k)]);
                         
        % Error norm 2
        Error_2(k) = sqrt((X_UIO2(1, k)-Yfail(1, k))^2 + (X_UIO2(2, k)-Yfail(2, k))^2 + (X_UIO2(3, k)-Yfail(3, k))^2);
        
        % Fault estimation 2
        Fact2(k) = mu_out(1, k)*(H2_U1_1*(X(:, k+1) - H2_C1_tilde_1*Phi_2(:, k+1)) + H2_A1_bar_22*H2_U1_1*(H2_C1_tilde_1*Phi_2(:, k) - Yfail(:, k)) - H2_A1_bar_21*Phi_2(:, k) - H2_B1_bar_2*U(:, k) -  H2_delta1_bar_2) ...
                      + mu_out(2, k)*(H2_U2_1*(X(:, k+1) - H2_C2_tilde_1*Phi_2(:, k+1)) + H2_A2_bar_22*H2_U2_1*(H2_C2_tilde_1*Phi_2(:, k) - Yfail(:, k)) - H2_A2_bar_21*Phi_2(:, k) - H2_B2_bar_2*U(:, k) -  H2_delta2_bar_2) ...
                      + mu_out(3, k)*(H2_U3_1*(X(:, k+1) - H2_C3_tilde_1*Phi_2(:, k+1)) + H2_A3_bar_22*H2_U3_1*(H2_C3_tilde_1*Phi_2(:, k) - Yfail(:, k)) - H2_A3_bar_21*Phi_2(:, k) - H2_B3_bar_2*U(:, k) -  H2_delta3_bar_2) ...
                      + mu_out(4, k)*(H2_U4_1*(X(:, k+1) - H2_C4_tilde_1*Phi_2(:, k+1)) + H2_A4_bar_22*H2_U4_1*(H2_C4_tilde_1*Phi_2(:, k) - Yfail(:, k)) - H2_A4_bar_21*Phi_2(:, k) - H2_B4_bar_2*U(:, k) -  H2_delta4_bar_2) ...
                      + mu_out(5, k)*(H2_U5_1*(X(:, k+1) - H2_C5_tilde_1*Phi_2(:, k+1)) + H2_A5_bar_22*H2_U5_1*(H2_C5_tilde_1*Phi_2(:, k) - Yfail(:, k)) - H2_A5_bar_21*Phi_2(:, k) - H2_B5_bar_2*U(:, k) -  H2_delta5_bar_2) ...
                      + mu_out(6, k)*(H2_U6_1*(X(:, k+1) - H2_C6_tilde_1*Phi_2(:, k+1)) + H2_A6_bar_22*H2_U6_1*(H2_C6_tilde_1*Phi_2(:, k) - Yfail(:, k)) - H2_A6_bar_21*Phi_2(:, k) - H2_B6_bar_2*U(:, k) -  H2_delta6_bar_2) ...                  
                      + mu_out(7, k)*(H2_U7_1*(X(:, k+1) - H2_C7_tilde_1*Phi_2(:, k+1)) + H2_A7_bar_22*H2_U7_1*(H2_C7_tilde_1*Phi_2(:, k) - Yfail(:, k)) - H2_A7_bar_21*Phi_2(:, k) - H2_B7_bar_2*U(:, k) -  H2_delta7_bar_2) ...
                      + mu_out(8, k)*(H2_U8_1*(X(:, k+1) - H2_C8_tilde_1*Phi_2(:, k+1)) + H2_A8_bar_22*H2_U8_1*(H2_C8_tilde_1*Phi_2(:, k) - Yfail(:, k)) - H2_A8_bar_21*Phi_2(:, k) - H2_B8_bar_2*U(:, k) -  H2_delta8_bar_2) ...
                      + mu_out(9, k)*(H2_U9_1*(X(:, k+1) - H2_C9_tilde_1*Phi_2(:, k+1)) + H2_A9_bar_22*H2_U9_1*(H2_C9_tilde_1*Phi_2(:, k) - Yfail(:, k)) - H2_A9_bar_21*Phi_2(:, k) - H2_B9_bar_2*U(:, k) -  H2_delta9_bar_2);

        if Error_2(k) > threshold(2, k)
            FQ2 = true;
        else
            FQ2 = false;
        end
        
        %% DLPV-UIOO 1
        Yest1(:, k) = T2_1*Yfail(:, k);
        Z1(:, k+1) = mu_in(1, k)*(N1_1*Z1(:, k) + L1_1*Yest1(:, k) + G1_1*U(:, k) + Tg1_1) ...
                         + mu_in(2, k)*(N1_2*Z1(:, k) + L1_2*Yest1(:, k) + G1_2*U(:, k) + Tg1_2) ...
                         + mu_in(3, k)*(N1_3*Z1(:, k) + L1_3*Yest1(:, k) + G1_3*U(:, k) + Tg1_3) ...
                         + mu_in(4, k)*(N1_4*Z1(:, k) + L1_4*Yest1(:, k) + G1_4*U(:, k) + Tg1_4) ...
                         + mu_in(5, k)*(N1_5*Z1(:, k) + L1_5*Yest1(:, k) + G1_5*U(:, k) + Tg1_5) ...
                         + mu_in(6, k)*(N1_6*Z1(:, k) + L1_6*Yest1(:, k) + G1_6*U(:, k) + Tg1_6) ...
                         + mu_in(7, k)*(N1_7*Z1(:, k) + L1_7*Yest1(:, k) + G1_7*U(:, k) + Tg1_7) ...
                         + mu_in(8, k)*(N1_8*Z1(:, k) + L1_8*Yest1(:, k) + G1_8*U(:, k) + Tg1_8) ...
                         + mu_in(9, k)*(N1_9*Z1(:, k) + L1_9*Yest1(:, k) + G1_9*U(:, k) + Tg1_9);

        X_UIOO1(:, k) = Z1(:, k) - E1*Yest1(:, k);

        % Residue 1
        res1(:, k) = Yfail(:, k) - C*X_UIOO1(:, k);
        
        % Error norm 1
        Error1(k) = sqrt(res1(1, k)^2);
        
        if Error1(k) > threshold(3, k)
            FO1 = true;
        else
            FO1 = false;
        end        

        %% DLPV-UIOO 2
        Yest2(:, k) = T2_2*Yfail(:, k);
        Z2(:, k+1) = mu_in(1, k)*(N2_1*Z2(:, k) + L2_1*Yest2(:, k) + G2_1*U(:, k) + Tg2_1) ...
                         + mu_in(2, k)*(N2_2*Z2(:, k) + L2_2*Yest2(:, k) + G2_2*U(:, k) + Tg2_2) ...
                         + mu_in(3, k)*(N2_3*Z2(:, k) + L2_3*Yest2(:, k) + G2_3*U(:, k) + Tg2_3) ...
                         + mu_in(4, k)*(N2_4*Z2(:, k) + L2_4*Yest2(:, k) + G2_4*U(:, k) + Tg2_4) ...
                         + mu_in(5, k)*(N2_5*Z2(:, k) + L2_5*Yest2(:, k) + G2_5*U(:, k) + Tg2_5) ...
                         + mu_in(6, k)*(N2_6*Z2(:, k) + L2_6*Yest2(:, k) + G2_6*U(:, k) + Tg2_6) ...
                         + mu_in(7, k)*(N2_7*Z2(:, k) + L2_7*Yest2(:, k) + G2_7*U(:, k) + Tg2_7) ...
                         + mu_in(8, k)*(N2_8*Z2(:, k) + L2_8*Yest2(:, k) + G2_8*U(:, k) + Tg2_8) ...
                         + mu_in(9, k)*(N2_9*Z2(:, k) + L2_9*Yest2(:, k) + G2_9*U(:, k) + Tg2_9);

        X_UIOO2(:, k) = Z2(:, k) - E2*Yest2(:, k);
        
        % Residue 2
        res2(:, k) = Yfail(:, k) - C*X_UIOO2(:, k);

        % Error norm 2
        Error2(k) = sqrt(res2(2, k)^2);
        
        if Error2(k) > threshold(4, k)
            FO2 = true;
        else
            FO2 = false;
        end
        
        %% Actuator fault estimation
        % Actuator fault 1
        if ~FQ1 && FQ2 && FO1 && ~FO2
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
        if FQ1 && ~FQ2 && ~FO1 && FO2
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
        if FO1 && ~FO2 && FQ1 && FQ2
            Fsen1(k) = res2(1, k);
        else
            Fsen1(k) = zeros(size(res2(1, k)));
        end
        
        % Sensor fault 2
        if ~FO1 && FO2 && FQ1 && FQ2
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
        % Only calefactor fluid
%         u0(1) = 100;
%         u0(2) = u0(2) + Kr(1)*(ek(1) - ek_1(1)) + Ts*Ki(1)*ek(1);
        % Mixed control
%         u0(1) = u0(1) + Kr(2)*(ek(2) - ek_1(2)) + Ts*Ki(2)*ek(2);
%         u0(2) = u0(2) + Kr(1)*(ek(1) - ek_1(1)) + Ts*Ki(1)*ek(1);
        % Direct control
        u0(1) = u0(1) + Kr(1)*(ek(1) - ek_1(1)) + Ts*Ki(1)*ek(1);
        u0(2) = u0(2) + Kr(2)*(ek(2) - ek_1(2)) + Ts*Ki(2)*ek(2);
        U(:, k+1) = [u0(1); u0(2)];

    end
end

save runHE

run enPlotHE