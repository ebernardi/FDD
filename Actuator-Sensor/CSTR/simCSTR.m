%% CSTR
clc; clear; close all;
yalmip('clear');
 
%% Load polytope and observer matrices
load polyObs

%% Simulation parameters
Time = 720.1;                         % Simulation end time
Ts = 0.05;                               % Sample time [min]
Nsim = Time/Ts;                     % Simulation steps
t = 0:Ts:Time-Ts;                    % Simulation time
Fact_1 = 5; Fact_2 = 5;          % Actuator fault magnitude [5% 5%]
Fsen_1 = 1.5; Fsen_2 = -4.5;        % Sensor fault magnitude [1.5% 0 1%]

% %% Polytope model
% V_min = 90;		% Volumen mínimo (m^3)
% V_mid = 98;	% Volumen medio (m^3)
% V_max = 110;	% Volumen máximo (m^3)
% % CA_min = 0.06;	% Concentración mínima (mol/l)
% % CA_mid = 0.087;	% Concentración media (mol/l)
% % CA_max = 0.12;	% Concentración máxima (mol/l)
% Tr_min = 440;	% Temperatura mínima (°K)
% Tr_mid = 445;	% Temperatura media (°K)
% Tr_max = 450;  % Temperatura máxima (°K)
% 
% run CSTR_polytope;
% M = 9;
% 
% % Observer start point
% Vr = V_mid;				% [l] Volumen del reactor
% Tr = Tr_min;             % [K] Temperatura de salida
% % Ca = CA_min;            % [mol/l] Concentración de salida
% run CSTR_linear;
% x0_obs = [Vr; Ca; Tr];
% 
% % System start point
% Vr = V_mid;				% [l] Volumen del reactor
% Tr = Tr_min;             % [K] Temperatura de salida
% % Ca = CA_min;            % [mol/l] Concentración de salida
% run CSTR_linear;
% x0 = [Vr; Ca; Tr];
% 
% % Reduced-order unknown input observer
% run RUIO;
% N = 2;
% 
% % Unknown input output observer
% run CSTR_DLPV_UIOO;
% 
% % Save observers' data
% save polyObs.mat

%% Noise
sig = 1e-3*([1e-1 1e-3 1])';    % Ouput noise sigma

rng default;                        % Random seed start
v = sig*randn(1, Nsim);    % Measurement noise v~N(0, sig)

%% Error detection threshold
Tau = 10;    % period
mag_1 = 8.5e-2;     % Value Q1
mag_2 = 4e-3;     % Value Q2
mag_3 = 2.2e-5;     % Value O1
mag_4 = 4e-4;     % Value O2 2e-1

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
Kr = [-2; -3]; 
Ki = [-1.5; -1.5];

% Ingreso las opciones de la ODE 'RelTol', 1e-6, 'AbsTol', 1e-6
options = odeset ('RelTol', 1e-12, 'AbsTol', 1e-12, ...
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
FQ1 = zeros(1, Nsim);                  % Fault detect Q1

% RUIO 2
Phi_2 = zeros(N, Nsim+1);          % Observer states
X_UIO2 = zeros(nx, Nsim);           % Estimated states
Error_2 = zeros(1, Nsim);             % Error
Fact2 = zeros(1, Nsim);                % Estimated control Input
FQ2 = zeros(1, Nsim);                  % Fault detect Q2

% UIOO 1
Z1 = zeros(nx, Nsim+1);              % Observer states
Yest1 = zeros(p, Nsim);                % Monitorated outputs
X_UIOO1 = zeros(nx, Nsim);         % Estimated states
res1 = zeros(nx, Nsim);                % Residue
Error1 = zeros(1, Nsim);               % Error
Fsen1 = zeros(1, Nsim);               % Estimated sensor fault
FO1 = zeros(1, Nsim);                  % Fault detect S1

% UIOO 2
Z2 = zeros(nx, Nsim+1);              % Observer states
Yest2 = zeros(p, Nsim);                % Monitorated outputs
X_UIOO2 = zeros(nx, Nsim);         % Estimated states
res2 = zeros(nx, Nsim);                % Residue
Error2 = zeros(1, Nsim);               % Error
Fsen2 = zeros(1, Nsim);               % Estimated sensor fault
FO2 = zeros(1, Nsim);                  % Fault detect S2

% Initial states and inputs
X(:, 1) = x0;
Xsp(:, 1) = x0;
U(:, 1) = U_lin;
X_UIO1(:, 1) = x0_obs;
X_UIO2(:, 1) = x0_obs;
X_UIOO1(:, 1) = x0_obs;
X_UIOO2(:, 1) = x0_obs;

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
        
        if tk > 400 && tk < 500
            Ufails(:, k) = [0; -Fact_2+Fact_2*(exp(-(tk-400)/10))];
            Ufail(:, k) = U(:, k) + Ufails(:, k);
        end
        
        %% Process simulation with ODE
        [tsim, x] = ode45(@(x, u) CSTR(X(:, k), Ufail(:, k)) , [0 Ts], X(:, k), options);
        X(:, k+1) = x(end, :)';
        Y(:, k) = C*X(:, k)+v(:, k);
        
        %% Sensor fault income
        Yfail(:, k) = Y(:, k);

        if tk > 220 && tk < 300
          Yfail(:, k) = Y(:, k) + [-Fsen_1 + Fsen_1*(exp(-(tk-220)/10)); 0; 0];  
        end
        
        if tk >= 300 && tk < 320
            Yfail(:, k) = Y(:, k) + [- Fsen_1*(exp(-(tk-300))); 0; 0];
        end

        if tk > 580 && tk < 660
            Yfail(:, k) = Y(:, k) +[0; 0; -Fsen_2 + Fsen_2*(exp(-(tk-580)/15))];
        end
        
        if tk >= 660 && tk < 680
            Yfail(:, k) = Y(:, k) + [0; 0; - Fsen_2*(exp(-(tk-660)))];
        end
        
        %% Setpoint
        Xsp(1, k) = V_mid-2;
        Xsp(3, k) = Tr_mid;
    
        if tk < 160
            Xsp(3, k) = Tr_min;
        elseif tk >= 160 && tk < 200
            Xsp(3, k) = Tr_min+((Tr_mid-Tr_min)*(tk-160)/40);
        elseif tk >= 200 && tk < 340
            Xsp(3, k) = Tr_mid;   
        elseif tk >= 340 && tk < 380
            Xsp(1, k) = V_mid-2+((V_max-V_mid+2)*(tk-340)/40);
        elseif tk >= 380 && tk < 520
            Xsp(1, k) = V_max;
        elseif tk >= 520 && tk < 560
            Xsp(3, k) = Tr_mid+((Tr_max-Tr_mid)*(tk-520)/40);
            Xsp(1, k) = V_max;
        elseif tk >= 560 && tk < 740
            Xsp(3, k) = Tr_max;
            Xsp(1, k) = V_max;
        end
        
        %% Membership
        mu_out(:, k) = membership(Yfail(:, k), V_min, V_mid, V_max, Tr_min, Tr_mid, Tr_max);
        mu_in(:, k) = membership(Y(:, k), V_min, V_mid, V_max, Tr_min, Tr_mid, Tr_max);
      
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
            FQ1(k) = true;
        else
            FQ1(k) = false;
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
            FQ2(k) = true;
        else
            FQ2(k) = false;
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
        Error1(k) = sqrt(res1(2, k)^2);
        
        if Error1(k) > threshold(3, k)
            FO1(k) = true;
        else
            FO1(k) = false;
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
        Error2(k) = sqrt(res2(3, k)^2);
        
        if Error2(k) > threshold(4, k)
            FO2(k) = true;
        else
            FO2(k) = false;
        end

        %% Actuator fault estimation
        % Actuator fault 1
        if ~FQ1(k) && FQ2(k) && ~FO1(k) && FO2(k)
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
        if FQ1(k) && ~FQ2(k) && FO1(k) && FO2(k)
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
        if FQ1(k) && ~FQ2(k) && FO1(k) && ~FO2(k)
            Fsen1(k) = res2(1, k);
        else
            Fsen1(k) = zeros(size(res2(1, k)));
        end
        
        % Sensor fault 2
        if FQ1(k) && FQ2(k) && ~FO1(k) && FO2(k)
            Fsen2(k) = res1(3, k);
        else
            Fsen2(k) = zeros(size(res1(3, k)));
        end
        
        %% PID
        u0(1) = U(1, k);
        u0(2) = U(2, k);
        ek_1 = ek;
        ek(1) = (Xsp(1, k) - Yfail(1, k));
        ek(2) = (Xsp(3, k) - Yfail(3, k));
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

save runCSTR

run enPlotCSTR