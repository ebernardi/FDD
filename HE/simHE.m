%% HE
clc; clear; yalmip('clear');
close all;

% ODE options 'RelTol', 1e-6, 'AbsTol', 1e-6
options = odeset ('RelTol', 1e-6, 'AbsTol', 1e-6, ...
	'NormControl', 'on', 'InitialStep', 1.0e-4, 'MaxStep', 1.0);

%% Load polytope and observer matrices
% Type of observer(N° observer). Sub-observer(N° sub-observer). Matrix 
RUIO = struct;
UIOO = struct;
load polyObs

%% Simulation parameters
Time = 720.1;                          % Simulation end time [min] (12 h)
Ts = 0.05;                                % Sample time [min] (3 seg)
Nsim = Time/Ts;                      % Simulation steps
t = 0:Ts:Time-Ts;                     % Simulation time
Fail_Q1 = 5; Fail_Q2 = 0.43;    % Actuator fault magnitude [5%, 5%]
Fail_S1 = 2.5; Fail_S2 = -3.5;	% Sensor fault magnitude [0.5% 0.5%]

%% Polytope model and observers
% This section is commented to reduce simulation time (using pre-calculated observer matrices)
% Theta_1s_min = 495;              % Minimum output fluid 1 temperature (K)
% Theta_1s_mid = 497.32;         % Middle output fluid 1 temperature (K)
% Theta_1s_max = 500;             % Maximum output fluid 1 temperature (K)
% 
% Theta_2s_min = 680;              % Minimum output fluid 2 temperature (K)
% Theta_2s_mid = 695.915;       % Middle output fluid 2 temperature (K)
% Theta_2s_max = 710;             % Maximum output fluid 2 temperature (K)
% 
% N = 2;                                      % Number of parameters
% L = 3;                                       % Linearization points per parameter
% M = L^N;                                 % Number of models
% run HE_polytope;                     % M models
% 
% % Observers start point
% Theta_1s = Theta_1s_mid;     % Output fluid 1 temperature (K)
% Theta_2s = Theta_2s_min;     % Output fluid 2 temperature (K)
% run HE_linear;
% x0_obs = [Theta_1s; Theta_2s; Theta_p];
% 
% % System start point
% Theta_1s = Theta_1s_min;     % Output fluid 1 temperature (K)
% Theta_2s = Theta_2s_mid;     % Output fluid 2 temperature (K)
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
sig = 3e-3*([1 1 1])';         % Ouput noise sigma

rng default;                        % Random seed start
v = sig*randn(1, Nsim);    % Measurement noise v~N(0, sig)

%% Error detection threshold
Tau = 10;                % Convergence period
mag_1 = 1.3e-1;     % Value Q1
mag_2 = 5e-2;        % Value Q2
mag_3 = 6e-2;        % Value O1
mag_4 = 2e-1;     % Value O2

threshold = zeros(4, Nsim);

for k = 1:Nsim
    threshold(1, k) = mag_1 + 1000*exp(-(k-1)/Tau);  % Q1
    threshold(2, k) = mag_2 + 900*exp(-(k-1)/Tau);    % Q2
    threshold(3, k) = mag_3 + 100*exp(-(k-1)/Tau);    % O1
    threshold(4, k) = mag_4 + 1000*exp(-(k-1)/Tau);  % O2
end

%% Parámetros del PID
ek = [0; 0]; ek_1 = [0; 0];
Kr = [-1; 5e-2]; 
Ki = [-2; 2e-1];

%% Simulation Setup
U = zeros(nu, Nsim);                   % Control Input
Ufail = zeros(nu, Nsim);               % Fault control Input
Ufails = zeros(nu, Nsim);             % Fails of control inputs
X = zeros(nx, Nsim+1);               % States
Y = zeros(ny, Nsim);                    % Measure outputs
Yfail = zeros(ny, Nsim);                % Faulty measure outputs
mu_out = zeros(M, Nsim);           % Membership
mu_in = zeros(M, Nsim);              % Membership
Xsp = zeros(nx, Nsim);                 % Set-point

% RUIOs
for j = 1:N
    RUIO(j).Phi = zeros(N, Nsim+1);     % Observer states
    RUIO(j).X = zeros(nx, Nsim);           % Estimated states
    RUIO(j).error = zeros(1, Nsim);       % Error
    RUIO(j).Fact = zeros(1, Nsim);        % Estimated control Input
    RUIO(j).FQ = zeros(1, Nsim);           % Fault detect Q
    RUIO(j).delay = 0;                            % Detection delay
end

% UIOOs
for j = 1:N
    UIOO(j).Z = zeros(nx, Nsim+1);      % Observer states
    UIOO(j).Ymon = zeros(N, Nsim);     % Monitorated outputs
    UIOO(j).X = zeros(nx, Nsim);           % Estimated states
    UIOO(j).Y = zeros(nx, Nsim);           % Estimated outputs
    UIOO(j).res = zeros(nx, Nsim);        % Residue
    UIOO(j).error = zeros(1, Nsim);       % Error
    UIOO(j).Fsen = zeros(1, Nsim);       % Estimated sensor fault
    UIOO(j).FO = zeros(1, Nsim);          % Fault detect S
end

% Initial states and inputs
X(:, 1) = x0;
Xsp(:, 1) = x0;
U(:, 1) = U_lin;
RUIO(1).X(:, 1) = x0_obs;
RUIO(2).X(:, 1) = x0_obs;
UIOO(1).X(:, 1) = x0_obs;
UIOO(2).X(:, 1) = x0_obs;

elapsed_time = zeros(Nsim, 1) ;  % initialize the elapsed times 

%% Simulation
for k = 1:Nsim
    tk = k*Ts; % Simulation time

    %% Actuator fault income
    Ufail(:, k) = U(:, k);      
    Ufails(:, k) = [0; 0];

    if tk > 30 && tk < 130
        Ufails(:, k) = [Fail_Q1; 0];
        Ufail(:, k) = U(:, k) + Ufails(:, k);
    end

    if tk > 580 && tk < 680
        Ufails(:, k) = [0; -Fail_Q2+Fail_Q2*(exp(-(tk-580)/10))];
        Ufail(:, k) = U(:, k) + Ufails(:, k);
    end

    %% Process simulation with ODE
    [tsim, x] = ode45(@(x, u) HE(X(:, k), Ufail(:, k)) , [0 Ts], X(:, k), options);
    X(:, k+1) = x(end, :)';
    Y(:, k) = C*X(:, k)+v(:, k);

    %% Sensor fault income
    Yfail(:, k) = Y(:, k);

    if tk > 220 && tk < 320
        Yfail(:, k) = Y(:, k) + [0; Fail_S2-Fail_S2*(exp(-(tk-220)/5)); 0];
    end

    if tk >400 && tk < 500
        Yfail(:, k) = Y(:, k) + [Fail_S1-Fail_S1*(exp(-(tk-400)/6)); 0; 0];
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

	t_tic = tic ;   % to get time evaluated
    
    %% membership
    mu_out(:, k) = membership(Yfail(:, k), Theta_1s_min, Theta_1s_mid, Theta_1s_max, Theta_2s_min, Theta_2s_mid, Theta_2s_max);
    mu_in(:, k) = membership(Yfail(:, k), Theta_1s_min, Theta_1s_mid, Theta_1s_max, Theta_2s_min, Theta_2s_mid, Theta_2s_max);

    %% LPV-RUIO 1
    RUIO(1).Phi(:, k+1) = zeros(N, 1);
    RUIO(1).X(:, k) = zeros(nx, 1);
    RUIO(1).Fact(k) = zeros(1, 1);
    for i = 1:M
        RUIO(1).Phi(:, k+1) = RUIO(1).Phi(:, k+1) + mu_out(i, k)*(RUIO(1).O(i).K*RUIO(1).Phi(:, k) + RUIO(1).O(i).L_ast*Yfail(:, k) + RUIO(1).O(i).B_bar_1*U(:, k) + RUIO(1).O(i).delta_bar_1);
        RUIO(1).X(:, k) = RUIO(1).X(:, k) + mu_out(i, k)*(RUIO(1).O(i).T*[RUIO(1).Phi(:, k); RUIO(1).O(i).U_1*Yfail(:, k)-RUIO(1).O(i).U_1*RUIO(1).O(i).C_tilde_1*RUIO(1).Phi(:, k)]);

        RUIO(1).Fact(k) = RUIO(1).Fact(k) + mu_out(i, k)*(RUIO(1).O(i).U_1*(X(:, k+1) - RUIO(1).O(i).C_tilde_1*RUIO(1).Phi(:, k+1)) + RUIO(1).O(i).A_bar_22*RUIO(1).O(i).U_1*(RUIO(1).O(i).C_tilde_1*RUIO(1).Phi(:, k) - Yfail(:, k)) - RUIO(1).O(i).A_bar_21*RUIO(1).Phi(:, k) - RUIO(1).O(i).B_bar_2*U(:, k) -  RUIO(1).O(i).delta_bar_2);
    end

    % Error norm 1
    RUIO(1).error(k) = sqrt((RUIO(1).X(1, k)-Yfail(1, k))^2 + (RUIO(1).X(2, k)-Yfail(2, k))^2 + (RUIO(1).X(3, k)-Yfail(3, k))^2);

    if RUIO(1).error(k) > threshold(1, k)
        RUIO(1).FQ(k) = true;
    else
        RUIO(1).FQ(k) = false;
    end

    %% LPV-RUIO 2
    RUIO(2).Phi(:, k+1) = zeros(N, 1);
    RUIO(2).X(:, k) = zeros(nx, 1);
    RUIO(2).Fact(k) = zeros(1, 1);
    for i = 1:M
        RUIO(2).Phi(:, k+1) = RUIO(2).Phi(:, k+1) + mu_out(i, k)*(RUIO(2).O(i).K*RUIO(2).Phi(:, k) + RUIO(2).O(i).L_ast*Yfail(:, k) + RUIO(2).O(i).B_bar_1*U(:, k) + RUIO(2).O(i).delta_bar_1);
        RUIO(2).X(:, k) = RUIO(2).X(:, k) + mu_out(i, k)*(RUIO(2).O(i).T*[RUIO(2).Phi(:, k); RUIO(2).O(i).U_1*Yfail(:, k)-RUIO(2).O(i).U_1*RUIO(2).O(i).C_tilde_1*RUIO(2).Phi(:, k)]);

        RUIO(2).Fact(k) = RUIO(2).Fact(k) + mu_out(i, k)*(RUIO(2).O(i).U_1*(X(:, k+1) - RUIO(2).O(i).C_tilde_1*RUIO(2).Phi(:, k+1)) + RUIO(2).O(i).A_bar_22*RUIO(2).O(i).U_1*(RUIO(2).O(i).C_tilde_1*RUIO(2).Phi(:, k) - Yfail(:, k)) - RUIO(2).O(i).A_bar_21*RUIO(2).Phi(:, k) - RUIO(2).O(i).B_bar_2*U(:, k) -  RUIO(2).O(i).delta_bar_2);
    end

    % Error norm 2
    RUIO(2).error(k) = sqrt((RUIO(2).X(1, k)-Yfail(1, k))^2 + (RUIO(2).X(2, k)-Yfail(2, k))^2 + (RUIO(2).X(3, k)-Yfail(3, k))^2);

    if RUIO(2).error(k) > threshold(2, k)
        RUIO(2).FQ(k) = true;
    else
        RUIO(2).FQ(k) = false;
    end

    %% LPV-UIOO 1
    UIOO(1).Ymon(:, k) = UIOO(1).T2*Yfail(:, k);
    UIOO(1).Z(:, k+1) = zeros(nx, 1);      
    for i = 1:M
        UIOO(1).Z(:, k+1) = UIOO(1).Z(:, k+1) + mu_in(i, k)*(UIOO(1).O(i).N*UIOO(1).Z(:, k) + UIOO(1).O(i).L*UIOO(1).Ymon(:, k) + UIOO(1).O(i).G*U(:, k) + UIOO(1).O(i).Tg);
    end

    UIOO(1).X(:, k) = UIOO(1).Z(:, k) - UIOO(1).E*UIOO(1).Ymon(:, k);
    UIOO(1).Y(:, k) = Cd*UIOO(1).X(:, k);

    % Residue 1
    UIOO(1).res(:, k) = Yfail(:, k) - UIOO(1).Y(:, k);

    % Error norm 1
    UIOO(1).error(k) = sqrt(UIOO(1).res(1, k)^2);

    if UIOO(1).error(k) > threshold(3, k)
        UIOO(1).FO(k) = true;
    else
        UIOO(1).FO(k) = false;
    end        

    %% LPV-UIOO 2
    UIOO(2).Ymon(:, k) = UIOO(2).T2*Yfail(:, k);
    UIOO(2).Z(:, k+1) = zeros(nx, 1);      
    for i = 1:M
        UIOO(2).Z(:, k+1) = UIOO(2).Z(:, k+1) + mu_in(i, k)*(UIOO(2).O(i).N*UIOO(2).Z(:, k) + UIOO(2).O(i).L*UIOO(2).Ymon(:, k) + UIOO(2).O(i).G*U(:, k) + UIOO(2).O(i).Tg);
    end

    UIOO(2).X(:, k) = UIOO(2).Z(:, k) - UIOO(2).E*UIOO(2).Ymon(:, k);
    UIOO(2).Y(:, k) = Cd*UIOO(2).X(:, k);

    % Residue 2
    UIOO(2).res(:, k) = Yfail(:, k) - UIOO(2).Y(:, k);

    % Error norm 2
    UIOO(2).error(k) = sqrt(UIOO(2).res(2, k)^2);

    if UIOO(2).error(k) > threshold(4, k)
        UIOO(2).FO(k) = true;
    else
        UIOO(2).FO(k) = false;
    end

    %% Actuator fault estimation
    % Actuator fault 1
    if ~RUIO(1).FQ(k) && RUIO(2).FQ(k) && UIOO(1).FO(k) && ~UIOO(2).FO(k)
        if RUIO(1).delay
            RUIO(1).Fact(k) = RUIO(1).Fact(k);
        else
            RUIO(1).delay = 1;
            RUIO(1).Fact(k) = 0;
        end
    else
        RUIO(1).delay = 0;
        RUIO(1).Fact(k) = 0;
    end

    % Actuator fault 2
    if RUIO(1).FQ(k) && ~RUIO(2).FQ(k) && ~UIOO(1).FO(k) && UIOO(2).FO(k)
        if RUIO(2).delay
            RUIO(2).Fact(k) = RUIO(2).Fact(k);
        else
            RUIO(2).delay = 1;
            RUIO(2).Fact(k) = 0;
        end
    else
        RUIO(2).delay = 0;
        RUIO(2).Fact(k) = 0;
    end

    %% Sensor fault estimation
    % Sensor fault 1
    if RUIO(1).FQ(k) && RUIO(2).FQ(k) && UIOO(1).FO(k) && ~UIOO(2).FO(k)
        UIOO(1).Fsen(k) = UIOO(2).res(1, k);
    else
        UIOO(1).Fsen(k) = zeros(size(UIOO(2).res(1, k)));
    end

    % Sensor fault 2
    if RUIO(1).FQ(k) && RUIO(2).FQ(k) && ~UIOO(1).FO(k) && UIOO(2).FO(k)
        UIOO(2).Fsen(k) = UIOO(1).res(2, k);
    else
        UIOO(2).Fsen(k) = zeros(size(UIOO(1).res(2, k)));
    end
    
	t_tic = toc(t_tic) ;              % get time elapsed
    elapsed_time(k) = t_tic ;   % store the time elapsed for the run

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

%% Get the overhead time 
time_avg = mean(elapsed_time) ;
msg = ['Mean time = ', num2str(time_avg)];
disp(msg)
msg = ['Mean time overhead = ', num2str(time_avg/(Ts*60)*100), '% @ Ts'];
disp(msg)
time_max = max(elapsed_time) ;
msg = ['Max time = ', num2str(time_max)];
disp(msg)
msg = ['Max time overhead = ', num2str(time_max/(Ts*60)*100), '% @ Ts'];
disp(msg)
time_min = min(elapsed_time) ;
msg = ['Min time = ', num2str(time_min)];
disp(msg)
msg = ['Min time overhead = ', num2str(time_min/(Ts*60)*100), '% @ Ts'];
disp(msg)

%% Save running data
save runHE

%% Plots
run enPlotHE