%% CSTR
clc; clear; close all;
yalmip('clear');
 
% ODE options 'RelTol', 1e-6, 'AbsTol', 1e-6
options = odeset ('RelTol', 1e-12, 'AbsTol', 1e-12, ...
	'NormControl', 'on', 'InitialStep', 1.0e-4, 'MaxStep', 1.0);

%% Load polytope and observer matrices
% Type of observer(N° observer). Sub-observer(N° sub-observer). Matrix 
RUIO = struct;
UIOO = struct;
load polyObs

%% Simulation parameters
Time = 720.1;                           % Simulation end time
Ts = 0.05;                                 % Sample time [min]  (3 seg)
Nsim = Time/Ts;                       % Simulation steps
t = 0:Ts:Time-Ts;                      % Simulation time
Fail_Q1 = 5; Fail_Q2 = 5;         % Actuator fault magnitude [5% 5%]
Fail_S1 = 1.5; Fail_S2 = -4.5;  % Sensor fault magnitude [1.5% 0 1%]

%% Polytope model and observers
% This section is comment on purpose to reduce simulation time (using pre-calculated observer matrices)
% V_min = 90;             % Minimum volume (m^3)
% V_mid = 98;             % Middle volume (m^3)
% V_max = 110;          % Maximum volume (m^3)
% 
% Tr_min = 440;           % Mimimum temperature (°K)
% Tr_mid = 445;           % Middle temperature (°K)
% Tr_max = 450;          % Maximum temperature (°K)
% 
% N = 2;                           % Number of parameters
% L = 3;                            % Linearization points per parameter
% M = L^N;                      % Number of models
% run CSTR_polytope;      % M Models
% 
% % Observer start point
% Vr = V_mid;                 % [l] Reactor volume
% Tr = Tr_min;                 % [K] Output temperature
% run CSTR_linear;
% x0_obs = [Vr; Ca; Tr];
% 
% % System start point
% Vr = V_mid;				  % [l] Reactor volume
% Tr = Tr_min;                 % [K] Output temperature
% run CSTR_linear;
% x0 = [Vr; Ca; Tr];
% 
% % Reduced-order unknown input observer
% run CSTR_DLPV_RUIO;
% 
% % Unknown input output observer
% run CSTR_DLPV_UIOO;
% 
% % Save observers' data
% save polyObs.mat

%% Noise
sig = 1e-3*([1e-1 1e-4 5e-1])';    % Ouput noise sigma

rng default;                        % Random seed start
v = sig*randn(1, Nsim);    % Measurement noise v~N(0, sig)

%% Error detection threshold
Tau = 10;              % Convergence period
mag_1 = 8.5e-2;  % Value Q1
mag_2 = 4e-3;     % Value Q2
mag_3 = 2e-5;     % Value O1
mag_4 = 3.5e-6;     % Value O2

threshold = zeros(4, Nsim);

for k = 1:Nsim
    threshold(1, k) = mag_1 + 1000*exp(-(k-1)/Tau);  % Q1
    threshold(2, k) = mag_2 + 900*exp(-(k-1)/Tau);  % Q2
    threshold(3, k) = mag_3 + 100*exp(-(k-1)/Tau);  % O1
    threshold(4, k) = mag_4 + 1000*exp(-(k-1)/Tau);  % O2
end

%% Parámetros del PID
ek = [0; 0]; ek_1 = [0; 0];
Kr = [-2; -3]; 
Ki = [-1.5; -1.5];

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

for k = 1:Nsim
    tk = k*Ts; % Simulation time

    %% Actuator fault income
    Ufail(:, k) = U(:, k);      
    Ufails(:, k) = [0; 0];

    if tk > 30 && tk < 130
        Ufails(:, k) = [Fail_Q1-Fail_Q1*(exp(-(tk-30)/10)); 0];
        Ufail(:, k) = U(:, k) + Ufails(:, k);
    end

    if tk > 400 && tk < 500
        Ufails(:, k) = [0; -Fail_Q2+Fail_Q2*(exp(-(tk-400)/10))];
        Ufail(:, k) = U(:, k) + Ufails(:, k);
    end

    %% Process simulation with ODE
    [tsim, x] = ode45(@(x, u) CSTR(X(:, k), Ufail(:, k)) , [0 Ts], X(:, k), options);
    X(:, k+1) = x(end, :)';
    Y(:, k) = C*X(:, k)+v(:, k);

    %% Sensor fault income
    Yfail(:, k) = Y(:, k);

    if tk > 220 && tk < 300
      Yfail(:, k) = Y(:, k) + [-Fail_S1 + Fail_S1*(exp(-(tk-220)/10)); 0; 0];  
    end

    if tk >= 300 && tk < 320
        Yfail(:, k) = Y(:, k) + [- Fail_S1*(exp(-(tk-300))); 0; 0];
    end

    if tk > 580 && tk < 660
        Yfail(:, k) = Y(:, k) +[0; 0; -Fail_S2 + Fail_S2*(exp(-(tk-580)/15))];
    end

    if tk >= 660 && tk < 680
        Yfail(:, k) = Y(:, k) + [0; 0; - Fail_S2*(exp(-(tk-660)))];
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
    
	t_tic = tic ;   % to get time evaluated

    %% Membership
    mu_out(:, k) = membership(Yfail(:, k), V_min, V_mid, V_max, Tr_min, Tr_mid, Tr_max);
    mu_in(:, k) = membership(Y(:, k), V_min, V_mid, V_max, Tr_min, Tr_mid, Tr_max);

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
    UIOO(1).Y(:, k) = C*UIOO(1).X(:, k);

    % Residue 1
    UIOO(1).res(:, k) = Yfail(:, k) - UIOO(1).Y(:, k);

    % Error norm 1
    UIOO(1).error(k) = sqrt(UIOO(1).res(2, k)^2);

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
    UIOO(2).Y(:, k) = C*UIOO(2).X(:, k);

    % Residue 2
    UIOO(2).res(:, k) = Yfail(:, k) - UIOO(2).Y(:, k);

    % Error norm 2
    UIOO(2).error(k) = sqrt(UIOO(2).res(3, k)^2);

    if UIOO(2).error(k) > threshold(4, k)
        UIOO(2).FO(k) = true;
    else
        UIOO(2).FO(k) = false;
    end

    %% Actuator fault estimation
    % Actuator fault 1
    if ~RUIO(1).FQ(k) && RUIO(2).FQ(k) && ~UIOO(1).FO(k) && ~UIOO(2).FO(k)
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
    if RUIO(1).FQ(k) && ~RUIO(2).FQ(k) && UIOO(1).FO(k) && UIOO(2).FO(k)
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
    if RUIO(1).FQ(k) && ~RUIO(2).FQ(k) && UIOO(1).FO(k) && ~UIOO(2).FO(k)
        UIOO(1).Fsen(k) = UIOO(2).res(1, k);
    else
        UIOO(1).Fsen(k) = zeros(size(UIOO(2).res(1, k)));
    end

    % Sensor fault 2
    if RUIO(1).FQ(k) && RUIO(2).FQ(k) && ~UIOO(1).FO(k) && UIOO(2).FO(k)
        UIOO(2).Fsen(k) = UIOO(1).res(3, k);
    else
        UIOO(2).Fsen(k) = zeros(size(UIOO(1).res(3, k)));
    end
    
	t_tic = toc(t_tic) ;              % get time elapsed
    elapsed_time(k) = t_tic ;   % store the time elapsed for the run
    
    %% PID
    u0(1) = U(1, k);
    u0(2) = U(2, k);
    ek_1 = ek;
    ek(1) = (Xsp(1, k) - Yfail(1, k));
    ek(2) = (Xsp(3, k) - Yfail(3, k));
    % Direct control
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
save runCSTR

%% Plots
run enPlotCSTR