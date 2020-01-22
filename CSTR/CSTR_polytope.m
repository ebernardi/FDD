%% CSTR Model
syms qs qc V CA T
sys = struct;

% Parameters
E_R = 1e4;                  % Activation energy [°K]
Te = 350;                    % Feed temperature [°K]
Tce = 350;                  % Inlet coolant temp. [°K]
dH = -2e5;                  % Heat of reaction [cal/mol]
Cp = 1;                        % Specific heats [cal/g °K]
rho = 1e3;                   % Liquid densities [g/l]
CAe = 1;                      % Feed concentration [mol/l]
ha = 7e5;                     % Heat transfer term [cal/min °K]
k0 = 7.2e10;                % Reaction rate constant [l/min]
k1 = dH*k0/(rho*Cp);
k2 = rho*Cp/(rho*Cp);
k3 = ha/(rho*Cp);
k4 = 10;                       % Valve constant [l/min m^3/2]
qe = 100;                     % Feed flow rate [l/min]

%% Non-Linear model
system = [(qe - qs);
          ((qe/V)*(CAe - CA) - k0*CA*exp(-E_R/T));
		  ((qe/V)*(Te - T) - k1*CA*exp(-E_R/T) + k2*(qc/V)*(1 - exp(-k3/qc))*(Tce - T))];

states = [V CA T];
outputs = [V CA T];
inputs = [qs qc];
nx = length(states); nu = length(inputs);  ny = length(outputs);
C = eye(ny, nx);       % Output matrix
D = zeros(ny, nu);    % Input/Output matrix

%% Linealization
% Symbolic matrices
A_sym = jacobian(system, states);
B_sym = jacobian(system, inputs);

for i = 1:M
    switch i
        case 1
            % System 1 (V = V_min [l]; T = T_min [K])
            Vr = V_min;				% [l] Reactor volume
            Tr = Tr_min;             % [K] Output temperature
        case 2
            % System 2 (V = V_min [l]; T = T_mid [K])
            Vr = V_min;				% [l] Reactor volume
            Tr = Tr_mid;             % [K] Output temperature
        case 3
            % System 3 (V = V_min [l]; T = T_max [K])
            Vr = V_min;				% [l] Reactor volume
            Tr = Tr_max;            % [K] Output temperature
        case 4
            % System 4 (V = V_mid [l]; T = T_min [K])
            Vr = V_mid;				% [l] Reactor volume
            Tr = Tr_min;             % [K] Output temperature
        case 5
            % System 5 (V = V_mid [l]; T = T_mid [K])
            Vr = V_mid;				% [l] Reactor volume
            Tr = Tr_mid;             % [K] Output temperature
        case 6
            % System 6 (V = V_mid [l]; T = T_max [K])
            Vr = V_mid;				% [l] Reactor volume
            Tr = Tr_max;            % [K] Output temperature
        case 7
            % System 7 (V = V_max [l]; T = T_min [K])
            Vr = V_max;			   % [l] Reactor volume
            Tr = Tr_min;             % [K] Output temperature
        case 8
            % System 8 (V = V_max [l]; T = T_mid [K])
            Vr = V_max;			   % [l] Reactor volume
            Tr = Tr_mid;             % [K] Output temperature
        case 9
            % System 9 (V = V_max [l]; T = T_max [K])
            Vr = V_max;				% [l] Reactor volume
            Tr = Tr_max;             % [K] Output temperature
        otherwise
    end
    
    qc0 = 103;              % [l/min] Coolant flow rate (Iteration start fsolve)

    % Reactor temperature
    % Tr = -E_R/log(-(q*(Ca-CAe))/(k0*Ca*Vr));

    % Output concentration
    Ca = CAe/(1+(k0*(Vr/qe)*exp(-E_R/Tr)));

    % States
    Xinit = [Vr; Ca; Tr];

    % Output flow rate
    Qs = k4*sqrt(Vr);

    % Coolant flow rate
    qc_fun = @(qc) (qe/Vr)*(Te-Tr) - k1*Ca*exp(-E_R/Tr) + k2*(qc/Vr)*(1-exp(-k3/qc))*(Tce-Tr);
    Qc = fsolve(qc_fun, qc0, optimoptions('fsolve', 'Display', 'off'));

    % Inputs
    Uinit = [Qs; Qc];

    % Linear systems matrices
    A = subs(A_sym, {CA, T, qs, qc, V}, {Ca, Tr, Qs, Qc, Vr});
    B = subs(B_sym, {CA, T, qs, qc, V}, {Ca, Tr, Qs, Qc, Vr});
    A = double(A);
    B = double(B);

    f = subs(system, {CA, T, qs, qc, V}, {Ca, Tr, Qs, Qc, Vr});
    f = double(f);

    % Constant term
    delta = f - (A*Xinit+B*Uinit);

    % Euler discretization method
    sys(i).Ad = (A*Ts) + eye(nx); sys(i).Bd = B*Ts; sys(i).deltad = delta*Ts;
    sys(i).C = C; sys(i).D = D;
end