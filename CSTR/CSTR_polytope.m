%% CSTR Model
syms qs qc V CA T
sys = struct;

% Parámetros
E_R = 1e4;              % [°K] (Energia de activación)
Te = 350;               % [°K] (Temperatura de entrada del reactante)
Tce = 350;              % [°K] (Temperatura del liquido refrigerante)
dH = -2e5;              % [cal/mol] (Calor de reacción)
Cp = 1;                 % [cal/g °K] (Calores específicos)
rho = 1e3;              % [g/l] (Densidad de los líquidos)
CAe = 1;                % [mol/l] (Concentración de A a la entrada)
ha = 7e5;               % [cal/min °K] (Coeficiente de transferencia de calor)
k0 = 7.2e10;            % [l/min] (Constante de velocidad de reacción)
k1 = dH*k0/(rho*Cp);
k2 = rho*Cp/(rho*Cp);
k3 = ha/(rho*Cp);
k4 = 10;                % [l/min m^3/2] (Constante de la válvula)
q = 100;                % [l/min] (Caudal de Entrada)
%qs = k4*sqrt(V);        % [l/min] (Caudal de salida)

%% Modelo no lineal
system = [(q - qs);
          ((q/V)*(CAe - CA) - k0*CA*exp(-E_R/T));
		  ((q/V)*(Te - T) - k1*CA*exp(-E_R/T) + k2*(qc/V)*(1 - exp(-k3/qc))*(Tce - T))];

states = [V CA T];
outputs = [V CA T];
inputs = [qs qc];
nx = length(states); nu = length(inputs);  ny = length(outputs);
C = eye(nx);			% Matriz de salida

%% Linealización
% Matrices simbólicas
A_sym = jacobian(system, states);
B_sym = jacobian(system, inputs);

for i = 1:M
    switch i
        case 1
            % System 1 (V = V_min [l]; T = T_min [K])% CA = CA_min [mol/l])
            Vr = V_min;				% [l] Volumen del reactor
            Tr = Tr_min;             % [K] Temperatura de salida
            % Ca = CA_min;            % [mol/l] Concentración de salida
        case 2
            % System 2 (V = V_min [l]; T = T_mid [K])% CA = CA_mid [mol/l])
            Vr = V_min;				% [l] Volumen del reactor
            Tr = Tr_mid;             % [K] Temperatura de salida
        case 3
            % System 3 (V = V_min [l]; T = T_max [K])% CA = CA_max [mol/l])
            Vr = V_min;				% [l] Volumen del reactor
            Tr = Tr_max;             % [K] Temperatura de salida
        case 4
            % System 4 (V = V_mid [l]; T = T_min [K])% CA = CA_min [mol/l])
            Vr = V_mid;				% [l] Volumen del reactor
            Tr = Tr_min;             % [K] Temperatura de salida
        case 5
            % System 5 (V = V_mid [l]; T = T_mid [K])% CA = CA_mid [mol/l])
            Vr = V_mid;				% [l] Volumen del reactor
            Tr = Tr_mid;             % [K] Temperatura de salida
        case 6
            % System 6 (V = V_mid [l]; T = T_max [K])% CA = CA_max [mol/l])
            Vr = V_mid;				% [l] Volumen del reactor
            Tr = Tr_max;             % [K] Temperatura de salida
        case 7
            % System 7 (V = V_max [l]; T = T_min [K])% CA = CA_min [mol/l])
            Vr = V_max;				% [l] Volumen del reactor
            Tr = Tr_min;             % [K] Temperatura de salida
        case 8
            % System 8 (V = V_max [l]; T = T_mid [K])% CA = CA_mid [mol/l])
            Vr = V_max;				% [l] Volumen del reactor
            Tr = Tr_mid;             % [K] Temperatura de salida
        case 9
            % System 9 (V = V_max [l]; T = T_max [K])% CA = CA_max [mol/l])
            Vr = V_max;				% [l] Volumen del reactor
            Tr = Tr_max;             % [K] Temperatura de salida
        otherwise
    end
    
    qc0 = 103;              % [l/min] Caudal refrigerante (inicio de iteración fsolve)

    % Obtengo la temperatura dentro del reactor
    % Tr = -E_R/log(-(q*(Ca-CAe))/(k0*Ca*Vr));

    % Obtengo la concentración de salida
    Ca = CAe/(1+(k0*(Vr/q)*exp(-E_R/Tr)));

    % Estados para la linealización
    Xinit = [Vr; Ca; Tr];

    % Obtengo el caudal de salida
    Qs = k4*sqrt(Vr);

    % Obtengo el caudal refrigerante
    qc_fun = @(qc) (q/Vr)*(Te-Tr) - k1*Ca*exp(-E_R/Tr) + k2*(qc/Vr)*(1-exp(-k3/qc))*(Tce-Tr);
    Qc = fsolve(qc_fun, qc0, optimoptions('fsolve', 'Display', 'off'));

    % Manipuladas para la linealización
    Uinit = [Qs; Qc];

    % Matrices del sistema lineal
    A = subs(A_sym, {CA, T, qs, qc, V}, {Ca, Tr, Qs, Qc, Vr});
    B = subs(B_sym, {CA, T, qs, qc, V}, {Ca, Tr, Qs, Qc, Vr});
    A = double(A);
    B = double(B);

    f = subs(system, {CA, T, qs, qc, V}, {Ca, Tr, Qs, Qc, Vr});
    f = double(f);

    % Desviación del modelo lineal
    delta = f - (A*Xinit+B*Uinit);

    % Euler discretization method
    sys(i).Ad = (A*Ts) + eye(nx); sys(i).Bd = B*Ts; sys(i).deltad = delta*Ts;
    sys(i).C = eye(ny, nx);
end