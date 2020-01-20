%% CSTR Model
syms qs qc V CA T

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
nx = length(states); nu = length(inputs); ny = length(outputs);
C = eye(ny, nx);        % Matriz de salida
D = zeros(ny, nu);     % Matriz entrada/salida

%% Linealización
% Matrices simbólicas
A_sym = jacobian(system, states);
B_sym = jacobian(system, inputs);

qc0 = 103;              % [l/min] Caudal refrigerante (inicio de iteración fsolve)

% Obtengo la temperatura dentro del reactor
% Tr = -E_R/log(-(q*(Ca-CAe))/(k0*Ca*Vr));

% Obtengo la concentración de salida
Ca = CAe/(1+(k0*(Vr/q)*exp(-E_R/Tr)));

% Estados para la linealización
X_lin = [Vr; Ca; Tr];

% Obtengo el caudal de salida
Qs = k4*sqrt(Vr);

% Obtengo el caudal refrigerante
qc_fun = @(qc) (q/Vr)*(Te-Tr) - k1*Ca*exp(-E_R/Tr) + k2*(qc/Vr)*(1-exp(-k3/qc))*(Tce-Tr);
Qc = fsolve(qc_fun, qc0, optimoptions('fsolve', 'Display', 'off'));

% Manipuladas para la linealización
U_lin = [Qs; Qc];

% Matrices del sistema lineal
A = subs(A_sym, {CA, T, qs, qc, V}, {Ca, Tr, Qs, Qc, Vr});
B = subs(B_sym, {CA, T, qs, qc, V}, {Ca, Tr, Qs, Qc, Vr});
A = double(A);
B = double(B);

f = subs(system, {CA, T, qs, qc, V}, {Ca, Tr, Qs, Qc, Vr});
f = double(f);

% Desviación del modelo lineal
delta = f - (A*X_lin+B*U_lin);

% Euler discretization method
Ad = (A*Ts) + eye(nx); Bd = B*Ts; deltad = delta*Ts;