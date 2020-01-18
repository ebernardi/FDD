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
nx = length(states); nu = length(inputs);  ny = length(outputs);
C = eye(nx);			% Matriz de salida

%% Linealización
% Matrices simbólicas
A_sym = jacobian(system, states);
B_sym = jacobian(system, inputs);

%% Sistema 1 (V = V_min [l]; T = T_min [K])% CA = CA_min [mol/l])
Vr = V_min;				% [l] Volumen del reactor
Tr = Tr_min;             % [K] Temperatura de salida
% Ca = CA_min;            % [mol/l] Concentración de salida
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
A1 = double(A);
B1 = double(B);

f = subs(system, {CA, T, qs, qc, V}, {Ca, Tr, Qs, Qc, Vr});
f1 = double(f);

% Desviación del modelo lineal
delta1 = f1 - (A1*Xinit+B1*Uinit);

% Euler discretization method
sys(1).Ad = (A1*Ts) + eye(nx); sys(1).Bd = B1*Ts; sys(1).deltad = delta1*Ts;
sys(1).C = eye(ny, nx);

%% Sistema 2 (V = V_min [l]; T = T_mid [K])% CA = CA_mid [mol/l])
Vr = V_min;				% [l] Volumen del reactor
Tr = Tr_mid;             % [K] Temperatura de salida
% Ca = CA_mid;            % [mol/l] Concentración de salida
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
A2 = double(A);
B2 = double(B);

f = subs(system, {CA, T, qs, qc, V}, {Ca, Tr, Qs, Qc, Vr});
f2 = double(f);

% Desviación del modelo lineal
delta2 = f2 - (A2*Xinit+B2*Uinit);

% Euler discretization method
sys(2).Ad = (A2*Ts) + eye(nx); sys(2).Bd = B2*Ts; sys(2).deltad = delta2*Ts;
sys(2).C = eye(ny, nx);

%% Sistema 3 (V = V_min [l]; T = T_max [K])% CA = CA_max [mol/l])
Vr = V_min;				% [l] Volumen del reactor
Tr = Tr_max;             % [K] Temperatura de salida
% Ca = CA_max;            % [mol/l] Concentración de salida
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
A3 = double(A);
B3 = double(B);

f = subs(system, {CA, T, qs, qc, V}, {Ca, Tr, Qs, Qc, Vr});
f3 = double(f);

% Desviación del modelo lineal
delta3 = f3 - (A3*Xinit+B3*Uinit);

% Euler discretization method
sys(3).Ad = (A3*Ts) + eye(nx); sys(3).Bd = B3*Ts; sys(3).deltad = delta3*Ts;
sys(3).C = eye(ny, nx);

%% Sistema 4 (V = V_mid [l]; T = T_min [K])% CA = CA_min [mol/l])
Vr = V_mid;				% [l] Volumen del reactor
Tr = Tr_min;             % [K] Temperatura de salida
% Ca = CA_min;            % [mol/l] Concentración de salida
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
A4 = double(A);
B4 = double(B);

f = subs(system, {CA, T, qs, qc, V}, {Ca, Tr, Qs, Qc, Vr});
f4 = double(f);

% Desviación del modelo lineal
delta4 = f4 - (A4*Xinit+B4*Uinit);

% Euler discretization method
sys(4).Ad = (A4*Ts) + eye(nx); sys(4).Bd = B4*Ts; sys(4).deltad = delta4*Ts;
sys(4).C = eye(ny, nx);

%% Sistema 5 (V = V_mid [l]; T = T_mid [K])% CA = CA_mid [mol/l])
Vr = V_mid;				% [l] Volumen del reactor
Tr = Tr_mid;             % [K] Temperatura de salida
% Ca = CA_mid;            % [mol/l] Concentración de salida
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
A5 = double(A);
B5 = double(B);

f = subs(system, {CA, T, qs, qc, V}, {Ca, Tr, Qs, Qc, Vr});
f5 = double(f);

% Desviación del modelo lineal
delta5 = f5 - (A5*Xinit+B5*Uinit);

% Euler discretization method
sys(5).Ad = (A5*Ts) + eye(nx); sys(5).Bd = B5*Ts; sys(5).deltad = delta5*Ts;
sys(5).C = eye(ny, nx);

%% Sistema 6 (V = V_mid [l]; T = T_max [K])% CA = CA_max [mol/l])
Vr = V_mid;				% [l] Volumen del reactor
Tr = Tr_max;             % [K] Temperatura de salida
% Ca = CA_max;            % [mol/l] Concentración de salida
qc0 = 110;              % [l/min] Caudal refrigerante (inicio de iteración fsolve)

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
A6 = double(A);
B6 = double(B);

f = subs(system, {CA, T, qs, qc, V}, {Ca, Tr, Qs, Qc, Vr});
f6 = double(f);

% Desviación del modelo lineal
delta6 = f6 - (A6*Xinit+B6*Uinit);

% Euler discretization method
sys(6).Ad = (A6*Ts) + eye(nx); sys(6).Bd = B6*Ts; sys(6).deltad = delta6*Ts;
sys(6).C = eye(ny, nx);

%% Sistema 7 (V = V_max [l]; T = T_min [K])% CA = CA_min [mol/l])
Vr = V_max;				% [l] Volumen del reactor
Tr = Tr_min;             % [K] Temperatura de salida
% Ca = CA_min;            % [mol/l] Concentración de salida
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
A7 = double(A);
B7 = double(B);

f = subs(system, {CA, T, qs, qc, V}, {Ca, Tr, Qs, Qc, Vr});
f7 = double(f);

% Desviación del modelo lineal
delta7 = f7 - (A7*Xinit+B7*Uinit);

% Euler discretization method
sys(7).Ad = (A7*Ts) + eye(nx); sys(7).Bd = B7*Ts; sys(7).deltad = delta7*Ts;
sys(7).C = eye(ny, nx);

%% Sistema 8 (V = V_max [l]; T = T_mid [K])% CA = CA_mid [mol/l])
Vr = V_max;				% [l] Volumen del reactor
Tr = Tr_mid;             % [K] Temperatura de salida
% Ca = CA_mid;            % [mol/l] Concentración de salida
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
A8 = double(A);
B8 = double(B);

f = subs(system, {CA, T, qs, qc, V}, {Ca, Tr, Qs, Qc, Vr});
f8 = double(f);

% Desviación del modelo lineal
delta8 = f8 - (A8*Xinit+B8*Uinit);

% Euler discretization method
sys(8).Ad = (A8*Ts) + eye(nx); sys(8).Bd = B8*Ts; sys(8).deltad = delta8*Ts;
sys(8).C = eye(ny, nx);

%% Sistema 9 (V = V_max [l]; T = T_max [K])% CA = CA_max [mol/l])
Vr = V_max;				% [l] Volumen del reactor
Tr = Tr_max;             % [K] Temperatura de salida
% Ca = CA_max;            % [mol/l] Concentración de salida
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
A9 = double(A);
B9 = double(B);

f = subs(system, {CA, T, qs, qc, V}, {Ca, Tr, Qs, Qc, Vr});
f9 = double(f);

% Desviación del modelo lineal
delta9 = f9 - (A9*Xinit+B9*Uinit);

% Euler discretization method
sys(9).Ad = (A9*Ts) + eye(nx); sys(9).Bd = B9*Ts; sys(9).deltad = delta9*Ts;
sys(9).C = eye(ny, nx);