%% Modelo HE
syms rho1 rho2 rhop Cp1 Cp2 Cpp Ar h1 h2 V1 V2 Vp theta1e theta2e q1 q2 theta1s theta2s thetap

% El fluido 2 es calefactor
% El fluido 1 es de proceso

%% Parámetros del HE
Rho1 = 1;               % Densidad del fluido 1 (kg/l)
Rho2 = 1;               % Densidad del fluido 2 (kg/l)
Rhop = 7.874;        % Densidad de la pared (kg/l)
Cp_1 = 1000;         % Calor especifico del fluido 1 (cal/kg K)
Cp_2 = 1000;          % Calor especifico del fluido 2 (cal/kg K)
Cp_p = 1075.53;     % Calor especifico de la pared (cal/kg K)
a = 0.881;               % Area de intercambio (m^2)
h_1 = 32374;          % Coeficiente de transferencia de calor para fluido 1 (cal/min K m^2)
h_2 = 14716.6667; % Coeficiente de transferencia de calor para fluido 2 (cal/min K m^2)
V_1 = 16;                % Volumen de tubos (l)
V_2 = 2.11;             % Volumen de carcaza (l)
V_p = 1.19;             % Volumen de pared (l)
Theta_1e = 480;       % Temperatura de entrada de fluido 1 (K)
Theta_2e = 900;       % Temperatura de entrada de fluido 2 (K)

%% Modelo no lineal
system = [(q1*rho1*Cp1*(theta1e-theta1s)-Ar*h1*(theta1s-thetap))/(rho1*V1*Cp1);
                  (q2*rho2*Cp2*(theta2e-theta2s)+Ar*h2*(thetap-theta2s))/(rho2*V2*Cp2);
                  (Ar*h1*(theta1s-thetap)-Ar*h2*(thetap-theta2s))/(rhop*Cpp*Vp)];

inputs = [q1 q2];
outputs = [theta1s theta2s thetap];
states = [theta1s theta2s thetap];
nx = length(states); nu = length(inputs); ny = length(outputs);
C = eye(ny, nx);        % Matriz de salida
D = zeros(ny, nu);     % Matriz entrada/salida

%% Linealización
% Matrices simbólicas
A_sym = jacobian(system, states);
B_sym = jacobian(system, inputs);

%% Sistema 1 (Theta_1s = Theta_1s_min [K]; Theta_2s = Theta_2s_min [K])
Theta_1s = Theta_1s_min;
Theta_2s = Theta_2s_min;

% Obtengo la temperatura de la pared
Theta_p = (h_2*Theta_2s + h_1*Theta_1s) / (h_2 + h_1);

% Estados para la linealización
Xinit = [Theta_1s; Theta_2s; Theta_p];

% Obtengo el caudal de salida del fluido 1
Q1 = ( a*h_1*(Theta_1s-Theta_p) ) / ( Rho1*Cp_1*(Theta_1e - Theta_1s) );

% Obtengo el caudal de salida del fluido 2
Q2 = ( -a*h_2*(Theta_p - Theta_2s) ) / ( Rho2*Cp_2*(Theta_2e - Theta_2s) );

% Comprobación                  
% Theta_2s = ( (h_1+h_2)*Theta_p - h_1*Theta_1s ) / h_2
% Theta_2s = (Q2*Rho2*Cp_2*Theta_2e + (a*h_2*h_1*Theta_1s / (h_1+h_2)) )/ ...
%                       (Q2*Rho2*Cp_2 + a*h_2 - (a*h_2^2 / (h_1+h_2)))
% Theta_p = ( (Q1*Rho1*Cp_1 + a*h_1)*Theta_1s - Q1*Rho1*Cp_1*Theta_1e ) / (a*h_1)
                  
% Manipuladas para la linealización
Uinit = [Q1; Q2];

% Matrices del sistema lineal
A = subs(A_sym, {rho1, rho2, rhop, Cp1, Cp2, Cpp, Ar, h1, h2, V1, V2, Vp, theta1e, theta2e, theta1s, theta2s, thetap, q1, q2}, ...
          {Rho1, Rho2, Rhop, Cp_1, Cp_2, Cp_p, a, h_1, h_2, V_1, V_2, V_p, Theta_1e, Theta_2e, Xinit(1), Xinit(2), Xinit(3), Uinit(1), Uinit(2)});
B = subs(B_sym, {rho1, rho2, rhop, Cp1, Cp2, Cpp, Ar, h1, h2, V1, V2, Vp, theta1e, theta2e, theta1s, theta2s, thetap, q1, q2}, ...
          {Rho1, Rho2, Rhop, Cp_1, Cp_2, Cp_p, a, h_1, h_2, V_1, V_2, V_p, Theta_1e, Theta_2e, Xinit(1), Xinit(2), Xinit(3), Uinit(1), Uinit(2)});
A1 = double(A);
B1 = double(B);

f = subs(system, {rho1, rho2, rhop, Cp1, Cp2, Cpp, Ar, h1, h2, V1, V2, Vp, theta1e, theta2e, theta1s, theta2s, thetap, q1, q2}, ...
          {Rho1, Rho2, Rhop, Cp_1, Cp_2, Cp_p, a, h_1, h_2, V_1, V_2, V_p, Theta_1e, Theta_2e, Xinit(1), Xinit(2), Xinit(3), Uinit(1), Uinit(2)});
f1 = double(f);

% Desviación del modelo lineal
delta1 = f1 - (A1*Xinit+B1*Uinit);

% Euler discretization method
sys(1).Ad = (A1*Ts) + eye(nx); sys(1).Bd = B1*Ts; sys(1).deltad = delta1*Ts;
sys(1).C = eye(ny, nx);

%% Sistema 2 (Theta_1s = Theta_1s_min [K]; Theta_2s = Theta_2s_mid [K])
Theta_1s = Theta_1s_min;
Theta_2s = Theta_2s_mid;

% Obtengo la temperatura de la pared
Theta_p = (h_2*Theta_2s + h_1*Theta_1s) / (h_2 + h_1);

% Estados para la linealización
Xinit = [Theta_1s; Theta_2s; Theta_p];

% Obtengo el caudal de salida del fluido 1
Q1 = ( a*h_1*(Theta_1s-Theta_p) ) / ( Rho1*Cp_1*(Theta_1e - Theta_1s) );

% Obtengo el caudal de salida del fluido 2
Q2 = ( -a*h_2*(Theta_p - Theta_2s) ) / ( Rho2*Cp_2*(Theta_2e - Theta_2s) );

% Comprobación                  
% Theta_2s = ( (h_1+h_2)*Theta_p - h_1*Theta_1s ) / h_2
% Theta_2s = (Q2*Rho2*Cp_2*Theta_2e + (a*h_2*h_1*Theta_1s / (h_1+h_2)) )/ ...
%                       (Q2*Rho2*Cp_2 + a*h_2 - (a*h_2^2 / (h_1+h_2)))
% Theta_p = ( (Q1*Rho1*Cp_1 + a*h_1)*Theta_1s - Q1*Rho1*Cp_1*Theta_1e ) / (a*h_1)

% Manipuladas para la linealización
Uinit = [Q1; Q2];

% Matrices del sistema lineal
A = subs(A_sym, {rho1, rho2, rhop, Cp1, Cp2, Cpp, Ar, h1, h2, V1, V2, Vp, theta1e, theta2e, theta1s, theta2s, thetap, q1, q2}, ...
          {Rho1, Rho2, Rhop, Cp_1, Cp_2, Cp_p, a, h_1, h_2, V_1, V_2, V_p, Theta_1e, Theta_2e, Xinit(1), Xinit(2), Xinit(3), Uinit(1), Uinit(2)});
B = subs(B_sym, {rho1, rho2, rhop, Cp1, Cp2, Cpp, Ar, h1, h2, V1, V2, Vp, theta1e, theta2e, theta1s, theta2s, thetap, q1, q2}, ...
          {Rho1, Rho2, Rhop, Cp_1, Cp_2, Cp_p, a, h_1, h_2, V_1, V_2, V_p, Theta_1e, Theta_2e, Xinit(1), Xinit(2), Xinit(3), Uinit(1), Uinit(2)});
A2 = double(A);
B2 = double(B);

f = subs(system, {rho1, rho2, rhop, Cp1, Cp2, Cpp, Ar, h1, h2, V1, V2, Vp, theta1e, theta2e, theta1s, theta2s, thetap, q1, q2}, ...
          {Rho1, Rho2, Rhop, Cp_1, Cp_2, Cp_p, a, h_1, h_2, V_1, V_2, V_p, Theta_1e, Theta_2e, Xinit(1), Xinit(2), Xinit(3), Uinit(1), Uinit(2)});
f2 = double(f);

% Desviación del modelo lineal
delta2 = f2 - (A2*Xinit+B2*Uinit);

% Euler discretization method
sys(2).Ad = (A2*Ts) + eye(nx); sys(2).Bd = B2*Ts; sys(2).deltad = delta2*Ts;
sys(2).C = eye(ny, nx);

%% Sistema 3 (Theta_1s = Theta_1s_min [K]; Theta_2s = Theta_2s_max [K])
Theta_1s = Theta_1s_min;
Theta_2s = Theta_2s_max;

% Obtengo la temperatura de la pared
Theta_p = (h_2*Theta_2s + h_1*Theta_1s) / (h_2 + h_1);

% Estados para la linealización
Xinit = [Theta_1s; Theta_2s; Theta_p];

% Obtengo el caudal de salida del fluido 1
Q1 = ( a*h_1*(Theta_1s-Theta_p) ) / ( Rho1*Cp_1*(Theta_1e - Theta_1s) );

% Obtengo el caudal de salida del fluido 2
Q2 = ( -a*h_2*(Theta_p - Theta_2s) ) / ( Rho2*Cp_2*(Theta_2e - Theta_2s) );

% Comprobación                  
% Theta_2s = ( (h_1+h_2)*Theta_p - h_1*Theta_1s ) / h_2
% Theta_2s = (Q2*Rho2*Cp_2*Theta_2e + (a*h_2*h_1*Theta_1s / (h_1+h_2)) )/ ...
%                       (Q2*Rho2*Cp_2 + a*h_2 - (a*h_2^2 / (h_1+h_2)))
% Theta_p = ( (Q1*Rho1*Cp_1 + a*h_1)*Theta_1s - Q1*Rho1*Cp_1*Theta_1e ) / (a*h_1)

% Manipuladas para la linealización
Uinit = [Q1; Q2];

% Matrices del sistema lineal
A = subs(A_sym, {rho1, rho2, rhop, Cp1, Cp2, Cpp, Ar, h1, h2, V1, V2, Vp, theta1e, theta2e, theta1s, theta2s, thetap, q1, q2}, ...
          {Rho1, Rho2, Rhop, Cp_1, Cp_2, Cp_p, a, h_1, h_2, V_1, V_2, V_p, Theta_1e, Theta_2e, Xinit(1), Xinit(2), Xinit(3), Uinit(1), Uinit(2)});
B = subs(B_sym, {rho1, rho2, rhop, Cp1, Cp2, Cpp, Ar, h1, h2, V1, V2, Vp, theta1e, theta2e, theta1s, theta2s, thetap, q1, q2}, ...
          {Rho1, Rho2, Rhop, Cp_1, Cp_2, Cp_p, a, h_1, h_2, V_1, V_2, V_p, Theta_1e, Theta_2e, Xinit(1), Xinit(2), Xinit(3), Uinit(1), Uinit(2)});
A3 = double(A);
B3 = double(B);

f = subs(system, {rho1, rho2, rhop, Cp1, Cp2, Cpp, Ar, h1, h2, V1, V2, Vp, theta1e, theta2e, theta1s, theta2s, thetap, q1, q2}, ...
          {Rho1, Rho2, Rhop, Cp_1, Cp_2, Cp_p, a, h_1, h_2, V_1, V_2, V_p, Theta_1e, Theta_2e, Xinit(1), Xinit(2), Xinit(3), Uinit(1), Uinit(2)});
f3 = double(f);

% Desviación del modelo lineal
delta3 = f3 - (A3*Xinit+B3*Uinit);

% Euler discretization method
sys(3).Ad = (A3*Ts) + eye(nx); sys(3).Bd = B3*Ts; sys(3).deltad = delta3*Ts;
sys(3).C = eye(ny, nx);

%% Sistema 4 (Theta_1s = Theta_1s_mid [K]; Theta_2s = Theta_2s_min [K])
Theta_1s = Theta_1s_mid;
Theta_2s = Theta_2s_min;

% Obtengo la temperatura de la pared
Theta_p = (h_2*Theta_2s + h_1*Theta_1s) / (h_2 + h_1);

% Estados para la linealización
Xinit = [Theta_1s; Theta_2s; Theta_p];

% Obtengo el caudal de salida del fluido 1
Q1 = ( a*h_1*(Theta_1s-Theta_p) ) / ( Rho1*Cp_1*(Theta_1e - Theta_1s) );

% Obtengo el caudal de salida del fluido 2
Q2 = ( -a*h_2*(Theta_p - Theta_2s) ) / ( Rho2*Cp_2*(Theta_2e - Theta_2s) );

% Comprobación                  
% Theta_2s = ( (h_1+h_2)*Theta_p - h_1*Theta_1s ) / h_2
% Theta_2s = (Q2*Rho2*Cp_2*Theta_2e + (a*h_2*h_1*Theta_1s / (h_1+h_2)) )/ ...
%                       (Q2*Rho2*Cp_2 + a*h_2 - (a*h_2^2 / (h_1+h_2)))
% Theta_p = ( (Q1*Rho1*Cp_1 + a*h_1)*Theta_1s - Q1*Rho1*Cp_1*Theta_1e ) / (a*h_1)
                  
% Manipuladas para la linealización
Uinit = [Q1; Q2];

% Matrices del sistema lineal
A = subs(A_sym, {rho1, rho2, rhop, Cp1, Cp2, Cpp, Ar, h1, h2, V1, V2, Vp, theta1e, theta2e, theta1s, theta2s, thetap, q1, q2}, ...
          {Rho1, Rho2, Rhop, Cp_1, Cp_2, Cp_p, a, h_1, h_2, V_1, V_2, V_p, Theta_1e, Theta_2e, Xinit(1), Xinit(2), Xinit(3), Uinit(1), Uinit(2)});
B = subs(B_sym, {rho1, rho2, rhop, Cp1, Cp2, Cpp, Ar, h1, h2, V1, V2, Vp, theta1e, theta2e, theta1s, theta2s, thetap, q1, q2}, ...
          {Rho1, Rho2, Rhop, Cp_1, Cp_2, Cp_p, a, h_1, h_2, V_1, V_2, V_p, Theta_1e, Theta_2e, Xinit(1), Xinit(2), Xinit(3), Uinit(1), Uinit(2)});
A4 = double(A);
B4 = double(B);

f = subs(system, {rho1, rho2, rhop, Cp1, Cp2, Cpp, Ar, h1, h2, V1, V2, Vp, theta1e, theta2e, theta1s, theta2s, thetap, q1, q2}, ...
          {Rho1, Rho2, Rhop, Cp_1, Cp_2, Cp_p, a, h_1, h_2, V_1, V_2, V_p, Theta_1e, Theta_2e, Xinit(1), Xinit(2), Xinit(3), Uinit(1), Uinit(2)});
f4 = double(f);

% Desviación del modelo lineal
delta4 = f4 - (A4*Xinit+B4*Uinit);

% Euler discretization method
sys(4).Ad = (A4*Ts) + eye(nx); sys(4).Bd = B4*Ts; sys(4).deltad = delta4*Ts;
sys(4).C = eye(ny, nx);

%% Sistema 5 (Theta_1s = Theta_1s_mid [K]; Theta_2s = Theta_2s_mid [K])
Theta_1s = Theta_1s_mid;
Theta_2s = Theta_2s_mid;

% Obtengo la temperatura de la pared
Theta_p = (h_2*Theta_2s + h_1*Theta_1s) / (h_2 + h_1);

% Estados para la linealización
Xinit = [Theta_1s; Theta_2s; Theta_p];

% Obtengo el caudal de salida del fluido 1
Q1 = ( a*h_1*(Theta_1s-Theta_p) ) / ( Rho1*Cp_1*(Theta_1e - Theta_1s) );

% Obtengo el caudal de salida del fluido 2
Q2 = ( -a*h_2*(Theta_p - Theta_2s) ) / ( Rho2*Cp_2*(Theta_2e - Theta_2s) );

% Comprobación                  
% Theta_2s = ( (h_1+h_2)*Theta_p - h_1*Theta_1s ) / h_2
% Theta_2s = (Q2*Rho2*Cp_2*Theta_2e + (a*h_2*h_1*Theta_1s / (h_1+h_2)) )/ ...
%                       (Q2*Rho2*Cp_2 + a*h_2 - (a*h_2^2 / (h_1+h_2)))
% Theta_p = ( (Q1*Rho1*Cp_1 + a*h_1)*Theta_1s - Q1*Rho1*Cp_1*Theta_1e ) / (a*h_1)

% Manipuladas para la linealización
Uinit = [Q1; Q2];

% Matrices del sistema lineal
A = subs(A_sym, {rho1, rho2, rhop, Cp1, Cp2, Cpp, Ar, h1, h2, V1, V2, Vp, theta1e, theta2e, theta1s, theta2s, thetap, q1, q2}, ...
          {Rho1, Rho2, Rhop, Cp_1, Cp_2, Cp_p, a, h_1, h_2, V_1, V_2, V_p, Theta_1e, Theta_2e, Xinit(1), Xinit(2), Xinit(3), Uinit(1), Uinit(2)});
B = subs(B_sym, {rho1, rho2, rhop, Cp1, Cp2, Cpp, Ar, h1, h2, V1, V2, Vp, theta1e, theta2e, theta1s, theta2s, thetap, q1, q2}, ...
          {Rho1, Rho2, Rhop, Cp_1, Cp_2, Cp_p, a, h_1, h_2, V_1, V_2, V_p, Theta_1e, Theta_2e, Xinit(1), Xinit(2), Xinit(3), Uinit(1), Uinit(2)});
A5 = double(A);
B5 = double(B);

f = subs(system, {rho1, rho2, rhop, Cp1, Cp2, Cpp, Ar, h1, h2, V1, V2, Vp, theta1e, theta2e, theta1s, theta2s, thetap, q1, q2}, ...
          {Rho1, Rho2, Rhop, Cp_1, Cp_2, Cp_p, a, h_1, h_2, V_1, V_2, V_p, Theta_1e, Theta_2e, Xinit(1), Xinit(2), Xinit(3), Uinit(1), Uinit(2)});
f5 = double(f);

% Desviación del modelo lineal
delta5 = f5 - (A5*Xinit+B5*Uinit);

% Euler discretization method
sys(5).Ad = (A5*Ts) + eye(nx); sys(5).Bd = B5*Ts; sys(5).deltad = delta5*Ts;
sys(5).C = eye(ny, nx);

%% Sistema 6 (Theta_1s = Theta_1s_mid [K]; Theta_2s = Theta_2s_max [K])
Theta_1s = Theta_1s_mid;
Theta_2s = Theta_2s_max;

% Obtengo la temperatura de la pared
Theta_p = (h_2*Theta_2s + h_1*Theta_1s) / (h_2 + h_1);

% Estados para la linealización
Xinit = [Theta_1s; Theta_2s; Theta_p];

% Obtengo el caudal de salida del fluido 1
Q1 = ( a*h_1*(Theta_1s-Theta_p) ) / ( Rho1*Cp_1*(Theta_1e - Theta_1s) );

% Obtengo el caudal de salida del fluido 2
Q2 = ( -a*h_2*(Theta_p - Theta_2s) ) / ( Rho2*Cp_2*(Theta_2e - Theta_2s) );

% Comprobación                  
% Theta_2s = ( (h_1+h_2)*Theta_p - h_1*Theta_1s ) / h_2
% Theta_2s = (Q2*Rho2*Cp_2*Theta_2e + (a*h_2*h_1*Theta_1s / (h_1+h_2)) )/ ...
%                       (Q2*Rho2*Cp_2 + a*h_2 - (a*h_2^2 / (h_1+h_2)))
% Theta_p = ( (Q1*Rho1*Cp_1 + a*h_1)*Theta_1s - Q1*Rho1*Cp_1*Theta_1e ) / (a*h_1)

% Manipuladas para la linealización
Uinit = [Q1; Q2];

% Matrices del sistema lineal
A = subs(A_sym, {rho1, rho2, rhop, Cp1, Cp2, Cpp, Ar, h1, h2, V1, V2, Vp, theta1e, theta2e, theta1s, theta2s, thetap, q1, q2}, ...
          {Rho1, Rho2, Rhop, Cp_1, Cp_2, Cp_p, a, h_1, h_2, V_1, V_2, V_p, Theta_1e, Theta_2e, Xinit(1), Xinit(2), Xinit(3), Uinit(1), Uinit(2)});
B = subs(B_sym, {rho1, rho2, rhop, Cp1, Cp2, Cpp, Ar, h1, h2, V1, V2, Vp, theta1e, theta2e, theta1s, theta2s, thetap, q1, q2}, ...
          {Rho1, Rho2, Rhop, Cp_1, Cp_2, Cp_p, a, h_1, h_2, V_1, V_2, V_p, Theta_1e, Theta_2e, Xinit(1), Xinit(2), Xinit(3), Uinit(1), Uinit(2)});
A6 = double(A);
B6 = double(B);

f = subs(system, {rho1, rho2, rhop, Cp1, Cp2, Cpp, Ar, h1, h2, V1, V2, Vp, theta1e, theta2e, theta1s, theta2s, thetap, q1, q2}, ...
          {Rho1, Rho2, Rhop, Cp_1, Cp_2, Cp_p, a, h_1, h_2, V_1, V_2, V_p, Theta_1e, Theta_2e, Xinit(1), Xinit(2), Xinit(3), Uinit(1), Uinit(2)});
f6 = double(f);

% Desviación del modelo lineal
delta6 = f6 - (A6*Xinit+B6*Uinit);

% Euler discretization method
sys(6).Ad = (A6*Ts) + eye(nx); sys(6).Bd = B6*Ts; sys(6).deltad = delta6*Ts;
sys(6).C = eye(ny, nx);

%% Sistema 7 (Theta_1s = Theta_1s_max [K]; Theta_2s = Theta_2s_min [K])
Theta_1s = Theta_1s_max;
Theta_2s = Theta_2s_min;

% Obtengo la temperatura de la pared
Theta_p = (h_2*Theta_2s + h_1*Theta_1s) / (h_2 + h_1);

% Estados para la linealización
Xinit = [Theta_1s; Theta_2s; Theta_p];

% Obtengo el caudal de salida del fluido 1
Q1 = ( a*h_1*(Theta_1s-Theta_p) ) / ( Rho1*Cp_1*(Theta_1e - Theta_1s) );

% Obtengo el caudal de salida del fluido 2
Q2 = ( -a*h_2*(Theta_p - Theta_2s) ) / ( Rho2*Cp_2*(Theta_2e - Theta_2s) );

% Comprobación                  
% Theta_2s = ( (h_1+h_2)*Theta_p - h_1*Theta_1s ) / h_2
% Theta_2s = (Q2*Rho2*Cp_2*Theta_2e + (a*h_2*h_1*Theta_1s / (h_1+h_2)) )/ ...
%                       (Q2*Rho2*Cp_2 + a*h_2 - (a*h_2^2 / (h_1+h_2)))
% Theta_p = ( (Q1*Rho1*Cp_1 + a*h_1)*Theta_1s - Q1*Rho1*Cp_1*Theta_1e ) / (a*h_1)
                  
% Manipuladas para la linealización
Uinit = [Q1; Q2];

% Matrices del sistema lineal
A = subs(A_sym, {rho1, rho2, rhop, Cp1, Cp2, Cpp, Ar, h1, h2, V1, V2, Vp, theta1e, theta2e, theta1s, theta2s, thetap, q1, q2}, ...
          {Rho1, Rho2, Rhop, Cp_1, Cp_2, Cp_p, a, h_1, h_2, V_1, V_2, V_p, Theta_1e, Theta_2e, Xinit(1), Xinit(2), Xinit(3), Uinit(1), Uinit(2)});
B = subs(B_sym, {rho1, rho2, rhop, Cp1, Cp2, Cpp, Ar, h1, h2, V1, V2, Vp, theta1e, theta2e, theta1s, theta2s, thetap, q1, q2}, ...
          {Rho1, Rho2, Rhop, Cp_1, Cp_2, Cp_p, a, h_1, h_2, V_1, V_2, V_p, Theta_1e, Theta_2e, Xinit(1), Xinit(2), Xinit(3), Uinit(1), Uinit(2)});
A7 = double(A);
B7 = double(B);

f = subs(system, {rho1, rho2, rhop, Cp1, Cp2, Cpp, Ar, h1, h2, V1, V2, Vp, theta1e, theta2e, theta1s, theta2s, thetap, q1, q2}, ...
          {Rho1, Rho2, Rhop, Cp_1, Cp_2, Cp_p, a, h_1, h_2, V_1, V_2, V_p, Theta_1e, Theta_2e, Xinit(1), Xinit(2), Xinit(3), Uinit(1), Uinit(2)});
f7 = double(f);

% Desviación del modelo lineal
delta7 = f7 - (A7*Xinit+B7*Uinit);

% Euler discretization method
sys(7).Ad = (A7*Ts) + eye(nx); sys(7).Bd = B7*Ts; sys(7).deltad = delta7*Ts;
sys(7).C = eye(ny, nx);

%% Sistema 8 (Theta_1s = Theta_1s_max [K]; Theta_2s = Theta_2s_mid [K])
Theta_1s = Theta_1s_max;
Theta_2s = Theta_2s_mid;

% Obtengo la temperatura de la pared
Theta_p = (h_2*Theta_2s + h_1*Theta_1s) / (h_2 + h_1);

% Estados para la linealización
Xinit = [Theta_1s; Theta_2s; Theta_p];

% Obtengo el caudal de salida del fluido 1
Q1 = ( a*h_1*(Theta_1s-Theta_p) ) / ( Rho1*Cp_1*(Theta_1e - Theta_1s) );

% Obtengo el caudal de salida del fluido 2
Q2 = ( -a*h_2*(Theta_p - Theta_2s) ) / ( Rho2*Cp_2*(Theta_2e - Theta_2s) );

% Comprobación                  
% Theta_2s = ( (h_1+h_2)*Theta_p - h_1*Theta_1s ) / h_2
% Theta_2s = (Q2*Rho2*Cp_2*Theta_2e + (a*h_2*h_1*Theta_1s / (h_1+h_2)) )/ ...
%                       (Q2*Rho2*Cp_2 + a*h_2 - (a*h_2^2 / (h_1+h_2)))
% Theta_p = ( (Q1*Rho1*Cp_1 + a*h_1)*Theta_1s - Q1*Rho1*Cp_1*Theta_1e ) / (a*h_1)

% Manipuladas para la linealización
Uinit = [Q1; Q2];

% Matrices del sistema lineal
A = subs(A_sym, {rho1, rho2, rhop, Cp1, Cp2, Cpp, Ar, h1, h2, V1, V2, Vp, theta1e, theta2e, theta1s, theta2s, thetap, q1, q2}, ...
          {Rho1, Rho2, Rhop, Cp_1, Cp_2, Cp_p, a, h_1, h_2, V_1, V_2, V_p, Theta_1e, Theta_2e, Xinit(1), Xinit(2), Xinit(3), Uinit(1), Uinit(2)});
B = subs(B_sym, {rho1, rho2, rhop, Cp1, Cp2, Cpp, Ar, h1, h2, V1, V2, Vp, theta1e, theta2e, theta1s, theta2s, thetap, q1, q2}, ...
          {Rho1, Rho2, Rhop, Cp_1, Cp_2, Cp_p, a, h_1, h_2, V_1, V_2, V_p, Theta_1e, Theta_2e, Xinit(1), Xinit(2), Xinit(3), Uinit(1), Uinit(2)});
A8 = double(A);
B8 = double(B);

f = subs(system, {rho1, rho2, rhop, Cp1, Cp2, Cpp, Ar, h1, h2, V1, V2, Vp, theta1e, theta2e, theta1s, theta2s, thetap, q1, q2}, ...
          {Rho1, Rho2, Rhop, Cp_1, Cp_2, Cp_p, a, h_1, h_2, V_1, V_2, V_p, Theta_1e, Theta_2e, Xinit(1), Xinit(2), Xinit(3), Uinit(1), Uinit(2)});
f8 = double(f);

% Desviación del modelo lineal
delta8 = f8 - (A8*Xinit+B8*Uinit);

% Euler discretization method
sys(8).Ad = (A8*Ts) + eye(nx); sys(8).Bd = B8*Ts; sys(8).deltad = delta8*Ts;
sys(8).C = eye(ny, nx);

%% Sistema 9 (Theta_1s = Theta_1s_max [K]; Theta_2s = Theta_2s_max [K])
Theta_1s = Theta_1s_max;
Theta_2s = Theta_2s_max;

% Obtengo la temperatura de la pared
Theta_p = (h_2*Theta_2s + h_1*Theta_1s) / (h_2 + h_1);

% Estados para la linealización
Xinit = [Theta_1s; Theta_2s; Theta_p];

% Obtengo el caudal de salida del fluido 1
Q1 = ( a*h_1*(Theta_1s-Theta_p) ) / ( Rho1*Cp_1*(Theta_1e - Theta_1s) );

% Obtengo el caudal de salida del fluido 2
Q2 = ( -a*h_2*(Theta_p - Theta_2s) ) / ( Rho2*Cp_2*(Theta_2e - Theta_2s) );

% Comprobación                  
% Theta_2s = ( (h_1+h_2)*Theta_p - h_1*Theta_1s ) / h_2
% Theta_2s = (Q2*Rho2*Cp_2*Theta_2e + (a*h_2*h_1*Theta_1s / (h_1+h_2)) )/ ...
%                       (Q2*Rho2*Cp_2 + a*h_2 - (a*h_2^2 / (h_1+h_2)))
% Theta_p = ( (Q1*Rho1*Cp_1 + a*h_1)*Theta_1s - Q1*Rho1*Cp_1*Theta_1e ) / (a*h_1)

% Manipuladas para la linealización
Uinit = [Q1; Q2];

% Matrices del sistema lineal
A = subs(A_sym, {rho1, rho2, rhop, Cp1, Cp2, Cpp, Ar, h1, h2, V1, V2, Vp, theta1e, theta2e, theta1s, theta2s, thetap, q1, q2}, ...
          {Rho1, Rho2, Rhop, Cp_1, Cp_2, Cp_p, a, h_1, h_2, V_1, V_2, V_p, Theta_1e, Theta_2e, Xinit(1), Xinit(2), Xinit(3), Uinit(1), Uinit(2)});
B = subs(B_sym, {rho1, rho2, rhop, Cp1, Cp2, Cpp, Ar, h1, h2, V1, V2, Vp, theta1e, theta2e, theta1s, theta2s, thetap, q1, q2}, ...
          {Rho1, Rho2, Rhop, Cp_1, Cp_2, Cp_p, a, h_1, h_2, V_1, V_2, V_p, Theta_1e, Theta_2e, Xinit(1), Xinit(2), Xinit(3), Uinit(1), Uinit(2)});
A9 = double(A);
B9 = double(B);

f = subs(system, {rho1, rho2, rhop, Cp1, Cp2, Cpp, Ar, h1, h2, V1, V2, Vp, theta1e, theta2e, theta1s, theta2s, thetap, q1, q2}, ...
          {Rho1, Rho2, Rhop, Cp_1, Cp_2, Cp_p, a, h_1, h_2, V_1, V_2, V_p, Theta_1e, Theta_2e, Xinit(1), Xinit(2), Xinit(3), Uinit(1), Uinit(2)});
f9 = double(f);

% Desviación del modelo lineal
delta9 = f9 - (A9*Xinit+B9*Uinit);

% Euler discretization method
sys(9).Ad = (A9*Ts) + eye(nx); sys(9).Bd = B9*Ts; sys(9).deltad = delta9*Ts;
sys(9).C = eye(ny, nx);