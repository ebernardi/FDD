%% Modelo del Intercambiador de calor
function out = HE(x, u)
	%
	% Funcion del Intercambiador de calor
	%
	% x: Matriz de estados [Theta_1s Theta_2s Theta_p]
	% u: Matriz de entradas [q1 q2]
	% out: Matriz de salidas [dTheta_1s dTheta_2s dTheta_p]

	% estados
	Theta_1s = x(1);
	Theta_2s = x(2);
	Theta_p = x(3);

	% entradas
	q1 = u(1);
	q2 = u(2);
    
    % El fluido 2 es calefactor
    % El fluido 1 es de proceso
	
    % Parámetros
    rho1 = 1;               % Densidad del fluido 1 (kg/l)
    rho2 = 1;               % Densidad del fluido 2 (kg/l)
    rhop = 7.874;        % Densidad de la pared (kg/l)
    Cp1 = 1000;         % Calor especifico del fluido 1 (cal/kg K)
    Cp2 = 1000;          % Calor especifico del fluido 2 (cal/kg K)
    Cpp = 1075.53;     % Calor especifico de la pared (cal/kg K)
    A = 0.881;               % Area de intercambio (m^2)
    h1 = 32374;          % Coeficiente de transferencia de calor para fluido 1 (cal/min K m^2)
    h2 = 14716.6667; % Coeficiente de transferencia de calor para fluido 2 (cal/min K m^2)
    V1 = 16;                % Volumen de tubos (l)
    V2 = 2.11;             % Volumen de carcaza (l)
    Vp = 1.19;             % Volumen de pared (l)
    Theta_1e = 480;       % Temperatura de entrada de fluido 1 (K) (435 +/-10)
    Theta_2e = 900;       % Temperatura de entrada de fluido 2 (K)
	
	% Balance de energía -> Temperatura de proceso
	dTheta_1s = (q1*rho1*Cp1*(Theta_1e-Theta_1s) - A*h1*(Theta_1s-Theta_p))/(rho1*V1*Cp1);
	% Balance de energía -> Temperatura pared
	dTheta_2s = (q2*rho2*Cp2*(Theta_2e-Theta_2s) + A*h2*(Theta_p-Theta_2s))/(rho2*V2*Cp2);
	% Balance de energía -> Temperatura de calefactor
	dTheta_p = (A*h1*(Theta_1s-Theta_p) - A*h2*(Theta_p-Theta_2s))/(rhop*Cpp*Vp);

 	out = [dTheta_1s; dTheta_2s; dTheta_p];
end
