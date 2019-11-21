%% Modelo del reactor CSTR
function out = CSTR(x, u)
	%
	% Funcion del reactor CSTR
	%
	% x: Matriz de estados [V CA T]
	% u: Matriz de entradas [qs qc]
	% out: Matriz de salidas [dV dCA dT]

	% estados
	V = x(1);
	CA = x(2);
	T = x(3);

	% entradas
	qs = u(1);
	qc = u(2);
	
	% Parámetros
	E_R = 1e4;			% [°K] (Energia de activación)
	Te = 350;			% [°K] (Temperatura de entrada del reactante)
	Tce = 350;			% [°K] (Temperatura del liquido refrigerante)
	dH = -2e5;			% [cal/mol] (Calor de reacción)
	Cp = 1;				% [cal/g °K] (Calores específicos)
	rho = 1e3;			% [g/l] (Densidad de los líquidos)
	CAe = 1;			% [mol/l] (Concentración de A a la entrada)
	ha = 7e5;			% [cal/min °K] (Coeficiente de transferencia de calor)
	k0 = 7.2e10;		% [l/min] (Constante de velocidad de reacción)
	k1 = dH*k0/(rho*Cp);
	k2 = rho*Cp/(rho*Cp);
	k3 = ha/(rho*Cp);
    q = 100;            % [l/min] (Caudal de Entrada)
% 	k4 = 10;			% [l/min m^3/2] (Constante de la válvula)
	
	% Balance de materia en el tanque -> Volumen
	dV = q - qs;
	% Balance de materia -> Concentración 
	dCA = (q/V)*(CAe-CA) - k0*CA*exp(-E_R/T);
	% Balance de energía en el reactor -> Temperatura
	dT = (q/V)*(Te-T) - k1*CA*exp(-E_R/T) + k2*(qc/V)*(1-exp(-k3/qc))*(Tce-T);

 	out = [dV; dCA; dT];
end
