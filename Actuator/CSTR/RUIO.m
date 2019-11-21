%% Obtengo la matriz de transformación T para cada modelo
% H1_N = [0 0; 0 0.001; 1 0];
H1_N = [0 0; 0 1; 1 1];

% Modelo 1
H1_D1 = B1d(:, 1);
H1_B1 = B1d;
H1_N1 = H1_N;
H1_T1 = [H1_N1 H1_D1];
H1_INV_T1 = inv(H1_T1);
H1_A1_bar = H1_T1\A1d*H1_T1;
H1_B1_bar = H1_T1\H1_B1;
H1_B1_bar_1 = H1_B1_bar(1:2, :);
H1_B1_bar_2 = H1_B1_bar(3, :);
H1_D1_bar = H1_T1\H1_D1;
H1_delta1_bar = H1_T1\delta1d;
H1_delta1_bar_1 = H1_delta1_bar(1:2, :);
H1_delta1_bar_2 = H1_delta1_bar(3, :);

% Modelo 2
H1_D2 = B2d(:, 1);
H1_B2 = B2d;
H1_N2 = H1_N;
H1_T2 = [H1_N2 H1_D2];
H1_INV_T2 = inv(H1_T2);
H1_A2_bar = H1_T2\A2d*H1_T2;
H1_B2_bar = H1_T2\H1_B2;
H1_B2_bar_1 = H1_B2_bar(1:2, :);
H1_B2_bar_2 = H1_B2_bar(3, :);
H1_D2_bar = H1_T2\H1_D2;
H1_delta2_bar = H1_T2\delta2d;
H1_delta2_bar_1 = H1_delta2_bar(1:2, :);
H1_delta2_bar_2 = H1_delta2_bar(3, :);

% Modelo 3
H1_D3 = B3d(:, 1);
H1_B3 = B3d;
H1_N3 = H1_N;
H1_T3 = [H1_N3 H1_D3];
H1_INV_T3 = inv(H1_T3);
H1_A3_bar = H1_T3\A3d*H1_T3;
H1_B3_bar = H1_T3\H1_B3;
H1_B3_bar_1 = H1_B3_bar(1:2, :);
H1_B3_bar_2 = H1_B3_bar(3, :);
H1_D3_bar = H1_T3\H1_D3;
H1_delta3_bar = H1_T3\delta3d;
H1_delta3_bar_1 = H1_delta3_bar(1:2, :);
H1_delta3_bar_2 = H1_delta3_bar(3, :);

% Modelo 4
H1_D4 = B4d(:, 1);
H1_B4 = B4d;
H1_N4 = H1_N;
H1_T4 = [H1_N4 H1_D4];
H1_A4_bar = H1_T4\A4d*H1_T4;
H1_B4_bar = H1_T4\H1_B4;
H1_B4_bar_1 = H1_B4_bar(1:2, :);
H1_B4_bar_2 = H1_B4_bar(3, :);
H1_D4_bar = H1_T4\H1_D4;
H1_delta4_bar = H1_T4\delta4d;
H1_delta4_bar_1 = H1_delta4_bar(1:2, :);
H1_delta4_bar_2 = H1_delta4_bar(3, :);

% Modelo 5
H1_D5 = B5d(:, 1);
H1_B5 = B5d;
H1_N5 = H1_N;
H1_T5 = [H1_N5 H1_D5];
H1_A5_bar = H1_T5\A5d*H1_T5;
H1_B5_bar = H1_T5\H1_B5;
H1_B5_bar_1 = H1_B5_bar(1:2, :);
H1_B5_bar_2 = H1_B5_bar(3, :);
H1_D5_bar = H1_T5\H1_D5;
H1_delta5_bar = H1_T5\delta5d;
H1_delta5_bar_1 = H1_delta5_bar(1:2, :);
H1_delta5_bar_2 = H1_delta5_bar(3, :);

% Modelo 6
H1_D6 = B6d(:, 1);
H1_B6 = B6d;
H1_N6 = H1_N;
H1_T6 = [H1_N6 H1_D6];
H1_A6_bar = H1_T6\A6d*H1_T6;
H1_B6_bar = H1_T6\H1_B6;
H1_B6_bar_1 = H1_B6_bar(1:2, :);
H1_B6_bar_2 = H1_B6_bar(3, :);
H1_D6_bar = H1_T6\H1_D6;
H1_delta6_bar = H1_T6\delta6d;
H1_delta6_bar_1 = H1_delta6_bar(1:2, :);
H1_delta6_bar_2 = H1_delta6_bar(3, :);

% Modelo 7
H1_D7 = B7d(:, 1);
H1_B7 = B7d;
H1_N7 = H1_N;
H1_T7 = [H1_N7 H1_D7];
H1_A7_bar = H1_T7\A7d*H1_T7;
H1_B7_bar = H1_T7\H1_B7;
H1_B7_bar_1 = H1_B7_bar(1:2, :);
H1_B7_bar_2 = H1_B7_bar(3, :);
H1_D7_bar = H1_T7\H1_D7;
H1_delta7_bar = H1_T7\delta7d;
H1_delta7_bar_1 = H1_delta7_bar(1:2, :);
H1_delta7_bar_2 = H1_delta7_bar(3, :);

% Modelo 8
H1_D8 = B8d(:, 1);
H1_B8 = B8d;
H1_N8 = H1_N;
H1_T8 = [H1_N8 H1_D8];
H1_A8_bar = H1_T8\A8d*H1_T8;
H1_B8_bar = H1_T8\H1_B8;
H1_B8_bar_1 = H1_B8_bar(1:2, :);
H1_B8_bar_2 = H1_B8_bar(3, :);
H1_D8_bar = H1_T8\H1_D8;
H1_delta8_bar = H1_T8\delta8d;
H1_delta8_bar_1 = H1_delta8_bar(1:2, :);
H1_delta8_bar_2 = H1_delta8_bar(3, :);

% Modelo 9
H1_D9 = B9d(:, 1);
H1_B9 = B9d;
H1_N9 = H1_N;
H1_T9 = [H1_N9 H1_D9];
H1_A9_bar = H1_T9\A9d*H1_T9;
H1_B9_bar = H1_T9\H1_B9;
H1_B9_bar_1 = H1_B9_bar(1:2, :);
H1_B9_bar_2 = H1_B9_bar(3, :);
H1_D9_bar = H1_T9\H1_D9;
H1_delta9_bar = H1_T9\delta9d;
H1_delta9_bar_1 = H1_delta9_bar(1:2, :);
H1_delta9_bar_2 = H1_delta9_bar(3, :);

%% Separo el vector de estados no perturbados (n-q) del vector de estados perturbados (q)

% Modelo 1
H1_A1_bar_11 = H1_A1_bar(1:2, 1:2);
H1_A1_bar_12 = H1_A1_bar(1:2, 3);
H1_A1_bar_21 = H1_A1_bar(3, 1:2);
H1_A1_bar_22 = H1_A1_bar(3, 3);

% Modelo 2
H1_A2_bar_11 = H1_A2_bar(1:2, 1:2);
H1_A2_bar_12 = H1_A2_bar(1:2, 3);
H1_A2_bar_21 = H1_A2_bar(3, 1:2);
H1_A2_bar_22 = H1_A2_bar(3, 3);

% Modelo 3
H1_A3_bar_11 = H1_A3_bar(1:2, 1:2);
H1_A3_bar_12 = H1_A3_bar(1:2, 3);
H1_A3_bar_21 = H1_A3_bar(3, 1:2);
H1_A3_bar_22 = H1_A3_bar(3, 3);

% Modelo 4
H1_A4_bar_11 = H1_A4_bar(1:2, 1:2);
H1_A4_bar_12 = H1_A4_bar(1:2, 3);
H1_A4_bar_21 = H1_A4_bar(3, 1:2);
H1_A4_bar_22 = H1_A4_bar(3, 3);

% Modelo 5
H1_A5_bar_11 = H1_A5_bar(1:2, 1:2);
H1_A5_bar_12 = H1_A5_bar(1:2, 3);
H1_A5_bar_21 = H1_A5_bar(3, 1:2);
H1_A5_bar_22 = H1_A5_bar(3, 3);

% Modelo 6
H1_A6_bar_11 = H1_A6_bar(1:2, 1:2);
H1_A6_bar_12 = H1_A6_bar(1:2, 3);
H1_A6_bar_21 = H1_A6_bar(3, 1:2);
H1_A6_bar_22 = H1_A6_bar(3, 3);

% Modelo 7
H1_A7_bar_11 = H1_A7_bar(1:2, 1:2);
H1_A7_bar_12 = H1_A7_bar(1:2, 3);
H1_A7_bar_21 = H1_A7_bar(3, 1:2);
H1_A7_bar_22 = H1_A7_bar(3, 3);

% Modelo 8
H1_A8_bar_11 = H1_A8_bar(1:2, 1:2);
H1_A8_bar_12 = H1_A8_bar(1:2, 3);
H1_A8_bar_21 = H1_A8_bar(3, 1:2);
H1_A8_bar_22 = H1_A8_bar(3, 3);

% Modelo 9
H1_A9_bar_11 = H1_A9_bar(1:2, 1:2);
H1_A9_bar_12 = H1_A9_bar(1:2, 3);
H1_A9_bar_21 = H1_A9_bar(3, 1:2);
H1_A9_bar_22 = H1_A9_bar(3, 3);

%% Armo la matriz no singular U
H1_Q = [0 0; 1 0; 0 1];

% Modelo 1
H1_rango_CD1 = rank(C*H1_D1);
H1_Q1 = H1_Q;
H1_U1 = [C*H1_D1 H1_Q1];
H1_inv_U1 = inv(H1_U1);
H1_U1_1 = H1_inv_U1(1, :);
H1_U1_2 = H1_inv_U1(2:3, :);

% Modelo 2
H1_rango_CD2 = rank(C*H1_D2);
H1_Q2 = H1_Q;
H1_U2 = [C*H1_D2 H1_Q2];
H1_inv_U2 = inv(H1_U2);
H1_U2_1 = H1_inv_U2(1, :);
H1_U2_2 = H1_inv_U2(2:3, :);

% Modelo 3
H1_rango_CD3 = rank(C*H1_D3);
H1_Q3 = H1_Q;
H1_U3 = [C*H1_D3 H1_Q3];
H1_inv_U3 = inv(H1_U3);
H1_U3_1 = H1_inv_U3(1, :);
H1_U3_2 = H1_inv_U3(2:3, :);

% Modelo 4
H1_rango_CD4 = rank(C*H1_D4);
H1_Q4 = H1_Q;
H1_U4 = [C*H1_D4 H1_Q4];
H1_inv_U4 = inv(H1_U4);
H1_U4_1 = H1_inv_U4(1, :);
H1_U4_2 = H1_inv_U4(2:3, :);

% Modelo 5
H1_rango_CD5 = rank(C*H1_D5);
H1_Q5 = H1_Q;
H1_U5 = [C*H1_D5 H1_Q5];
H1_inv_U5 = inv(H1_U5);
H1_U5_1 = H1_inv_U5(1, :);
H1_U5_2 = H1_inv_U5(2:3, :);

% Modelo 6
H1_rango_CD6 = rank(C*H1_D6);
H1_Q6 = H1_Q;
H1_U6 = [C*H1_D6 H1_Q6];
H1_inv_U6 = inv(H1_U6);
H1_U6_1 = H1_inv_U6(1, :);
H1_U6_2 = H1_inv_U6(2:3, :);

% Modelo 7
H1_rango_CD7 = rank(C*H1_D7);
H1_Q7 = H1_Q;
H1_U7 = [C*H1_D7 H1_Q7];
H1_inv_U7 = inv(H1_U7);
H1_U7_1 = H1_inv_U7(1, :);
H1_U7_2 = H1_inv_U7(2:3, :);

% Modelo 8
H1_rango_CD8 = rank(C*H1_D8);
H1_Q8 = H1_Q;
H1_U8 = [C*H1_D8 H1_Q8];
H1_inv_U8 = inv(H1_U8);
H1_U8_1 = H1_inv_U8(1, :);
H1_U8_2 = H1_inv_U8(2:3, :);

% Modelo 9
H1_rango_CD9 = rank(C*H1_D9);
H1_Q9 = H1_Q;
H1_U9 = [C*H1_D9 H1_Q9];
H1_inv_U9 = inv(H1_U9);
H1_U9_1 = H1_inv_U9(1, :);
H1_U9_2 = H1_inv_U9(2:3, :);

%% Obtengo matrices de utilidad y compruebo observabilidad
% Modelo 1
H1_A1_tilde_1 = H1_A1_bar_11 - H1_A1_bar_12*H1_U1_1*C*H1_N1;
H1_C1_tilde_1 = C*H1_N1;
H1_E1_1 = H1_A1_bar_12*H1_U1_1;

H1_rango_C1_tilde_1 = rank(H1_C1_tilde_1);
H1_O_M1 = [H1_C1_tilde_1' (H1_C1_tilde_1*H1_A1_tilde_1)' (H1_C1_tilde_1*H1_A1_tilde_1*H1_A1_tilde_1)']';
H1_rank_Obs_M1 = rank(H1_O_M1);

% Modelo 2
H1_A2_tilde_1 = H1_A2_bar_11 - H1_A2_bar_12*H1_U2_1*C*H1_N2;
H1_C2_tilde_1 = C*H1_N2;
H1_E2_1 = H1_A2_bar_12*H1_U2_1;

H1_rango_C2_tilde_1 = rank(H1_C2_tilde_1);
H1_O_M2 = [H1_C2_tilde_1' (H1_C2_tilde_1*H1_A2_tilde_1)' (H1_C2_tilde_1*H1_A2_tilde_1*H1_A2_tilde_1)']';
H1_rank_Obs_M2 = rank(H1_O_M2);

% Modelo 3
H1_A3_tilde_1 = H1_A3_bar_11 - H1_A3_bar_12*H1_U3_1*C*H1_N3;
H1_C3_tilde_1 = C*H1_N3;
H1_E3_1 = H1_A3_bar_12*H1_U3_1;

H1_rango_C3_tilde_1 = rank(H1_C3_tilde_1);
H1_O_M3 = [H1_C3_tilde_1' (H1_C3_tilde_1*H1_A3_tilde_1)' (H1_C3_tilde_1*H1_A3_tilde_1*H1_A3_tilde_1)']';
H1_rank_Obs_M3 = rank(H1_O_M3);

% Modelo 4
H1_A4_tilde_1 = H1_A4_bar_11 - H1_A4_bar_12*H1_U4_1*C*H1_N4;
H1_C4_tilde_1 = C*H1_N4;
H1_E4_1 = H1_A4_bar_12*H1_U4_1;

H1_rango_C4_tilde_1 = rank(H1_C4_tilde_1);

H1_O_M4 = [H1_C4_tilde_1' (H1_C4_tilde_1*H1_A4_tilde_1)' (H1_C4_tilde_1*H1_A4_tilde_1*H1_A4_tilde_1)']';
H1_rank_Obs_M4 = rank(H1_O_M4);

% Modelo 5
H1_A5_tilde_1 = H1_A5_bar_11 - H1_A5_bar_12*H1_U5_1*C*H1_N5;
H1_C5_tilde_1 = C*H1_N5;
H1_E5_1 = H1_A5_bar_12*H1_U5_1;

H1_rango_C5_tilde_1 = rank(H1_C5_tilde_1);

H1_O_M5 = [H1_C5_tilde_1' (H1_C5_tilde_1*H1_A5_tilde_1)' (H1_C5_tilde_1*H1_A5_tilde_1*H1_A5_tilde_1)']';
H1_rank_Obs_M5 = rank(H1_O_M5);

% Modelo 6
H1_A6_tilde_1 = H1_A6_bar_11 - H1_A6_bar_12*H1_U6_1*C*H1_N6;
H1_C6_tilde_1 = C*H1_N6;
H1_E6_1 = H1_A6_bar_12*H1_U6_1;

H1_rango_C6_tilde_1 = rank(H1_C6_tilde_1);

H1_O_M6 = [H1_C6_tilde_1' (H1_C6_tilde_1*H1_A6_tilde_1)' (H1_C6_tilde_1*H1_A6_tilde_1*H1_A6_tilde_1)']';
H1_rank_Obs_M6 = rank(H1_O_M6);

% Modelo 7
H1_A7_tilde_1 = H1_A7_bar_11 - H1_A7_bar_12*H1_U7_1*C*H1_N7;
H1_C7_tilde_1 = C*H1_N7;
H1_E7_1 = H1_A7_bar_12*H1_U7_1;

H1_rango_C7_tilde_1 = rank(H1_C7_tilde_1);

H1_O_M7 = [H1_C7_tilde_1' (H1_C7_tilde_1*H1_A7_tilde_1)' (H1_C7_tilde_1*H1_A7_tilde_1*H1_A7_tilde_1)']';
H1_rank_Obs_M7 = rank(H1_O_M7);

% Modelo 8
H1_A8_tilde_1 = H1_A8_bar_11 - H1_A8_bar_12*H1_U8_1*C*H1_N8;
H1_C8_tilde_1 = C*H1_N8;
H1_E8_1 = H1_A8_bar_12*H1_U8_1;

H1_rango_C8_tilde_1 = rank(H1_C8_tilde_1);

H1_O_M8 = [H1_C8_tilde_1' (H1_C8_tilde_1*H1_A8_tilde_1)' (H1_C8_tilde_1*H1_A8_tilde_1*H1_A8_tilde_1)']';
H1_rank_Obs_M8 = rank(H1_O_M8);

% Modelo 9
H1_A9_tilde_1 = H1_A9_bar_11 - H1_A9_bar_12*H1_U9_1*C*H1_N9;
H1_C9_tilde_1 = C*H1_N9;
H1_E9_1 = H1_A9_bar_12*H1_U9_1;

H1_rango_C9_tilde_1 = rank(H1_C9_tilde_1);

H1_O_M9 = [H1_C9_tilde_1' (H1_C9_tilde_1*H1_A9_tilde_1)' (H1_C9_tilde_1*H1_A9_tilde_1*H1_A9_tilde_1)']';
H1_rank_Obs_M9 = rank(H1_O_M9);


%% LMI NRUIO
% Dimension of system matrices
n = size(H1_A1_tilde_1, 1);    % n = 2
p = size(H1_C1_tilde_1, 1);    % p = 3

% %% Variables to be determined NUIO1
% yalmip('clear');
% X = sdpvar(n);
% 
% W1 = sdpvar(n, p);
% W2 = sdpvar(n, p);
% W3 = sdpvar(n, p);
% W4 = sdpvar(n, p);
% W5 = sdpvar(n, p);
% W6 = sdpvar(n, p);
% W7 = sdpvar(n, p);
% W8 = sdpvar(n, p);
% W9 = sdpvar(n, p);
% 
% alpha = -1;
% 
% LMI_1 =[2*alpha*X H1_A1_tilde_1'*X-H1_C1_tilde_1'*W1';
%              X*H1_A1_tilde_1-W1*H1_C1_tilde_1 2*alpha*X];
% 
% LMI_2 =[2*alpha*X H1_A2_tilde_1'*X-H1_C2_tilde_1'*W2';
%              X*H1_A2_tilde_1-W2*H1_C2_tilde_1 2*alpha*X];
% 
% LMI_3 =[2*alpha*X H1_A3_tilde_1'*X-H1_C3_tilde_1'*W3';
%              X*H1_A3_tilde_1-W3*H1_C3_tilde_1 2*alpha*X];
% 
% LMI_4 =[2*alpha*X H1_A4_tilde_1'*X-H1_C4_tilde_1'*W4';
%              X*H1_A4_tilde_1-W4*H1_C4_tilde_1 2*alpha*X];
%          
% LMI_5 =[2*alpha*X H1_A5_tilde_1'*X-H1_C5_tilde_1'*W5';
%              X*H1_A5_tilde_1-W1*H1_C5_tilde_1 2*alpha*X];
% 
% LMI_6 =[2*alpha*X H1_A6_tilde_1'*X-H1_C6_tilde_1'*W6';
%              X*H1_A6_tilde_1-W6*H1_C6_tilde_1 2*alpha*X];
% 
% LMI_7 =[2*alpha*X H1_A7_tilde_1'*X-H1_C7_tilde_1'*W7';
%              X*H1_A7_tilde_1-W7*H1_C7_tilde_1 2*alpha*X];
% 
% LMI_8 =[2*alpha*X H1_A8_tilde_1'*X-H1_C8_tilde_1'*W8';
%              X*H1_A8_tilde_1-W8*H1_C8_tilde_1 2*alpha*X];
% 
% LMI_9 =[2*alpha*X H1_A9_tilde_1'*X-H1_C9_tilde_1'*W9';
%              X*H1_A9_tilde_1-W9*H1_C9_tilde_1 2*alpha*X];
% 
% % Restricciones
% const = [LMI_1 <= 0 LMI_2 <= 0 LMI_3 <= 0 ...
%                LMI_4 <= 0 LMI_5 <= 0 LMI_6 <= 0 ...
%                LMI_7 <= 0 LMI_8 <= 0 LMI_8 <= 0 ... 
%                X >= 0];
% 
% diagnostics = optimize(const);
% 
% string = yalmiperror(diagnostics.problem);
% if diagnostics.problem == 0
%     disp('Factible!')
% else
%     disp('Infactible!')
%     return
% end
% 
% % Matrices a determinar
% X = double(X);
% 
% W1 = double(W1);
% W2 = double(W2);
% W3 = double(W3);
% W4 = double(W4);
% W5 = double(W5);
% W6 = double(W6);
% W7 = double(W7);
% W8 = double(W8);
% W9 = double(W9);
% 
% W1(:, 1) = [0; 0];
% W2(:, 1) = [0; 0];
% W3(:, 1) = [0; 0];
% W4(:, 1) = [0; 0];
% W5(:, 1) = [0; 0];
% W6(:, 1) = [0; 0];
% W7(:, 1) = [0; 0];
% W8(:, 1) = [0; 0];
% W9 = [0 0 0; 0 0 0];
% 
% H1_L1 = X\W1;
% H1_L2 = X\W2;
% H1_L3 = X\W3;
% H1_L4 = X\W4;
% H1_L5 = X\W5;
% H1_L6 = X\W6;
% H1_L7 = X\W7;
% H1_L8 = X\W8;
% H1_L9 = X\W9;
% 
% H1_K1 = H1_A1_tilde_1 - H1_L1*H1_C1_tilde_1;
% H1_K2 = H1_A2_tilde_1 - H1_L2*H1_C2_tilde_1;
% H1_K3 = H1_A3_tilde_1 - H1_L3*H1_C3_tilde_1;
% H1_K4 = H1_A4_tilde_1 - H1_L4*H1_C4_tilde_1;
% H1_K5 = H1_A5_tilde_1 - H1_L5*H1_C5_tilde_1;
% H1_K6 = H1_A6_tilde_1 - H1_L6*H1_C6_tilde_1;
% H1_K7 = H1_A7_tilde_1 - H1_L7*H1_C7_tilde_1;
% H1_K8 = H1_A8_tilde_1 - H1_L8*H1_C8_tilde_1;
% H1_K9 = H1_A9_tilde_1 - H1_L9*H1_C9_tilde_1;
% 
% H1_L1_ast = H1_L1 + H1_E1_1;
% H1_L2_ast = H1_L2 + H1_E2_1;
% H1_L3_ast = H1_L3 + H1_E3_1;
% H1_L4_ast = H1_L4 + H1_E4_1;
% H1_L5_ast = H1_L5 + H1_E5_1;
% H1_L6_ast = H1_L6 + H1_E6_1;
% H1_L7_ast = H1_L7 + H1_E7_1;
% H1_L8_ast = H1_L8 + H1_E8_1;
% H1_L9_ast = H1_L9 + H1_E9_1;

setlmis([]);
X = lmivar(1, [n 1]);
W1 = lmivar(2, [n p]);
W2 = lmivar(2, [n p]);
W3 = lmivar(2, [n p]);
W4 = lmivar(2, [n p]);
W5 = lmivar(2, [n p]);
W6 = lmivar(2, [n p]);
W7 = lmivar(2, [n p]);
W8 = lmivar(2, [n p]);
W9 = lmivar(2, [n p]);
% alpha = lmivar(1, [n 0]);
alpha = -1;

% LMI #1: X > 0
lmiterm([-1 1 1 X], 1, 1);                  % LMI #1: X

% LMI #2: M < 0
lmiterm([2 1 1 X], 2*alpha, 1);  % LMI #2: 2*alpha*X; −left hand side
lmiterm([2 2 1 W1], -1, H1_C1_tilde_1, 's'); % LMI #2: -W1*H1_C1_tilde_1; −left hand side
lmiterm([2 2 1 X], 1, H1_A1_tilde_1, 's'); % LMI #2: X*H1_A1_tilde_1; −left hand side
lmiterm([2 2 2 X], 2*alpha, 1);  % LMI #2: 2*alpha*X; −left hand side

% LMI #3: M < 0
lmiterm([3 1 1 X], 2*alpha, 1);  % LMI #3: 2*alpha*X; −left hand side
lmiterm([3 2 1 W2], -1, H1_C2_tilde_1, 's'); % LMI #3: -W1*H1_C2_tilde_1; −left hand side
lmiterm([3 2 1 X], 1, H1_A2_tilde_1, 's'); % LMI #3: X*H1_A2_tilde_1; −left hand side
lmiterm([3 2 2 X], 2*alpha, 1);  % LMI #3: 2*alpha*X; −left hand side

% LMI #4: M < 0
lmiterm([4 1 1 X], 2*alpha, 1);  % LMI #4: 2*alpha*X; −left hand side
lmiterm([4 2 1 W3], -1, H1_C3_tilde_1, 's'); % LMI #4: -W1*H1_C3_tilde_1; −left hand side
lmiterm([4 2 1 X], 1, H1_A3_tilde_1, 's'); % LMI #4: X*H1_A3_tilde_1; −left hand side
lmiterm([4 2 2 X], 2*alpha, 1);  % LMI #4: 2*alpha*X; −left hand side

% LMI #5: M < 0
lmiterm([5 1 1 X], 2*alpha, 1);  % LMI #5: 2*alpha*X; −left hand side
lmiterm([5 2 1 W4], -1, H1_C4_tilde_1, 's'); % LMI #5: -W1*H1_C4_tilde_1; −left hand side
lmiterm([5 2 1 X], 1, H1_A4_tilde_1, 's'); % LMI #5: X*H1_A4_tilde_1; −left hand side
lmiterm([5 2 2 X], 2*alpha, 1);  % LMI #5: 2*alpha*X; −left hand side

% LMI #6: M < 0
lmiterm([6 1 1 X], 2*alpha, 1);  % LMI #6: 2*alpha*X; −left hand side
lmiterm([6 2 1 W5], -1, H1_C5_tilde_1, 's'); % LMI #6: -W1*H1_C5_tilde_1; −left hand side
lmiterm([6 2 1 X], 1, H1_A5_tilde_1, 's'); % LMI #6: X*H1_A5_tilde_1; −left hand side
lmiterm([6 2 2 X], 2*alpha, 1);  % LMI #6: 2*alpha*X; −left hand side

% LMI #7: M < 0
lmiterm([7 1 1 X], 2*alpha, 1);  % LMI #7: 2*alpha*X; −left hand side
lmiterm([7 2 1 W6], -1, H1_C6_tilde_1, 's'); % LMI #7: -W1*H1_C6_tilde_1; −left hand side
lmiterm([7 2 1 X], 1, H1_A6_tilde_1, 's'); % LMI #7: X*H1_A6_tilde_1; −left hand side
lmiterm([7 2 2 X], 2*alpha, 1);  % LMI #7: 2*alpha*X; −left hand side

% LMI #8: M < 0
lmiterm([8 1 1 X], 2*alpha, 1);  % LMI #8: 2*alpha*X; −left hand side
lmiterm([8 2 1 W7], -1, H1_C7_tilde_1, 's'); % LMI #8: -W1*H1_C7_tilde_1; −left hand side
lmiterm([8 2 1 X], 1, H1_A7_tilde_1, 's'); % LMI #8: X*H1_A7_tilde_1; −left hand side
lmiterm([8 2 2 X], 2*alpha, 1);  % LMI #8: 2*alpha*X; −left hand side

% LMI #9: M < 0
lmiterm([9 1 1 X], 2*alpha, 1);  % LMI #9: 2*alpha*X; −left hand side
lmiterm([9 2 1 W8], -1, H1_C8_tilde_1, 's'); % LMI #9: -W1*H1_C8_tilde_1; −left hand side
lmiterm([9 2 1 X], 1, H1_A8_tilde_1, 's'); % LMI #9: X*H1_A8_tilde_1; −left hand side
lmiterm([9 2 2 X], 2*alpha, 1);  % LMI #9: 2*alpha*X; −left hand side

% LMI #10: M < 0
lmiterm([10 1 1 X], 2*alpha, 1);  % LMI #10: 2*alpha*X; −left hand side
lmiterm([10 2 1 W9], -1, H1_C9_tilde_1, 's'); % LMI #10: -W1*H1_C9_tilde_1; −left hand side
lmiterm([10 2 1 X], 1, H1_A9_tilde_1, 's'); % LMI #10: X*H1_A9_tilde_1; −left hand side
lmiterm([10 2 2 X], 2*alpha, 1);  % LMI #10: 2*alpha*X; −left hand side

LMIs = getlmis;

[~, xfeas] = feasp(LMIs);

X = dec2mat(LMIs, xfeas, X);
W1 = dec2mat(LMIs, xfeas, W1);
W2 = dec2mat(LMIs, xfeas, W2);
W3 = dec2mat(LMIs, xfeas, W3);
W4 = dec2mat(LMIs, xfeas, W4);
W5 = dec2mat(LMIs, xfeas, W5);
W6 = dec2mat(LMIs, xfeas, W6);
W7 = dec2mat(LMIs, xfeas, W7);
W8 = dec2mat(LMIs, xfeas, W8);
W9 = dec2mat(LMIs, xfeas, W9);

H1_L1 = X\W1;
H1_L2 = X\W2;
H1_L3 = X\W3;
H1_L4 = X\W4;
H1_L5 = X\W5;
H1_L6 = X\W6;
H1_L7 = X\W7;
H1_L8 = X\W8;
H1_L9 = X\W9;

H1_K1 = H1_A1_tilde_1 - H1_L1*H1_C1_tilde_1;
H1_K2 = H1_A2_tilde_1 - H1_L2*H1_C2_tilde_1;
H1_K3 = H1_A3_tilde_1 - H1_L3*H1_C3_tilde_1;
H1_K4 = H1_A4_tilde_1 - H1_L4*H1_C4_tilde_1;
H1_K5 = H1_A5_tilde_1 - H1_L5*H1_C5_tilde_1;
H1_K6 = H1_A6_tilde_1 - H1_L6*H1_C6_tilde_1;
H1_K7 = H1_A7_tilde_1 - H1_L7*H1_C7_tilde_1;
H1_K8 = H1_A8_tilde_1 - H1_L8*H1_C8_tilde_1;
H1_K9 = H1_A9_tilde_1 - H1_L9*H1_C9_tilde_1;

H1_L1_ast = H1_L1 + H1_E1_1;
H1_L2_ast = H1_L2 + H1_E2_1;
H1_L3_ast = H1_L3 + H1_E3_1;
H1_L4_ast = H1_L4 + H1_E4_1;
H1_L5_ast = H1_L5 + H1_E5_1;
H1_L6_ast = H1_L6 + H1_E6_1;
H1_L7_ast = H1_L7 + H1_E7_1;
H1_L8_ast = H1_L8 + H1_E8_1;
H1_L9_ast = H1_L9 + H1_E9_1;

%% Obtengo la matriz de transformación T para cada modelo RNUIO 2
H2_N =[1 0; 0 1; 0 0];

% Modelo 1
H2_D1 = B1d(:, 2);
H2_B1 = B1d;
H2_N1 = H2_N;
H2_T1 = [H2_N1 H2_D1];
H2_A1_bar = H2_T1\A1d*H2_T1;
H2_B1_bar = H2_T1\H2_B1;
H2_B1_bar_1 = H2_B1_bar(1:2, :);
H2_B1_bar_2 = H2_B1_bar(3, :);
H2_D1_bar = H2_T1\H2_D1;
H2_delta1_bar = H2_T1\delta1d;
H2_delta1_bar_1 = H2_delta1_bar(1:2, :);
H2_delta1_bar_2 = H2_delta1_bar(3, :);

% Modelo 2
H2_D2 = B2d(:, 2);
H2_B2 = B2d;
H2_N2 = H2_N;
H2_T2 = [H2_N2 H2_D2];
H2_A2_bar = H2_T2\A2d*H2_T2;
H2_B2_bar = H2_T2\H2_B2;
H2_B2_bar_1 = H2_B2_bar(1:2, :);
H2_B2_bar_2 = H2_B2_bar(3, :);
H2_D2_bar = H2_T2\H2_D2;
H2_delta2_bar = H2_T2\delta2d;
H2_delta2_bar_1 = H2_delta2_bar(1:2, :);
H2_delta2_bar_2 = H2_delta2_bar(3, :);

% Modelo 3
H2_D3 = B3d(:, 2);
H2_B3 = B3d;
H2_N3 = H2_N;
H2_T3 = [H2_N3 H2_D3];
H2_A3_bar = H2_T3\A3d*H2_T3;
H2_B3_bar = H2_T3\H2_B3;
H2_B3_bar_1 = H2_B3_bar(1:2, :);
H2_B3_bar_2 = H2_B3_bar(3, :);
H2_D3_bar = H2_T3\H2_D3;
H2_delta3_bar = H2_T3\delta3d;
H2_delta3_bar_1 = H2_delta3_bar(1:2, :);
H2_delta3_bar_2 = H2_delta3_bar(3, :);

% Modelo 4
H2_D4 = B4d(:, 2);
H2_B4 = B4d;
H2_N4 = H2_N;
H2_T4 = [H2_N4 H2_D4];
H2_A4_bar = H2_T4\A4d*H2_T4;
H2_B4_bar = H2_T4\H2_B4;
H2_B4_bar_1 = H2_B4_bar(1:2, :);
H2_B4_bar_2 = H2_B4_bar(3, :);
H2_D4_bar = H2_T4\H2_D4;
H2_delta4_bar = H2_T4\delta4d;
H2_delta4_bar_1 = H2_delta4_bar(1:2, :);
H2_delta4_bar_2 = H2_delta4_bar(3, :);

% Modelo 5
H2_D5 = B5d(:, 2);
H2_B5 = B5d;
H2_N5 = H2_N;
H2_T5 = [H2_N5 H2_D5];
H2_A5_bar = H2_T5\A5d*H2_T5;
H2_B5_bar = H2_T5\H2_B5;
H2_B5_bar_1 = H2_B5_bar(1:2, :);
H2_B5_bar_2 = H2_B5_bar(3, :);
H2_D5_bar = H2_T5\H2_D5;
H2_delta5_bar = H2_T5\delta5d;
H2_delta5_bar_1 = H2_delta5_bar(1:2, :);
H2_delta5_bar_2 = H2_delta5_bar(3, :);

% Modelo 6
H2_D6 = B6d(:, 2);
H2_B6 = B6d;
H2_N6 = H2_N;
H2_T6 = [H2_N6 H2_D6];
H2_A6_bar = H2_T6\A6d*H2_T6;
H2_B6_bar = H2_T6\H2_B6;
H2_B6_bar_1 = H2_B6_bar(1:2, :);
H2_B6_bar_2 = H2_B6_bar(3, :);
H2_D6_bar = H2_T6\H2_D6;
H2_delta6_bar = H2_T6\delta6d;
H2_delta6_bar_1 = H2_delta6_bar(1:2, :);
H2_delta6_bar_2 = H2_delta6_bar(3, :);

% Modelo 7
H2_D7 = B7d(:, 2);
H2_B7 = B7d;
H2_N7 = H2_N;
H2_T7 = [H2_N7 H2_D7];
H2_A7_bar = H2_T7\A7d*H2_T7;
H2_B7_bar = H2_T7\H2_B7;
H2_B7_bar_1 = H2_B7_bar(1:2, :);
H2_B7_bar_2 = H2_B7_bar(3, :);
H2_D7_bar = H2_T7\H2_D7;
H2_delta7_bar = H2_T7\delta7d;
H2_delta7_bar_1 = H2_delta7_bar(1:2, :);
H2_delta7_bar_2 = H2_delta7_bar(3, :);

% Modelo 8
H2_D8 = B8d(:, 2);
H2_B8 = B8d;
H2_N8 = H2_N;
H2_T8 = [H2_N8 H2_D8];
H2_A8_bar = H2_T8\A8d*H2_T8;
H2_B8_bar = H2_T8\H2_B8;
H2_B8_bar_1 = H2_B8_bar(1:2, :);
H2_B8_bar_2 = H2_B8_bar(3, :);
H2_D8_bar = H2_T8\H2_D8;
H2_delta8_bar = H2_T8\delta8d;
H2_delta8_bar_1 = H2_delta8_bar(1:2, :);
H2_delta8_bar_2 = H2_delta8_bar(3, :);

% Modelo 9
H2_D9 = B9d(:, 2);
H2_B9 = B9d;
H2_N9 = H2_N;
H2_T9 = [H2_N9 H2_D9];
H2_A9_bar = H2_T9\A9d*H2_T9;
H2_B9_bar = H2_T9\H2_B9;
H2_B9_bar_1 = H2_B9_bar(1:2, :);
H2_B9_bar_2 = H2_B9_bar(3, :);
H2_D9_bar = H2_T9\H2_D9;
H2_delta9_bar = H2_T9\delta9d;
H2_delta9_bar_1 = H2_delta9_bar(1:2, :);
H2_delta9_bar_2 = H2_delta9_bar(3, :);

%% Separo el vector de estados no perturbados (n-q) del vector de estados perturbados (q)
% Modelo 1
H2_A1_bar_11 = H2_A1_bar(1:2, 1:2);
H2_A1_bar_12 = H2_A1_bar(1:2, 3);
H2_A1_bar_21 = H2_A1_bar(3, 1:2);
H2_A1_bar_22 = H2_A1_bar(3, 3);

% Modelo 2
H2_A2_bar_11 = H2_A2_bar(1:2, 1:2);
H2_A2_bar_12 = H2_A2_bar(1:2, 3);
H2_A2_bar_21 = H2_A2_bar(3, 1:2);
H2_A2_bar_22 = H2_A2_bar(3, 3);

% Modelo 3
H2_A3_bar_11 = H2_A3_bar(1:2, 1:2);
H2_A3_bar_12 = H2_A3_bar(1:2, 3);
H2_A3_bar_21 = H2_A3_bar(3, 1:2);
H2_A3_bar_22 = H2_A3_bar(3, 3);

% Modelo 4
H2_A4_bar_11 = H2_A4_bar(1:2, 1:2);
H2_A4_bar_12 = H2_A4_bar(1:2, 3);
H2_A4_bar_21 = H2_A4_bar(3, 1:2);
H2_A4_bar_22 = H2_A4_bar(3, 3);

% Modelo 5
H2_A5_bar_11 = H2_A5_bar(1:2, 1:2);
H2_A5_bar_12 = H2_A5_bar(1:2, 3);
H2_A5_bar_21 = H2_A5_bar(3, 1:2);
H2_A5_bar_22 = H2_A5_bar(3, 3);

% Modelo 6
H2_A6_bar_11 = H2_A6_bar(1:2, 1:2);
H2_A6_bar_12 = H2_A6_bar(1:2, 3);
H2_A6_bar_21 = H2_A6_bar(3, 1:2);
H2_A6_bar_22 = H2_A6_bar(3, 3);

% Modelo 7
H2_A7_bar_11 = H2_A7_bar(1:2, 1:2);
H2_A7_bar_12 = H2_A7_bar(1:2, 3);
H2_A7_bar_21 = H2_A7_bar(3, 1:2);
H2_A7_bar_22 = H2_A7_bar(3, 3);

% Modelo 8
H2_A8_bar_11 = H2_A8_bar(1:2, 1:2);
H2_A8_bar_12 = H2_A8_bar(1:2, 3);
H2_A8_bar_21 = H2_A8_bar(3, 1:2);
H2_A8_bar_22 = H2_A8_bar(3, 3);

% Modelo 9
H2_A9_bar_11 = H2_A9_bar(1:2, 1:2);
H2_A9_bar_12 = H2_A9_bar(1:2, 3);
H2_A9_bar_21 = H2_A9_bar(3, 1:2);
H2_A9_bar_22 = H2_A9_bar(3, 3);

%% Armo la matriz no singular U
H2_Q = [0 1; 1 0; 0 0];

% Modelo 1
H2_rango_CD1 = rank(C*H2_D1);
H2_Q1 = H2_Q;
H2_U1 = [C*H2_D1 H2_Q1];
H2_inv_U1 = inv(H2_U1);
H2_U1_1 = H2_inv_U1(1, :);
H2_U1_2 = H2_inv_U1(2:3, :);

% Modelo 2
H2_rango_CD2 = rank(C*H2_D2);
H2_Q2 = H2_Q;
H2_U2 = [C*H2_D2 H2_Q2];
H2_inv_U2 = inv(H2_U2);
H2_U2_1 = H2_inv_U2(1, :);
H2_U2_2 = H2_inv_U2(2:3, :);

% Modelo 3
H2_rango_CD3 = rank(C*H2_D3);
H2_Q3 = H2_Q;
H2_U3 = [C*H2_D3 H2_Q3];
H2_inv_U3 = inv(H2_U3);
H2_U3_1 = H2_inv_U3(1, :);
H2_U3_2 = H2_inv_U3(2:3, :);

% Modelo 4
H2_rango_CD4 = rank(C*H2_D4);
H2_Q4 = H2_Q;
H2_U4 = [C*H2_D4 H2_Q4];
H2_inv_U4 = inv(H2_U4);
H2_U4_1 = H2_inv_U4(1, :);
H2_U4_2 = H2_inv_U4(2:3, :);

% Modelo 5
H2_rango_CD5 = rank(C*H2_D5);
H2_Q5 = H2_Q;
H2_U5 = [C*H2_D5 H2_Q5];
H2_inv_U5 = inv(H2_U5);
H2_U5_1 = H2_inv_U5(1, :);
H2_U5_2 = H2_inv_U5(2:3, :);

% Modelo 6
H2_rango_CD6 = rank(C*H2_D6);
H2_Q6 = H2_Q;
H2_U6 = [C*H2_D6 H2_Q6];
H2_inv_U6 = inv(H2_U6);
H2_U6_1 = H2_inv_U6(1, :);
H2_U6_2 = H2_inv_U6(2:3, :);

% Modelo 7
H2_rango_CD7 = rank(C*H2_D7);
H2_Q7 = H2_Q;
H2_U7 = [C*H2_D7 H2_Q7];
H2_inv_U7 = inv(H2_U7);
H2_U7_1 = H2_inv_U7(1, :);
H2_U7_2 = H2_inv_U7(2:3, :);

% Modelo 8
H2_rango_CD8 = rank(C*H2_D8);
H2_Q8 = H2_Q;
H2_U8 = [C*H2_D8 H2_Q8];
H2_inv_U8 = inv(H2_U8);
H2_U8_1 = H2_inv_U8(1, :);
H2_U8_2 = H2_inv_U8(2:3, :);

% Modelo 9
H2_rango_CD9 = rank(C*H2_D9);
H2_Q9 = H2_Q;
H2_U9 = [C*H2_D9 H2_Q9];
H2_inv_U9 = inv(H2_U9);
H2_U9_1 = H2_inv_U9(1, :);
H2_U9_2 = H2_inv_U9(2:3, :);

%% Obtengo matrices de utilidad y compruebo observabilidad
% Modelo 1
H2_A1_tilde_1 = H2_A1_bar_11 - H2_A1_bar_12*H2_U1_1*C*H2_N1;
H2_C1_tilde_1 = C*H2_N1;
H2_E1_1 = H2_A1_bar_12*H2_U1_1;

H2_rango_C1_tilde_1 = rank(H2_C1_tilde_1);

H2_O_M1 = [H2_C1_tilde_1' (H2_C1_tilde_1*H2_A1_tilde_1)' (H2_C1_tilde_1*H2_A1_tilde_1*H2_A1_tilde_1)']';
H2_rank_Obs_M1 = rank(H2_O_M1);

% Modelo 2
H2_A2_tilde_1 = H2_A2_bar_11 - H2_A2_bar_12*H2_U2_1*C*H2_N2;
H2_C2_tilde_1 = C*H2_N2;
H2_E2_1 = H2_A2_bar_12*H2_U2_1;

H2_rango_C2_tilde_1 = rank(H2_C2_tilde_1);

H2_O_M2 = [H2_C2_tilde_1' (H2_C2_tilde_1*H2_A2_tilde_1)' (H2_C2_tilde_1*H2_A2_tilde_1*H2_A2_tilde_1)']';
H2_rank_Obs_M2 = rank(H2_O_M2);

% Modelo 3
H2_A3_tilde_1 = H2_A3_bar_11 - H2_A3_bar_12*H2_U3_1*C*H2_N3;
H2_C3_tilde_1 = C*H2_N3;
H2_E3_1 = H2_A3_bar_12*H2_U3_1;

H2_rango_C3_tilde_1 = rank(H2_C3_tilde_1);

H2_O_M3 = [H2_C3_tilde_1' (H2_C3_tilde_1*H2_A3_tilde_1)' (H2_C3_tilde_1*H2_A3_tilde_1*H2_A3_tilde_1)']';
H2_rank_Obs_M3 = rank(H2_O_M3);

% Modelo 4
H2_A4_tilde_1 = H2_A4_bar_11 - H2_A4_bar_12*H2_U4_1*C*H2_N4;
H2_C4_tilde_1 = C*H2_N4;
H2_E4_1 = H2_A4_bar_12*H2_U4_1;

H2_rango_C4_tilde_1 = rank(H2_C4_tilde_1);

H2_O_M4 = [H2_C4_tilde_1' (H2_C4_tilde_1*H2_A4_tilde_1)' (H2_C4_tilde_1*H2_A4_tilde_1*H2_A4_tilde_1)']';
H2_rank_Obs_M4 = rank(H2_O_M4);

% Modelo 5
H2_A5_tilde_1 = H2_A5_bar_11 - H2_A5_bar_12*H2_U5_1*C*H2_N5;
H2_C5_tilde_1 = C*H2_N5;
H2_E5_1 = H2_A5_bar_12*H2_U5_1;

H2_rango_C5_tilde_1 = rank(H2_C5_tilde_1);

H2_O_M5 = [H2_C5_tilde_1' (H2_C5_tilde_1*H2_A5_tilde_1)' (H2_C5_tilde_1*H2_A5_tilde_1*H2_A5_tilde_1)']';
H2_rank_Obs_M5 = rank(H2_O_M5);

% Modelo 6
H2_A6_tilde_1 = H2_A6_bar_11 - H2_A6_bar_12*H2_U6_1*C*H2_N6;
H2_C6_tilde_1 = C*H2_N6;
H2_E6_1 = H2_A6_bar_12*H2_U6_1;

H2_rango_C6_tilde_1 = rank(H2_C6_tilde_1);

H2_O_M6 = [H2_C6_tilde_1' (H2_C6_tilde_1*H2_A6_tilde_1)' (H2_C6_tilde_1*H2_A6_tilde_1*H2_A6_tilde_1)']';
H2_rank_Obs_M6 = rank(H2_O_M6);

% Modelo 7
H2_A7_tilde_1 = H2_A7_bar_11 - H2_A7_bar_12*H2_U7_1*C*H2_N7;
H2_C7_tilde_1 = C*H2_N7;
H2_E7_1 = H2_A7_bar_12*H2_U7_1;

H2_rango_C7_tilde_1 = rank(H2_C7_tilde_1);

H2_O_M7 = [H2_C7_tilde_1' (H2_C7_tilde_1*H2_A7_tilde_1)' (H2_C7_tilde_1*H2_A7_tilde_1*H2_A7_tilde_1)']';
H2_rank_Obs_M7 = rank(H2_O_M7);

% Modelo 8
H2_A8_tilde_1 = H2_A8_bar_11 - H2_A8_bar_12*H2_U8_1*C*H2_N8;
H2_C8_tilde_1 = C*H2_N8;
H2_E8_1 = H2_A8_bar_12*H2_U8_1;

H2_rango_C8_tilde_1 = rank(H2_C8_tilde_1);

H2_O_M8 = [H2_C8_tilde_1' (H2_C8_tilde_1*H2_A8_tilde_1)' (H2_C8_tilde_1*H2_A8_tilde_1*H2_A8_tilde_1)']';
H2_rank_Obs_M8 = rank(H2_O_M8);

% Modelo 9
H2_A9_tilde_1 = H2_A9_bar_11 - H2_A9_bar_12*H2_U9_1*C*H2_N9;
H2_C9_tilde_1 = C*H2_N9;
H2_E9_1 = H2_A9_bar_12*H2_U9_1;

H2_rango_C9_tilde_1 = rank(H2_C9_tilde_1);

H2_O_M9 = [H2_C9_tilde_1' (H2_C9_tilde_1*H2_A9_tilde_1)' (H2_C9_tilde_1*H2_A9_tilde_1*H2_A9_tilde_1)']';
H2_rank_Obs_M9 = rank(H2_O_M9);

%% LMI
% Dimension of system matrices
n = size(H2_A1_tilde_1, 1);    % n = 1
p = size(H2_C1_tilde_1, 1);    % p = 1

setlmis([]);
X = lmivar(1, [n 1]);
W1 = lmivar(2, [n p]);
W2 = lmivar(2, [n p]);
W3 = lmivar(2, [n p]);
W4 = lmivar(2, [n p]);
W5 = lmivar(2, [n p]);
W6 = lmivar(2, [n p]);
W7 = lmivar(2, [n p]);
W8 = lmivar(2, [n p]);
W9 = lmivar(2, [n p]);
%alpha = lmivar(1, [n 0]);
alpha = -1;

% LMI #1: X > 0
lmiterm([-1 1 1 X], 1, 1);  % X

% LMI #2: M < 0
lmiterm([2 1 1 X], 2*alpha, 1);  % LMI #2: 2*alpha*X; −left hand side
lmiterm([2 2 1 W1], -1, H2_C1_tilde_1, 's'); % LMI #2: -W1*H2_C1_tilde_1; −left hand side
lmiterm([2 2 1 X], 1, H2_A1_tilde_1, 's'); % LMI #2: X*H2_A1_tilde_1; −left hand side
lmiterm([2 2 2 X], 2*alpha, 1);  % LMI #2: 2*alpha*X; −left hand side

% LMI #3: M < 0
lmiterm([3 1 1 X], 2*alpha, 1);  % LMI #3: 2*alpha*X; −left hand side
lmiterm([3 2 1 W2], -1, H2_C2_tilde_1, 's'); % LMI #3: -W2*H2_C2_tilde_1; −left hand side
lmiterm([3 2 1 X], 1, H2_A2_tilde_1, 's'); % LMI #3: X*H2_A2_tilde_1; −left hand side
lmiterm([3 2 2 X], 2*alpha, 1);  % LMI #3: 2*alpha*X; −left hand side

% LMI #4: M < 0
lmiterm([4 1 1 X], 2*alpha, 1);  % LMI #4: 2*alpha*X; −left hand side
lmiterm([4 2 1 W3], -1, H2_C3_tilde_1, 's'); % LMI #4: -W3*H2_C3_tilde_1; −left hand side
lmiterm([4 2 1 X], 1, H2_A3_tilde_1, 's'); % LMI #4: X*H2_A3_tilde_1; −left hand side
lmiterm([4 2 2 X], 2*alpha, 1);  % LMI #4: 2*alpha*X; −left hand side

% LMI #5: M < 0
lmiterm([5 1 1 X], 2*alpha, 1);  % LMI #5: 2*alpha*X; −left hand side
lmiterm([5 2 1 W4], -1, H2_C4_tilde_1, 's'); % LMI #5: -W4*H2_C4_tilde_1; −left hand side
lmiterm([5 2 1 X], 1, H2_A4_tilde_1, 's'); % LMI #5: X*H2_A4_tilde_1; −left hand side
lmiterm([5 2 2 X], 2*alpha, 1);  % LMI #5: 2*alpha*X; −left hand side

% LMI #6: M < 0
lmiterm([6 1 1 X], 2*alpha, 1);  % LMI #6: 2*alpha*X; −left hand side
lmiterm([6 2 1 W5], -1, H2_C5_tilde_1, 's'); % LMI #6: -W5*H2_C5_tilde_1; −left hand side
lmiterm([6 2 1 X], 1, H2_A5_tilde_1, 's'); % LMI #6: X*H2_A5_tilde_1; −left hand side
lmiterm([6 2 2 X], 2*alpha, 1);  % LMI #6: 2*alpha*X; −left hand side

% LMI #7: M < 0
lmiterm([7 1 1 X], 2*alpha, 1);  % LMI #7: 2*alpha*X; −left hand side
lmiterm([7 2 1 W6], -1, H2_C6_tilde_1, 's'); % LMI #7: -W6*H2_C6_tilde_1; −left hand side
lmiterm([7 2 1 X], 1, H2_A6_tilde_1, 's'); % LMI #7: X*H2_A6_tilde_1; −left hand side
lmiterm([7 2 2 X], 2*alpha, 1);  % LMI #7: 2*alpha*X; −left hand side

% LMI #8: M < 0
lmiterm([8 1 1 X], 2*alpha, 1);  % LMI #8: 2*alpha*X; −left hand side
lmiterm([8 2 1 W7], -1, H2_C7_tilde_1, 's'); % LMI #8: -W7*H2_C7_tilde_1; −left hand side
lmiterm([8 2 1 X], 1, H2_A7_tilde_1, 's'); % LMI #8: X*H2_A7_tilde_1; −left hand side
lmiterm([8 2 2 X], 2*alpha, 1);  % LMI #8: 2*alpha*X; −left hand side

% LMI #9: M < 0
lmiterm([9 1 1 X], 2*alpha, 1);  % LMI #9: 2*alpha*X; −left hand side
lmiterm([9 2 1 W8], -1, H2_C8_tilde_1, 's'); % LMI #9: -W8*H2_C8_tilde_1; −left hand side
lmiterm([9 2 1 X], 1, H2_A8_tilde_1, 's'); % LMI #9: X*H2_A8_tilde_1; −left hand side
lmiterm([9 2 2 X], 2*alpha, 1);  % LMI #9: 2*alpha*X; −left hand side

% LMI #10: M < 0
lmiterm([10 1 1 X], 2*alpha, 1);  % LMI #10: 2*alpha*X; −left hand side
lmiterm([10 2 1 W9], -1, H2_C9_tilde_1, 's'); % LMI #10: -W9*H2_C9_tilde_1; −left hand side
lmiterm([10 2 1 X], 1, H2_A9_tilde_1, 's'); % LMI #10: X*H2_A9_tilde_1; −left hand side
lmiterm([10 2 2 X], 2*alpha, 1);  % LMI #10: 2*alpha*X; −left hand side

LMIs = getlmis;

[~, xfeas] = feasp(LMIs);

X = dec2mat(LMIs, xfeas, X);
W1 = dec2mat(LMIs, xfeas, W1);
W2 = dec2mat(LMIs, xfeas, W2);
W3 = dec2mat(LMIs, xfeas, W3);
W4 = dec2mat(LMIs, xfeas, W4);
W5 = dec2mat(LMIs, xfeas, W5);
W6 = dec2mat(LMIs, xfeas, W6);
W7 = dec2mat(LMIs, xfeas, W7);
W8 = dec2mat(LMIs, xfeas, W8);
W9 = dec2mat(LMIs, xfeas, W9);

H2_L1 = X\W1;
H2_L2 = X\W2;
H2_L3 = X\W3;
H2_L4 = X\W4;
H2_L5 = X\W5;
H2_L6 = X\W6;
H2_L7 = X\W7;
H2_L8 = X\W8;
H2_L9 = X\W9;

H2_K1 = H2_A1_tilde_1 - H2_L1*H2_C1_tilde_1;
H2_K2 = H2_A2_tilde_1 - H2_L2*H2_C2_tilde_1;
H2_K3 = H2_A3_tilde_1 - H2_L3*H2_C3_tilde_1;
H2_K4 = H2_A4_tilde_1 - H2_L4*H2_C4_tilde_1;
H2_K5 = H2_A5_tilde_1 - H2_L5*H2_C5_tilde_1;
H2_K6 = H2_A6_tilde_1 - H2_L6*H2_C6_tilde_1;
H2_K7 = H2_A7_tilde_1 - H2_L7*H2_C7_tilde_1;
H2_K8 = H2_A8_tilde_1 - H2_L8*H2_C8_tilde_1;
H2_K9 = H2_A9_tilde_1 - H2_L9*H2_C9_tilde_1;

H2_L1_ast = H2_L1 + H2_E1_1;
H2_L2_ast = H2_L2 + H2_E2_1;
H2_L3_ast = H2_L3 + H2_E3_1;
H2_L4_ast = H2_L4 + H2_E4_1;
H2_L5_ast = H2_L5 + H2_E5_1;
H2_L6_ast = H2_L6 + H2_E6_1;
H2_L7_ast = H2_L7 + H2_E7_1;
H2_L8_ast = H2_L8 + H2_E8_1;
H2_L9_ast = H2_L9 + H2_E9_1;