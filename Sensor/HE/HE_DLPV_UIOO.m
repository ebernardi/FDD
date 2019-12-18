%% LMI LPV-UIOO 1
% F1_1 = zeros(size(B1d));
% F1_2 = zeros(size(B2d));
% F1_3 = zeros(size(B3d));
% F1_4 = zeros(size(B4d));
% F1_5 = zeros(size(B5d));
% F1_6 = zeros(size(B6d));
% F1_7 = zeros(size(B7d));
% F1_8 = zeros(size(B8d));
% F1_9 = zeros(size(B9d));

F1_1 = B1d(:, 1);
F1_2 = B2d(:, 1);
F1_3 = B3d(:, 1);
F1_4 = B4d(:, 1);
F1_5 = B5d(:, 1);
F1_6 = B6d(:, 1);
F1_7 = B7d(:, 1);
F1_8 = B8d(:, 1);
F1_9 = B9d(:, 1);

H_1 = zeros(size(C));
H_1(2, :) = C(2, :);
T2_1 = null(H_1, 'r')';
J_1 = T2_1*C;

% J_1 = C;                      % Copy the entire C matrix
% J_1(2, :) = [];              % Delete non-monitored outputs

% Dimension of system matrices
n = size(A1d, 1);    % n = 3
k = size(B1d, 2);    % k = 2
p = size(J_1, 1);      % p = 2

% Variables to be determined
yalmip('clear');
X = sdpvar(n);
S = sdpvar(n, p);

W1 = sdpvar(n, p);
W2 = sdpvar(n, p);
W3 = sdpvar(n, p);
W4 = sdpvar(n, p);
W5 = sdpvar(n, p);
W6 = sdpvar(n, p);
W7 = sdpvar(n, p);
W8 = sdpvar(n, p);
W9 = sdpvar(n, p);

alpha = 0.5;

% LMI 1
LMI_1 = [2*alpha*X, (A1d'*X+A1d'*J_1'*S'-J_1'*W1');
            (X*A1d+S*J_1*A1d-W1*J_1) 2*alpha*X];

% LMI 2
LMI_2 = [2*alpha*X (A2d'*X+A2d'*J_1'*S'-J_1'*W2');
            (X*A2d+S*J_1*A2d-W2*J_1) 2*alpha*X];

% LMI 3
LMI_3 = [2*alpha*X (A3d'*X+A3d'*J_1'*S'-J_1'*W3');
            (X*A3d+S*J_1*A3d-W3*J_1) 2*alpha*X];

% LMI 4
LMI_4 = [2*alpha*X (A4d'*X+A4d'*J_1'*S'-J_1'*W4');
            (X*A4d+S*J_1*A4d-W4*J_1) 2*alpha*X];

% LMI 5
LMI_5 = [2*alpha*X (A5d'*X+A5d'*J_1'*S'-J_1'*W5');
            (X*A5d+S*J_1*A5d-W5*J_1) 2*alpha*X];

% LMI 6
LMI_6 = [2*alpha*X (A6d'*X+A6d'*J_1'*S'-J_1'*W6');
            (X*A6d+S*J_1*A6d-W6*J_1) 2*alpha*X];

% LMI 7
LMI_7 = [2*alpha*X (A7d'*X+A7d'*J_1'*S'-J_1'*W7');
            (X*A7d+S*J_1*A7d-W7*J_1) 2*alpha*X];

% LMI 8
LMI_8 = [2*alpha*X (A8d'*X+A8d'*J_1'*S'-J_1'*W8');
            (X*A8d+S*J_1*A8d-W8*J_1) 2*alpha*X];

% LMI 9
LMI_9 = [2*alpha*X (A9d'*X+A9d'*J_1'*S'-J_1'*W9');
            (X*A9d+S*J_1*A9d-W9*J_1) 2*alpha*X];
        
% Constraints
const = [LMI_1 <= 0, LMI_2 <= 0, LMI_3 <= 0 ...
               LMI_4 <= 0, LMI_5 <= 0, LMI_6 <= 0 ...
               LMI_7 <= 0, LMI_8 <= 0, LMI_9 <= 0 ...
               X >= 0 ...
               (X+S*J_1)*F1_1 == 0, (X+S*J_1)*F1_2 == 0 ...
               (X+S*J_1)*F1_3 == 0, (X+S*J_1)*F1_4 == 0 ...
               (X+S*J_1)*F1_5 == 0, (X+S*J_1)*F1_6 == 0 ...
               (X+S*J_1)*F1_7 == 0, (X+S*J_1)*F1_8 == 0 ...
               (X+S*J_1)*F1_9 == 0 ...
               ];

diagnostics = solvesdp(const);

string = yalmiperror(diagnostics.problem);
if diagnostics.problem == 0
    disp('Factible!')
else
    disp('Infactible!')
    return
end

% Matrices to be determinated
X = double(X);
S = double(S);

W1 = double(W1);
W2 = double(W2);
W3 = double(W3);
W4 = double(W4);
W5 = double(W5);
W6 = double(W6);
W7 = double(W7);
W8 = double(W8);
W9 = double(W9);

E1 = X\S;

T1_1 = (eye(n) + E1*J_1);

K1 = X\W1;
K2 = X\W2;
K3 = X\W3;
K4 = X\W4;
K5 = X\W5;
K6 = X\W6;
K7 = X\W7;
K8 = X\W8;
K9 = X\W9;

G1_1 = T1_1*B1d;
G1_2 = T1_1*B2d;
G1_3 = T1_1*B3d;
G1_4 = T1_1*B4d;
G1_5 = T1_1*B5d;
G1_6 = T1_1*B6d;
G1_7 = T1_1*B7d;
G1_8 = T1_1*B8d;
G1_9 = T1_1*B9d;

Tg1_1 = T1_1*delta1d;
Tg1_2 = T1_1*delta2d;
Tg1_3 = T1_1*delta3d;
Tg1_4 = T1_1*delta4d;
Tg1_5 = T1_1*delta5d;
Tg1_6 = T1_1*delta6d;
Tg1_7 = T1_1*delta7d;
Tg1_8 = T1_1*delta8d;
Tg1_9 = T1_1*delta9d;

N1_1 = T1_1*A1d - K1*J_1;
N1_2 = T1_1*A2d - K2*J_1;
N1_3 = T1_1*A3d - K3*J_1;
N1_4 = T1_1*A4d - K4*J_1;
N1_5 = T1_1*A5d - K5*J_1;
N1_6 = T1_1*A6d - K6*J_1;
N1_7 = T1_1*A7d - K7*J_1;
N1_8 = T1_1*A8d - K8*J_1;
N1_9 = T1_1*A9d - K9*J_1;

L1_1 = K1 - N1_1*E1;
L1_2 = K2 - N1_2*E1;
L1_3 = K3 - N1_3*E1;
L1_4 = K4 - N1_4*E1;
L1_5 = K5 - N1_5*E1;
L1_6 = K6 - N1_6*E1;
L1_7 = K7 - N1_7*E1;
L1_8 = K8 - N1_8*E1;
L1_9 = K9 - N1_9*E1;

%% LMI LPV-UIOO 2
% F2_1 = zeros(size(B1d));
% F2_2 = zeros(size(B2d));
% F2_3 = zeros(size(B3d));
% F2_4 = zeros(size(B4d));
% F2_5 = zeros(size(B5d));
% F2_6 = zeros(size(B6d));
% F2_7 = zeros(size(B7d));
% F2_8 = zeros(size(B8d));
% F2_9 = zeros(size(B9d));

F2_1 = B1d(:, 1);
F2_2 = B2d(:, 1);
F2_3 = B3d(:, 1);
F2_4 = B4d(:, 1);
F2_5 = B5d(:, 1);
F2_6 = B6d(:, 1);
F2_7 = B7d(:, 1);
F2_8 = B8d(:, 1);
F2_9 = B9d(:, 1);

H_2 = zeros(size(C));
H_2(1, :) = C(1, :);
T2_2 = null(H_2, 'r')';
J_2 = T2_2*C;

% J_2 = C;                      % Copy the entire C matrix
% J_2(1, :) = [];              % Delete non-monitored outputs

% Dimension of system matrices
n = size(A1d, 1);    % n = 3
k = size(B1d, 2);    % k = 2
p = size(J_2, 1);         % p = 2

% Variables to be determined
yalmip('clear');
X = sdpvar(n);
S = sdpvar(n, p);

W1 = sdpvar(n, p);
W2 = sdpvar(n, p);
W3 = sdpvar(n, p);
W4 = sdpvar(n, p);
W5 = sdpvar(n, p);
W6 = sdpvar(n, p);
W7 = sdpvar(n, p);
W8 = sdpvar(n, p);
W9 = sdpvar(n, p);

alpha = 0.1;

% LMI 1
LMI_1 = [2*alpha*X, (A1d'*X+A1d'*J_2'*S'-J_2'*W1');
            (X*A1d+S*J_2*A1d-W1*J_2) 2*alpha*X];

% LMI 2
LMI_2 = [2*alpha*X (A2d'*X+A2d'*J_2'*S'-J_2'*W2');
            (X*A2d+S*J_2*A2d-W2*J_2) 2*alpha*X];

% LMI 3
LMI_3 = [2*alpha*X (A3d'*X+A3d'*J_2'*S'-J_2'*W3');
            (X*A3d+S*J_2*A3d-W3*J_2) 2*alpha*X];

% LMI 4
LMI_4 = [2*alpha*X (A4d'*X+A4d'*J_2'*S'-J_2'*W4');
            (X*A4d+S*J_2*A4d-W4*J_2) 2*alpha*X];

% LMI 5
LMI_5 = [2*alpha*X (A5d'*X+A5d'*J_2'*S'-J_2'*W5');
            (X*A5d+S*J_2*A5d-W5*J_2) 2*alpha*X];

% LMI 6
LMI_6 = [2*alpha*X (A6d'*X+A6d'*J_2'*S'-J_2'*W6');
            (X*A6d+S*J_2*A6d-W6*J_2) 2*alpha*X];

% LMI 7
LMI_7 = [2*alpha*X (A7d'*X+A7d'*J_2'*S'-J_2'*W7');
            (X*A7d+S*J_2*A7d-W7*J_2) 2*alpha*X];

% LMI 8
LMI_8 = [2*alpha*X (A8d'*X+A8d'*J_2'*S'-J_2'*W8');
            (X*A8d+S*J_2*A8d-W8*J_2) 2*alpha*X];

% LMI 9
LMI_9 = [2*alpha*X (A9d'*X+A9d'*J_2'*S'-J_2'*W9');
            (X*A9d+S*J_2*A9d-W9*J_2) 2*alpha*X];

% Constraints
const = [LMI_1 <= 0, LMI_2 <= 0, LMI_3 <= 0 ...
               LMI_4 <= 0, LMI_5 <= 0, LMI_6 <= 0 ...
               LMI_7 <= 0, LMI_8 <= 0, LMI_9 <= 0 ...
               X >= 0 ...
               (X+S*J_2)*F2_1 == 0, (X+S*J_2)*F2_2 == 0 ...
               (X+S*J_2)*F2_3 == 0, (X+S*J_2)*F2_4 == 0 ...
               (X+S*J_2)*F2_5 == 0, (X+S*J_2)*F2_6 == 0 ...
               (X+S*J_2)*F2_7 == 0, (X+S*J_2)*F2_8 == 0 ...
               (X+S*J_2)*F2_9 == 0 ...
               ];

diagnostics = optimize(const);

string = yalmiperror(diagnostics.problem);
if diagnostics.problem == 0
    disp('Factible!')
else
    disp('Infactible!')
    return
end

% Matrices to be determinated
X = double(X);

S = double(S);

W1 = double(W1);
W2 = double(W2);
W3 = double(W3);
W4 = double(W4);
W5 = double(W5);
W6 = double(W6);
W7 = double(W7);
W8 = double(W8);
W9 = double(W9);

E2 = X\S;
T1_2 = (eye(n) + E2*J_2);

K1 = X\W1;
K2 = X\W2;
K3 = X\W3;
K4 = X\W4;
K5 = X\W5;
K6 = X\W6;
K7 = X\W7;
K8 = X\W8;
K9 = X\W9;

G2_1 = T1_2*B1d;
G2_2 = T1_2*B2d;
G2_3 = T1_2*B3d;
G2_4 = T1_2*B4d;
G2_5 = T1_2*B5d;
G2_6 = T1_2*B6d;
G2_7 = T1_2*B7d;
G2_8 = T1_2*B8d;
G2_9 = T1_2*B9d;

Tg2_1 = T1_2*delta1d;
Tg2_2 = T1_2*delta2d;
Tg2_3 = T1_2*delta3d;
Tg2_4 = T1_2*delta4d;
Tg2_5 = T1_2*delta5d;
Tg2_6 = T1_2*delta6d;
Tg2_7 = T1_2*delta7d;
Tg2_8 = T1_2*delta8d;
Tg2_9 = T1_2*delta9d;

N2_1 = T1_2*A1d - K1*J_2;
N2_2 = T1_2*A2d - K2*J_2;
N2_3 = T1_2*A3d - K3*J_2;
N2_4 = T1_2*A4d - K4*J_2;
N2_5 = T1_2*A5d - K5*J_2;
N2_6 = T1_2*A6d - K6*J_2;
N2_7 = T1_2*A7d - K7*J_2;
N2_8 = T1_2*A8d - K8*J_2;
N2_9 = T1_2*A9d - K9*J_2;

L2_1 = K1 - N2_1*E2;
L2_2 = K2 - N2_2*E2;
L2_3 = K3 - N2_3*E2;
L2_4 = K4 - N2_4*E2;
L2_5 = K5 - N2_5*E2;
L2_6 = K6 - N2_6*E2;
L2_7 = K7 - N2_7*E2;
L2_8 = K8 - N2_8*E2;
L2_9 = K9 - N2_9*E2;