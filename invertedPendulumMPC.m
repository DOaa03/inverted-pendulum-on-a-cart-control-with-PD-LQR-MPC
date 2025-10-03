clear; close all; clc;

% parameters
m = 1;    % pendulum mass
M = 5;    % cart mass
L = 2;    % pendulum length
g = 10;   % gravity
d = 1;    % cart viscous damping

% State order: [ x; x_dot; theta; theta_dot ]
% Continuous linearized dynamics (small angle)
A = [ 0,          1,                      0,                      0;
      0,      -d/M,                 (m*g)/M,                  0;
      0,          0,                      0,                      1;
      0,   d/(M*L),    -((M+m)*g)/(M*L),               0 ];

B = [ 0;
      1/M;
      0;
     -1/(M*L) ];   % note the negative sign on the last entry

C = eye(4);
D = zeros(4,1);

ts = 0.005; % sampling time
plant = ss(A,B,C,D);
planted = c2d(plant,ts,'zoh');

% MPC setup (same structure as yours)
Prediction_horizon = 300;
Control_horizon = 20;
mpc_obj = mpc(planted,ts,Prediction_horizon, Control_horizon);

mpc_obj.MV.Min = -15;
mpc_obj.MV.Max = 15;
mpc_obj.Model.Plant.InputName = 'Motor Torque';
mpc_obj.Model.Plant.InputUnit = 'Nm';

mpc_obj.Model.Nominal.X = [0;0;0;0];
mpc_obj.Model.Nominal.Y = [0;0;0;0];
mpc_obj.Model.Nominal.U = 0;
mpc_obj.Model.Nominal.DX = [0;0;0;0];

mpc_obj.Model.Plant.OutputName = {'x','x_dot','theta','theta_dot'};
mpc_obj.Model.Plant.OutputUnit = {'m','m/s','rad','rad/s'};

% constraints on theta (small angle)
mpc_obj.OutputVariables(3).Min = -0.05;
mpc_obj.OutputVariables(3).Max =  0.05;
mpc_obj.OutputVariables(3).MinECR = 1;
mpc_obj.OutputVariables(3).MaxECR = 1;

% weights
mpc_obj.Weights.OutputVariables = [50 1 500 5];
mpc_obj.Weights.MV = 0.2;
mpc_obj.Weights.MVRate = 0.05;

% initial
initial_state = [3; 0; 0.01; 0]; % {x, x_dot, theta, theta_dot}
state = mpcstate(mpc_obj);
state.Plant = initial_state;

t = 0:ts:10;
N = length(t);
y = zeros(N,4);
u = zeros(N,1);
r = zeros(4,1);

for k=1:N
    y(k,:) = state.Plant.';       % log current plant state (row)
    u(k) = mpcmove(mpc_obj, state, y(k,:).', r);  % column vector expected
    % simulate plant one step
    state.Plant = planted.A * state.Plant + planted.B * u(k);
end



Title='MPC';
draw4(y,L,Title,u,ts)
