clear; close all; clc;

% parameters
m = 1;    % pendulum mass
M = 5;    % cart mass
L = 2;    % pendulum length
g = 10;   % gravity
d = 4;    % cart viscous damping

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
Prediction_horizon = 400;
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
initial_state = [0; 0; 20*pi/180; 0]; % {x, x_dot, theta, theta_dot}
state = mpcstate(mpc_obj);
state.Plant = initial_state;

t = 0:ts:15;
N = length(t);
y = zeros(N,4);
y(1,:) = initial_state';
u = zeros(N,1);
r = zeros(4,1);

for k=1:N
    % switching controller
    y(k,:) = state.Plant.';
    if abs(y(k,3)) < (15*pi/180) 
        y(k,:) = state.Plant.';       % log current plant state (row)
        u(k) = mpcmove(mpc_obj, state, y(k,:).', r);  % column vector expected
        % simulate plant one step
        state.Plant = planted.A * state.Plant + planted.B * u(k);
   
    else
        % Non-linear System / Swing-up Control
        
        % Calculate energy
        E = 0.5 * m * L^2 * y(k,4)^2 - m * g * L * cos(y(k,3));
        E_up = m * g * L;
        ke = 10; 

        % Calculate swing-up control input and save it to the history array
        u_val = -ke * sign((E - E_up) * y(k,4) * cos(y(k,3)));
        
        % Clamp the input to the motor torque limits
        u(k) = max(min(u_val, mpc_obj.MV.Max), mpc_obj.MV.Min); 

        % --- Non-linear simulation of one step (using u(k)) ---
        % Current state for convenience
        x_curr = y(k,1);
        x_dot_curr = y(k,2);
        theta_curr = y(k,3);
        theta_dot_curr = y(k,4);
        
        % Right-hand side of the differential equations (dx)
        % This is the continuous-time dx/dt
        denominator = (L * (4/3 - (m * cos(theta_curr)^2) / (M + m)));
        
        dx_dot = (g * sin(theta_curr) + cos(theta_curr) * ((-u(k) - m * L * theta_dot_curr^2 * sin(theta_curr) + d * x_dot_curr) / (M + m))) / denominator;
        
        ddx = (u(k) + m * L * (theta_dot_curr^2 * sin(theta_curr) - dx_dot * cos(theta_curr)) - d * x_dot_curr) / (M + m);
        
        dx = [x_dot_curr; ddx; theta_dot_curr; dx_dot];

        % Euler integration to get the next state
        next_state = state.Plant + dx * ts;
        
        % Update the plant state for the next iteration
        state.Plant = next_state;
    end
    
    % The angle wrapping should use the updated state.Plant
    state.Plant(3) = atan2(sin(state.Plant(3)), cos(state.Plant(3))); 
    
end


Title='MPC';
drawIP(y,L,Title,u,ts)
