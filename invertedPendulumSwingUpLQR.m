% this section controls the system LQR
clear all 
close all
clc
%initializing parameters
m=1; %pendulum mass
M=5; %cart mass
L=2; %pendulum length
g=10; %gravity acceleration
d=3; %damping coefficient


A = [ 0,          1,                      0,                      0;
      0,      -d/M,                 (m*g)/M,                  0;
      0,          0,                      0,                      1;
      0,   d/(M*L),    -((M+m)*g)/(M*L),               0 ];

B = [ 0;
      1/M;
      0;
     -1/(M*L) ];   % note the negative sign on the last entry

Q=[100,0,0,0
    0,1,0,0
    0,0,100,0
    0,0,0,1000];
R=1;
K=lqr(A,B,Q,R);

%linearizd control using state-space representation 
dt = 0.03;
t = 0 : dt : 15;
x0 = [0; 0; 20*pi/180; 0]; % 初期値
u = 0; % 入力の初期値
x = x0;

s_x1 = [];
s_x2 = [];
s_x3 = [];
s_x4 = [];
s_u = [];
for n = t
    % switching controller
    if abs(x(3)) < (15*pi/180) 
        u = -K*x;
        dx = A * x + B * u;
        x = x + dx * dt;
   
    else
        % Swing-up:using energy law
        E = 0.5 * m * L^2 * x(4)^2 - m * g * L * cos(x(3));
        E_up = m * g * L;
        ke = 1; %gain can be manipulated depending on response

        u = -ke * sign((E - E_up) * x(4) * cos(x(3)));

        %non-linear system because angle is large
        dx(1,1) = x(2);
        dx(4,1) = (g * sin(x(3)) + cos(x(3)) * ((-u - m * L * x(4)^2 * sin(x(3)) + d * x(2)) / (M + m))) / ...
            (L * (4/3 - (m * cos(x(3))^2) / (M + m)));
        dx(2,1)= (u + m * L * (x(4)^2 * sin(x(3)) - dx(4,1) * cos(x(3))) - d * x(2)) / (M + m);
        dx(3,1) = x(4);
        
        dx= [dx(1,1); dx(2,1); dx(3,1); dx(4,1)];
        x = x + dx * dt;
    end
    %wrap angles between +-180
    x(3) = atan2(sin(x(3)), cos(x(3))); 
   
    
    s_x1 = [s_x1 x(1)];
    s_x2 = [s_x2 x(2)];
    s_x3 = [s_x3 x(3)];
    s_x4 = [s_x4 x(4)];
    s_u = [s_u u];

end

T1='Swing up wirh LQR control';
y=[s_x1',s_x2',s_x3',s_x4'];
draw4(y,L,T1,s_u,dt)

    

   
