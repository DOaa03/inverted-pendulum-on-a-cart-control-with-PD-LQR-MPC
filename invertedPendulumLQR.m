%% %% this section controls the system LQR
clear all 
close all
clc
%initializing parameters
m=1; %pendulum mass
M=5; %cart mass
L=2; %pendulum length
g=10; %gravity acceleration
d=1; %damping coefficient


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
dt = 0.3;
t = 0 : dt : 10;
x0 = [1; 0; 15*pi/180; 0.05]; % 初期値
u = 0; % 入力の初期値
x = x0;

s_x1 = [];
s_x2 = [];
s_x3 = [];
s_x4 = [];
s_u = [];
for n = t
    dx = A * x + B * u;
    x = x + dx * dt;
    u = -K*x;
    
    s_x1 = [s_x1 x(1)];
    s_x2 = [s_x2 x(2)];
    s_x3 = [s_x3 x(3)];
    s_x4 = [s_x4 x(4)];
    s_u = [s_u u];

end

T1='LQR control';
y=[s_x1',s_x2',s_x3',s_x4'];
draw4(y,L,T1,s_u,dt)