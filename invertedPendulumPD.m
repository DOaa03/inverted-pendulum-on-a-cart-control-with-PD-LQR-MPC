%this script describes the controls inverted pendulum motion using PD
%control
%initializing parameters
m=1; %pendulum mass
M=5; %cart mass
L=2; %pendulum length
g=10; %gravity acceleration
d=1; %damping coefficient
A = [0,0,1,0;
         0,(-d/M),(-m*g/M),0;
         0,0,0,1;
         0,(-d/M*L),(-g*(m+M)/(M*L)),0];
B = [0; 1/M; 0; 1/(M*L)];

%PD gaind
kp_x = -200;
kd_x = 90;
kp_theta = 200;
kd_theta = 20;

%initial state
% y=[ x, x dot, theta,theta dot];
dt = 0.1;
t = 0 : dt : 20;
x0 = [0; 0; 15*pi/180; 3]; % 初期値
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
    theta= x(3);
    cy=cos(theta);
    sy=sin(theta);
    D=m*L*L*(M+m*(1-cy^2));

    x_error =  - x(1);
    x__dot_error = - x(2);
    theta_error =  - theta;
    theta__dot_error = - x(4);

    u_x= kp_x * x_error + kd_x * x__dot_error;
    u_theta= kp_theta * theta_error + kd_theta * theta__dot_error;
    u =  u_x + u_theta ;
    
    s_x1 = [s_x1 x(1)];
    s_x2 = [s_x2 x(2)];
    s_x3 = [s_x3 x(3)];
    s_x4 = [s_x4 x(4)];
    s_u = [s_u u];

end


t1='PD controller';
y=[s_x1',s_x2',s_x3',s_x4'];
drawIP(y,L,t1,s_u,dt)





