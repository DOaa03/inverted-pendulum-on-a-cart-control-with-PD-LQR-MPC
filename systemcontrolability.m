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
     -1/(M*L) ];  

%checking controlability

controliability_matrix=ctrb(A,B);
Rank=rank(controliability_matrix);

if Rank== length(A)
    disp('this system is controllable')
else
    disp('this uncontrollable')
end
