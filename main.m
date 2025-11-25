clc;
clear all;
close all;
%% inital parameter
% Masses of the first and second pendulum links.
m1 = 0.2;  
m2 = 0.2;
% Lengths of the first and second pendulum links.
r1=0.2;
r2=0.2;
% Initial angles of the first and second pendulum links.
theta10 = 0;
theta20 = 0;
% Spring constants for rotational springs.
ktheta1 = 1;
ktheta2 = 1;
%  Damping coefficients for rotational damping.
Dtheta1 = 0.06;
Dtheta2 = 0.06;
%  Moments of inertia for the first and second pendulum links.
jtheta1 = 1;
jtheta2 = 1;
% Acceleration due to gravity.
g = 9.8;


syms theta1 theta1_d theta1_dd theta2...
    theta2_d theta2_dd u ...
    x1 x1_d x2 x2_d x3 x3_d x4 x4_d s ...
    x__01 x__02 x__03 x__04

eqns1 = [((m1+m2)*r1^2+jtheta1)*theta1_dd+m2*r1*r2*theta2_dd*(cos(theta1-theta2))+m2*r1*r2*theta2_d^2*(sin(theta1-theta2))+(theta1-theta10)/ktheta1+Dtheta1*theta1_d+(m1+m2)*g*r1*sin(theta1)== 0
        (m2*r2^2+jtheta2)*theta2_dd+m2*r1*r2*theta1_dd*cos(theta1-theta2)-m2*r1*r2*theta1_d^2*sin(theta1-theta2)+(theta2-theta20)/ktheta2+Dtheta2*theta2_d+m2*g*r2*sin(theta2)== 0];
    

S1 = solve(eqns1,[theta1_dd  theta2_dd]);

theta1_dd=S1.theta1_dd-16.01*u;
theta2_dd=S1.theta2_dd+38.84*u;

%% Linearization around thetha1 , theta2 = 0

X = [theta1 theta1_d theta2 theta2_d];

f = [theta1_d;theta1_dd;theta2_d;theta2_dd];

As = jacobian(f,X);
A = subs(As, [X,u],[0.000001 0 0 0 0]);
A1 = double(A);
A1
Bs = jacobian(f,u);
B = subs(Bs, [X,u],[0.000001 0 0 0 0]);
B1 = double(B);
B1
C = [0  0 1 0];

D = 0;

eigen_vals1 = eig(A1);
%%jordan 
[V1 , J1] = jordan(A1);
[V1 , J1] = cdf2rdf(V1,J1);
V1
Aj = inv(V1)*A1*V1;
Bj = inv(V1)*B1;
Cj = C*V1;
Aj

% transfer func;
sys1 = ss(A1,B1,C,D);
g1 = tf(sys1);

figure(1)
rlocus(g1);

% Marix phi(t)

phi_s1 = inv(s*eye(4)-A1);
phi_s1 = simplify(phi_s1);


% step 
figure(3);
step(g1);
stepinfo(g1)


% Zero Input & Zero State
U = [1/s 1/s 1/s 1/s];
X0 = [0; 0.5; 0; 1];

XS = phi_s1 * X0 + phi_s1 * B * U;
YS = C * phi_s1 * X0 + C * phi_s1 * B * U + D * U;

XT = ilaplace(XS);
XT = vpa(simplify(XT))


YT = ilaplace(YS);
YT = vpa(simplify(YT))


% freq without response
inv(V1)*[0;0;0;1]
eqns_freq = [ inv(V1) * [x__01 ;x__02; x__03 ;x__04] == [0 0 0 0]];

freq = solve(eqns_freq,[x__01 x__02 x__03 x__04]);



%% Linearization around thetha1 , theta2 = pi

X = [theta1 theta1_d theta2 theta2_d];

f = [theta1_d;theta1_dd;theta2_d;theta2_dd];

As = jacobian(f,X);
A = subs(As, [X,u],[pi+0.000001 0 pi 0 0]);
A2 = double(A);
A2
Bs = jacobian(f,u);
B = subs(Bs, [X,u],[pi+0.000001 0 pi 0 0]);
B2 = double(B);
B2
C = [0  0 1 0];

D = 0;

eigen_vals2 = eig(A2);
%%jordan 
[V2 , J2] = jordan(A2);
[V2 , J2] = cdf2rdf(V2,J2);

Ajpi = inv(V2)*A2*V2;
Bjpi = inv(V2)*B2;
Cjpi = C*V2;

sys2 = ss(A2,B2,C,D);
g2 = tf(sys2);

figure(3)
rlocus(g2);

phi_s2 = inv(s*eye(4)-A2);
phi_s2 = simplify(phi_s2);




% syms x0_1 x0_2 x0_3 x0_4 x0_5 x0_6 x0_7 x0_8
syms x1 x1_d x2 x2_d x3 x3_d x4 x4_d s
Aj=[-0.0297    1.1746   -0.0000   -0.0000;
   -1.1746   -0.0297         0    0.0000;
   -0.0000   -0.0000   -0.0295    1.3250;
    0.0000    0.0000   -1.3250   -0.0295;];

T=[0.0007   -0.0246    1.3894   20.2767;
    0.0289    0.0015  -26.9070    1.2418;
   -0.0215   -0.8508   -0.0168   -0.7544;
    1.0000         0    1.0000         0;];
T;
x0=[x1; x2; x3; x4;];

% step 
figure(4);
step(g2);
stepinfo(g2)

C_AB = [B, A*B, A^2 * B, A^3 * B, A^4 * B];
r_C_AB = rank(C_AB)

O_AC = [C; C*A; C * A^2; C * A^3; C * A^4;];
r_O_AC = rank(C_AB)


% kallmann

x1_d = Aj(1,1) * x1 + Aj(1,2) * x2 + Bj(1) * u;
x2_d = Aj(2,1) * x1 + Aj(2,2) * x2 + Bj(2) * u;
x3_d = Aj(3,3) * x3 + Aj(3,4) * x4 + Bj(3) * u;
x4_d = Aj(4,3) * x3 + Aj(4,4) * x4 + Bj(4) * u;

X_dot = [x1_d;x2_d;x3_d;x4_d;] ;
pretty(X_dot);
