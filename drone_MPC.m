%% Quadcopter state-space model
clear all
close all
%% constants
m=0.65; % mass kg
g=9.81; 

Ix=7.5E-3; % Inertia o x axis kg*m^2
Iy=7.5E-3; % Inertia o y axis kg*m^2
Iz=1.3E-2; % Inertia o z axis kg*m^2

dx=0; dy=0; dz=0; % disturbances
kx=0.25; ky=0.25; kz=0.25; % air friction

% States
% x=[x y z vx vy vz roll pitch yaw wr wp wy];

% Control
% u=[T tr tp ty];

%% Space-state model
%{
syms x1 x2 x3 x4 x5 x6 x7 x8 x9 x10 x11 x12
syms u1 u2 u3 u4

xdot1=x4;
xdot2=x5;
xdot3=x6;
xdot4=(u1/m)*(cos(x9)*sin(x8)*cos(x7)+sin(x9)*sin(x7))-kx*x4/m+dx/m;
xdot5=(u1/m)*(sin(x9)*sin(x8)*cos(x7)-cos(x9)*sin(x7))-ky*x5/m+dy/m;
xdot6=(u1/m)*cos(x8)*cos(x7)-g-kz*x6/m+dz/m;
xdot7=x10+x11*sin(x7)*tan(x8)+x12*cos(x7)*tan(x8);
xdot8=x11*cos(x7)-x12*sin(x7);
xdot9=sin(x7)*x11/cos(x8)+cos(x7)*x12/cos(x8);
xdot10=(u2/Ix)-((Iy-Iz)/Ix)*x11*x12;
xdot11=(u3/Iy)-((Iz-Ix)/Iy)*x10*x12;
xdot12=(u4/Iz)-((Ix-Iy)/Iz)*x10*x11;


xdot=[xdot1 xdot2 xdot3 xdot4 xdot5 xdot6 xdot7 xdot8 xdot9 xdot10 xdot11 xdot12]';
x=[x1 x2 x3 x4 x5 x6 x7 x8 x9 x10 x11 x12]';
u=[u1 u2 u3 u4];
y=[x1 x2 x3 x7 x8 x9]';

% No se pueden encontrar los puntos de equilibrio -> hay que simplificar el
% modelo
%xdot=subs(xdot,[u1 u2 u3 u4],[m*g 0 0 0])
%[e1 e2 e3 e4 e5 e6 e7 e8 e9 e10 e11 e12]=solve(xdot,[x1 x2 x3 x4 x5 x6 x7 x8 x9 x10 x11 x12])

%}

%% Model simplification
% sin x -> x; cos x -> 1; tan x -> x; no distrurbances
syms x1 x2 x3 x4 x5 x6 x7 x8 x9 x10 x11 x12
syms u1 u2 u3 u4

xdot1=x4;
xdot2=x5;
xdot3=x6;
xdot4=(u1/m)*(x8+x9*x7)-kx*x4/m;
xdot5=(u1/m)*(x9*x8-x7)-ky*x5/m;
xdot6=(u1/m)-g-kz*x6/m;
xdot7=x10+x11*x7*x8+x12*x8;
xdot8=x11-x12*x7;
xdot9=x7*x11+x12;
xdot10=(u2/Ix)-((Iy-Iz)/Ix)*x11*x12;
xdot11=(u3/Iy)-((Iz-Ix)/Iy)*x10*x12;
xdot12=(u4/Iz)-((Ix-Iy)/Iz)*x10*x11;

xdot=[xdot1 xdot2 xdot3 xdot4 xdot5 xdot6 xdot7 xdot8 xdot9 xdot10 xdot11 xdot12].';
x=[x1 x2 x3 x4 x5 x6 x7 x8 x9 x10 x11 x12].';
u=[u1 u2 u3 u4].';
y=[x1 x2 x3 x9].';

[x_size,aux]=size(x);
[y_size,aux]=size(y);
[u_size,aux]=size(u);

%xdot=subs(xdot,[u1 u2 u3 u4],[m*g 0 0 0])
%[e1 e2 e3 e4 e5 e6 e7 e8 e9 e10 e11 e12]=solve(xdot,[x1 x2 x3 x4 x5 x6 x7 x8 x9 x10 x11 x12])
% Equilibrium points
%ue=[m*g 0 0 0].';
%xe=[x1 x2 x3 0 0 0 0 0 0 0 0 0].';

%% Jacobian linearization
% Equilibrium points
ue=[m*g 0 0 0].';
xe=[x1 x2 x3 0 0 0 0 0 0 0 0 0].';


JA=jacobian(xdot,x.');
JB=jacobian(xdot,u.');
JC=jacobian(y, x.');

A=subs(JA,[x,u],[xe,ue]); A=eval(A);
B=subs(JB,[x,u],[xe,ue]); B=eval(B);
C=subs(JC,[x,u],[xe,ue]); C=eval(C);

%% Discrete state-space model
fs=50; % 100 Hz
Ts=1/fs; 
sysc=ss(A,B,C,0);
sysd=c2d(sysc,Ts);

Am=sysd.A;
Bm=sysd.B;
Cm=sysd.C;
Dm=zeros(x_size,1);

%% MPC
%% Initzialization
N_sim=10*fs; %% samples = seconds*frequency

% tunning parameters
Nc=10;  % control horizon
Np=50; % prediction horizon
R = 0.001;   % control weighting

% Init control, reference and output signal
u=zeros(u_size,1);  
y=zeros(y_size,1); 

rx=0*ones(1,N_sim);
ry=0*ones(1,N_sim);
rz=0*ones(1,N_sim);
ryaw=0*ones(1,N_sim);

xm_vector=[];
u_vector=[];
deltau_vector=[];
y_vector=[];

% Init system states
xm=zeros(x_size,1); % states vector
Xf=zeros(x_size+y_size,1); % augmented incremetal state [deltax y]'

%% Get the augmented incremental model and the parameters of the incremental trajectory control
[GG, GF, GR, A, B, C , GD, F, G]=mpcgain_mimo(Am,Bm,Cm,Nc,Np,Dm); 

%% Constant part of the incremental trajectory control
f1=GG+R*eye(Nc*u_size,Nc*u_size); % E for the cost function J=xEx'+x'F

%% Constrain matrix M
ymax=[0.8 1 2];  % output max limits (1 x n_y)
ymin=[-0.3 -1 -1.5];  % output min limits (1 x n_y)

% hay que probar
deltaumax=500;
deltaumin=-500;

% para u negativa hay que invertir el motor que aumenta o reduce para
% cambiar el sentido de giro
% u1 hay que limitarlo (dividir /2) porque controla 4 motores en vez de 2
umax=500;
umin=-500;
% ymax;ymin for output number 'n' where G(n,:); -G(n,:); 
%M_output=[G(1,:); -G(1,:); % y1
%            G(3,:); -G(3,:)]; % y3
M_output=[];
[gm,gn]=size(G);
aux=eye(gn);
M_deltau=[aux(1,:);-aux(1,:);
    aux(2,:);-aux(2,:);
    aux(3,:);-aux(3,:);
    aux(4,:);-aux(4,:)];

M_u=[aux(1,:);-aux(1,:);
    aux(2,:);-aux(2,:);
    aux(3,:);-aux(3,:);
    aux(4,:);-aux(4,:)];

M=[M_output; M_deltau; M_u];

%% loop
for k=1:N_sim;  
    %% Reference
    rx(k)=sin(0.05*k);    
    ry(k)=cos(0.05*k);       
    rz(k)=0.01*k;    
    ryaw(k)=45*pi/180;
    
    r=[rx; ry; rz; ryaw];
    
    %% Calculate DeltaU
    % Get the second part of DeltaU    
    f2=GR*r(:,k)-GF*Xf; % F for the cost function J=xEx'+x'F
    DeltaU=inv(f1)*f2; % Get DeltaU without constrains
    
    %% Calculate DeltaU with constrains
    % output number 'n' where ymax(n) -ymin(n) -F(n,:) F(n,:)
    %gamma_output=[[ymax(1);-ymin(1)]+[-F(1,:); F(1,:)]*Xf; % y1
    %              [ymax(3);-ymin(3)]+[-F(3,:); F(3,:)]*Xf;]; % y3
    gamma_output=[];
    gamma_deltau=[deltaumax;-deltaumin;
        deltaumax;-deltaumin;
        deltaumax;-deltaumin;
        deltaumax;-deltaumin];
    gamma_u=[umax-u(1);0+u(1);
        umax-u(2);-umin+u(2);
        umax-u(3);-umin+u(3);
        umax-u(4);-umin+u(4)];
    
    gamma=[gamma_output; gamma_deltau; gamma_u];
    
    %DeltaU=optim(f1,-f2,M,gamma,DeltaU); %% Get DeltaU with constrains (Hildreth's algorithm)
    
    %% Calculate u
    deltau=DeltaU(1:4);  % Apply the receding control horizon (take only the first element)
    u=u+deltau; % (u(k)=u(k-1)+deltau(k))
    u=u-[m*g;0;0;0]; % equilibrium point
    %u=[0;0;0;0]; % open-loop simulation
    %% Get the incremental states vector prediction     
    xm_old=xm; 
    xm=Am*xm+Bm*u; 
    y=Cm*xm;    
    %y=y+rand()*[0.1;0.2;0.2;0.1]; % distrurbances noise 
    Xf=[xm-xm_old;y]; 
    
    %% add them to vectors
    xm_vector=[xm_vector xm];
    y_vector=[y_vector y];
    u_vector=[u_vector u];
    deltau_vector=[deltau_vector deltau];
  
end

%% Plot
p=0:N_sim-1;
% output and reference
figure
for i=1:4 
    subplot(4,1,i)
    plot(p,y_vector(i,:),'LineWidth', 2);
    hold on
    grid on
    plot(p,r(i,:));
    
end
subplot(4,1,1); xlabel('x (m)');
subplot(4,1,2); xlabel('y (m)');
subplot(4,1,3); xlabel('z (m)');
subplot(4,1,4); xlabel('yaw (rad)');
% u
figure
for i=1:4
    subplot(4,1,i)
    plot(p,u_vector(i,:),'LineWidth', 2);
    xlabel('u'+string(i));
    grid on
end

% deltau
figure
for i=1:4
    subplot(4,1,i)
    plot(p,deltau_vector(i,:),'LineWidth', 2);
    xlabel('deltau'+string(i));
    grid on
end

% xm
figure
for i=1:12
    subplot(4,3,i)
    plot(p,xm_vector(i,:));
    hold on
    grid on
    xlabel('x'+string(i));    
end

figure
plot3(y_vector(1,:),y_vector(2,:),y_vector(3,:),'LineWidth', 2)
hold on
grid on
plot3(r(1,:),r(2,:),r(3,:))
xlabel('x (m)');ylabel('y (m)');zlabel('z (m)');