clc;
clear all;
close all;

%% Initialize Quad
%Quadorotor parameters
global Ub Cb Jxx Jyy Jzz J g told

Ub = 12; %V
Cb = 5; %A*sec*60
Jxx = 1; %??
Jyy = 1;
Jzz = 1;
J = [Jxx 0 0; 0 Jyy 0; 0 0 Jzz];
g = 9.81; %m/sec^2
told = 0;

%Coefficients
global Dp Hp Bp lambda_c sigma A K0 epsilon alpha0 Cfd rho0 h Tt e R CR

Dp = 10*0.0254; %in->m
Hp = 4.5*.0254; %in->m
Bp = 2; %no of blades
lambda_c = .75; %unitless
sigma = .5; %unitless
A = 5; %unitless 
K0 = 6.11; %unitless
epsilon = .85; %unitless
alpha0 = 0; %unitless
Cfd = .015; %unitless

rho0 = 1.293; %kg/m^3
h = 10; %m
Tt = 25; %degC

e = .83; %unitless

%R = (Dp/2)/sin(pi/4); %units of Dp(m)
R = .1796;

%Calculate coefficients
global Pa rho Ct Cm Cd cT cM

Pa = (101325*(1-((0.0065*h)/(273+Tt))).^5.2561); %Pa
rho = (rho0*273*Pa)/(101325*(273+Tt)); %kg/m^3

Ct = .25*((pi).^3)*(lambda_c)*(sigma)*Bp*K0*((epsilon*(atan(Hp/(pi*Dp)))-alpha0)/(pi*A + K0));
Cd = Cfd + (pi*A*(K0.^2)/e)*((epsilon*(atan(Hp/(pi*Dp)))-alpha0)/((pi*A + K0).^2));
Cm = (1/8*A)*(pi.^2)*Cd*(sigma.^2)*lambda_c*(Bp.^2);

%cT = ((1/4)*pi.^2)*(rho*(Dp.^4)*Ct); %change
cT = .0024;
%cM = ((1/4)*pi.^2)*(rho*(Dp.^5)*Cm); %change
cM = .0028;
%CR = Cb*Ub; %V*A*sec*60
CR = 60;
%% Initialize values for controller
global e_pz eint_px eint_py eint_pz

e_pz = 0;
eint_px = 0;
eint_py = 0;
eint_pz = 0;

%% Set throttle and initial values
%set throttle between 0 and 1
global throttle
throttle = [.653555;.653555;.653555;.653555]; %throttle@hover ~ .653555

%Initial values
xinit = 0;
yinit = 0;
zinit = -10;
xdotinit = 0;
ydotinit = 0;
zdotinit = 0;
phiinit = 0;
thetainit = 0;
psiinit = 0;
pinit = 0;
qinit = 0;
rinit = 0;

%% Integrate thrust over stvec

tspan = [0 10];

stateinit = [xinit yinit zinit xdotinit ydotinit zdotinit phiinit thetainit psiinit pinit qinit rinit];

[t, stvec] = ode45('odefunction', tspan, stateinit);

%% Prepare for plotting

%check for 'crash'
k=length(stvec);
for i=2:length(stvec)
    if stvec(i,3)>=0
        k=i;
        break;
    end
end

if k ~= length(stvec)
    disp('CRASH!')
end

x = stvec(1:k,1);
y = stvec(1:k,2);
z = stvec(1:k,3);
xdot = stvec(1:k,4);
ydot = stvec(1:k,5);
zdot = -stvec(1:k,6);
phi = stvec(1:k,7);
theta = stvec(1:k,8);
psi = stvec(1:k,9);
p = stvec(1:k,10);
q = stvec(1:k,11);
r = stvec(1:k,12);
t = t(1:k);

%% Plot linear position and velocity
figure (1)

subplot(6,1,1)
plot(t,x)
grid on;
xlabel('t')
ylabel('x')

subplot(6,1,2)
plot(t,y)
grid on;
xlabel('t')
ylabel('y')

subplot(6,1,3)
plot(t,z)
set (gca, 'ydir', 'reverse');
grid on;
xlabel('t');
ylabel('z');

subplot(6,1,4)
plot(t,xdot)
grid on;
xlabel('t')
ylabel('xdot')

subplot(6,1,5)
plot(t,ydot)
grid on;
xlabel('t')
ylabel('ydot')

subplot(6,1,6)
plot(t,zdot)
grid on;
xlabel('t');
ylabel('zdot');

%% Plot angular position and velocities
figure (2)
title('angular positions and velocities')

subplot(6,1,1)
plot(t,phi)
grid on;
xlabel('t')
ylabel('phi')

subplot(6,1,2)
plot(t,theta)
grid on;
xlabel('t')
ylabel('theta')

subplot(6,1,3)
plot(t,psi)
grid on;
xlabel('t')
ylabel('psi')

subplot(6,1,4)
plot(t,p)
grid on;
xlabel('t')
ylabel('p')

subplot(6,1,5)
plot(t,q)
grid on;
xlabel('t')
ylabel('q')

subplot(6,1,6)
plot(t,r)
grid on;
xlabel('t')
ylabel('r')

%% Plot Quad
figure(3)
plot3(x,y,z)
set (gca, 'zdir', 'reverse');
grid on;
xlabel('x')
ylabel('y')
zlabel('z')

