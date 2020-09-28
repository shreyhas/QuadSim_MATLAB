function dzdt = odefunction(t,stvec)
%% Initialize variables

global Ub Cb Tm s varpib Jxx Jyy Jzz J g
global Pa rho Ct Cm Cd cT cM R CR
global throttle 



x = stvec(1);
y = stvec(2);
z = stvec(3);
xdot = stvec(4);
ydot = stvec(5);
zdot = stvec(6);
phi = stvec(7);
theta = stvec(8);
psi = stvec(9);
p = stvec(10);
q = stvec(11);
r = stvec(12);


%% Calculate/Modulate thrust and torque
global m

m = 1.5;

%throttle = [.653555;.653555;.653555;.653555] + .1*sin(t); % change 653555

stvec_des = [0 0 -10 0 0 0 0 0 1 0 0 0];

throttle = PIDController3(stvec, stvec_des,t);
%ftau = PIDController3(stvec, stvec_des,t);

varpi1 = (CR*throttle(1));
varpi2 = (CR*throttle(2));
varpi3 = (CR*throttle(3));
varpi4 = (CR*throttle(4));


M4 = [cT cT cT cT;
     (sqrt(2)/2)*R*cT -(sqrt(2)/2)*R*cT -(sqrt(2)/2)*R*cT (sqrt(2)/2)*R*cT;
     (sqrt(2)/2)*R*cT (sqrt(2)/2)*R*cT -(sqrt(2)/2)*R*cT -(sqrt(2)/2)*R*cT;
      cM -cM cM -cM];
  
ftau = M4*[varpi1^2;varpi2^2;varpi3^2;varpi4^2];
f = ftau(1);
taux = ftau(2);
tauy = ftau(3);
tauz = ftau(4);

%{
f = cT*varpi1^2 + cT*varpi2^2 + cT*varpi3^2 + cT*varpi4^2; % kg*m/sec^2; max ~ 34.5 or 2.3*m*g

taux = R*cT*((sqrt(2)/2)*varpi1^2 - (sqrt(2)/2)*varpi2^2 - (sqrt(2)/2)*varpi3^2 + (sqrt(2)/2)*varpi4^2);
tauy = R*cT((sqrt(2)/2)*varpi1^2 + (sqrt(2)/2)*varpi2^2 - (sqrt(2)/2)*varpi3^2 - (sqrt(2)/2)*varpi4^2);
tauz = cM*varpi1^2 - cM*varpi2^2 + cM*varpi3^2 - cM*varpi4^2;
%}
tau = [taux;tauy;tauz]; % kg*m^2/sec^2 R

Ga = [0;0;0]; %??


%% Equations of Motion

%linear velocities
x = xdot; %m/sec
y = ydot; %m/sec
z = zdot; %m/sec

%linear accelerations
Rot = [cos(theta)*cos(psi) cos(psi)*sin(theta)*sin(phi)-sin(psi)*cos(phi) cos(psi)*sin(theta)*cos(phi)+sin(psi)*sin(phi);
       cos(theta)*sin(psi) sin(psi)*sin(theta)*sin(phi)+cos(psi)*cos(phi) sin(psi)*sin(theta)*cos(phi)-cos(psi)*sin(phi);
       -sin(theta) sin(phi)*cos(theta) cos(phi)*cos(theta)];
   
e3 = [0;0;1];

%Rote3 = [theta*cos(psi) + phi*sin(psi);
%         theta*sin(psi) - phi*cos(psi);
%         1];
vdot = g*e3 - (f/m)*(Rot*e3);

%px_double_dot = xdot  py_double_dot = ydot  pz_double_dot = zdot
xdot = vdot(1); %m/sec^2
ydot = vdot(2); %m/sec^2
zdot = vdot(3); %m/sec^2

%angular velocities
 
phi = p; %rad/sec
theta = q; %rad/sec
psi = r; %rad/sec

%angular accelerations

omegab_dot = tau;

p = omegab_dot(1); %rad/sec^2
q = omegab_dot(2); %rad/sec^2
r = omegab_dot(3); %rad/sec^2

%% Update stvec

dzdt = zeros(size(stvec));

dzdt(1) = x;
dzdt(2) = y;
dzdt(3) = z;
dzdt(4) = xdot;
dzdt(5) = ydot;
dzdt(6) = zdot;
dzdt(7) = phi;
dzdt(8) = theta;
dzdt(9) = psi;
dzdt(10)= p;
dzdt(11)= q;
dzdt(12)= r;

end

