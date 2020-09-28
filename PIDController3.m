function throttle = PIDController(stvec,stvec_des,t)
  
global g m cT cM R CR
global e_pz
global eint_px eint_py eint_pz told

deltat = t - told;

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

x_des = stvec_des(1);
y_des = stvec_des(2);
z_des = stvec_des(3);
xdot_des = stvec_des(4);
ydot_des = stvec_des(5);
zdot_des = stvec_des(6);
psi_des = stvec_des(9);
p_des = stvec_des(10);
q_des = stvec_des(11);
r_des = stvec_des(12);
%% Position Control
%horizontal position (x)
%gains
kp_px=.1;
ki_px = 0;
kd_px=.1;

%error vectors
e_px = x_des-x;
edot_px = xdot_des-xdot;
eint_px = eint_px + e_px*deltat;

eint_px_max = .1; %saturation function
if eint_px > eint_px_max
    eint_px = eint_px_max;
end

theta_des = -kp_px*e_px - kd_px*edot_px - ki_px*eint_px; % Pitch forward is negative

theta_des_max = 0.15;  % Saturation bound for theta
if theta_des > theta_des_max
    theta_des = theta_des_max;
elseif theta_des < -theta_des_max
    theta_des = -theta_des_max;
end

%horizontal position (y)
%gains
kp_py=.1;
ki_py = 0;
kd_py=.1;

%error vectors
e_py = y_des-y;
edot_py = ydot_des-ydot;
eint_py = eint_py + e_py*deltat;

eint_py_max = .1;
if eint_py > eint_py_max %saturation function
    eint_py = eint_py_max;
end

phi_des = kp_py*e_py + kd_py*edot_py + ki_py*eint_py;

phi_des_max = 0.15;  % Saturation bound for phi
if phi_des > phi_des_max
    phi_des = phi_des_max;
elseif phi_des < -phi_des_max
    phi_des = -phi_des_max;
end

%% Attitude Control
%pitch
%gains
kp_theta=10;
kd_theta=2;

%error vectors
e_theta = theta_des - theta;
edot_theta = q_des - q;

tauy_des = kp_theta*e_theta + kd_theta*edot_theta;


%roll
%gains
kp_phi=10;
kd_phi=2;

%error vectors
e_phi = phi_des - phi;
edot_phi = p_des - p;

taux_des = kp_phi*e_phi + kd_phi*edot_phi;

%yaw
%gains
kp_psi=10;
kd_psi=5;

%error vectors
e_psi = psi_des - psi;
edot_psi = r_des - r;

tauz_des = kp_psi*e_psi + kd_psi*edot_psi;

tau_des = [taux_des;tauy_des;tauz_des];
%% Altitude Control

pzdotdot_des = 0;

%error vectors
e_pz = z_des - z;
edot_pz = zdot_des - zdot;
eint_pz = eint_pz + e_pz*deltat; %multiply e_pz_old*dt

%altitude control gains
kd_pz = 12;
ki_pz = 0;
kp_pz = 30;

f_des = m*g - m*(kd_pz*edot_pz + kp_pz*e_pz + ki_pz*eint_pz); %eint_pz


f_des_max = 30; %saturation bound (approx. 1.8*m*g)
if f_des > f_des_max
    f_des = f_des_max
elseif f_des < 0
    f_des = 0
end

%ftau = [f_des;tau_des(1);tau_des(2);tau_des(3)]; %Works when ftau is output(NOT THROTTLE)

%% Control Allocation
%control allocation matrix
M4 = [cT cT cT cT;
     (sqrt(2)/2)*R*cT -(sqrt(2)/2)*R*cT -(sqrt(2)/2)*R*cT (sqrt(2)/2)*R*cT;
     (sqrt(2)/2)*R*cT (sqrt(2)/2)*R*cT -(sqrt(2)/2)*R*cT -(sqrt(2)/2)*R*cT;
      cM -cM cM -cM];
%M4 = [cT cT cT cT;
%     0 -R*cT 0 R*cT;
%     R*cT 0 -R*cT 0;
%     cM -cM cM -cM];

P4 = inv(M4);

varpi_des = P4*[f_des;tau_des(1);tau_des(2);tau_des(3)];

%% Motor Control

throttle = zeros(4,1);

for i = 1:4
    throttle(i) = sqrt(varpi_des(i))/CR;
    if throttle(i) > 1
        throttle(i) = 1;
    end
    if throttle(i) < 0
        throttle(i) = 0;
    end
end

end

