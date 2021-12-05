clc
close all

% CA1CA3 Briefing MATLAB Demostration (State Augmentation)

% Formulate the Augmented State-Space Model

Fbar = [0,1,0;-1.20,-3.71,0;1,0,0]; % The process matrix for the augmented system
Gu = [0;1;0]; % The input matrix for u
Gr = [0;0;-1]; % The input matrix for r
Gv = [0;1;0]; % Used for Disturbance Analysis; The input matrix for disturbance v
Hbar = [1,0,0]; % The output matrix for y



% Design Using LQR

%for i = 1:5
  Q_LQR = [1,0,0;0,1,0;0,0,200];
  R_LQR = 1;
  [K_LQR,~,~] = lqr(Fbar,Gu,Q_LQR,R_LQR)

  sys_ag_LQR = ss(Fbar-Gu*K_LQR,Gr,Hbar,0);
  figure(1)
  bode(sys_ag_LQR)
  grid on
  hold on
  figure(2)
  step(sys_ag_LQR)
  grid on
  hold on
  
  sys_ag_LQR_Control = ss(Fbar-Gu*K_LQR,Gr,-K_LQR,0);
 
% In sys_ag_LQR_Control, Hbar is changed to -K_LQR
% In this case, MATLAB would treat -K_LQR*x as the output of the system,
% which is nothing else but the control signal
% Therefore, by doing a unit step response analysis, the control signal
% response can be displayed
  figure(3)
  step(sys_ag_LQR_Control)
  grid on
  hold on

%end

ITAEPoles = 2*[-0.7081; -0.5210+1.0680*1i;-0.5210-1.0680*1i];
K_ITAE = acker(Fbar,Gu,ITAEPoles)
sys_ag_ITAE = ss(Fbar-Gu*K_ITAE,Gr,Hbar,0);


BesselPoles = [-2.8260; -2.2365+2.1336*1i; -2.2365-2.1336*1i];
K_Bessel = acker(Fbar,Gu,BesselPoles)
sys_ag_Bessel = ss(Fbar-Gu*K_Bessel,Gr,Hbar,0);

SODPoles = [-7.0;-1.4140+1.4144*1i;-1.4140-1.4144*1i];
K_SOD = acker(Fbar,Gu,SODPoles)
sys_ag_SOD = ss(Fbar-Gu*K_SOD,Gr,Hbar,0)

% Disturbance Study
sys_ag_SODDisturb = ss(Fbar-Gu*K_SOD,Gv,Hbar,0);
% Gr is changed to Gv
% For sys_ag_Disturb, the input matrix is set Gv in order to study the
% system response when disturbance v comes in
% In this case study, the reference signal r is zero.
figure(21)
bode(sys_ag_SODDisturb)
grid on
figure(22)
step(sys_ag_SODDisturb)
grid on
sys_ag_SODDisturbControl = ss(Fbar-Gu*K_SOD,Gv,-K_SOD,0);
figure(23)
step(sys_ag_SODDisturbControl)
grid on

[num, den]=ss2tf(A, B, C, D, 2);
