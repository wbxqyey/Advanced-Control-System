clc
close all

% Open Loop System Analysis

F_Plant = [0,1;-1.20,-3.71]; % Process Matrix
G_Plant = [0;1]; % Input Matrix
H_Plant_State1 = [1,0]; % Output Matrix
H_Plant_State2 = [0,1]; % Output Matrix Assuming x2 is the Output
H_Plant_FullState = [1,0;0,1]; % Output Matrix Assuming Both States are Outputs (for State-Feedback Purpose)

[num, den]=ss2tf(F_Plant, G_Plant , H_Plant_State1, 0, 2);


sys1 = ss(F_Plant,G_Plant,H_Plant_State1,0); % Use function 'ss' to establish state-space model
sys2 = ss(F_Plant,G_Plant,H_Plant_State2,0);
figure(1) % Open a new figure window named 'figure 1'
bode(sys1) % Bode plot from u to x1; Use function 'bode' to plot bode graph
grid on
figure(2)
bode(sys2) % Bode plot from u to x2
grid on

Poles = eig(F_Plant); % The open-loop poles are the eigenvalue of the original plant process matrix
% Use function 'eig' to calculate the eigenvalue of a square matrix
disp('The open-loop system transfer function poles are located at:') 
display(Poles)
% Use function 'disp' and 'display' to display messages on Command Window

ControllabilityMatrix = [G_Plant,F_Plant*G_Plant];


if (det(ControllabilityMatrix)==0)
    error('The original plant is not controllable.') 
    % An 'error' command will stop the MATLAB program and pop-up an error message
    % If the controllability matrix has determinant of zero, the plant is not controllable.
else
    disp('The original plant is controllable.')
end


% Design Using Ackermann's Formula

sys_fullstateplant = ss(F_Plant,G_Plant,H_Plant_FullState,[0;0]);
% The original plant with full state vector as output

K_SimpleExp = [0.1,0.1];
Signal_KX_SimpleExp = ss(0,[0,0],0,K_SimpleExp); % Dummy system representing the value of K*x

sys_fb_SimpleExp = feedback(sys_fullstateplant,Signal_KX_SimpleExp,-1); % Form a feedback system.
% Use function 'feedback' to establish a closed-loop system with a feedback
% controller
% Note that this system is 1-input-2-output system. The input is r (with
% Ks=1 by default) and the outputs are x1 and x2.

figure(3)
bode(sys_fb_SimpleExp) % Bode plot from r to x1 and x2

F_ClosedLoop = F_Plant-G_Plant*K_SimpleExp;
Poles = eig(F_ClosedLoop); % The closed-loop poles are the eigenvalue of the closed-loop process matrix

display(Poles)

ITAEPoles = 4*[-0.7071+0.7071*1i;-0.7071-0.7071*1i]; % ITAE Design Poles as a column vector
% In MATLAB, 'a+b*1i' is common way of defining complex value
K_ITAE = acker(F_Plant,G_Plant,ITAEPoles)
% Use function 'acker' to calculate the controller gain using Ackermann's Formula
Signal_KX_ITAE = ss(0,[0,0],0,K_ITAE)
sys_fb_ITAE = feedback(sys_fullstateplant,Signal_KX_ITAE,-1)

Ks_ITAE = 1/dcgain(sys_fb_ITAE) % Calculate the steady state gain of the feedback system
% Use function 'dcgain' to calculate the steady state gain of a system
% The returned value of a function can be directly used as part of
% other calculations.
sys_cl_ITAE = Ks_ITAE*sys_fb_ITAE; 
% The feedback system together with the scaling gain Ks forms the final design of closed-loop system
% Note that sys_cl_ITAE is SINGLE output system with output y=x1

figure(4)
bode(sys_cl_ITAE) % Bode plot from r to y
grid on
figure(5)
step(sys_cl_ITAE) % Unit step response from r to y; Use function 'step' to observe the step response of a system
grid on % Add grids to the step response figure
BW = bandwidth(sys_cl_ITAE)
[Mr,Wr,Wr_3db] = mr(sys_cl_ITAE)

% Similarly for Bessel Prototype
BesselPoles = 4*[-0.8660+0.5000*1i;-0.8660-0.5000*1i];
K_Bessel = acker(F_Plant,G_Plant,BesselPoles)
Signal_KX_Bessel = ss(0,[0,0],0,K_Bessel);
sys_fb_Bessel = feedback(sys_fullstateplant,Signal_KX_Bessel,-1);

Ks_Bessel = 1/dcgain(sys_fb_Bessel)
sys_cl_Bessel = Ks_Bessel*sys_fb_Bessel; 


figure(6)
bode(sys_cl_Bessel)
grid on
figure(7)
step(sys_cl_Bessel)
grid on

DampRatio = 0.707; % Define Damping Ratio for Reference Model
NatrualFreq = 3.6; % Define Natural Frequency for Reference Model
BandWidthDesire = NatrualFreq*sqrt((1-2*DampRatio^2)+sqrt((1-2*DampRatio^2)^2+1))
% Calculate the Bandwidth of the reference model
SODPoles = roots([1,2*DampRatio*NatrualFreq,NatrualFreq^2])
% Calculate the pole positions of the reference model, which is the root of
% pole polynomial
% Use function 'roots' to calculate the roots of a polynomial
K_SOD = acker(F_Plant,G_Plant,SODPoles);
Signal_KX_SOD = ss(0,[0,0],0,K_SOD);
sys_fb_SOD = feedback(sys_fullstateplant,Signal_KX_SOD,-1);

Ks_SOD = 1/dcgain(sys_fb_SOD);
sys_cl_SOD = Ks_SOD*sys_fb_SOD; 

figure(8)
bode(sys_cl_SOD)
grid on
figure(9)
step(sys_cl_SOD)
grid on


% Design Using LQR

Q_LQR = [1,0;0,1]; % Define penalty matrix Q
R_LQR = 1; % Define penalty matrix R (in this case, a scalar)
[K_LQR,~,~] = lqr(F_Plant,G_Plant,Q_LQR,R_LQR);
% Use function 'lqr' to calculate the controller gain for LQR approach
% 'lqr' is a multiple output function and only second/third output is not used in the rest of the script
Signal_KX_LQR = ss(0,[0,0],0,K_LQR);
sys_fb_LQR = feedback(sys_fullstateplant,Signal_KX_LQR,-1);

Ks_LQR = 1/dcgain(sys_fb_LQR);
sys_cl_LQR = Ks_LQR*sys_fb_LQR; 

figure(10)
bode(sys_cl_LQR)
figure(11)
step(sys_cl_LQR)
grid on
