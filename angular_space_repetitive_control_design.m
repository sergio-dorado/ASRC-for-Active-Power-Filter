%% ANGULAR SPACE REPETITIVE CONTROL FOR AN ACTIVE POWER FILTER APPLICATION (2016)

% Authors:  Sergio A. Dorado-Rojas (sergio.dorado.rojas@gmail.com)
%           Felipe Galarza-Jiménez (fgalarzaj@unal.edu.co)
%           Prof. Germán A. Ramos Ph.D (garamosf@unal.edu.co)

% Modified and uploaded by: Sergio A. Dorado-Rojas (2019)

% Note 1: This code and its associated Simulink file must be run using a
% MATLAB version prior to release R2015a.  We did not found the reason why

% Initial clear-up
clc
clear all
close all

%% Plant parameters

s=tf('s');

Ts=1/12000;
% This has to coincide with the Ts of the Simulink library if
% the LTI blocks are to be used for the compensators.
% However, both Ts do not have to be equal. If a different
% Ts is to be used for either the simulation or the
% compensator calculation, the controllers have to be
% implemented as z-transfer functions in Simulink with the
% coefficients obtained for their respective discretization
% at the given Ts.
   
% Plant parameters

% Capacitance of the DC bus
C=1000e-6;
% Capacitance resistance
Rc=8.2e3;
% Inductance of the filter
L=1.2e-3;
% Resistance of the filter branch
Rl=1.4;
% Load resistance
R=0.2;

% Voltage-loop transfer function: notice that both transfer functions have
% a '+' sign if d2 is applied on the first transistor of the first arm;
% that is, alpha=2d-1. If alpha=1-2d and d1 is applied to that transistor
% instead of d2, both plants have a '-' sign.

Gu=Rc/(Rc*C*s+1);

% Current-loop transfer function
Gi=-1/(s*L*(120*pi)+Rl);

%% Voltage-loop controller

% Voltage-loop controller (Version 2016)
kp=0.2;
ki=1.5;
Gcuz=c2d(ki/s+kp,Ts,'tustin');

%% Repetitive control - Current loop

% Current loop controller
Gcz=-tf([7.976 6.657],[1 0.9511],pi/100);

% DT Plant
Gz=c2d(Gi,pi/100,'zoh');

% Inner closed-loop transfer function
Goz=feedback(Gz*Gcz,1);

% Poles check
[z_,p_,k_]=zpkdata(Goz,'v');

% Gx calculation
rd=length(p_)-length(z_); % relative degree of Go
Gx=(1/(Goz))*tf([1],[1,zeros(1,rd)],pi/100);

% Filter H calculation
q0=0.5;
H=tf([(1-q0)/2,q0,(1-q0)/2],[0,1,0],pi/100);

% Repetitive control computing
N=200;
Ret=[1 zeros(1,N-rd)];
Retz=tf(1,Ret,pi/100);
Ret=[1 zeros(1,rd)];
Retz2=tf(1,Ret,pi/100);
repe=feedback(Retz*H,Retz2,1);
Kr=0.3;

disp('The furthest pole from the origin is located at:')
max(abs(pole(feedback((1+repe*Gx*Kr)*Gcz*Gz,1))))

% Closed-loop feedback transfer function with repetitive
CL=feedback((1+repe*Gx*Kr)*Gcz*Gz,1);

%%%
% dc voltage filter
z=tf('z',pi/100); 
F1=(1/N)*(1-z^-N)/(1-z^-1);
