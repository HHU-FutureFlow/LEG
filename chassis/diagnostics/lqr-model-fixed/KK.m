% Run AB.m in this directory first. These gains use u=-K*(X-X_target).
clear; clc;
syms Mp M L Lm l g Rw Iw Mw Ip Im real
output_dir = fileparts(mfilename('fullpath'));
load(fullfile(output_dir,'AB_symbolic.mat'),'A','B');
parameters = [Mp,M,L,Lm,l,g,Rw,Iw,Mw,Ip,Im];
values = [1.674,4.326,0.2933,0.1067,0.00494,9.8, ...
          0.06,0.000695,0.5,0.0257,0.0419];
A_val = double(subs(A,parameters,values));
B_val = double(subs(B,parameters,values));
assert(rank(ctrb(A_val,B_val))==6,'Model must be controllable.');
Q = diag([150,1,50,1,200,1]);
% Raise wheel torque cost from 4 to 8; retain the current leg cost of 0.5.
R_input = diag([8,0.5]);
K = lqr(A_val,B_val,Q,R_input);
poles_corrected = eig(A_val-B_val*K);
K_current = [22.8061,1.2335,7.0178,2.3379,1.9520,0.4453; ...
             -1.4104,-0.0769,1.7325,0.2979,-28.2688,-2.4956];
poles_current = eig(A_val-B_val*K_current);
assert(all(real(poles_corrected)<0),'Corrected model must be stable.');
format long g
disp('A physical ='); disp(A_val);
disp('B physical ='); disp(B_val);
disp('K physical, rows [Tw,Tp], columns [theta,dtheta,x,dx,phi,dphi] =');
disp(K);
disp('Corrected closed-loop poles ='); disp(poles_corrected);
disp('Current firmware K closed-loop poles ='); disp(poles_current);
save(fullfile(output_dir,'LQR_numeric.mat'), ...
     'A_val','B_val','Q','R_input','K','poles_corrected','poles_current');
