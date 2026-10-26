% Correct coupled acceleration solve, using the requested physical signs.
% x: wheel axle position, forward positive.
% theta: hip-to-wheel angle from downward vertical, wheel ahead positive.
% phi: body COM behind hip positive (backward body lean).
% Tw: wheel torque driving forward; Tp: torque on leg swinging wheel forward.
% This writes model artifacts only; it does not change MCU firmware.
clear; clc;
syms Mp M L Lm l g Rw Iw Mw Ip Im real
syms theta theta_dot x_pos x_dot phi phi_dot real
syms ax ath aph Tw Tp real
q = [x_pos; theta; phi];
qd = [x_dot; theta_dot; phi_dot];
qdd = [ax; ath; aph];

leg_com = [x_pos-L*sin(theta); L*cos(theta)];
body_com = [x_pos-(L+Lm)*sin(theta)-l*sin(phi); ...
            (L+Lm)*cos(theta)+l*cos(phi)];
leg_vel = jacobian(leg_com,q)*qd;
body_vel = jacobian(body_com,q)*qd;
leg_acc = jacobian(leg_vel,q)*qd + jacobian(leg_vel,qd)*qdd;
body_acc = jacobian(body_vel,q)*qd + jacobian(body_vel,qd)*qdd;

Nm = M*body_acc(1);
Pm = M*(body_acc(2)+g);
N = Mp*leg_acc(1)+Nm;
P = Mp*(leg_acc(2)+g)+Pm;

% Every equation uses the SAME scalar acceleration symbols before solving.
equations = [ ...
    (Mw+Iw/Rw^2)*ax + N - Tw/Rw; ...
    Ip*ath - (P*L+Pm*Lm)*sin(theta) ...
           - (N*L+Nm*Lm)*cos(theta) - Tw - Tp; ...
    Im*aph + Tp - Nm*l*cos(phi) - Pm*l*sin(phi)];
[mass_matrix, force_vector] = equationsToMatrix(equations,qdd);
accelerations = simplify(mass_matrix\force_vector);
assert(~any(has(accelerations,qdd),'all'), ...
    'Acceleration solve must not retain any acceleration symbols.');
assert(isequal(simplify(mass_matrix-mass_matrix.'),sym(zeros(3))), ...
    'The generalized mass matrix must be symmetric.');

X = [theta; theta_dot; x_pos; x_dot; phi; phi_dot];
U = [Tw; Tp];
f = [theta_dot; accelerations(2); x_dot; accelerations(1); ...
     phi_dot; accelerations(3)];
equilibrium = [X; U];
A = simplify(subs(jacobian(f,X),equilibrium,zeros(8,1)));
B = simplify(subs(jacobian(f,U),equilibrium,zeros(8,1)));
output_dir = fileparts(mfilename('fullpath'));
save(fullfile(output_dir,'AB_symbolic.mat'),'A','B');
disp('Corrected physical-coordinate symbolic A and B saved.');
