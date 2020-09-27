% test rotation perturbance

clear all;
close all;
clc;


%q =  q_hat o_plus q_tilde

q_hat = [0.1 0.3 -0.4 1.2]';
q_hat = q_hat / norm(q_hat);

theta = [0.001, 0.002 , -0.003]';

q_tilde = [0.5 * theta; 0.99];
q_tilde = q_tilde / norm(q_tilde);



q = quatMult(q_hat, q_tilde);

R_hat = quatToRotMat(q_hat);
R_tilde = quatToRotMat(q_tilde);
R =  R_hat * R_tilde;

R

R1 = quatToRotMat(q)

R1' * R




