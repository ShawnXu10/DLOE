clear all;
close all;
clc;
warning off;
dbstop if error

addpath(genpath('src'))
addpath(genpath('../spams-matlab'))
load('datasets/demo_dataset_jog.mat')

paramWL.const1 = 0.9;
paramWL.const2 = 0.5;
paramWL.lemma1 = 0.0015;
paramWL.lemma2 = 1e10;
paramWL.lemma3 = 0.03;
paramWL.itermax = 100;
paramWL.thres = 1e-4;


paramWL.cam_index = cam_index;
paramWL.Convdisp = true;

paramWL.f = 'X';
% paramWL.dist_Type = 'Time';
% paramWL.Seriation = 'MDS';

[ X_init,~,~] = X_initial( ray, t, cam_index);
[ ~, mean_errorinit, std_errorinit ] = reconstruction_error( X_GT', X_init);
[Xopt, Wopt, Dopt] = Triconvex_opt(X_init, ray, t, cam_index,paramWL, false);
[ ~, mean_erroropt, std_erroropt ] = reconstruction_error( X_GT', Xopt);



Animate_3D_Pose(Xopt(order,:)',ray(order),t(order),1,0,0.2, R(order), []);
figure
image(Wopt(order,order)*225)
figure
image(Dopt(order,order)*225)