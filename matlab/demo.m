clear all;
close all;
clc;
warning off;
dbstop if error

addpath(genpath('src'))
load('datasets/demo_dataset_jog.mat')

paramWL.const1 = 0.9;
paramWL.const2 = 0.5;
paramWL.lemma1 = 0.0015;
paramWL.lemma2 = 1e10;
paramWL.lemma3 = 0.03;
paramWL.itermax = 50;
paramWL.thres = 1e-4;


paramWL.cam_index = cam_index;
paramWL.Convdisp = true;

paramWL.f = 'X';

[ X_init,~,~] = X_initial( ray, t, cam_index);
[ ~, mean_errorinit, std_errorinit ] = reconstruction_error( X_GT', X_init);
[Xopt, Wopt, Dopt] = Triconvex_opt(X_init, ray, t, cam_index,paramWL, false);
[ ~, mean_erroropt, std_erroropt ] = reconstruction_error( X_GT', Xopt);

param_lap.Ln = 'Simple';
param_lap.cam_index = cam_index;
param_lap.dist_Type = 'Arc';
param_lap.Seriation = 'SpRank'; %'MDS'; %'Laplacian';
[f_node, sequence, eigparam_X] = SequenceDReduce( Xopt', param_lap );

Animate_3D_Pose(Xopt(sequence,:)',ray(sequence),t(sequence),0,0,0.02, R(sequence), []);
figure
image(Wopt(sequence,sequence)*225)

