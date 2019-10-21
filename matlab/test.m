clear all;
close all;
clc;
warning off;
dbstop if error

addpath(genpath('src'))
load('datasets/demo_dataset_jog.test.mat')

paramWL.Wprior = 0.1;
paramWL.Dconst = 0.5;
paramWL.lambda1 = 0.0015;
paramWL.lambda2 = 1e10;
paramWL.lambda3 = 0.03;
paramWL.itermax = 50;
paramWL.thres = 1e-4;
paramWL.Convdisp = true;
paramWL.f = 'SpPrior';

[ X_init,~,~] = X_initial( ray, t, cam_index);
%3D reconstruction error of triangulaton
[ ~, mean_errorinit, std_errorinit ] = reconstruction_error( X_GT', X_init);
[Xopt, Wopt, Dopt] = Triconvex_opt(X_init, ray, t, cam_index,paramWL, false);
%reconstrution error of DLOE
[ ~, mean_erroropt, std_erroropt ] = reconstruction_error( X_GT', Xopt);

param.Ln = 'Simple';
param.cam_index = cam_index;
param.dist_Type = 'Arc';
param.Seriation = 'SpRank';
param.cam_index = cam_index;
[~, sequence, ~] = SequenceDReduce( Xopt', param );

AnimParam.mode = 0;
AnimParam.t_pause = 0.02;
Animate_3D_Pose(Xopt(sequence,:)',t(sequence), R(sequence), [], AnimParam);

%W matrix which show the pairwise affinity relation
figure
image(Wopt(sequence,sequence)*225)