clear all;
close all;
clc;
warning off;
dbstop if error

addpath(genpath('src'))
load('datasets/demo_dataset_jog_demo.mat')

[Xopt,Lopt,sequence] = DLOE_DynRec(caminput,'param.yaml');

%==================== dynamice structure animation ========================
AnimParam.mode = 0;
AnimParam.t_pause = 0.02;
Animate_3D_Pose(Xopt(order,:)',caminput.t(order), caminput.R(order), [], AnimParam);