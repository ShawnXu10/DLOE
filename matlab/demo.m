clear all;
close all;
clc;
warning off;
dbstop if error

addpath(genpath('src'))


[Xopt,Lopt,sequence,data] = DLOE_DynRec('config.yaml');

%==================== dynamice structure animation ========================
AnimParam.mode = 0;
AnimParam.t_pause = 0.02;
Animate_3D_Pose(Xopt(sequence,:)',data.t(sequence), data.R(sequence), [], AnimParam);