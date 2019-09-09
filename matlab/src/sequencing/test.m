clear all;
close all;
clc;
dbstop if error

addpath(genpath('parser')) 
addpath(genpath('animate'))
addpath('quaternions')
addpath(genpath('spams-matlab'))
addpath(genpath('Xiangyu Method'))
addpath(genpath('lap_fun_constrains'))
%addpath('HDM_01-01_amc')

%global cam1 cam2 cam3 cam4 limbdist_mean W_GT X_GT
%% load input files
[skel,mot]       = readMocap('data/HDM_bk.asf', 'data/HDM_bk_hopBothLegs3hops_006_120.amc');
%[skel,mot]       = readMocap('data/HDM_bd.asf', 'data/HDM_bd_walkRightCircle6StepsLstart_001_120.amc');
%[skel,mot]       = readMocap('data/HDM_tr.asf', 'data/HDM_tr_hopRLeg1hops_033_120.amc');
%[skel,mot]       = readMocap('data/HDM_tr.asf', 'data/HDM_tr_jumpingJack3Reps_013_120.amc');
%[skel,mot]       = readMocap('data/HDM_tr.asf', 'data/HDM_tr_sneak4StepsRStart_016_120.amc');
%[skel,mot]       = readMocap('data/HDM_tr.asf', 'data/HDM_tr_jogOnPlaceStartFloor4StepsRStart_014_120.amc');
%[skel,mot]       = readMocap('data/HDM_tr.asf', 'data/HDM_tr_elbowToKnee3RepsLelbowStart_011_120.amc');

mot.nframes = 192;
for i = 1:31
    mot.jointTrajectories{i} = mot.jointTrajectories{i}(:,31:222);
end

%%initail cameras struct
y = 31;
K = [1000 0 500;0 1000 500;0 0 1];
R1 = cell(mot.nframes,1);
R2 = cell(mot.nframes,1);
R3 = cell(mot.nframes,1);
R4 = cell(mot.nframes,1);
t1 = cell(mot.nframes,1);
t2 = cell(mot.nframes,1);
t3 = cell(mot.nframes,1);
t4 = cell(mot.nframes,1);
Rc1 = zeros(1, mot.nframes);
Rc2 = zeros(1, mot.nframes);
Rc3 = zeros(1, mot.nframes);
Rc4 = zeros(1, mot.nframes);
R = 200;

%LimbRelation
limbSeq1 = [1 2;2 3;3 4;4 5;5 6];
limbSeq2 = [1 7;7 8;8 9;9 10;10 11];
limbSeq3 = [1 12;12 13;13 14;14 15;15 16;16 17];
limbSeq4 = [14 18;18 19;19 20;20 21;21 22;22 23];
limbSeq5 = [21 24];
limbSeq6 = [14 25;25 26;26 27;27 28;28 29;29 30];
limbSeq7 = [28 31];
limbSeq = [limbSeq1;limbSeq2;limbSeq3;limbSeq4;limbSeq5;limbSeq6;limbSeq7];

X_grdT = cell2mat(mot.jointTrajectories);
center = [mean(X_grdT(1,:)) mean(X_grdT(2,:)) mean(X_grdT(3,:)) ];

for i= 1:mot.nframes
    Rc1(i) = R+0*sin(i/mot.nframes*4*pi);
    Rc2(i) = R+0*sin(i/mot.nframes*2*pi);
    Rc3(i) = R+0*sin(i/mot.nframes*6.5*pi);
    Rc4(i) = R+0*sin(i/mot.nframes*10*pi);
    Theta1 = -pi/4 +pi/(3*(mot.nframes-1))*(i-1)+pi;
    Theta2 = pi/4 +pi/(3*(mot.nframes-1))*(i-1)+pi;
    Theta3 = 3*pi/4 +pi/(3*(mot.nframes-1))*(i-1)+pi;
    Theta4 = -3*pi/4 +pi/(3*(mot.nframes-1))*(i-1)+pi;
    R1{i} = [cos(Theta1)     0     sin(Theta1);0     1    0;-sin(Theta1)     0     cos(Theta1)];
    R2{i} = [cos(Theta2)     0     sin(Theta2);0     1    0;-sin(Theta2)     0     cos(Theta2)];
    R3{i} = [cos(Theta3)     0     sin(Theta3);0     1    0;-sin(Theta3)     0     cos(Theta3)];
    R4{i} = [cos(Theta4)     0     sin(Theta4);0     1    0;-sin(Theta4)     0     cos(Theta4)];
    t1{i} = [Rc1(i)*sin(Theta1)+center(1) y -Rc1(i)*cos(Theta1)+center(3)];
    t2{i} = [Rc2(i)*sin(Theta2)+center(1) y -Rc2(i)*cos(Theta2)+center(3)];
    t3{i} = [Rc3(i)*sin(Theta3)+center(1) y -Rc3(i)*cos(Theta3)+center(3)];
    t4{i} = [Rc4(i)*sin(Theta4)+center(1) y -Rc4(i)*cos(Theta4)+center(3)];
end


data = cell(5,4);
lemma3_data = [1e7 0.026 0.022 0.019 0.013 0.009];
lemma3Q_data = [1e7 0.026 0.022 0.019 0.013 0.009];
lemma4Q_data = [0.01 0.005 0.0025];%[0.01 0.01 0.01 0.01 0.01 0.01];%[0.02 0.02 0.015 0.013 0.01 0.01];
lemma3Q1_data = [1e7 0.026 0.022 0.019 0.013 0.009];%[4.5e6 0.017 0.011 0.002 0.0015 0.001];%
lemma4Q1_data = [0.002 0.0015 0.0005];%[0.02 0.02 0.015 0.013 0.01 0.01];


freq = [30 15 7.5];
noise_data = [0 1 2 3 4 5];

avg_error = zeros(1,6);
mean_error = zeros(1,6);
avg_errorQ = zeros(1,6);
mean_errorQ = zeros(1,6);
avg_errorQ1 = zeros(1,6);
mean_errorQ1 = zeros(1,6);
avg_errorWLMDS = zeros(1,6);
mean_errorWLMDS = zeros(1,6);

std_avg_error = zeros(1,6);
std_error = zeros(1,6);
std_avg_errorQ = zeros(1,6);
std_errorQ = zeros(1,6);
std_avg_errorQ1 = zeros(1,6);
std_errorQ1 = zeros(1,6);
std_avg_errorWLMDS = zeros(1,6);
std_errorWLMDS = zeros(1,6);

avg_rhoLap = zeros(1,6);
avg_rhoLap_init = zeros(1,6);
avg_rhoLapArc = zeros(1,6);
avg_rhoLapArc_init = zeros(1,6);
avg_rhoMDS = zeros(1,6);
avg_rhoMDS_init = zeros(1,6);
avg_rhoMDSArc = zeros(1,6);
avg_rhoMDSArc_init = zeros(1,6);
avg_rhoSHP = zeros(1,6);
avg_rhoSHP_init = zeros(1,6);
avg_rhoSHPArc = zeros(1,6);
avg_rhoSHPArc_init = zeros(1,6);

for noise = 1:6
    percentage{noise} = zeros(1,10);
    percentageQ{noise} = zeros(1,10);
    percentageQ1{noise} = zeros(1,10);
    avg_percentageQ{noise} = zeros(1,10);
    avg_percentageQ1{noise} = zeros(1,10);
    avg_percentageWLMDS{noise} = zeros(1,10);
    avg_percentage{noise} = zeros(1,10);
end
%predefined for parpool
cam1 = cell(1,6);
cam2 = cell(1,6);
cam3 = cell(1,6);
cam4 = cell(1,6);
cam1_t = cell(1,6);
cam2_t = cell(1,6);
cam3_t = cell(1,6);
cam4_t = cell(1,6);
cam1_ray = cell(1,6);
cam2_ray = cell(1,6);
cam3_ray = cell(1,6);
cam4_ray = cell(1,6);
X_cam1 = cell(1,6);
X_cam2 = cell(1,6);
X_cam3 = cell(1,6);
X_cam4 = cell(1,6);
X = cell(1,6);
W = cell(1,6);
XQ = cell(1,6);
WQ = cell(1,6);
Q = cell(1,6);
XQ1 = cell(1,6);
WQ1 = cell(1,6);
Q1 = cell(1,6);
error = cell(1,6);
errorQ = cell(1,6);
errorQ1 = cell(1,6);
param = cell(1,6);
paramWL = cell(1,6);
X_init = cell(1,6);
X_initXX = cell(1,6);
ray_sum = cell(1,6);
t_sum = cell(1,6);
cam_index = cell(1,6);
Sequence_GT = cell(1,6);
Order_GT = cell(1,6);
param_lap = cell(1,6);
eigparam_X = cell(1,6);
eigparam_X_init = cell(1,6);
sequenceMDSArc = cell(1,6);
OrderMDSArc = cell(1,6);
sequenceMDS = cell(1,6);
OrderMDS = cell(1,6);
sequenceLap = cell(1,6);
OrderLap = cell(1,6);
sequenceLapArc = cell(1,6);
OrderLapArc = cell(1,6);
sequenceSHP = cell(1,6);
OrderSHP = cell(1,6);
sequenceSHPArc = cell(1,6);
OrderSHPArc = cell(1,6);
pardistArc = cell(1,6);

sequenceMDSArc_init = cell(1,6);
OrderMDSArc_init = cell(1,6);
sequenceMDS_init = cell(1,6);
OrderMDS_init = cell(1,6);
sequenceLap_init = cell(1,6);
OrderLap_init = cell(1,6);
sequenceLapArc_init = cell(1,6);
OrderLapArc_init = cell(1,6);
sequenceSHP_init = cell(1,6);
OrderSHP_init = cell(1,6);
sequenceSHPArc_init = cell(1,6);
OrderSHPArc_init = cell(1,6);
pardistArc_init = cell(1,6);


for iter = 1:1
    for noise = 1:1
        for percent_miss = 1:1
            for nf = 1:1
                cam1{noise} = CameraStruct(mot, K, R1, t1, Rc1, noise_data(noise), 1, freq(nf));
                cam2{noise} = CameraStruct(mot, K, R2, t2, Rc2, noise_data(noise), 1+mot.samplingRate/freq(nf)/4, freq(nf));
                cam3{noise} = CameraStruct(mot, K, R3, t3, Rc3, noise_data(noise), 1+mot.samplingRate/freq(nf)*2/4, freq(nf));
                cam4{noise} = CameraStruct(mot, K, R4, t4, Rc4, noise_data(noise), 1+mot.samplingRate/freq(nf)*3/4, freq(nf)); 

                %animate(skel, mot);

                cam1_t{noise} = cam1{noise}.t(~cellfun('isempty',cam1{noise}.ray));
                cam2_t{noise} = cam2{noise}.t(~cellfun('isempty',cam2{noise}.ray));
                cam3_t{noise} = cam3{noise}.t(~cellfun('isempty',cam3{noise}.ray));
                cam4_t{noise} = cam4{noise}.t(~cellfun('isempty',cam4{noise}.ray));

                cam1_ray{noise} = cam1{noise}.ray(~cellfun('isempty',cam1{noise}.ray));
                cam2_ray{noise} = cam2{noise}.ray(~cellfun('isempty',cam2{noise}.ray));
                cam3_ray{noise} = cam3{noise}.ray(~cellfun('isempty',cam3{noise}.ray));
                cam4_ray{noise} = cam4{noise}.ray(~cellfun('isempty',cam4{noise}.ray));

                ray_sum{noise} = [cam1_ray{noise}; cam2_ray{noise}; cam3_ray{noise}; cam4_ray{noise}];
                t_sum{noise} = [cam1_t{noise}; cam2_t{noise}; cam3_t{noise}; cam4_t{noise}];

                frames = size(ray_sum{noise},1);
                joints = mot.njoints;

                %generate X for each camera, and in grdt temperal sequence
                X_cam1{noise} =  X_grdT(:, cam1{noise}.start:mot.samplingRate/cam1{noise}.frequency:end);
                X_cam2{noise} =  X_grdT(:, cam2{noise}.start:mot.samplingRate/cam2{noise}.frequency:end);
                X_cam3{noise} =  X_grdT(:, cam3{noise}.start:mot.samplingRate/cam3{noise}.frequency:end);
                X_cam4{noise} =  X_grdT(:, cam4{noise}.start:mot.samplingRate/cam4{noise}.frequency:end);
                cam_index{noise} = [size(X_cam1{noise}, 2) size(X_cam1{noise}, 2)+size(X_cam2{noise}, 2) size(X_cam1{noise}, 2)+size(X_cam2{noise}, 2)+size(X_cam3{noise}, 2) size(X_cam1{noise}, 2)+size(X_cam2{noise}, 2)+size(X_cam3{noise}, 2)+size(X_cam4{noise}, 2)];


                X_GT = X_grdT(:, [cam1{noise}.start:mot.samplingRate/cam1{noise}.frequency:end cam2{noise}.start:mot.samplingRate/cam2{noise}.frequency:end...
                    cam3{noise}.start:mot.samplingRate/cam3{noise}.frequency:end cam4{noise}.start:mot.samplingRate/cam4{noise}.frequency:end]);
                Sequence_GT{noise} = [cam1{noise}.start:mot.samplingRate/cam1{noise}.frequency:size(X_grdT,2) cam2{noise}.start:mot.samplingRate/cam2{noise}.frequency:size(X_grdT,2)...
                    cam3{noise}.start:mot.samplingRate/cam3{noise}.frequency:size(X_grdT,2) cam4{noise}.start:mot.samplingRate/cam4{noise}.frequency:size(X_grdT,2)];
                Order_GT{noise} = 1:length(Sequence_GT{noise});
    %==================================================================================            
                %randomly drop some frames

                percent_frame_miss = (percent_miss-1)*0.1;
                [ X_GT, ray_sum{noise}, t_sum{noise}, cam_index{noise} ] = FrameDrop(X_GT, ray_sum{noise}, t_sum{noise}, cam_index{noise}, percent_frame_miss);
                frames = size(ray_sum{noise},1);
    %==================================================================================
                %W_GT = getrealWGT(cam_index);
                %W_GT{noise,nf,percent_miss} = getWGT(X_GT, cam_index{noise,nf,percent_miss});

                [ X_init{noise}, ~ ] = X_initial( ray_sum{noise}, t_sum{noise}, cam_index{noise} );
                [ X_initXX{noise}, ~ ,~,~,~] = X_initialXX( ray_sum{noise}, t_sum{noise}, cam_index{noise} );

                
%                 param{noise}.lemma1 = 0.05;
%                 param{noise}.lemma2 = 0.5;
%                 param{noise}.lemma3 = lemma3_data(noise);%-10+10*(n_lemma3-1);
%                 param{noise}.noise_valid = 1;
%                 param{noise}.opt_index = 1;
%                 param{noise}.hpfilter = 0;
%                 [X{noise}, W{noise}] = biconvex_opt(X_init{noise}, ray_sum{noise}, t_sum{noise}, cam_index{noise},param{noise});
%                 [ error{noise}, mean_error(noise), std_error(noise) ] = reconstruction_error( X_GT, X{noise});          
%                 percentage{noise} = ThresholdError( error{noise} );
%                  %Animate_3D_Pose(X_2,ray_sum,t_sum,2,1,0.5);
%                  
                %============Xiangyu's Method1===============================
                paramWL{noise}.const2 = 0.001;
                paramWL{noise}.const1 = 0.95;
                paramWL{noise}.lemma2 = 0;
                paramWL{noise}.lemma3 = lemma3Q_data(noise);
                paramWL{noise}.lemma4 = lemma4Q_data(nf);
                paramWL{noise}.noise_valid = 1;
                paramWL{noise}.f = 'X';
%                 [XQ{noise}, WQ{noise}, Q{noise}] = Triconvex_optQ(X_initXX{noise}, ray_sum{noise}, t_sum{noise}, cam_index{noise},paramWL{noise});
%                 [ errorQ{noise}, mean_errorQ(noise), std_errorQ(noise) ] = reconstruction_error( X_GT, XQ{noise}); 
%                 percentageQ{noise} = ThresholdError( errorQ{noise} );
                
                paramWL{noise}.lemma3 = lemma3Q1_data(noise);
                paramWL{noise}.lemma4 = lemma4Q1_data(nf); 
                
                [XQ12{noise}, WQ12{noise}, Q12{noise}] = Triconvex_optQ1(X_initXX{noise}, ray_sum{noise}, t_sum{noise}, cam_index{noise},paramWL{noise});
                [ errorQ12{noise}, mean_errorQ12(noise), std_errorQ12(noise) ] = reconstruction_error( X_GT, XQ12{noise}); 
                percentageQ12{noise} = ThresholdError( errorQ12{noise} );
               
                %====================================
                paramWL{noise}.f = 'fiedler';
                paramWL{noise}.dist_Type = 'Time';
                paramWL{noise}.Seriation = 'Laplacian';
                %====================================                
                
                [XQ13{noise}, WQ13{noise}, Q13{noise}] = Triconvex_optQ1(X_initXX{noise}, ray_sum{noise}, t_sum{noise}, cam_index{noise},paramWL{noise});
                [ errorQ13{noise}, mean_errorQ13(noise), std_errorQ13(noise) ] = reconstruction_error( X_GT, XQ13{noise}); 
                percentageQ13{noise} = ThresholdError( errorQ13{noise} );
                                 
                %====================================
                paramWL{noise}.f = 'fiedler';
                paramWL{noise}.dist_Type = 'Time';
                paramWL{noise}.Seriation = 'MDS';
                %====================================
                
                [XQ1{noise}, WQ1{noise}, Q1{noise}] = Triconvex_optQ1(X_initXX{noise}, ray_sum{noise}, t_sum{noise}, cam_index{noise},paramWL{noise});
                [ errorQ1{noise}, mean_errorQ1(noise), std_errorQ1(noise) ] = reconstruction_error( X_GT, XQ1{noise}); 
                percentageQ1{noise} = ThresholdError( errorQ1{noise} );
                Loss = LossFWL( XQ1{noise}, XQ1{noise}', WQ1{noise}, Q1{noise}, t_sum{noise}, ray_sum{noise}, cam_index{noise}, paramWL{noise}.lemma2, paramWL{noise}.lemma3, paramWL{noise}.lemma4);
%               
                param_lap{noise}.dist = 1;
                param_lap{noise}.Ln = 'Simple';
                param_lap{noise}.cam_index = cam_index{noise};
                param_lap{noise}.t = 1;
                param_lap{noise}.dist_criter = 1.5;
                param_lap{noise}.DMtype = 'inv';
                param_lap{noise}.graph = 'undirected';
                param_lap{noise}.Version = 2;
                
                param_lap{noise}.Seriation = 'MDS'; %'MDS'; %'Laplacian';
%===============================MDS_time===================================
                param_lap{noise}.dist_Type = 'Time';
                [~, ~,  eigparam_X{noise}] = SequenceDReduce( XQ1{noise}, param_lap{noise} );
                [~,sequenceMDSArc{noise}] = sort(eigparam_X{noise}.EmbedV);
                %[~,indexMDS] = sort(sequenceMDS);
                OrderMDSArc{noise} = Sequence_GT{noise}(sequenceMDSArc{noise});
                rhoMDSArc(noise) = abs(corr(Order_GT{noise}',OrderMDSArc{noise}', 'Type', 'Kendall'));
                
                
                [~, ~,  eigparam_X_init{noise}] = SequenceDReduce( X_initXX{noise}, param_lap{noise} );
                [~,sequenceMDSArc_init{noise}] = sort(eigparam_X_init{noise}.EmbedV);
                %[~,indexMDS_init] = sort(sequenceMDS_init);
                OrderMDSArc_init{noise} = Sequence_GT{noise}(sequenceMDSArc_init{noise});
                rhoMDSArc_init(noise) = abs(corr(Order_GT{noise}',OrderMDSArc_init{noise}', 'Type', 'Kendall'));
%==========================================================================                
                
                
                
%===============================MDS_space===================================                    
                param_lap{noise}.dist_Type = 'Space';
                [~, ~,  eigparam_X{noise}] = SequenceDReduce( XQ1{noise}, param_lap{noise} );
                [~,sequenceMDS{noise}] = sort(eigparam_X{noise}.EmbedV);
                %[~,indexMDS2] = sort(sequenceMDS2);
                OrderMDS{noise} = Sequence_GT{noise}(sequenceMDS{noise});
                rhoMDS(noise) = abs(corr(Order_GT{noise}',OrderMDS{noise}', 'Type', 'Kendall'));
                
                [~, ~,  eigparam_X_init{noise}] = SequenceDReduce( X_initXX{noise}, param_lap{noise} );
                [~,sequenceMDS_init{noise}] = sort(eigparam_X_init{noise}.EmbedV);
                %[~,indexMDS2_init] = sort(sequenceMDS2_init);
                OrderMDS_init{noise} = Sequence_GT{noise}(sequenceMDS_init{noise});
                rhoMDS_init(noise) = abs(corr(Order_GT{noise}',OrderMDS_init{noise}', 'Type', 'Kendall'));
%==========================================================================                
                
                
                param_lap{noise}.Seriation = 'Laplacian'; %'MDS'; %'Laplacian';
%===============================Lap_time===================================
                param_lap{noise}.dist_Type = 'Time';
                [~, ~,  eigparam_X{noise}] = SequenceDReduce( XQ1{noise}, param_lap{noise} );
                [~,sequenceLapArc{noise}] = sort(eigparam_X{noise}.fedler);
                %[~,indexMDS] = sort(sequenceMDS);
                OrderLapArc{noise} = Sequence_GT{noise}(sequenceLapArc{noise});
                rhoLapArc(noise) = abs(corr(Order_GT{noise}',OrderLapArc{noise}', 'Type', 'Kendall'));
                
                
                [~, ~,  eigparam_X_init{noise}] = SequenceDReduce( X_initXX{noise}, param_lap{noise} );
                [~,sequenceLapArc_init{noise}] = sort(eigparam_X_init{noise}.fedler);
                %[~,indexMDS_init] = sort(sequenceMDS_init);
                OrderLapArc_init{noise} = Sequence_GT{noise}(sequenceLapArc_init{noise});
                rhoLapArc_init(noise) = abs(corr(Order_GT{noise}',OrderLapArc_init{noise}', 'Type', 'Kendall'));

%===============================Lap_space===================================                    
                param_lap{noise}.dist_Type = 'Space';
                [~, ~,  eigparam_X{noise}] = SequenceDReduce( XQ1{noise}, param_lap{noise} );
                [~,sequenceLap{noise}] = sort(eigparam_X{noise}.fedler);
                %[~,indexMDS2] = sort(sequenceMDS2);
                OrderLap{noise} = Sequence_GT{noise}(sequenceLap{noise});
                rhoLap(noise) = abs(corr(Order_GT{noise}',OrderLap{noise}', 'Type', 'Kendall'));
                
                [~, ~,  eigparam_X_init{noise}] = SequenceDReduce( X_initXX{noise}, param_lap{noise} );
                [~,sequenceLap_init{noise}] = sort(eigparam_X_init{noise}.fedler);
                %[~,indexMDS2_init] = sort(sequenceMDS2_init);
                OrderLap_init{noise} = Sequence_GT{noise}(sequenceLap_init{noise});
                rhoLap_init(noise) = abs(corr(Order_GT{noise}',OrderLap_init{noise}', 'Type', 'Kendall'));
                
% %================================adjacency matrix==========================
%                 param_lap.t = 0;
%                 [~, ~,  eigparam_W] = SequenceDReduce( W{noise}, param_lap );
%                 [~,sequenceLapW] = sort(eigparam_W.fedler);
%                 [~,indexMDS2] = sort(sequenceMDS2);
%                 OrderLapW = Sequence_GT(sequenceLapW);
%                 
% %==========================================================================
                
%===============================SHP_space==================================                 
                sequenceSHP{noise} = TspToShp( XQ1{noise});
                %[~,indexTsp] = sort(sequenceTsp);
                OrderSHP{noise} = Sequence_GT{noise}(sequenceSHP{noise});
                rhoSHP(noise) = abs(corr(Order_GT{noise}',OrderSHP{noise}', 'Type', 'Kendall'));
                
                sequenceSHP_init{noise} = TspToShp( X_initXX{noise});
                %[~,indexTsp_init] = sort(sequenceTsp_init);
                OrderSHP_init{noise} = Sequence_GT{noise}(sequenceSHP_init{noise});
                rhoSHP_init(noise) = abs(corr(Order_GT{noise}',OrderSHP_init{noise}', 'Type', 'Kendall'));
                
%===============================SHP_time==================================

                pardistArc{noise} = pdistArc( XQ1{noise}, cam_index{noise}, param_lap{noise});
                sequenceSHPArc{noise} = TspToShp( XQ1{noise}, pardistArc{noise});
                %[~,indexTspArc] = sort(sequenceTspArc);
                OrderSHPArc{noise} = Sequence_GT{noise}(sequenceSHPArc{noise});
                rhoSHPArc(noise) = abs(corr(Order_GT{noise}',OrderSHPArc{noise}', 'Type', 'Kendall'));
                
                pardistArc_init{noise} = pdistArc( X_initXX{noise}, cam_index{noise}, param_lap{noise});
                sequenceSHPArc_init{noise} = TspToShp( X_initXX{noise}, pardistArc_init{noise});
                %[~,indexTspArc_init] = sort(sequenceTspArc_init);
                OrderSHPArc_init{noise} = Sequence_GT{noise}(sequenceSHPArc_init{noise});
                rhoSHPArc_init(noise) = abs(corr(Order_GT{noise}',OrderSHPArc_init{noise}', 'Type', 'Kendall'));
                
                

            end
        end
    end
    avg_rhoLap = avg_rhoLap + rhoLap;
    avg_rhoLap_init = avg_rhoLap_init + rhoLap_init;
    avg_rhoLapArc = avg_rhoLapArc + rhoLapArc;
    avg_rhoLapArc_init = avg_rhoLapArc_init + rhoLapArc_init;
    avg_rhoMDS = avg_rhoMDS + rhoMDS;
    avg_rhoMDS_init = avg_rhoMDS_init + rhoMDS_init;
    avg_rhoMDSArc = avg_rhoMDSArc + rhoMDSArc;
    avg_rhoMDSArc_init = avg_rhoMDSArc_init + rhoMDSArc_init;
    avg_rhoSHP = avg_rhoSHP + rhoSHP;
    avg_rhoSHP_init = avg_rhoSHP_init + rhoSHP_init;
    avg_rhoSHPArc = avg_rhoSHPArc + rhoSHPArc;
    avg_rhoSHPArc_init = avg_rhoSHPArc_init + rhoSHPArc_init;
 end
    avg_rhoLap = avg_rhoLap/iter;
    avg_rhoLap_init = avg_rhoLap_init/iter;
    avg_rhoLapArc = avg_rhoLapArc/iter;
    avg_rhoLapArc_init = avg_rhoLapArc_init/iter;
    avg_rhoMDS = avg_rhoMDS/iter;
    avg_rhoMDS_init = avg_rhoMDS_init/iter;
    avg_rhoMDSArc = avg_rhoMDSArc/iter;
    avg_rhoMDSArc_init = avg_rhoMDSArc_init/iter;
    avg_rhoSHP = avg_rhoSHP/iter;
    avg_rhoSHP_init = avg_rhoSHP_init/iter;
    avg_rhoSHPArc = avg_rhoSHPArc/iter;
    avg_rhoSHPArc_init = avg_rhoSHPArc_init/iter;
%[~,index_GT] = sort(Sequence_GT);
%[~,index] = sort(sequenceMDSArc);
%ShowLine(9, X_GT(1:3,Sequence_GT), 'b')
                figure(5)
                subplot(6,2,1)
                bar(OrderMDS_init{noise})
                title(['MDSInit: ' num2str(rhoMDS_init(noise))])
                subplot(6,2,2)
                bar(fliplr(OrderMDS{noise}))
                title(['MDS: ' num2str(rhoMDS(noise))])
                subplot(6,2,3)
                bar(fliplr(OrderMDSArc_init{noise}))
                title(['MDSArcInit: ' num2str(rhoMDSArc_init(noise))])
                subplot(6,2,4)
                bar(fliplr(fliplr(OrderMDSArc{noise})))
                title(['MDSArc: ' num2str(rhoMDSArc(noise))])    
                
                subplot(6,2,5)
                bar(fliplr(OrderLap_init{noise}))
                title(['LapInit: ' num2str(rhoLap_init(noise))])
                subplot(6,2,6)
                bar(fliplr(OrderLap{noise}))
                title(['Lap: ' num2str(rhoLap(noise))])
                subplot(6,2,7)
                bar(fliplr(OrderLapArc_init{noise}))
                title(['LapArcInit: ' num2str(rhoLapArc_init(noise))])
                subplot(6,2,8)
                bar(fliplr(OrderLapArc{noise}))
                title(['LapArc: ' num2str(rhoLapArc(noise))])  
                
                subplot(6,2,9)
                bar(OrderSHP_init{noise})
                title(['SHPInit: ' num2str(rhoSHP_init(noise))])
                subplot(6,2,10)
                bar(OrderSHP{noise})
                title(['SHP: ' num2str(rhoSHP(noise))])               
                subplot(6,2,11)  
                bar(OrderSHPArc_init{noise})
                title(['SHPInitArc: ' num2str(rhoSHPArc_init(noise))])
                subplot(6,2,12)
                bar(OrderSHPArc{noise})
                title(['SHPArc: ' num2str(rhoSHPArc(noise))])