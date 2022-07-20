function MainFreq()
dbstop if error
warning off;
addpath(genpath('../../Dynamic_3D_Reconstruction'))
addpath(genpath('src'))
load('../../Dynamic_3D_Reconstruction/HDM05_cut_amc/sequencedata.mat')



motionindex = [1 5 30 45 52 82 84 95 101 115];

sequencedata2 = sequencedata(motionindex,:);

itern = length(motionindex);
%predefined for parpool
cam1 = cell(itern,1);
cam2 = cell(itern,1);
cam3 = cell(itern,1);
cam4 = cell(itern,1);

paramWL = cell(itern,1);
X_init = cell(itern,1);
X_opt = cell(itern,1);

ray_sum = cell(itern,1);
t_sum = cell(itern,1);
cam_index = cell(itern,1);
X_GT = cell(itern,1);


y = cell(itern,1);
K = cell(itern,1);
R1 = cell(itern,1);
R2 = cell(itern,1);
R3 = cell(itern,1);
R4 = cell(itern,1);
t1 = cell(itern,1);
t2 = cell(itern,1);
t3 = cell(itern,1);
t4 = cell(itern,1);
Rc1 = cell(itern,1);
Rc2 = cell(itern,1);
Rc3 = cell(itern,1);
Rc4 = cell(itern,1);
R = cell(itern,1);
X_grdT = cell(itern,1);
center = cell(itern,1);
Theta1 = cell(itern,1);
Theta2 = cell(itern,1);
Theta3 = cell(itern,1);
Theta4 = cell(itern,1);
mot = cell(itern,1);

lemma3_data = [1e10 100 100 100 100 100];%[1e7 0.026 0.022 0.019 0.013 0.009];%
lemma3Q1_data = [1e10 0.02 0.02 0.02 0.02 0.02];%[1e10 0.15 0.15 0.15 0.15 0.15];%
lemma4Q1_data = [0.0015 0.0015 0.0015 0.0015];
epsilon_data = [1e-5 1e-5 1e-5];
lambda2_data = [1e-5 1e-5 1e-5];
const2_data = [0.5 0.5 0.5];
const1_data = [0.9 0.9 0.9];

freq = [30 15 7.5 3];
noise_data = [0 1 2 3 4 5];
%run_time = [];
%run_mem = [];
load('runtime3.mat')

%% load input files

%     parfor_progress(itern,1);
for iter = 1:itern
%     parfor_progress(-1,1);
    [~,mot{iter}]       = readMocap(sequencedata2{iter,2}, sequencedata2{iter,1});
    %%initail cameras struct
    y{iter} = 31;
    K{iter} = [1000 0 500;0 1000 500;0 0 1];
    R1{iter} = cell(mot{iter}.nframes,1);
    R2{iter} = cell(mot{iter}.nframes,1);
    R3{iter} = cell(mot{iter}.nframes,1);
    R4{iter} = cell(mot{iter}.nframes,1);
    t1{iter} = cell(mot{iter}.nframes,1);
    t2{iter} = cell(mot{iter}.nframes,1);
    t3{iter} = cell(mot{iter}.nframes,1);
    t4{iter} = cell(mot{iter}.nframes,1);
    Rc1{iter} = zeros(1, mot{iter}.nframes);
    Rc2{iter} = zeros(1, mot{iter}.nframes);
    Rc3{iter} = zeros(1, mot{iter}.nframes);
    Rc4{iter} = zeros(1, mot{iter}.nframes);
    X_grdT{iter} = cell2mat(mot{iter}.jointTrajectories);
    R{iter} = max(200, 1.5*norm([max(X_grdT{iter}(1,:)) max(X_grdT{iter}(2,:)) max(X_grdT{iter}(3,:))] - [min(X_grdT{iter}(1,:)) min(X_grdT{iter}(2,:)) min(X_grdT{iter}(3,:))]));

    
    center{iter} = [mean(X_grdT{iter}(1,:)) mean(X_grdT{iter}(2,:)) mean(X_grdT{iter}(3,:))];
    for i= 1:mot{iter}.nframes
        Rc1{iter}(i) = R{iter};
        Rc2{iter}(i) = R{iter};
        Rc3{iter}(i) = R{iter};
        Rc4{iter}(i) = R{iter};
        Theta1{iter} = -pi/4;
        Theta2{iter} = pi/4;
        Theta3{iter} = 3*pi/4;
        Theta4{iter} = -3*pi/4;
        R1{iter}{i} = [cos(Theta1{iter})     0     sin(Theta1{iter});0     1    0;-sin(Theta1{iter})     0     cos(Theta1{iter})];
        R2{iter}{i} = [cos(Theta2{iter})     0     sin(Theta2{iter});0     1    0;-sin(Theta2{iter})     0     cos(Theta2{iter})];
        R3{iter}{i} = [cos(Theta3{iter})     0     sin(Theta3{iter});0     1    0;-sin(Theta3{iter})     0     cos(Theta3{iter})];
        R4{iter}{i} = [cos(Theta4{iter})     0     sin(Theta4{iter});0     1    0;-sin(Theta4{iter})     0     cos(Theta4{iter})];
        t1{iter}{i} = [Rc1{iter}(i)*sin(Theta1{iter})+center{iter}(1) y{iter} -Rc1{iter}(i)*cos(Theta1{iter})+center{iter}(3)];
        t2{iter}{i} = [Rc2{iter}(i)*sin(Theta2{iter})+center{iter}(1) y{iter} -Rc2{iter}(i)*cos(Theta2{iter})+center{iter}(3)];
        t3{iter}{i} = [Rc3{iter}(i)*sin(Theta3{iter})+center{iter}(1) y{iter} -Rc3{iter}(i)*cos(Theta3{iter})+center{iter}(3)];
        t4{iter}{i} = [Rc4{iter}(i)*sin(Theta4{iter})+center{iter}(1) y{iter} -Rc4{iter}(i)*cos(Theta4{iter})+center{iter}(3)];
    end

        for noise = 1:1
            for percent_miss = 1:6
                for nf = 1:1

                    cam1{iter} = CameraStruct(mot{iter}, K{iter}, R1{iter}, t1{iter}, noise_data(noise), 1, freq(nf));
                    cam2{iter} = CameraStruct(mot{iter}, K{iter}, R2{iter}, t2{iter}, noise_data(noise), 1+mot{iter}.samplingRate/freq(nf)/4, freq(nf));
                    cam3{iter} = CameraStruct(mot{iter}, K{iter}, R3{iter}, t3{iter}, noise_data(noise), 1+mot{iter}.samplingRate/freq(nf)*2/4, freq(nf));
                    cam4{iter} = CameraStruct(mot{iter}, K{iter}, R4{iter}, t4{iter}, noise_data(noise), 1+mot{iter}.samplingRate/freq(nf)*3/4, freq(nf)); 

                    ray_sum{iter} = [cam1{iter}.ray; cam2{iter}.ray; cam3{iter}.ray; cam4{iter}.ray];
                    t_sum{iter} = [cam1{iter}.t; cam2{iter}.t; cam3{iter}.t; cam4{iter}.t];
                    R_sum{iter} = [cam1{iter}.R; cam2{iter}.R; cam3{iter}.R; cam4{iter}.R];

                    nframes = size(ray_sum{iter},1);

                    cam_index{iter} = [cam1{iter}.frames cam1{iter}.frames+cam2{iter}.frames cam1{iter}.frames+cam2{iter}.frames+cam3{iter}.frames cam1{iter}.frames+cam2{iter}.frames+cam3{iter}.frames+cam4{iter}.frames];


                    X_GT{iter} = X_grdT{iter}(:, [cam1{iter}.start:mot{iter}.samplingRate/cam1{iter}.frequency:end cam2{iter}.start:mot{iter}.samplingRate/cam2{iter}.frequency:end...
                        cam3{iter}.start:mot{iter}.samplingRate/cam3{iter}.frequency:end cam4{iter}.start:mot{iter}.samplingRate/cam4{iter}.frequency:end]);

                    Sequence_GT = [cam1{iter}.start:mot{iter}.samplingRate/cam1{iter}.frequency:size(X_grdT{iter},2) cam2{iter}.start:mot{iter}.samplingRate/cam2{iter}.frequency:size(X_grdT{iter},2)...
                    cam3{iter}.start:mot{iter}.samplingRate/cam3{iter}.frequency:size(X_grdT{iter},2) cam4{iter}.start:mot{iter}.samplingRate/cam4{iter}.frequency:size(X_grdT{iter},2)];
                    [tmp,order] = sort(Sequence_GT);
                    
                    %====================================Waiting to be fixed==========================            
                    %randomly drop some frames
                    percent_frame_miss = (percent_miss-1)*0.1;
                    [ X_GT{iter}, ray_sum{iter}, t_sum{iter}, cam_index{iter}, NumMiss ] = FrameDrop(X_GT{iter}, ray_sum{iter}, t_sum{iter}, cam_index{iter}, percent_frame_miss);
                    Sequence_GT(NumMiss) = [];
                    [tmp,order] = sort(Sequence_GT);
                    nframes = size(ray_sum{iter},1);
                    %==================================================================================
                    
                    paramWL{iter}.Wprior = 0.1;
                    paramWL{iter}.Dconst = 0.5;
                    paramWL{iter}.lambda1 = 0.0015;
                    paramWL{iter}.lambda2 = 1e10;
                    paramWL{iter}.lambda3 = 0.03;
                    paramWL{iter}.itermax = 50;
                    paramWL{iter}.thres = 1e-4;
                    paramWL{iter}.Convdisp = false;
                    paramWL{iter}.f = 'SpPrior';

                    [ X_init{iter},~,~] = X_initial( ray_sum{iter}, t_sum{iter}, cam_index{iter});
                    %3D reconstruction error of triangulaton
                    %[user, sys] = memory;
%                    poolobj = gcp('nocreate');
%                     delete(poolobj)
%                     [tmp pid] = system('pgrep MATLAB');
%                     [tmp mem_usage] = system(['cat /proc/' strtrim(pid) '/status | grep VmSize']);
%                     mem_start = round(str2num(strtrim(extractAfter(extractBefore(mem_usage, ' kB'), ':'))) / 1000);
                    %fprintf("%i MB\n", round(str2num(strtrim(extractAfter(extractBefore(mem_usage, ' kB'), ':'))) / 1000));
                    tic;
                    [Xopt{iter}, ~, ~] = Triconvex_opt(X_init{iter}, ray_sum{iter}, t_sum{iter}, cam_index{iter},paramWL{iter}, false);
                    t_inter = toc;
%                     [tmp pid] = system('pgrep MATLAB');
%                     [tmp mem_usage] = system(['cat /proc/' strtrim(pid) '/status | grep VmSize']);
%                     mem_end = round(str2num(strtrim(extractAfter(extractBefore(mem_usage, ' kB'), ':'))) / 1000);
                    %[user2, sys2] = memory;
                    run_time = [run_time;[nframes, t_inter]];
                    %run_mem = [run_mem;[nframes, mem_end-mem_start]];
                    disp(run_time)
                    save('runtime3.mat', 'run_time')
                    %save('runmem3.mat', 'run_mem')
                    
                end
            end
        end
        
end

% run_time[2:5:end,:] = round(run_time[2:5:end,:]*90)

% [r,w] = unix('free | grep Mem');
% stats = str2double(regexp(w, '[0-9]*', 'match'));
% memsize = stats(1)/1e6;
% freemem = (stats(3)+stats(end))/1e6;

end
