close all
clear all
clc

load runtime3.mat
run_time3 = run_time;
load runtime.mat
load run_time1.mat

run_time2 = pickle_data;
load run_time2.mat
run_time2 = [run_time2;pickle_data];

run_time_avg = zeros(6, 1);

tmp = run_time(find(run_time(:,1) < 100),2);
run_time_avg(1) = sum(tmp)/length(tmp);

tmp = run_time(find(run_time(:,1) >= 100 & run_time(:,1) < 200),2);
run_time_avg(2) = sum(tmp)/length(tmp);

tmp = run_time(find(run_time(:,1) >= 200 & run_time(:,1) < 300),2);
run_time_avg(3) = sum(tmp)/length(tmp);

tmp = run_time(find(run_time(:,1) >= 300 & run_time(:,1) < 400),2);
run_time_avg(4) = sum(tmp)/length(tmp);

tmp = run_time(find(run_time(:,1) >= 400 & run_time(:,1) < 500),2);
run_time_avg(5) = sum(tmp)/length(tmp);

tmp = run_time(find(run_time(:,1) >= 500),2);
run_time_avg(6) = sum(tmp)/length(tmp);

%=================== GTT ====================
run_time_avg3 = zeros(6, 1);
tmp = run_time3(find(run_time3(:,1) < 100),2);
run_time_avg3(1) = sum(tmp)/length(tmp);

tmp = run_time3(find(run_time3(:,1) >= 100 & run_time3(:,1) < 200),2);
run_time_avg3(2) = sum(tmp)/length(tmp);

tmp = run_time3(find(run_time3(:,1) >= 200 & run_time3(:,1) < 300),2);
run_time_avg3(3) = sum(tmp)/length(tmp);

tmp = run_time3(find(run_time3(:,1) >= 300 & run_time3(:,1) < 400),2);
run_time_avg3(4) = sum(tmp)/length(tmp);

tmp = run_time3(find(run_time3(:,1) >= 400 & run_time3(:,1) < 500),2);
run_time_avg3(5) = sum(tmp)/length(tmp);

tmp = run_time3(find(run_time3(:,1) >= 500),2);
run_time_avg3(6) = sum(tmp)/length(tmp);

%=================== GTT ====================
run_time_avg2 = zeros(6, 1);
tmp = run_time2(find(run_time2(:,1) < 100),2);
run_time_avg2(1) = sum(tmp)/length(tmp);

tmp = run_time2(find(run_time2(:,1) >= 100 & run_time2(:,1) < 200),2);
run_time_avg2(2) = sum(tmp)/length(tmp);

tmp = run_time2(find(run_time2(:,1) >= 200 & run_time2(:,1) < 300),2);
run_time_avg2(3) = sum(tmp)/length(tmp);

tmp = run_time2(find(run_time2(:,1) >= 300 & run_time2(:,1) < 400),2);
run_time_avg2(4) = sum(tmp)/length(tmp);

tmp = run_time2(find(run_time2(:,1) >= 400 & run_time2(:,1) < 500),2);
run_time_avg2(5) = sum(tmp)/length(tmp);

tmp = run_time2(find(run_time2(:,1) >= 500),2);
run_time_avg2(6) = sum(tmp)/length(tmp);


figure
hold on
grid on
X = categorical({'100~200','200~300','300~400','400~500','500+'});
X = reordercats(X,{'100~200','200~300','300~400','400~500','500+'});
Y = [run_time_avg(2:end), run_time_avg3(2:end), run_time_avg2(2:end)];
plot(X, run_time_avg(2:end), 'y', 'LineWidth', 2)
plot(X, run_time_avg3(2:end), 'b', 'LineWidth', 2)
plot(X, run_time_avg2(2:end), 'r', 'LineWidth', 2)
b = bar(X, Y);
b(1).FaceColor = 'y';
b(2).FaceColor = 'b';
b(3).FaceColor = 'r';
legend({'DLOE','DLOE-parallel','GTT-Net'}, 'FontSize', 15)
xlabel('Number of frames', 'FontSize', 20)
ylabel('Running time(s)', 'FontSize', 20)
