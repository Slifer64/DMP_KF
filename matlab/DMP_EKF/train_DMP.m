function train_DMP()

set_matlab_utils_path();

%% Load training data
load('data/training_data.mat','Data');
    
data = Data{1};

Timed = data.Time;
Yd_data = data.Y;
dYd_data = data.dY;
ddYd_data = data.ddY;

Dim = size(Yd_data,1);

%% initialize DMP
N_kernels = 10;
a_z = 16;
b_z = a_z/4;
train_method = 'LWR';
can_clock_ptr = CanonicalClock();
dmp = cell(Dim,1);
shapeAttrGatingPtr = SigmoidGatingFunction(1.0, 0.5);
% shapeAttrGatingPtr = LinGatingFunction(1.0, 0.0);
% shapeAttrGatingPtr = ExpGatingFunction(1.0, 0.05);
for i=1:Dim 
    dmp{i} = DMP(N_kernels, a_z, b_z, can_clock_ptr, shapeAttrGatingPtr);
end

%% Train the DMP
disp('DMP training...')
tic
offline_train_mse = zeros(Dim,1); 
n_data = size(Yd_data,2);
for i=1:Dim
    [offline_train_mse(i), F_train, Fd_train] = dmp{i}.train(train_method, Timed, Yd_data(i,:), dYd_data(i,:), ddYd_data(i,:));      
end
offline_train_mse

toc

%% DMP simulation
disp('DMP simulation...');
tic
y0 = Yd_data(:,1);
g = Yd_data(:,end); 
T = Timed(end);
dt = 0.005;
[Time, Y_data, dY_data, ddY_data] = simulateDMP(dmp, y0, g, T, dt);
toc


%% save data
dmp_data = struct('dmp',{dmp}, 'g', g, 'y0',y0, 'tau',T, 'train_method',train_method);
save('data/dmp_data.mat', 'dmp_data');


%% plot data
fontsize = 14;
figure('NumberTitle', 'off', 'Name', ['Demo ' num2str(1)]);
k = 1;
for i=1:Dim
    subplot(Dim,3,k);
    plot(Time,Y_data(i,:), Timed,Yd_data(i,:));
    if (i==1), title('pos [$m$]','interpreter','latex','fontsize',fontsize); end
    if (i==Dim), xlabel('time [$s$]','interpreter','latex','fontsize',fontsize); end
    legend('dmp','demo');
    subplot(Dim,3,k+1);
    plot(Time,dY_data(i,:), Timed,dYd_data(i,:));
    if (i==1), title('vel [$m/s$]','interpreter','latex','fontsize',fontsize); end
    if (i==Dim), xlabel('time [$s$]','interpreter','latex','fontsize',fontsize); end
    subplot(Dim,3,k+2);
    plot(Time,ddY_data(i,:), Timed,ddYd_data(i,:));
    if (i==1), title('accel [$m/s^2$]','interpreter','latex','fontsize',fontsize); end
    if (i==Dim), xlabel('time [$s$]','interpreter','latex','fontsize',fontsize); end
    k = k+3;
end

end


