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
N_kernels = 8;
a_z = 16;
b_z = a_z/4;
train_method = 'LS';
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
linewidth = 1.5;
figure('NumberTitle', 'off', 'Name', ['Demo ' num2str(1)]);
k = 1;
ax_cell = cell(3,3);
for i=1:3
    for j=1:3
        ax_cell{i,j} = subplot(3,3,k);
        hold(ax_cell{i,j}, 'on');
        k = k + 1;
    end
end
for i=1:3
    plot(Timed,Yd_data(i,:), 'LineWidth',linewidth, 'Color',[0.85 0.33 0.1], 'Parent',ax_cell{1,i});
    plot(Time,Y_data(i,:), 'LineWidth',linewidth, 'Color','blue', 'Parent',ax_cell{1,i}); 
end
for i=1:3
    plot(Timed,dYd_data(i,:), 'LineWidth',linewidth, 'Color',[0.85 0.33 0.1], 'Parent',ax_cell{2,i});
    plot(Time,dY_data(i,:), 'LineWidth',linewidth, 'Color','blue', 'Parent',ax_cell{2,i}); 
end
for i=1:3
    plot(Timed,ddYd_data(i,:), 'LineWidth',linewidth, 'Color',[0.85 0.33 0.1], 'Parent',ax_cell{3,i});
    plot(Time,ddY_data(i,:), 'LineWidth',linewidth, 'Color','blue', 'Parent',ax_cell{3,i}); 
end
legend(ax_cell{1,3}, {'dmp','demo'},'interpreter','latex','fontsize',fontsize);
title(ax_cell{1,1}, '$X$','interpreter','latex','fontsize',fontsize);
title(ax_cell{1,2}, '$Y$','interpreter','latex','fontsize',fontsize);
title(ax_cell{1,3}, '$Z$','interpreter','latex','fontsize',fontsize);
ylabel(ax_cell{1,1}, 'position [$m$]','interpreter','latex','fontsize',fontsize);
ylabel(ax_cell{2,1}, 'velocity [$m/s$]','interpreter','latex','fontsize',fontsize);
ylabel(ax_cell{3,1}, 'acceleration [$m/s^2$]','interpreter','latex','fontsize',fontsize);
title(ax_cell{3,1}, 'time [$s$]','interpreter','latex','fontsize',fontsize);
title(ax_cell{3,2}, 'time [$s$]','interpreter','latex','fontsize',fontsize);
title(ax_cell{3,3}, 'time [$s$]','interpreter','latex','fontsize',fontsize);

end


