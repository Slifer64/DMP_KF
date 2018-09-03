clc;
% close all;
clear;

set_matlab_utils_path();

% load('data/demo_data.mat','Data');
load('data/training_data.mat','Data');

N = length(Data);

N_kernels = 30;
a_z = 16;
b_z = a_z/4;
train_method = 'LWR';
dt = 0.005;

can_clock_ptr = CanonicalClock();


Data_sim = cell(N,1);
DMP_data = cell(N,1);

for n=1:N
    
    data = Data{n};
    
    Timed = data.Time;
    yd_data = data.Y;
    dyd_data = data.dY;
    ddyd_data = data.ddY;
    
    D = size(yd_data,1);

    dmp = cell(D,1);
    
    for i=1:D
        shapeAttrGatingPtr = SigmoidGatingFunction(1.0, 0.97);
%         shapeAttrGatingPtr = LinGatingFunction(1.0, 0.0);
%         shapeAttrGatingPtr = ExpGatingFunction(1.0, 0.05);
        dmp{i} = DMP(N_kernels, a_z, b_z, can_clock_ptr, shapeAttrGatingPtr);
    end
    
    %% Train the DMP
    disp('DMP training...')
    tic
    offline_train_mse = zeros(D,1); 
    n_data = size(yd_data,2);
    for i=1:D
        T = Timed;
        yd = yd_data(i,:);
        dyd = dyd_data(i,:);
        ddyd = ddyd_data(i,:);

        [offline_train_mse(i), F_train, Fd_train] = dmp{i}.train(train_method, T, yd, dyd, ddyd);      
    end
    
    offline_train_mse

    toc
    
    %% DMP simulation
    % set initial values
    y0 = yd_data(:,1);
    g0 = yd_data(:,end); 
    g = g0;
    x = 0.0;
    dx = 0.0;
    ddy = zeros(D,1);
    dy = zeros(D,1);
    y = y0;
    t = 0.0;
    dz = zeros(D,1);
    z = zeros(D,1);

    t_end = Timed(end);
    can_clock_ptr.setTau(t_end);
    
    iters = 0;
    Time = [];
    y_data = [];
    dy_data = [];
    ddy_data = [];
    dz_data = [];
    x_data = [];

    disp('DMP simulation...')
    tic
    while (true)

        %% data logging

        Time = [Time t];

        y_data = [y_data y];
        dy_data = [dy_data dy];  
        ddy_data = [ddy_data ddy];

        x_data = [x_data x];

        %% DMP simulation
        for i=1:D

            y_c = 0.0;
            z_c = 0.0;

            [dy(i), dz(i)] = dmp{i}.getStatesDot(x, y(i), z(i), y0(i), g(i), y_c, z_c);

            ddy(i) = dz(i)/dmp{i}.getTau();

        end

        %% Update phase variable
        dx = can_clock_ptr.getPhaseDot(x);

        %% Stopping criteria
        err_p = max(abs(g0-y));
        if (err_p <= 1e-3 && t>=t_end)
            break; 
        end

        iters = iters + 1;
        if (t>=t_end && norm(dy)<0.005)
            warning('Iteration limit reached. Stopping simulation...\n');
            break;
        end

        %% Numerical integration
        t = t + dt;

        x = x + dx*dt;

        y = y + dy*dt;
        z = z + dz*dt;
        
    end
    toc
    
    Data_sim{n} = struct('Time',Time, 'Y',y_data, 'dY',dy_data, 'ddY',ddy_data);

    DMP_data{n} = dmp;
    
    fontsize = 14;
    figure('NumberTitle', 'off', 'Name', ['Demo ' num2str(n)]);
    k = 1;
    for i=1:D
        subplot(D,3,k);
        plot(Time,y_data(i,:), Timed,yd_data(i,:));
        if (i==1), title('pos [$m$]','interpreter','latex','fontsize',fontsize); end
        if (i==D), xlabel('time [$s$]','interpreter','latex','fontsize',fontsize); end
        legend('dmp','demo');
        subplot(D,3,k+1);
        plot(Time,dy_data(i,:), Timed,dyd_data(i,:));
        if (i==1), title('vel [$m/s$]','interpreter','latex','fontsize',fontsize); end
        if (i==D), xlabel('time [$s$]','interpreter','latex','fontsize',fontsize); end
        subplot(D,3,k+2);
        plot(Time,ddy_data(i,:), Timed,ddyd_data(i,:));
        if (i==1), title('accel [$m/s^2$]','interpreter','latex','fontsize',fontsize); end
        if (i==D), xlabel('time [$s$]','interpreter','latex','fontsize',fontsize); end
        k = k+3;
    end

end

ax = cell(D,1);

figure;
for i=1:D
   ax{i} = subplot(D,1,i);
   ax{i}.NextPlot = 'add';
end

colors = {[0.75 0.75 0], [0.75 0 0.75], [0 0.75 0.75], [0 0 1], [0 0.5 0], [1 0.84 0], ...
    [0 0.45 0.74], [0.85 0.33 0.1], [1 0 0], [0.6 0.2 0], [1 0.6 0.78], [0.49 0.18 0.56]};

legends = cell(D,1);

for n=1:N
    dmp = DMP_data{n};
    for i=1:D
        legends{i} = [legends{i}, {['dim ' num2str(i) ' - demo ' num2str(n)]} ];
        bar(ax{i}, dmp{i}.w, 'BarWidth',1.0/n, 'FaceColor',colors{mod(n-1,length(colors))+1});
    end
end
for i=1:D
    legend(ax{i}, legends{i}, 'interpreter','latex', 'fontsize',14);
end
title(ax{1},'DMP weights');

save('data/dmp_data.mat', 'DMP_data');

