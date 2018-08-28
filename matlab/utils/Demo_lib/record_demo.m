function Data = record_demo(N_demos, Ts, N_kernels, dt)

if (nargin < 4), dt = Ts; end
if (nargin < 3), N_kernels = 10; end

% N_demos = 4;
% Ts = 0.01;
% N_kernels = 5;
% dt = 0.01;

raw_demo_data = record_raw_demo(N_demos);

demo_data = process_demo_data(raw_demo_data, Ts);

Data = process_data(demo_data, N_kernels, dt);

plot_demos(Data);

end

% ####################################################################
% ####################################################################

function demo_data = record_raw_demo(N_demos)

demo_data = cell(N_demos,1);
Time = cell(N_demos,1);

figure;
xlim([-1.5 1.5]);
ylim([-1.5 1.5]);
    
for n=1:N_demos
    
    %% Draw an abstract shape
    disp(['Recording demo ' num2str(n) '/' num2str(N_demos) '...']);
    h = imfreehand;
    disp('(Double click on the recorded path to finish recording)');
    pos_data = wait(h); % double click on the line to continue
    disp('Recorded!');

    n_points = size(pos_data,1);
    demo_data{n} = pos_data;
    
end

end

% ####################################################################
% ####################################################################


function Data = process_demo_data(demo_data, Ts)

N = length(demo_data);

Data = cell(N,1);

for n=1:N
    data = demo_data{n}';
    
    data_dot = [zeros(2,1) diff(data,1,2)]/Ts;
    
    data_ddot = [zeros(2,1) diff(data_dot,1,2)]/Ts;
    
    Time = (0:(size(data,2)-1))*Ts;
    
    Data{n} = struct('Time',Time, 'Y',data, 'dY',data_dot, 'ddY',data_ddot);
    
    disp(['Processed demo' num2str(n) '/' num2str(N) ]);
    
end

end

% ####################################################################
% ####################################################################

function Data2 = process_data(Data, N_kernels, dt)

N = length(Data);

a_z = 16;
b_z = a_z/4;
train_method = 'LWR';

canClockPtr = LinCanonicalClock();

Data_sim = cell(N,1);

for n=1:N
    
    data = Data{n};
    
    Timed = data.Time;
    yd_data = data.Y;
    dyd_data = data.dY;
    ddyd_data = data.ddY;
    
    D = size(yd_data,1);

    dmp = cell(D,1);
    
    for i=1:D
        dmp{i} = DMP(N_kernels, a_z, b_z, canClockPtr);
    end
    
    %% Train the DMP
    offline_train_mse = zeros(D,1); 
    for i=1:D
        T = Timed;
        yd = yd_data(i,:);
        dyd = dyd_data(i,:);
        ddyd = ddyd_data(i,:);

        [offline_train_mse(i), F_train, Fd_train] = dmp{i}.train(train_method, T, yd, dyd, ddyd);      
    end
    
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
    canClockPtr.setTau(t_end);

    iters = 0;
    Time = [];
    y_data = [];
    dy_data = [];
    ddy_data = [];

    while (true)

        %% data logging

        Time = [Time t];

        y_data = [y_data y];
        dy_data = [dy_data dy];   
        ddy_data = [ddy_data ddy];

        %% DMP simulation
        for i=1:D

            y_c = 0.0;
            z_c = 0.0;

            [dy(i), dz(i)] = dmp{i}.getStatesDot(x, y(i), z(i), y0(i), g(i), y_c, z_c);

            ddy(i) = dz(i)/dmp{i}.getTau();

        end

        %% Update phase variable
        dx = canClockPtr.getPhaseDot(x);

        %% Stopping criteria
        err_p = max(abs(g0-y));
        if (err_p <= 1e-3 ...
            && t>=t_end)
            break; 
        end

        iters = iters + 1;
        if (t>=t_end && norm(dy)<0.001) %&& iters>=600)
%             warning('Iteration limit reached. Stopping simulation...\n');
            break;
        end

        %% Numerical integration
        t = t + dt;

        x = x + dx*dt;

        y = y + dy*dt;
        z = z + dz*dt;
        
    end
%     toc
    
    Data_sim{n} = struct('Time',Time, 'Y',y_data, 'dY',dy_data, 'ddY',ddy_data);
    
    disp(['Processed demo' num2str(n) '/' num2str(N) ]);
     
end

Data2 = Data_sim;

end


% ####################################################################
% ####################################################################


function plot_demos(Data)

    N = length(Data);

    for n=1:N

        Time = Data{n}.Time;
        y_data = Data{n}.Y;
        dy_data = Data{n}.dY;
        ddy_data = Data{n}.ddY;

        D = size(y_data,1);

        fontsize = 14;
        figure('NumberTitle', 'off', 'Name', ['Demo ' num2str(n)]);
        for i=1:D
            subplot(D,3,(i-1)*(D+1)+1);
            plot(Time,y_data(i,:));
            ylabel(['dim-$' num2str(i) '$'],'interpreter','latex','fontsize',fontsize);
            if (i==1), title('pos [$m$]','interpreter','latex','fontsize',fontsize); end
            if (i==D), xlabel('time [$s$]','interpreter','latex','fontsize',fontsize); end
            subplot(D,3,(i-1)*(D+1)+2);
            plot(Time,dy_data(i,:));
            if (i==1), title('vel [$m/s$]','interpreter','latex','fontsize',fontsize); end
            if (i==D), xlabel('time [$s$]','interpreter','latex','fontsize',fontsize); end
            subplot(D,3,(i-1)*(D+1)+3);
            plot(Time,ddy_data(i,:));
            if (i==1), title('accel [$m/s^2$]','interpreter','latex','fontsize',fontsize); end
            if (i==D), xlabel('time [$s$]','interpreter','latex','fontsize',fontsize); end
        end

    end

end