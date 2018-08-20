clc;
close all;
clear;

set_matlab_utils_path();

load('data/dmp_data.mat', 'DMP_data');

N = length(DMP_data);
D = length(DMP_data{1});

DMP_w = cell(D,1);

for d=1:D
    
    DMP_w{d} = [];
    
    for n=1:N
        DMP_w{d} = [DMP_w{d} DMP_data{n}{d}.w];
    end
    
end

dmpm = cell(D,1);

for d=1:D
    
    w_data = DMP_w{d};
    
    D_w = size(w_data,1);
    N_w = size(w_data,2);
    
    Mu_w = sum(w_data,2)/N_w;
    
    temp = w_data - repmat(Mu_w, 1, N_w);
    Sigma_w = temp*temp'/N_w;

    dmpm{d} = struct('Priors',1.0, 'Mu',Mu_w, 'Sigma',Sigma_w);
end




load('data/data.mat','Data');

N = length(Data);

dt = 0.01;

Data_sim = cell(N,1);

for n=1:N
    
    data = Data{n};
    
    Timed = data.Time;
    yd_data = data.Y;
    dyd_data = data.dY;
    ddyd_data = data.ddY;
    
    D = size(yd_data,1);

    dmp = cell(D,1);
    
    for d=1:D
        dmp{d} = DMP_data{1}{d};
        
%         for k=1:length(dmpm{d}.Priors)
%             dmpm{d}.Sigma(:,:,k) = 100*eye(size(dmpm{d}.Sigma(:,:,k)));
%         end
    end

    canClockPtr = dmp{1}.canClockPtr;
    
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
    dz_data = [];
    x_data = [];
    
%     figure('NumberTitle', 'off', 'Name', ['Sim ' num2str(n)]);

    it = 0;
    
    disp('DMPM simulation...')
    tic
    
    N_data = size(yd_data,2);
    
    f = [];
    C = [];
    
    Mu_w = cell(D,1);
    Sigma_w = cell(D,1);
    for d=1:D
       Mu_w{d} = dmpm{d}.Mu;
%         Mu_w{d} = zeros(size(dmpm{d}.Mu));
       
       Sigma_w{d} = dmpm{d}.Sigma;
%        Sigma_w{d} = eye(size(dmpm{d}.Sigma))*10000;
       
       dmp{d}.w = Mu_w{d};
    end
    
    
    
    for j=1:N_data

        %% data logging

        Time = [Time t];

        y_data = [y_data y];
        dy_data = [dy_data dy];  
        ddy_data = [ddy_data ddy];

        x_data = [x_data x];

        %% DMP simulation
        for i=1:D
            
            yd = yd_data(:,j);
            dyd = dyd_data(:,j);
            ddyd = ddyd_data(:,j);
            
            if (it == 0)
                
%                 it = it + 1;
                
                % update DMP weights
                tau = dmp{i}.getTau();
%                 f = [f; ddyd(i)*tau^2 - dmp{i}.a_z * (dmp{i}.b_z*(g(i)-yd(i)) - dyd(i)*tau)];
                f = ddyd(i)*tau^2 - dmp{i}.a_z * (dmp{i}.b_z*(g(i)-yd(i)) - dyd(i)*tau);
                psi = dmp{i}.kernelFunction(x);
                psi = psi / (sum(psi)+dmp{i}.zero_tol);
%                 C = [C dmp{i}.shapeAttrGating(x) * dmp{i}.forcingTermScaling(y0(i), g(i))*psi];
                C = dmp{i}.shapeAttrGating(x) * dmp{i}.forcingTermScaling(y0(i), g(i))*psi;
                f_hat = C'*dmp{i}.w;
                
                Mu_f = C'*Mu_w{i};
                Sigma_f = C'*Sigma_w{i}*C;
                Sigma_f = Sigma_f + 0.0*eye(size(Sigma_f));

                Sigma_wf = Sigma_w{i}*C;
%                 inv_Sigma_f = inv(Sigma_f);
                
                K_g = Sigma_wf/Sigma_f;

%                 Mu_w_temp = Mu_w{i} + K_g*(f-f_hat);
%                 dmp{i}.w = Mu_w{i};
                
                Mu_w{i} = Mu_w{i} + K_g*(f-f_hat);
                Sigma_w{i} = Sigma_w{i} - K_g*Sigma_wf';
                dmp{i}.w = Mu_w{i};

%                 figure;
%                 hold on;
%                 bar(w_new, 'BarWidth',1.0, 'FaceColor','blue');
%                 bar(w1, 'BarWidth',0.5, 'FaceColor','red');
%                 hold off;
                
            
            end

            y_c = 0.0;
            z_c = 0.0;

            [dy(i), dz(i)] = dmp{i}.getStatesDot(x, y(i), z(i), y0(i), g(i), y_c, z_c);

            ddy(i) = dz(i)/dmp{i}.getTau();

        end

        %% Update phase variable
        dx = canClockPtr.getPhaseDot(x);

        %% Stopping criteria
        err_p = max(abs(g0-y));
        if (err_p <= 1e-3 && t>=t_end)
            break; 
        end

        iters = iters + 1;
        if (t>=t_end && iters>=600)
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
    for i=1:D
        subplot(D,3,(i-1)*(D+1)+1);
        plot(Time,y_data(i,:), Timed,yd_data(i,:));
        legend('dmp','demo');
        if (i==1), title('pos [$m$]','interpreter','latex','fontsize',fontsize); end
        if (i==D), xlabel('time [$s$]','interpreter','latex','fontsize',fontsize); end
        subplot(D,3,(i-1)*(D+1)+2);
        plot(Time,dy_data(i,:), Timed,dyd_data(i,:));
        if (i==1), title('vel [$m/s$]','interpreter','latex','fontsize',fontsize); end
        if (i==D), xlabel('time [$s$]','interpreter','latex','fontsize',fontsize); end
        subplot(D,3,(i-1)*(D+1)+3);
        plot(Time,ddy_data(i,:), Timed,ddyd_data(i,:));
        if (i==1), title('accel [$m/s^2$]','interpreter','latex','fontsize',fontsize); end
        if (i==D), xlabel('time [$s$]','interpreter','latex','fontsize',fontsize); end
    end
    

end


figure;
ax = cell(D,1);
for i=1:D
    ax{i} = subplot(D,1,i);
    ax{i}.NextPlot = 'add';
end
for i=1:D
    for n=1:length(Data_sim)
        plot(ax{i}, Data_sim{n}.Time,Data_sim{n}.Y(i,:));
    end
end
