function create_demo()

clc;
close all
clear;

Ts = 0.002;
t_end = 6;

Time = 0:Ts:t_end;

y0 = [0.5; 0.1; 0.3];
g = y0 + [1.0; 0.79; 0.9];

D = size(g,1);
N_data = length(Time);

y_data = y0 + (g-y0)*(10*(Time/t_end).^3 - 15*(Time/t_end).^4 + 6*(Time/t_end).^5) + 0.8*[0.2; 0.3; 0.1]*exp(-2*(Time-3.0).^2) + 0.85*[0.2; 0.3; 0.1]*exp(-1.5*(Time-1.9).^2);
dy_data = (g-y0)*(3*10*(Time/t_end).^2 - 4*15*(Time/t_end).^3 + 5*6*(Time/t_end).^4)/t_end;
ddy_data = (g-y0)*(2*3*10*(Time/t_end).^1 - 3*4*15*(Time/t_end).^2 + 4*5*6*(Time/t_end).^3)/t_end^2;

for i=1:size(y_data,1)
    dy_data(i,:) = diff([y_data(i,1) y_data(i,:)])/Ts;
    ddy_data(i,:) = diff([dy_data(i,1) dy_data(i,:)])/Ts;
end
% dy_hat = diff([y(1) y])/Ts;
% ddy_hat = diff([dy(1) dy])/Ts;
% 
% figure;
% subplot(3,1,1);
% plot(t, y);
% subplot(3,1,2);
% plot(t, dy, t,dy_hat);
% subplot(3,1,3);
% plot(t, ddy, t,ddy_hat);

N_demos = 1;
Data = cell(N_demos,1);

Data{1} = struct('Time',Time, 'Y',y_data, 'dY',dy_data, 'ddY',ddy_data);

plot_demos(Data);
figure;
plot(y_data(1,:), y_data(2,:));

save('data/demo_data.mat','Data');

end

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
        k = 1;
        for i=1:D
            subplot(D,3,k);
            plot(Time,y_data(i,:));
            ylabel(['dim-$' num2str(i) '$'],'interpreter','latex','fontsize',fontsize);
            if (i==1), title('pos [$m$]','interpreter','latex','fontsize',fontsize); end
            if (i==D), xlabel('time [$s$]','interpreter','latex','fontsize',fontsize); end
            subplot(D,3,k+1);
            plot(Time,dy_data(i,:));
            if (i==1), title('vel [$m/s$]','interpreter','latex','fontsize',fontsize); end
            if (i==D), xlabel('time [$s$]','interpreter','latex','fontsize',fontsize); end
            subplot(D,3,k+2);
            plot(Time,ddy_data(i,:));
            if (i==1), title('accel [$m/s^2$]','interpreter','latex','fontsize',fontsize); end
            if (i==D), xlabel('time [$s$]','interpreter','latex','fontsize',fontsize); end
            k = k+3;
        end

    end

end