function plotData(Data)

    N = length(Data);

    for n=1:N

        Time = Data{n}.Time;
        y_data = Data{n}.Y;
        dy_data = Data{n}.dY;
        ddy_data = Data{n}.ddY;

        D = size(y_data,1);

        k = 1;
        
        fontsize = 14;
        figure('NumberTitle', 'off', 'Name', ['Data ' num2str(n)]);
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