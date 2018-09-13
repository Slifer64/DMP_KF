function plot_estimation_results(Time, g, g_data, tau, tau_data, P_data, F_data, mf_data, plot_1sigma, Y_data)

    fontsize = 16;
    linewidth = 1.5;
    
    D = length(g);
    
    axis_name = {'x', 'y', 'z'};
    
    figure;
    for i=1:D
        subplot(D+1,1,i);
        hold on;
        plot([Time(1) Time(end)],[g(i) g(i)],'r--', 'LineWidth',2);
        plot(Time,g_data(i,:),'b-', 'LineWidth',linewidth);
%         plot(Time,Y_data(i,:),'g-.', 'LineWidth',1.2);
        legend_labels = {['$y_{g,' num2str(i) '}$'], ['$\hat{y}_{g,' num2str(i) '}$']};
%         legend_labels = {['$y_{g,' num2str(i) '}$'], ['$\hat{y}_{g,' num2str(i) '}$'], ['$y_' num2str(i) '$']};
        if (plot_1sigma)
            plot(Time,g_data(i,:)+P_data(i,:),'c-.', 'LineWidth',linewidth);
        	plot(Time,g_data(i,:)-P_data(i,:),'c-.', 'LineWidth',linewidth);
            legend_labels = [legend_labels, ['$\pm1\sigma$ bound']];
        end
        legend(legend_labels,'interpreter','latex','fontsize',fontsize);
        if (i==1), title('EKF prediction','interpreter','latex','fontsize',fontsize); end
        ylabel([axis_name{i} ' [$m$]'],'interpreter','latex','fontsize',fontsize);
        axis tight
        hold off;
    end
    subplot(D+1,1,D+1);
    hold on;
%     plot([Time(1) Time(end)],[tau tau],'r--', 'LineWidth',2);
    plot(Time,tau_data,'b-', 'LineWidth',1.5);
    legend_labels = {['$\tau$'], ['$\hat{\tau}$']};
    if (plot_1sigma)
        plot(Time,tau_data+P_data(end,:),'c-.', 'LineWidth',linewidth);
    	plot(Time,tau_data-P_data(end,:),'c-.', 'LineWidth',linewidth);
        legend_labels = [legend_labels, ['$\pm1\sigma$ bound']];
    end
%     legend(legend_labels,'interpreter','latex','fontsize',fontsize);
    xlabel('time [$s$]','interpreter','latex','fontsize',fontsize);
    ylabel('$\tau$ [$m$]','interpreter','latex','fontsize',fontsize);
    axis tight
    hold off;
    
%     f_label = {'f_x', 'f_y', 'f_z'};
%     figure;
%     for i=1:D
%         subplot(D,1,i);
%         plot(Time, F_data(i,:),'b-', 'LineWidth',1.5);
%         if (i==1), title('Interaction force','interpreter','latex','fontsize',fontsize); end
%         ylabel(['$' f_label{i} '$[$N$]'],'interpreter','latex','fontsize',fontsize);
%         if (i==D), xlabel('time [$s$]','interpreter','latex','fontsize',fontsize); end
%     end

    figure;
    hold on;
    plot(Time, F_data(1,:), 'LineWidth',linewidth);
    plot(Time, F_data(2,:), 'LineWidth',linewidth);
    plot(Time, F_data(3,:), 'LineWidth',linewidth);
    ylabel(['Force [$N$]'],'interpreter','latex','fontsize',fontsize);
    xlabel('time [$s$]','interpreter','latex','fontsize',fontsize);
    legend({'$f_x$','$f_y$','$f_z$'},'interpreter','latex','fontsize',fontsize);
    title('Interaction force','interpreter','latex','fontsize',fontsize);
    
    figure;
    hold on;
    plot(Time, Y_data(1,:), 'LineWidth',linewidth);
    plot(Time, Y_data(2,:), 'LineWidth',linewidth);
    plot(Time, Y_data(3,:), 'LineWidth',linewidth);
    xlabel('time [$s$]', 'interpreter','latex', 'fontsize',fontsize);
    ylabel('position [$m$]', 'interpreter','latex', 'fontsize',fontsize);
    legend({'$X$','$Y$','$Z$'}, 'interpreter','latex', 'fontsize',fontsize);
    title('Robot end-effector Cartesian position','interpreter','latex','fontsize',fontsize);
    hold off;
    
    
    figure;
    hold on;
    plot(Time, mf_data,'b-', 'LineWidth',1.5);
    plot(Time, 1-mf_data,'g-', 'LineWidth',1.5);
    title('Leader-follower role','interpreter','latex','fontsize',fontsize);
    legend({'leader','$follower$'},'interpreter','latex','fontsize',fontsize);
    ylabel('activation','interpreter','latex','fontsize',fontsize);
    xlabel('time [$s$]','interpreter','latex','fontsize',fontsize);
    hold off;

end