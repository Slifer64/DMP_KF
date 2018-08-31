function plot_estimation_results(Time, g, g_data, tau, tau_data, P_data, F_data, mf_data, plot_1sigma)

    fontsize = 14;
    
    D = length(g);
    
    figure;
    for i=1:D
        subplot(D+1,1,i);
        hold on;
        plot([Time(1) Time(end)],[g(i) g(i)],'r--', 'LineWidth',2);
        plot(Time,g_data(i,:),'b-', 'LineWidth',1.5);
        legend_labels = {['$g_' num2str(i) '$'], ['$\hat{g}_' num2str(i) '$']};
        if (plot_1sigma)
            plot(Time,g_data(i,:)+P_data(i,:),'c-.', 'LineWidth',1.5);
        	plot(Time,g_data(i,:)-P_data(i,:),'c-.', 'LineWidth',1.5);
            legend_labels = [legend_labels, ['$\pm1\sigma$ bound']];
        end
        legend(legend_labels,'interpreter','latex','fontsize',fontsize);
        if (i==1), title('EKF-matlab prediction','interpreter','latex','fontsize',fontsize); end
        hold off;
    end
    subplot(D+1,1,D+1);
    hold on;
    plot([Time(1) Time(end)],[tau tau],'r--', 'LineWidth',2);
    plot(Time,tau_data,'b-', 'LineWidth',1.5);
    legend_labels = {['$\tau$'], ['$\hat{\tau}$']};
    if (plot_1sigma)
        plot(Time,tau_data+P_data(end,:),'c-.', 'LineWidth',1.5);
    	plot(Time,tau_data-P_data(end,:),'c-.', 'LineWidth',1.5);
        legend_labels = [legend_labels, ['$\pm1\sigma$ bound']];
    end
    legend(legend_labels,'interpreter','latex','fontsize',fontsize);
    xlabel('time [$s$]','interpreter','latex','fontsize',fontsize);
    hold off;
    
    f_label = {'f_x', 'f_y', 'f_z'};
    figure;
    for i=1:D
        subplot(D,1,i);
        plot(Time, F_data(i,:),'b-', 'LineWidth',1.5);
        if (i==1), title('Interaction force','interpreter','latex','fontsize',fontsize); end
        ylabel(['$' f_label{i} '$[$N$]'],'interpreter','latex','fontsize',fontsize);
        if (i==D), xlabel('time [$s$]','interpreter','latex','fontsize',fontsize); end
    end
    
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