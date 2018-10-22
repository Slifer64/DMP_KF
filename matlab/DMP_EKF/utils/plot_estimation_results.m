function plot_estimation_results(Time, g, g_data, tau, tau_data, P_data, F_data, mf_data, plot_1sigma, y_data)

    fontsize = 16;
    linewidth = 1.5;
    
    D = length(g);
    
    axis_name = {'x', 'y', 'z'};
    
    goal_color = {[1 0 0], [0 0.5 0], [0 0 1]};
    goal_est_color = {[1 0 1], [0 1 0], [0 1 1]};
    pos_color = {[0.85 0.7 1], [0.75 0.75 0], [0 0.45 0.75]};
    tau_color = [0 0 1];
    tau_hat_color = [0.85 0.33 0.1];
    
    figure;
    subplot(2,1,1);
    hold on;
    legend_labels = {};
    for i=1:D
        % subplot(D+1,1,i);
        % hold on;
        plot([Time(1) Time(end)],[g(i) g(i)], 'LineStyle','--', 'Color',goal_color{i} ,'LineWidth',2);
        plot(Time,g_data(i,:), 'LineStyle','-', 'Color',goal_est_color{i}, 'LineWidth',linewidth);
        plot(Time,y_data(i,:), 'LineStyle','-', 'Color',pos_color{i}, 'LineWidth',linewidth);
        legend_labels = [legend_labels, ['$\mathbf{y}_{g,' axis_name{i} '}$'], ['$\hat{\mathbf{y}}_{g,' axis_name{i} '}$'], ['$\mathbf{y}_{' axis_name{i} '}$']];
        if (plot_1sigma)
            plot(Time,g_data(i,:)+P_data(i,:),'c-.', 'LineWidth',linewidth);
        	plot(Time,g_data(i,:)-P_data(i,:),'c-.', 'LineWidth',linewidth);
            legend_labels = [legend_labels, ['$\pm1\sigma$']];
        end
        % ylabel([axis_name{i} ' [$m$]'],'interpreter','latex','fontsize',fontsize);
        ylabel('[$m$]','interpreter','latex','fontsize',fontsize);
        % if (i==1), title('EKF prediction','interpreter','latex','fontsize',fontsize); end
        axis tight;
        % hold off;
    end
    legend(legend_labels,'interpreter','latex','fontsize',fontsize);
    hold off;
    % subplot(D+1,1,D+1);
    subplot(2,1,2);
    hold on;
    plot([Time(1) Time(end)],[tau tau], 'LineStyle','--', 'Color',tau_color, 'LineWidth',2);
    plot(Time,tau_data, 'LineStyle','-', 'Color',tau_hat_color, 'LineWidth',linewidth);
    legend_labels = {['$\tau$'], ['$\hat{\tau}$']};
    if (plot_1sigma)
        plot(Time,tau_data+P_data(end,:),'c-.', 'LineWidth',linewidth);
    	plot(Time,tau_data-P_data(end,:),'c-.', 'LineWidth',linewidth);
        legend_labels = [legend_labels, ['$\pm1\sigma$']];
    end
    legend(legend_labels,'interpreter','latex','fontsize',fontsize);
    ylabel('$\tau$ [$s$]','interpreter','latex','fontsize',fontsize);
    xlabel('time [$s$]','interpreter','latex','fontsize',fontsize);
    axis tight;
    hold off;
    
%     f_label = {'f_x', 'f_y', 'f_z'};
%     figure;
%     for i=1:D
%         subplot(D,1,i);
%         plot(Time, F_data(i,:),'b-', 'LineWidth',linewidth);
%         if (i==1), title('Interaction force','interpreter','latex','fontsize',fontsize); end
%         ylabel(['$' f_label{i} '$[$N$]'],'interpreter','latex','fontsize',fontsize);
%         if (i==D), xlabel('time [$s$]','interpreter','latex','fontsize',fontsize); end
%     end
    
    f_norm = zeros(size(F_data,2),1);
    for i=1:length(f_norm)
        f_norm(i) = norm(F_data(:,i));
    end

    figure;

    subplot(2,1,1);
    plot(Time, f_norm,'b-', 'LineWidth',linewidth);
    title('Interaction force','interpreter','latex','fontsize',fontsize);
    ylabel(['$||\mathbf{f}_{ext}||$ [$N$]'],'interpreter','latex','fontsize',fontsize);
    xlabel('time [$s$]','interpreter','latex','fontsize',fontsize);
    axis tight;

    subplot(2,1,2);
    hold on;
    plot(Time, mf_data,'b-', 'LineWidth',linewidth);
    plot(Time, 1-mf_data,'g-', 'LineWidth',linewidth);
    title('Leader-follower role','interpreter','latex','fontsize',fontsize);
    legend({'leader','$follower$'},'interpreter','latex','fontsize',fontsize);
    ylabel('activation','interpreter','latex','fontsize',fontsize);
    xlabel('time [$s$]','interpreter','latex','fontsize',fontsize);
    axis tight;
    hold off;
    

end