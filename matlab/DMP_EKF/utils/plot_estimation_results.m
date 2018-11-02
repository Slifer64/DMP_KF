function plot_estimation_results(Time, Yg, Yg_data, tau, tau_data, Sigma_theta_data, plot_1sigma, F_data, Y_data, dY_data)

    fontsize = 16;
    linewidth = 1.5;
    
    D = length(Yg);
    
    axis_name = {'x', 'y', 'z'};
    
    goal_color = {[1 0 0], [0 0.5 0], [0 0 1]};
    goal_est_color = {[1 0 1], [0 1 0], [0 1 1]};
    pos_color = {[0.85 0.7 1], [0.75 0.75 0], [0 0.45 0.75]};
    tau_color = [0 0 1];
    tau_hat_color = [0.85 0.33 0.1];
    
    fig1 = figure('Position', [150 150 800 1000]);
    axes1 = axes('Parent',fig1, 'Position',[0.13 0.595 0.775 0.33]);
    hold(axes1,'on');
    legend_labels = {};
    for i=1:D
        plot([Time(1) Time(end)],[Yg(i) Yg(i)], 'LineStyle','--', 'Color',goal_color{i} ,'LineWidth',2, 'Parent',axes1);
        plot(Time,Yg_data(i,:), 'LineStyle','-', 'Color',goal_est_color{i}, 'LineWidth',linewidth, 'Parent',axes1);
        plot(Time,Y_data(i,:), 'LineStyle','-', 'Color',pos_color{i}, 'LineWidth',linewidth, 'Parent',axes1);
        legend_labels = [legend_labels, ['$\mathbf{y}_{g,' axis_name{i} '}$'], ['$\hat{\mathbf{y}}_{g,' axis_name{i} '}$'], ['$\mathbf{y}_{' axis_name{i} '}$']];
        if (plot_1sigma)
            plot(Time,Yg_data(i,:)+Sigma_theta_data(i,:),'c-.', 'LineWidth',linewidth, 'Parent',axes1);
        	plot(Time,Yg_data(i,:)-Sigma_theta_data(i,:),'c-.', 'LineWidth',linewidth, 'Parent',axes1);
            legend_labels = [legend_labels, ['$\pm1\sigma$']];
        end
        ylabel('[$m$]','interpreter','latex','fontsize',fontsize, 'Parent',axes1);
        axis tight;
    end
    legend(axes1, legend_labels,'interpreter','latex','fontsize',fontsize, 'Position',[0.005 0.95 0.99 0.036], 'Orientation','horizontal');
    hold(axes1,'off');

    axes2 = axes('Parent',fig1, 'Position',[0.13 0.347 0.775 0.189]);
    hold(axes2,'on');
    plot([Time(1) Time(end)],[tau tau], 'LineStyle','--', 'Color',tau_color, 'LineWidth',2, 'Parent',axes2);
    plot(Time,tau_data, 'LineStyle','-', 'Color',tau_hat_color, 'LineWidth',linewidth, 'Parent',axes2);
    legend_labels = {['$\tau$'], ['$\hat{\tau}$']};
    if (plot_1sigma)
        plot(Time,tau_data+Sigma_theta_data(end,:),'c-.', 'LineWidth',linewidth, 'Parent',axes2);
    	plot(Time,tau_data-Sigma_theta_data(end,:),'c-.', 'LineWidth',linewidth, 'Parent',axes2);
        legend_labels = [legend_labels, ['$\pm1\sigma$']];
    end
    legend(axes2, legend_labels,'interpreter','latex','fontsize',fontsize);
    ylabel('$\tau$ [$s$]','interpreter','latex','fontsize',fontsize, 'Parent',axes2);
    axis tight;
    hold(axes2,'off');
    
    f_norm = zeros(size(F_data,2),1);
    for i=1:length(f_norm)
        f_norm(i) = norm(F_data(:,i));
    end

    axes3 = axes('Parent',fig1, 'Position',[0.13 0.0845 0.775 0.216]);
    hold(axes3,'on');
    plot(Time, f_norm,'b-', 'LineWidth',linewidth, 'Parent',axes3);
    ylabel(['$||\mathbf{f}_{ext}||$ [$N$]'],'interpreter','latex','fontsize',fontsize, 'Parent',axes3);
    xlabel('time [$s$]','interpreter','latex','fontsize',fontsize, 'Parent',axes3);
    axis tight;
    hold(axes3,'off');
    
%% ==================================================
%% ===============   Calc effort  ===================
    
    dt = Time(2)-Time(1);

    effort = 0.0;
    sum_f2 = 0.0;
    P = zeros(length(Time),1);
    F_square = zeros(length(Time),1);
    for i=1:size(F_data,2)
        P(i) = abs(F_data(:,i)'*dY_data(:,i));
        effort = effort + abs(P(i))*dt;
        F_square(i) = F_data(:,i)'*F_data(:,i);
        sum_f2 = sum_f2 + F_square(i)*dt;
    end
    goal_err = norm(Yg-Yg_data(:,end));
    
%% ================================================
%% ================  Plot effort  =================

    linewidth = 2.0;
    figure;
    subplot(2,1,1);
    plot(Time, P, 'LineWidth',linewidth)
    xlabel('time [$s$]', 'interpreter','latex', 'fontsize',fontsize);
    ylabel('Power [$Watt$]', 'interpreter','latex', 'fontsize',fontsize);
    axis tight;

    subplot(2,1,2);
    plot(Time, F_square, 'LineWidth',linewidth)
    xlabel('time [$s$]', 'interpreter','latex', 'fontsize',fontsize);
    ylabel('$||\mathbf{f}_{ext}||^2$ [$N$]', 'interpreter','latex', 'fontsize',fontsize);
    axis tight;

    metrics_results = {'Work', 'Power_F', 'target error'; effort, sum_f2, goal_err};
    disp(metrics_results);

end