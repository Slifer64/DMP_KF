function plot_estimation_results(Time, g, g_data, tau, tau_data, P_data, F_data, mf_data, plot_1sigma, Y_data, dY_data)

    fontsize = 18;
    linewidth = 1.5;
    
    D = length(g);
    
    axis_name = {'x', 'y', 'z'};
    
    goal_color = {[1 0 0], [0 0.5 0], [0 0 1]};
    goal_est_color = {[1 0 1], [0 1 0], [0 1 1]};
    pos_color = {[0.85 0.7 1], [0.75 0.75 0], [0 0.45 0.75]};
    tau_color = [0 0 1];
    tau_hat_color = [0.85 0.33 0.1];
    
    s1_scale = 0.1;
    
    
    % Create figure
    figure1 = figure('pos',[100 100 700 700]);
    
    % Create axes
    axes1 = axes('Parent',figure1, 'Position',[0.13 0.374 0.775 0.551]);
    hold(axes1,'on');
    legend_labels = {};
    for i=1:D
        plot([Time(1) Time(end)],[g(i) g(i)], 'LineStyle','--', 'Color',goal_color{i} ,'LineWidth',2, 'Parent',axes1);
        plot(Time,g_data(i,:), 'LineStyle','-', 'Color',goal_est_color{i}, 'LineWidth',linewidth, 'Parent',axes1);
        legend_labels_i = {['$\mathbf{y}_{g,' axis_name{i} '}$'], ['$\hat{\mathbf{y}}_{g,' axis_name{i} '}$']};
        if (plot_1sigma)
            plot(Time,g_data(i,:)+P_data(i,:)*s1_scale, 'LineStyle',':', 'Color',goal_est_color{i}*0.75, 'LineWidth',linewidth, 'Parent',axes1);
        	plot(Time,g_data(i,:)-P_data(i,:)*s1_scale, 'LineStyle',':', 'Color',goal_est_color{i}*0.75, 'LineWidth',linewidth, 'Parent',axes1, 'HandleVisibility','off');
            legend_labels_i = [legend_labels_i, ['$\pm1\sigma$']];
        end
        plot(Time,Y_data(i,:), 'LineStyle','-.', 'Color',pos_color{i}, 'LineWidth',linewidth, 'Parent',axes1);
        legend_labels_i = [legend_labels_i, ['$\mathbf{y}_{' axis_name{i} '}$']];
        legend_labels = [legend_labels, legend_labels_i];
    end
    ylabel('target estimation [$m$]','interpreter','latex','fontsize',fontsize, 'Parent',axes1);
    legend(axes1, legend_labels,'interpreter','latex', 'fontsize',fontsize, ...
          'Orientation','horizontal', 'Position',[0.04 0.938 0.927 0.046]);
    axis(axes1,'tight');
    hold(axes1,'off');
    
    % Create axes
    axes2 = axes('Parent',figure1, 'Position',[0.13 0.11 0.775 0.206]);
    hold(axes2,'on');
    legend_labels = {['$\hat{\tau}$']};
    hold(axes2,'on');
    if (~isempty(tau))
        plot([Time(1) Time(end)],[tau tau], 'LineStyle','--', 'Color',tau_color, 'LineWidth',2, 'Parent',axes2);
        legend_labels = [['$\tau$'], legend_labels];
    end
    plot(Time,tau_data, 'LineStyle','-', 'Color',tau_hat_color, 'LineWidth',linewidth, 'Parent',axes2);
    if (plot_1sigma)
        plot(Time,tau_data+P_data(end,:)*s1_scale, 'LineStyle',':', 'Color',tau_hat_color*0.75, 'LineWidth',linewidth, 'Parent',axes2);
    	plot(Time,tau_data-P_data(end,:)*s1_scale, 'LineStyle',':', 'Color',tau_hat_color*0.75, 'LineWidth',linewidth, 'Parent',axes2, 'HandleVisibility','off');
        legend_labels = [legend_labels, ['$\pm1\sigma$']];
    end
    legend(axes2, legend_labels,'interpreter','latex','fontsize',fontsize);
    ylabel('$\tau$ [$s$]','interpreter','latex','fontsize',fontsize, 'Parent',axes2);
    xlabel('time [$s$]','interpreter','latex','fontsize',fontsize, 'Parent',axes2);
    axis(axes2,'tight');
    hold(axes2,'off');
    
%     f_label = {'f_x', 'f_y', 'f_z'};
%     figure;
%     hold on;
%     plot(Time, F_data(1,:), 'LineWidth',linewidth);
%     plot(Time, F_data(2,:), 'LineWidth',linewidth);
%     plot(Time, F_data(3,:), 'LineWidth',linewidth);
%     ylabel(['Force [$N$]'],'interpreter','latex','fontsize',fontsize);
%     xlabel('time [$s$]','interpreter','latex','fontsize',fontsize);
%     legend({'$f_x$','$f_y$','$f_z$'},'interpreter','latex','fontsize',fontsize);
%     title('Interaction force','interpreter','latex','fontsize',fontsize);
    
%     figure;
%     hold on;
%     plot(Time, y_data(1,:), 'LineWidth',linewidth);
%     plot(Time, y_data(2,:), 'LineWidth',linewidth);
%     plot(Time, y_data(3,:), 'LineWidth',linewidth);
%     xlabel('time [$s$]', 'interpreter','latex', 'fontsize',fontsize);
%     ylabel('position [$m$]', 'interpreter','latex', 'fontsize',fontsize);
%     legend({'$X$','$Y$','$Z$'}, 'interpreter','latex', 'fontsize',fontsize);
%     title('Robot end-effector Cartesian position','interpreter','latex','fontsize',fontsize);
%     hold off;
    
    
    figure;
    hold on;
    plot(Time, mf_data,'b-', 'LineWidth',1.5);
    plot(Time, 1-mf_data,'g-', 'LineWidth',1.5);
    %title('Leader-follower role','interpreter','latex','fontsize',fontsize);
    legend({'DMP','admittance'},'interpreter','latex','fontsize',fontsize, 'orientation','horizontal');
    ylabel('m($\mathbf{f}_{ext}$)','interpreter','latex','fontsize',fontsize);
    xlabel('time [$s$]','interpreter','latex','fontsize',fontsize);
    hold off;
    
    
    dt = Time(2)-Time(1);

    effort = 0.0;
    sum_f2 = 0.0;
    P = zeros(length(Time),1);
    F_square = zeros(length(Time),1);
    for i=1:size(F_data,2)
        P(i) = F_data(:,i)'*dY_data(:,i);
        effort = effort + P(i)*dt;
        F_square(i) = F_data(:,i)'*F_data(:,i);
        sum_f2 = sum_f2 + F_square(i)*dt;
    end

    effort
    sum_f2
    goal_err = norm(g-g_data(:,end))

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
    
    n = length(mf_data);
    leader = sum(mf_data >= 0.99)/n
    follower = sum(mf_data <= 0.01)/n
    mix = sum(mf_data>0.01 & mf_data<0.99)/n
    
    
end