function plotEstSettlings(Time, Y_data, Yg_data, Yg, tau_data, tau)

% p_set = 0.5*5/100; % assuming max dist 0.5 m define the settling at 5% of that dist
p_set = [ones(3,1)*0.005; 0.05]; % settling at 0.5 cm for pos and 50 ms for tau

Yg = [Yg; tau];
Yg_data = [Yg_data; tau_data];
Y_data = [Y_data; repmat(tau,1,size(Y_data,2))];

Dim = size(Y_data,1);

goal_color = {[1 0 0], [0 0.5 0], [0 0 1], [0 0 1]};
goal_est_color = {[1 0 1], [0 1 0], [0 1 1], [0.85 0.33 0.1]};
fontsize = 16;
linewidth = 1.5;

legend_labels = {'$\mathbf{y}_{g,x}-\hat{\mathbf{y}}_{g,x}$', ...
                '$\mathbf{y}_{g,y}-\hat{\mathbf{y}}_{g,y}$', ...
                '$\mathbf{y}_{g,z}-\hat{\mathbf{y}}_{g,z}$', ...
                '$\tau-\hat{\tau}$'};

fig = figure('pos',[70 70 1800 400]);
ax_cell = cell(Dim,1);
ax_width = 0.225;
ax_height = 0.85;
ax_cell{1} = axes('Parent',fig, 'Position',[0.03           0.10 ax_width ax_height]);
ax_cell{2} = axes('Parent',fig, 'Position',[0.05+ax_width  0.10 ax_width ax_height]);
ax_cell{3} = axes('Parent',fig, 'Position',[0.07+2*ax_width 0.10 ax_width ax_height]);
ax_cell{4} = axes('Parent',fig, 'Position',[0.09+3*ax_width 0.10 ax_width ax_height]);

for i=1:Dim
    y_data = Y_data(i,:);
    yg_data = Yg_data(i,:);
    yg = Yg(i);
    
    d_y_yg = abs(yg-y_data); %/abs(yg);
    j = find( d_y_yg>p_set(i) , 1, 'last');
    if (isempty(j)), j=1; end
    t_y = Time(j);
    
    d_yg_yg = abs(yg-yg_data); %/abs(yg);
    j = find( d_yg_yg>p_set(i) , 1, 'last');
    if (isempty(j)), j=1; end
    t_yg = Time(j);
    
    p_t0 = t_yg/Time(end)*100;
    p_t = (t_y-t_yg)/Time(end)*100;
    
    ind = find(d_yg_yg<d_y_yg);
    p_t2 = length(ind)/length(Time)*100;
    
    Time_n = Time/Time(end);
    t_yg = t_yg/Time(end);
    t_y = t_y/Time(end);

    ax = ax_cell{i};
    hold(ax, 'on');
    plot(Time_n, yg-yg_data, 'LineStyle','-', 'Color',goal_est_color{i}, 'LineWidth',1.5, 'Parent',ax);
    %plot(Time_n, yg-y_data, 'LineStyle',':', 'Color',pos_color{i}, 'LineWidth',1.5, 'Parent',ax);
    axis(ax, 'tight');
    y_lim = ylim(ax);
    y_lim = y_lim + [-0.1 0.1];
    ylim(ax, y_lim);
    % plot([t_yg t_yg], y_lim, 'LineStyle','--', 'Color',[0.85 0.33 0.1], 'LineWidth',1.5, 'Parent',ax);%, 'HandleVisibility','off');
    % plot([t_y t_y], y_lim, 'LineStyle','--', 'Color',[0 0 0], 'LineWidth',1.5, 'Parent',ax);%, 'HandleVisibility','off');
    plot([Time_n(1) Time_n(end)], [p_set(i) p_set(i)], 'LineStyle','--', 'Color',[1 0 0], 'LineWidth',1.5, 'Parent',ax, 'HandleVisibility','off');
    plot([Time_n(1) Time_n(end)], [0 0], 'LineStyle','-', 'Color',[0 0 0], 'LineWidth',1.5, 'Parent',ax, 'HandleVisibility','off');
    plot([Time_n(1) Time_n(end)], [-p_set(i) -p_set(i)], 'LineStyle','--', 'Color',[1 0 0], 'LineWidth',1.5, 'Parent',ax, 'HandleVisibility','off');
    legend(ax, legend_labels(i),'interpreter','latex','fontsize',fontsize, 'Orientation','horizontal');
    xlabel('time [$s$]','interpreter','latex','fontsize',fontsize, 'Parent',ax);
    hold(ax, 'off');
end
ylabel('[$m$]','interpreter','latex','fontsize',fontsize, 'Parent',ax_cell{1});


end

