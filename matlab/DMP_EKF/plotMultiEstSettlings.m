function plotMultiEstSettlings(Data)

    if (nargin < 1)
        load('data/multisim_data.mat', 'Data');
    end
    
    Colors = [ 51, 255,   0;
              204, 204,   0;
              255, 102,   0;
              255,   0,   0;
              153,   0, 153;
                0,   0, 153;
              102, 102, 102;
                10,  5,  10
             ]/255.0;
    
    n_colors = size(Colors,1);
    
    % p_set = 0.5*5/100; % assuming max dist 0.5 m define the settling at 5% of that dist
    p_set = [ones(3,1)*0.005; 0.1]; % settling at 0.5 cm for pos and 100 ms for tau

    Dim = 4;

    goal_est_color = {[1 0 1], [0 1 0], [0 1 1], [0.85 0.33 0.1]};
    fontsize = 16;
    linewidth = 1.5;

    legend_labels = {'$\mathbf{y}_{g,x}-\hat{\mathbf{y}}_{g,x}$', ...
                    '$\mathbf{y}_{g,y}-\hat{\mathbf{y}}_{g,y}$', ...
                    '$\mathbf{y}_{g,z}-\hat{\mathbf{y}}_{g,z}$', ...
                    '$\tau-\hat{\tau}$'};

    fig = figure('pos',[70 70 1800 550]);
    ax_cell = cell(Dim,1);
    ax_width = 0.225;
    ax_height = 0.85;
    ax_cell{1} = axes('Parent',fig, 'Position',[0.03           0.10 ax_width ax_height]);
    ax_cell{2} = axes('Parent',fig, 'Position',[0.05+ax_width  0.10 ax_width ax_height]);
    ax_cell{3} = axes('Parent',fig, 'Position',[0.07+2*ax_width 0.10 ax_width ax_height]);
    ax_cell{4} = axes('Parent',fig, 'Position',[0.09+3*ax_width 0.10 ax_width ax_height]);

    for i=1:length(ax_cell)
        ax = ax_cell{i};
        hold(ax, 'on');
        plot([0 1], [p_set(i) p_set(i)], 'LineStyle','--', 'Color',[1 0 0], 'LineWidth',linewidth, 'Parent',ax, 'HandleVisibility','off');
        plot([0 1], [0 0], 'LineStyle','-', 'Color',[0 0 0], 'LineWidth',linewidth, 'Parent',ax, 'HandleVisibility','off');
        plot([0 1], [-p_set(i) -p_set(i)], 'LineStyle','--', 'Color',[1 0 0], 'LineWidth',linewidth, 'Parent',ax, 'HandleVisibility','off');
        xlabel('normalized time','interpreter','latex','fontsize',fontsize, 'Parent',ax);
    end
    ylabel('[$m$]','interpreter','latex','fontsize',fontsize, 'Parent',ax_cell{1});
    
    s_times = zeros(Dim,length(Data));
    s2_times = zeros(Dim,length(Data));
    
    time_offset_max = 14.0;
    time_color_bounds = (1:n_colors)*time_offset_max/n_colors;
    
    for k=1:length(Data)
        
        Time = Data{k}.Time;
        Y_data = Data{k}.Y_data;
        Yg_data = Data{k}.Yg_data;
        Yg = Data{k}.Yg;
        tau_data = Data{k}.tau_data;
        tau = Data{k}.tau; 
        
        time_offset = abs(Data{k}.tau_offset);
        for i=1:n_colors
            if (time_offset <= time_color_bounds(i)), break; end
        end
        color = Colors(i,:);

        Yg = [Yg; tau];
        Yg_data = [Yg_data; tau_data];
        Y_data = [Y_data; repmat(tau,1,size(Y_data,2))];

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

            p_t0 = t_yg/Time(end);
            p_t = (t_y-t_yg)/Time(end);
            
            s_times(i,k) = p_t0;
            s2_times(i,k) = p_t;

            ind = find(d_yg_yg<d_y_yg);
            p_t2 = length(ind)/length(Time);

            Time_n = Time/Time(end);
            t_yg = t_yg/Time(end);
            t_y = t_y/Time(end);

            ax = ax_cell{i};
            plot_h = plot(Time_n, yg-yg_data, 'LineStyle','-', 'LineWidth',1.0, 'Color',color, 'Parent',ax);
            if (k>1)
                set(plot_h, 'HandleVisibility','off');
            end
        end

    end
    
    s_times_mean = mean(s_times,2);
    s_times_sigma = std(s_times,0,2);
    
    s_times_mean
    s_times_sigma

    for i=1:length(ax_cell)
        ax = ax_cell{i};
        axis(ax, 'tight');
        y_lim = ylim(ax);
        y_lim = y_lim + [-0.1 0.1];
        ylim(ax, y_lim);
        mu = s_times_mean(i);
        mps = mu + s_times_sigma(i);
        mms = mu - s_times_sigma(i);
        plot([mu mu], y_lim, 'LineStyle','--', 'Color',[0.85 0.33 0.1], 'LineWidth',linewidth, 'Parent',ax);
        plot([mps mps], y_lim, 'LineStyle',':', 'Color',0.5*[0.85 0.33 0.1], 'LineWidth',linewidth, 'Parent',ax);
        plot([mms mms], y_lim, 'LineStyle',':', 'Color',0.5*[0.85 0.33 0.1], 'LineWidth',linewidth, 'Parent',ax, 'HandleVisibility','off');
        legend(ax, [legend_labels(i) ['$\bar{t}_s$'] ['$\bar{t}_s \pm \sigma$']],'interpreter','latex','fontsize',fontsize, 'Orientation','vertical');
        title(['Average settling at $' num2str(s_times_mean(i)*100,2) '\pm' num2str(s_times_sigma(i)*100,2) '\%$ of the movement'], ...
            'interpreter','latex','fontsize',fontsize, 'Parent',ax);
    end
    
    ind = [];
    s_t = 0.95*ones(4,1); %s_times_mean+s_times_sigma; % 0.65*ones(4,1);
    for i=1:size(s_times,1)
        ind = [ind find(s_times(i,:) > s_t(i))];
    end
    ind = unique(ind, 'sorted');
    
    if (~isempty(ind)), fprintf('Simulations that settled after the mean+sigma bound: (%d)\n',length(ind)); end
    for k=1:length(ind)
        i = ind(k);
        goal_offset = Data{i}.Yg_offset;
        tau_offset = Data{i}.tau_offset;
        fprintf('=== Sim %d ===\n', i);
        fprintf('Target offset: (%.2f, %.2f, %.2f) m\n', goal_offset(1), goal_offset(2), goal_offset(3));
        fprintf('Tau offset: %.2f sec\n', tau_offset);
    end
    
    colormap(ax_cell{4}, Colors);
    c_bar = colorbar(ax_cell{4});
    c_bar.Label.FontSize = 14;
    c_bar.Label.String = '$||\tau - \hat{\tau}||$';
    c_bar.Label.Interpreter = 'latex';
    tick_labels = cellstr(split(sprintf('%.1f ',time_color_bounds),' '));
    c_bar.TickLabels = tick_labels(1:end-1);
    c_bar.FontSize = 14;
    c_bar.TickLabelInterpreter = 'latex';
    c_bar.Ticks = time_color_bounds/time_offset_max;
    %c_bar.Direction = 'reverse';
    

end
