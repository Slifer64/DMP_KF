function processTrainingData(ind, v_start, v_stop)
%% Loads the binary data from 'data/bin' folder and trims the start and end of the
%% movement according to the velocity thresholds specified as arguments.
%% The processed data are save in 'data/training_data.mat'.
% @param[in] ind: Rowvector with indexes of the training data to be processed. (default = [1])
% @param[in] v_start: Start velocity threshold. (default = 0.01)
% @param[in] v_stop: Stop velocity threshold. (default = 0.01)
%

if (nargin < 1), ind = 2; end
if (nargin < 2), v_start = 0.01; end
if (nargin < 3), v_stop = 0.01; end

set_matlab_utils_path();

prefix_path = 'data/bin/';

files = dir([prefix_path 'training_data_*.bin']);

files = {files.name};
files = files(ind);

Data = {};

for k=1:length(files)
    
    filename = [prefix_path files{k}];
    
    binary = true;
    fid = fopen(filename);
    if (fid < 0)
        warning('Could not load %s\n', filename);
        continue;
    end

    q_start = read_mat(fid, binary);
    Time = read_mat(fid, binary);
    Y_data = read_mat(fid, binary);
    dY_data = read_mat(fid, binary);
    ddY_data = read_mat(fid, binary);
    
    fclose(fid);

    if (isempty(Time))
        error('The loaded data are empty %s\n', filename);
    end
    
    norm_dY = zeros(length(Time),1);
    for i=1:length(norm_dY)
        norm_dY(i) = norm(dY_data(:,i));
    end

    i1 = find(norm_dY>=v_start, 1, 'first');
    i2 = find(norm_dY>=v_stop, 1, 'last');
    ind = i1:i2;

    dt = Time(2)-Time(1);
    Time = (0:length(ind)+1)*dt;
    Y_data = Y_data(:,ind);      Y_data = [Y_data(:,1) Y_data Y_data(:,end)];
    Y_data = Y_data - repmat(Y_data(:,end), 1, size(Y_data,2));
    dY_data = dY_data(:,ind);    dY_data = [zeros(3,1) dY_data zeros(3,1)];
    ddY_data = ddY_data(:,ind);  ddY_data = [zeros(3,1) ddY_data zeros(3,1)];

    Data{k} = struct('Time',Time, 'Y',Y_data, 'dY',dY_data, 'ddY',ddY_data);
    
end

save('data/training_data.mat','Data');


fig = figure;
ax = axes('Position',[0.12 0.12 0.85 0.85], 'Parent',fig);
hold(ax, 'on');
legends = {};
for i=1:length(Data)
    Y_data = Data{i}.Y;
    plot3(Y_data(1,:), Y_data(2,:), Y_data(3,:), 'LineWidth',2.0, 'Parent',ax); 
    h_start = plot3(Y_data(1,1), Y_data(2,1), Y_data(3,1), 'Color','blue', 'LineWidth',4.0, 'Marker','o', 'MarkerSize',12, 'HandleVisibility','off', 'Parent',ax); 
    h_end = plot3(Y_data(1,end), Y_data(2,end), Y_data(3,end), 'Color','red', 'LineWidth',4.0, 'Marker','x', 'MarkerSize',12, 'HandleVisibility','off', 'Parent',ax); 
    legends = [legends ['demo ' num2str(i)]];
    if (i==length(Data))
        h_start.HandleVisibility = 'on';
        h_end.HandleVisibility = 'on';
        legends = [legends 'start' 'target'];
    end
end
legend(ax, legends, 'interpreter','latex', 'fontsize',15);
xlabel('x [$m$]', 'interpreter','latex', 'fontsize',15, 'Parent',ax);
ylabel('y [$m$]', 'interpreter','latex', 'fontsize',15, 'Parent',ax);
zlabel('z [$m$]', 'interpreter','latex', 'fontsize',15, 'Parent',ax);
hold(ax, 'off');


% fig = figure;
% ax = axes('Position',[0.12 0.12 0.85 0.85], 'Parent',fig);
% hold(ax, 'on');
% legends = {};
% for i=1:length(Data)
%     Y_data = Data{i}.dY;
%     plot3(Y_data(1,:), Y_data(2,:), Y_data(3,:), 'LineWidth',2.0); 
%     legends = [legends ['demo ' num2str(i)]];
% end
% legend(ax, legends, 'interpreter','latex', 'fontsize',15);
% xlabel('$\dot{x}$ [$m/s$]', 'interpreter','latex', 'fontsize',15, 'Parent',ax);
% ylabel('$\dot{y}$ [$m/s$]', 'interpreter','latex', 'fontsize',15, 'Parent',ax);
% zlabel('$\dot{z}$ [$m/s$]', 'interpreter','latex', 'fontsize',15, 'Parent',ax);
% hold(ax, 'off');
% 
% 
% fig = figure;
% ax = axes('Position',[0.12 0.12 0.85 0.85], 'Parent',fig);
% hold(ax, 'on');
% legends = {};
% for i=1:length(Data)
%     Y_data = Data{i}.ddY;
%     plot3(Y_data(1,:), Y_data(2,:), Y_data(3,:), 'LineWidth',2.0); 
%     legends = [legends ['demo ' num2str(i)]];
% end
% legend(ax, legends, 'interpreter','latex', 'fontsize',15);
% xlabel('$\ddot{x}$ [$m/s$]', 'interpreter','latex', 'fontsize',15, 'Parent',ax);
% ylabel('$\ddot{y}$ [$m/s$]', 'interpreter','latex', 'fontsize',15, 'Parent',ax);
% zlabel('$\ddot{z}$ [$m/s$]', 'interpreter','latex', 'fontsize',15, 'Parent',ax);
% hold(ax, 'off');

% plotData(Data);


end