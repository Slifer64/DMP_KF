function plotModelWeights()

addpath('utils/');

binary = true;
filename = '../data/model_data.bin';
fid = fopen(filename);
if (fid < 0)
    error('Could not load %s\n', filename);
end

q_start = read_mat(fid, binary);
g_d = read_mat(fid, binary);
tau_d = read_scalar(fid, binary, 'double');

dim = read_scalar(fid, binary, 'int64');

weights = cell(dim,1);
N_kernels = zeros(1, dim);
a_z = zeros(1, dim);
b_z = zeros(1, dim);

for i=1:dim
    dmp_params = read_mat(fid, binary);
    
    N_kernels(i) = dmp_params(1);
    a_z(i) = dmp_params(2);
    b_z(i) = dmp_params(3);

    weights{i} = dmp_params(4:end);
end

fclose(fid);

dim
N_kernels
a_z
b_z


ax = cell(dim,1);
figure;
for i=1:dim
   ax{i} = subplot(dim,1,i);
   ax{i}.NextPlot = 'add';
end

colors = {[0.75 0 0.75], [0.75 0.75 0], [0 0.75 0.75], [0 0 1], [0 0.5 0], [1 0.84 0], ...
    [0 0.45 0.74], [0.85 0.33 0.1], [1 0 0], [0.6 0.2 0], [1 0.6 0.78], [0.49 0.18 0.56]};

legends = cell(dim,1);
n = 1;
for i=1:dim
    legends{i} = [legends{i}, {['dim ' num2str(i)]} ];
    bar(ax{i}, weights{i}, 'BarWidth',1.0/n, 'FaceColor',colors{mod(n-1,length(colors))+1});
end

for i=1:dim
    legend(ax{i}, legends{i}, 'interpreter','latex', 'fontsize',14);
end
title(ax{1},'DMP weights');


end