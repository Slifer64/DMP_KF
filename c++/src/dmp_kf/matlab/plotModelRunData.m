function plotModelRunData()

addpath('utils/');

binary = true;
filename = '../data/model_run_data.bin';
fid = fopen(filename);
if (fid < 0)
    error('Could not load %s\n', filename);
end

Time = read_mat(fid, binary);
y_data = read_mat(fid, binary);
dy_data = read_mat(fid, binary);
ddy_data = read_mat(fid, binary);

fclose(fid);

if (isempty(Time))
    error('The loaded data are empty %s\n', filename);
end

D = size(y_data,1);

k = 1;
fontsize = 14;
figure('NumberTitle', 'off', 'Name', 'Training Data');
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