clc;
close all;
clear;

y0 = [-0.0408 0.4509 0.1900]';

g = [0.1087 0.7139 0.3904]';
    
g1 = [-0.2162 0.7392 0.1913]';

g2 = [0.3128 0.5661 0.4707]';

g3 = [0.3794 0.3919 0.1449]';

g4 = [-0.1292 0.7078 0.5318]';

g5 = [-0.1144 0.7004 0.4828]';

g6 = [0.3152 0.5273 0.1901]';

G = [g1 g2 g3 g4 g5 g6];

N_g = size(G,2);

markersize = 10;
linewidth = 2.0;
fontsize = 20;

text_offset = 0.01;

fig = figure;
hold on;
plot3(y0(1), y0(2), y0(3), 'Marker','*','Markersize',markersize, 'LineWidth',linewidth, 'Color','green', 'HandleVisibility','off');
text(y0(1)+text_offset, y0(2)+text_offset, y0(3)+text_offset, '$y_0$', 'interpreter','latex', 'fontsize',fontsize);
plot3(g(1), g(2), g(3), 'Marker','*','Markersize',markersize, 'LineWidth',linewidth, 'Color','blue', 'HandleVisibility','off');
text(g(1)+text_offset, g(2)+text_offset, g(3)+text_offset, '$g_0$', 'interpreter','latex', 'fontsize',fontsize);
legend_labels = cell(1,N_g);
for i=1:N_g
    gi = G(:,i);
    plot3(gi(1), gi(2), gi(3), 'Marker','*','Markersize',markersize, 'LineWidth',linewidth, 'Color','red');
    text(gi(1)+text_offset, gi(2)+text_offset, gi(3)+text_offset, ['$g_' num2str(i) '$'], 'interpreter','latex', 'fontsize',fontsize);
    legend_labels{i} = ['$||\mathbf{g}_' num2str(i) '-\mathbf{g}_0||=' num2str(norm(gi-g),2) '$'];
end
xlabel('x [$m$]', 'interpreter','latex', 'fontsize',fontsize);
ylabel('y [$m$]', 'interpreter','latex', 'fontsize',fontsize);
zlabel('z [$m$]', 'interpreter','latex', 'fontsize',fontsize);
legend(legend_labels, 'interpreter','latex', 'fontsize',fontsize);
hold off;

