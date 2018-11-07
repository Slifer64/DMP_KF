clc;
close all;
clear;

y0 = [0.0181 0.3929 0.1634]';

g = [0.1822 0.6528 0.3463]';
    
g1 = [-0.1928 0.7584 0.2019]';

g2 = [-0.1121 0.7589 0.4564]';

G = [g1 g2];

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

