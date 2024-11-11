function h = personal_plot1(x, y, x_label_latex, y_label_latex, pdf_name)

set(0, 'DefaultTextInterpreter', 'latex')
set(0, 'DefaultLegendInterpreter', 'latex')
set(0, 'DefaultAxesTickLabelInterpreter', 'latex')
lw = 2;

h = figure('Renderer', 'painters', 'Position', [10 10 900 300]);

plot(x, y, 'k-', 'Linewidth', lw ,'Color', [0.2, 0.2, 0.2]);

xlabel(x_label_latex)
ylabel(y_label_latex)
set(gca, 'FontSize',28);
grid on
box on
set(gcf,'color','w');
set(h, 'MenuBar', 'none');
set(h, 'ToolBar', 'none');
xlim([x(1) x(end)])
exportgraphics(h, pdf_name);

end
