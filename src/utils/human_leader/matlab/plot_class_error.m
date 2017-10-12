%show the error in classification as a graph, for all the test set, shown
%in the x axis, input is a matrix copied from excel, containing the value
%of the errors, but could come directly from evaluate_model function. it
%has been done this way because excel already had all the errors in tables.

function [] = plot_class_error(input)

F = figure; hold;
set(F,'defaultlinelinewidth',3);
set(F,'defaultaxeslinewidth',2);
set(F,'defaulttextfontsize',12);
set(F,'defaultaxesfontsize',12);

plot(input(1,:),'bs-','color',[1 0.82 0.13]);
plot(input(2,:),'md-','color',[1 0.26 0.06]);
plot(input(3,:),'ko-','color',[0 0.27 0.53]);

label = {'st04','st08','gd02','gd06','as02','as03',...
    'fr04','nm05','nm03','od01','od10','total'};

axis tight, grid on;
set(gcf,'position',[175 338 1000 300]);
set(gca,'ylim',[0 0.4]);
set(gca,'xtick',1:12)
set(gca,'XTickLabel',label);
legend('false good','false bad','false total','location','eastoutside');
