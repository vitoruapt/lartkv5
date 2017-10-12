%compare the labels created by the three evaluators with, also plotting the
%final tag, for illustrative purposes only.

function [] = adaboost_vs_ann()

% Create figure
% F = figure('position',[360 260 750 600]);
F = figure('position',[360 260 750 500]);
set(F,'defaultlinelinewidth',3);
set(F,'defaultaxeslinewidth',1.5);
set(F,'defaulttextfontsize',12);
set(F,'defaultaxesfontsize',12);

%%%%%%%%%%% false good %%%%%%%%%%
% FG = subplot(3,1,1,'position',[0.065 0.70 0.90 0.20]);
FG = subplot(3,1,1,'position',[0.075 0.65 0.90 0.25]);
hold on; grid on;

set(gca,'xtick',1:12);
set(FG,'xlim',[1 12]);
set(FG,'ylim',[0 0.4]);
ax_x = get(FG, 'xlim');
ax_y = get(FG, 'ylim');
box on;
set(gca,'XTickLabel',[]);
ylabel('false good');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%% false bad %%%%%%%%%%
% FB = subplot(3,1,2,'position',[0.065 0.4125 0.90 0.20]);
FB = subplot(3,1,2,'position',[0.075 0.36 0.90 0.25]);
hold on; grid on;

set(gca,'xtick',1:12);
set(FB,'xlim',ax_x);
set(FB,'ylim',ax_y);
set(gca,'XTickLabel',[]);
box on;
ylabel('false bad');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%% false total %%%%%%%%%%
FT = subplot(3,1,3,'position',[0.075 0.066 0.90 0.25]);
hold on; grid on;

set(gca,'xtick',1:12);
set(FT,'xlim',ax_x);
set(FT,'ylim',ax_y);
box on;

label = {'st04','st08','gd02','gd06','as02','as03',...
    'fr04','nm05','nm03','od01','od10','total'};
set(gca,'XTickLabel',label);
ylabel('false total');

large_markers

%%%%%%%%%%%%%%%%%%%%%%%%
end

function large_markers
ax1 = gca;
ax2 = axes;
hold;
plot(0,0,'b','linewidth',10,'parent',ax2);
plot(0,0,'r','linewidth',10,'parent',ax2);

hL = legend(ax2,'AdaBoost','ANN','location','eastOutside');
set(hL,'orientation', 'horizontal');
set(hL,'position', [0.37 0.92 0.25 0.05]);
set(ax2,'visible','off')
axis(ax1, 'tight');
end