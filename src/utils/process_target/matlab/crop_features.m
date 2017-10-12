%crop variable containing leader features
%so transitory measurments from beginning and
%end can be removed

%inputs:
%in_data : input data
%x1: inferior crop limit
%x2: superior crop limit

function [out_data] = crop_features(in_data,x1,x2)

% Show the data
H = figure;
set(H,'defaultlinelinewidth',3);
set(H,'defaultaxeslinewidth',1.5);
set(H,'defaulttextfontsize',12);
set(H,'defaultaxesfontsize',12);
hold on, grid on; title('Cropped Fatures');
set(gcf,'position',[200 200 800 300]);

[val1 ind1] = min(abs(in_data(:,1)-x1));
[val2 ind2] = min(abs(in_data(:,1)-x2));
top = max(max(in_data(:,3:9)));
bottom = min(min(in_data(:,3:9)));

patch([x1 x1 x2 x2],...
    [bottom top top bottom],...
    [0.7 0.7 0.7]);%,'faceAlpha',0.5);

plot(in_data(:,1),in_data(:,3),'bo');
plot(in_data(:,1),in_data(:,4),'co');
plot(in_data(:,1),in_data(:,5),'ko');
plot(in_data(:,1),in_data(:,6),'go');
plot(in_data(:,1),in_data(:,7),'yo');
plot(in_data(:,1),in_data(:,8),'mo');
plot(in_data(:,1),in_data(:,9),'o','color',[0.5 0 0]);

legend('vel','vel diff','head diff','angle', 'dist diff',...
   'Location','eastOutside');
xlabel('t(s)');



axis tight;
out_data = in_data(ind1:ind2,:);

%large markers
ax1 = gca;
ax2 = axes('position',get(ax1,'position')); 
hold;
plot(0,0,'b','linewidth',10,'parent',ax2);
plot(0,0,'c','linewidth',10,'parent',ax2);
plot(0,0,'k','linewidth',10,'parent',ax2);
plot(0,0,'g','linewidth',10,'parent',ax2);
plot(0,0,'y','linewidth',10,'parent',ax2);
plot(0,0,'m','linewidth',10,'parent',ax2);
plot(0,0,'color',[0.5 0 0],'linewidth',10,'parent',ax2);
plot(0,0,'color',[0.7 0.7 0.7],'linewidth',10,'parent',ax2);
%set(ax2,'position',get(ax1,'position'));
HL = legend(ax2,'velocity (m/s)','lateral disp. (m)',...
    'relative head. (rad)','angle (rad)','distance (m)',...
    'relative vel. x (m/s)','relative vel. y (m/s)','cropped region',...
    'location','eastOutside');
set(HL,'position',[0.7650 0.35 0.23 0.37]);
set(ax2,'visible','off')
%axis(ax1, 'tight');
