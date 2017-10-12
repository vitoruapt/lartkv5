%compares pro tag before and after shift, 

function [] = plot_test(test_set)

features = test_set(:,3:end);
good_tag = test_set(:,2);    %second column has leader tag
bad_tag = test_set(good_tag == 1,1);
low_plot = min(min(features));
up_plot = max(max(features));
time = test_set(:,1);

% Show the data
H = figure;
set(H,'defaultlinelinewidth',3);
set(H,'defaultaxeslinewidth',2);
set(H,'defaulttextfontsize',12);
set(H,'defaultaxesfontsize',12);
hold on, grid on; title('Features over time');
if ~isempty(bad_tag)
    line([bad_tag bad_tag],[low_plot up_plot],'Color',[1 0 0.1]);
end

v = plot(time,features(:,1),'b.'); %velocity
ld = plot(time,features(:,2),'c.'); %lateral displacement
rh = plot(time,features(:,3),'k.'); %heading diff
a = plot(time,features(:,4),'g.'); %angle to robot
d = plot(time,features(:,5),'y.'); %distance
rv_x = plot(time,-features(:,6),'m.'); %relative velocity x
rv_y = plot(time,features(:,7),'color',[0.5 0 0]); %relative velocity y

legend([v,ld,rh,a,d,rv_x,rv_y],'velocity','lateral displacement',...
    'relative heading','angle','distance',...
    'relative vel. x','relative vel. y',...
    'location','eastOutside');
xlabel('t(s)');

%large markers
ax1 = gca;
ax2 = axes; 
hold;
plot(0,0,'b','linewidth',10,'parent',ax2);
plot(0,0,'c','linewidth',10,'parent',ax2);
plot(0,0,'k','linewidth',10,'parent',ax2);
plot(0,0,'g','linewidth',10,'parent',ax2);
plot(0,0,'y','linewidth',10,'parent',ax2);
plot(0,0,'m','linewidth',10,'parent',ax2);
plot(0,0,'color',[0.5 0 0],'linewidth',10,'parent',ax2);
%set(ax2,'position',get(ax1,'position'));
legend(ax2,'velocity','lateral displacement',...
    'relative heading','angle','distance',...
    'relative vel. x','relative vel. y',...
    'location','eastOutside');
set(ax2,'visible','off')
axis(ax1, 'tight');
set(gcf,'position',[200 200 800 400]);



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% features = test_b(:,5:end);
% %test(test(:,2)==0,2)=-1; %transform from 0 to -1
% gd_class = test_b(:,2);    %rich tag
% low_plot = min(min(features));
% up_plot = max(max(features));
% time = test_b(:,1);
% 
% bad_tag = test_b(gd_class == 1,1);
% 
% % Show the data
% I = subplot(2,1,2); hold on, grid on;
% if ~isempty(bad_tag)
%     line([bad_tag bad_tag],[low_plot up_plot],'Color',[1 0 0.1]);
% end
% plot(time,features(:,1),'b.');
% plot(time,features(:,2),'m.');
% plot(time,features(:,3),'k.');
% plot(time,features(:,4),'g.');
% plot(time,features(:,5),'y.');
% %axis equal;
% axis tight;
% %set(I,'xlim',ax_x);
% set(I,'ylim',ax_y);

