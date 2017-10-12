%compare the labels created by the three evaluators with, also plotting the
%final tag, for illustrative purposes only.

function [] = plot_labels(test,final_tag)

features = test(:,5:end);
low_plot = min(min(features));
up_plot = max(max(features));
time = test(:,1);

% Create figure
F = figure('position',[360 260 900 600]);
set(F,'defaultlinelinewidth',3);
set(F,'defaultaxeslinewidth',1.5);
set(F,'defaulttextfontsize',12);
set(F,'defaultaxesfontsize',12);

%%%%%%%%%%% pro plot %%%%%%%%%%
gd_class = test(:,2);
bad_tag = test(gd_class == 1,1);
H = subplot(3,1,1,'position',[0.065 0.70 0.70 0.20]);
hold on, grid on;
if ~isempty(bad_tag)
    left = bad_tag(1);
    right = bad_tag(end);
    top = up_plot;
    bottom = low_plot;
    patch([left left right right],...
        [bottom top top bottom],...
        [0.7 0.7 0.7],...
        'FaceAlpha',0.5);
    line([final_tag final_tag],...
        [bottom top],...
        'color',[1 0 0]);
end
plot(time,features(:,1),'b.');
plot(time,features(:,2),'c.');
plot(time,features(:,3),'k.');
plot(time,features(:,4),'g.');
plot(time,features(:,5),'y.');
% plot(time,features(:,6),'m.');
% plot(time,features(:,7),'.','color',[0.7 0 0]);
%axis equal;
axis tight;
ax_x = get(H, 'xlim');
ax_y = get(H, 'ylim');
box on;
set(gca,'XTickLabel',[]);
%set(gca,'YTickLabel',[]);
ylabel('pro');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%% rich plot %%%%%%%%%%
gd_class = test(:,3);    %rich tag
bad_tag = test(gd_class == 1,1);

% Show the data
I = subplot(3,1,2,'position',[0.065 0.4125 0.70 0.20]);
hold on, grid on;
if ~isempty(bad_tag)
    left = bad_tag(1);
    right = bad_tag(end);
    top = up_plot;
    bottom = low_plot;
    patch([left left right right],...
        [bottom top top bottom],...
        [0.7 0.7 0.7],...
        'FaceAlpha',0.5);
    line([final_tag final_tag],...
        [bottom top],...
        'color',[1 0 0]);
end
plot(time,features(:,1),'b.');
plot(time,features(:,2),'c.');
plot(time,features(:,3),'k.');
plot(time,features(:,4),'g.');
plot(time,features(:,5),'y.');
% plot(time,features(:,6),'m.');
% plot(time,features(:,7),'.','color',[0.7 0 0]);
%axis equal;
axis tight;
set(I,'xlim',ax_x);
set(I,'ylim',ax_y);
set(gca,'XTickLabel',[]);
box on;
%set(gca,'YTickLabel',[]);
ylabel('rich');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%% jor plot %%%%%%%%%%
gd_class = test(:,4);    %jorge_tag
bad_tag = test(gd_class == 1,1);

% Show the data
I = subplot(3,1,3,'position',[0.065 0.1250 0.70 0.20]);
hold on, grid on;
if ~isempty(bad_tag)
    left = bad_tag(1);
    right = bad_tag(end);
    top = up_plot;
    bottom = low_plot;
    patch([left left right right],...
        [bottom top top bottom],...
        [0.7 0.7 0.7],...
        'FaceAlpha',0.5);
    line([final_tag final_tag],...
        [bottom top],...
        'color',[1 0 0]);
end
plot(time,features(:,1),'b.');
plot(time,features(:,2),'c.');
plot(time,features(:,3),'k.');
plot(time,features(:,4),'g.');
plot(time,features(:,5),'y.');
% plot(time,features(:,6),'m.');
% plot(time,features(:,7),'.','color',[0.7 0 0]);%axis equal;
axis tight;
set(I,'xlim',ax_x);
set(I,'ylim',ax_y);
box on;
xlabel('t(s)');
%set(gca,'YTickLabel',[]);
ylabel('jor');

large_markers

%%%%%%%%%%%%%%%%%%%%%%%%%%%
p_tag = test(diff(test(:,2))==1,1);
r_tag = test(diff(test(:,3))==1,1);
j_tag = test(diff(test(:,4))==1,1);

[p_tag r_tag j_tag]

%%%%%%%%%%%%%%%%%%%%%%%%
end

function large_markers
xlabel('t(s)');
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
plot(0,0,'color',[0.7 0.7 0.7],'linewidth',10,'parent',ax2);
% hL = legend(ax2,'velocity','rel velocity',...
%     'rel heading','angle','distance','bad leader region',...
%     'final tag',...
%     'location','eastOutside');
hL = legend(ax2,'velocity (m/s)','lateral disp. (m)',...
    'relative head. (rad)','angle (rad)','distance (m)',...
    'relative vel. x (m/s)','relative vel. y (m/s)','cropped region',...
    'location','eastOutside');
set(hL,'position', [0.80 0.35 0.15 0.30]);
set(ax2,'visible','off')
axis(ax1, 'tight');
end