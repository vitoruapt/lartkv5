%compare the labels created by the three evaluators with, also plotting the
%final tag, for illustrative purposes only.
%input must be xx files

function [] = plot_labels(test)

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
H = figure;
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
