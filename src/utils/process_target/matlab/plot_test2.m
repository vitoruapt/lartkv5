%used in do_tag to show comparison between labels

function [] = plot_test2(test_a)

features = test_a(:,5:end);
%test(test(:,2)==0,2)=-1; %transform from 0 to -1
gd_class = test_a(:,2);    %second column has leader tag
low_plot = min(min(features));
up_plot = max(max(features));
time = test_a(:,1);

bad_tag = test_a(gd_class == 1,1);

% Show the data
F = figure;
set(F,'defaultlinelinewidth',3);
set(F,'defaultaxeslinewidth',2);
set(F,'defaulttextfontsize',12);
set(F,'defaultaxesfontsize',12);
H = subplot(3,1,1); hold on, grid on;
if ~isempty(bad_tag)
    line([bad_tag bad_tag],[low_plot up_plot],'Color',[0.5 0.5 0.5]);
end
plot(time,features(:,1),'b.');
plot(time,features(:,2),'m.');
plot(time,features(:,3),'k.');
plot(time,features(:,4),'g.');
plot(time,features(:,5),'y.');
%axis equal;
axis tight;
ax_x = get(H, 'xlim');
ax_y = get(H, 'ylim');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

features = test_a(:,5:end);
%test(test(:,2)==0,2)=-1; %transform from 0 to -1
gd_class = test_a(:,3);    %rich tag
low_plot = min(min(features));
up_plot = max(max(features));
time = test_a(:,1);

bad_tag = test_a(gd_class == 1,1);

% Show the data
I = subplot(3,1,2); hold on, grid on;
if ~isempty(bad_tag)
    line([bad_tag bad_tag],[low_plot up_plot],'Color',[0.5 0.5 0.5]);
end
plot(time,features(:,1),'b.');
plot(time,features(:,2),'m.');
plot(time,features(:,3),'k.');
plot(time,features(:,4),'g.');
plot(time,features(:,5),'y.');
%axis equal;
axis tight;
set(I,'xlim',ax_x);
set(I,'ylim',ax_y);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

features = test_a(:,5:end);
%test(test(:,2)==0,2)=-1; %transform from 0 to -1
gd_class = test_a(:,4);    %jorge_tag
low_plot = min(min(features));
up_plot = max(max(features));
time = test_a(:,1);

bad_tag = test_a(gd_class == 1,1);

% Show the data
I = subplot(3,1,3); hold on, grid on;
if ~isempty(bad_tag)
    line([bad_tag bad_tag],[low_plot up_plot],'Color',[0.5 0.5 0.5]);
end
plot(time,features(:,1),'b.');
plot(time,features(:,2),'m.');
plot(time,features(:,3),'k.');
plot(time,features(:,4),'g.');
plot(time,features(:,5),'y.');
%axis equal;
axis tight;
set(I,'xlim',ax_x);
set(I,'ylim',ax_y);
xlabel('t(s)');

%%%%%%%%%%%%%%%%%%%%%%%%%%%
p_tag = test_a(diff(test_a(:,2))==1,1);
r_tag = test_a(diff(test_a(:,3))==1,1);
j_tag = test_a(diff(test_a(:,4))==1,1);

[p_tag r_tag j_tag]