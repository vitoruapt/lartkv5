% i think this was created to comparte an adaboost classificer with
% a neural network classifier

%test neural network

function [] = compare_classifiers(model,net,test)

%prepare the data
features = test(:,3:7);
input_val = features';
target_val = test(:,2)';
target_val(2,find(target_val == 0)) = 1; %for ann
test(test(:,2)==0,2)=-1; %for ada: transform from 0 to -1
gd_class = test(:,2);    %second column has leader tag

%for plot purposes
low_plot = min(min(features));
up_plot = max(max(features));
time = test(:,1);

%%%%%%%%%%%%% ANN Classifier %%%%%%%%%%
% Test the Network
outputs = net(input_val);

% binarize classification = good/bad
ann_class = round(outputs(2,:));

%diferences = abs(test_class - gd_class);
differences = abs(target_val(2,:) - ann_class);
error = length(find(differences == 1));
error_ratio = error / length(differences);
ann_string =  ...
    sprintf('NeuralNet Classification. error:%.2f',error_ratio);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%% Adaboost Classifier %%%%%%%%%%

% Classify the features with the trained model
if(isstruct(model))
    %classic adaboost
    ada_class = adaboost('apply',features, model);
else
    %matlab adaboost
    ada_class = predict(model, features);
end

diferences = abs(ada_class - gd_class);
error = length(find(diferences == 2));
error_ratio = error / length(diferences);
ada_string = ...
    sprintf('Adaboost Classification. error:%.2f',error_ratio);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%% groundtruth plot %%%%%%%%%%
bad_tag = test(gd_class == 1,1);

% Show groundtruth
H = figure;
set(H,'defaultlinelinewidth',3);
set(H,'defaultaxeslinewidth',2);
set(H,'defaulttextfontsize',12);
set(H,'defaultaxesfontsize',12);
set(H,'position',[740 110 550 550]);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
subplot(3,1,1), hold on, grid on;
if ~isempty(bad_tag)
    line([bad_tag bad_tag],[low_plot up_plot],'Color',[1 0 0.1]);
end
vel = plot(time,features(:,1),'b');
vld = plot(time,features(:,2),'m');
hdd = plot(time,features(:,3),'k');
ang = plot(time,features(:,4),'g');
dst = plot(time,features(:,5),'y');
l_handle = legend([vel, vld, hdd, ang, dst],...
      'vel','vel diff','head diff','angle', 'distance');
set(l_handle,'orientation','horizontal',...
      'Position',[0.12 0.0 0.8 0.05]);
axis tight;
set(gca,'XTickLabel',[]);
title('Ground Truth','FontWeight','bold');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%% neural net plot %%%%%%%%%%%
bad_tag = test(ann_class == 0,1);

% Show classification
subplot(3,1,2), hold on, grid on;
if ~isempty(bad_tag)
    line([bad_tag bad_tag],[low_plot up_plot],'Color',[1 0 0.1]);
end
plot(time,features(:,1),'b');
plot(time,features(:,2),'m');
plot(time,features(:,3),'k');
plot(time,features(:,4),'g');
plot(time,features(:,5),'y');
axis tight;
set(gca,'XTickLabel',[]);
title(ann_string,'FontWeight','bold');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%% Adaboost plot %%%%%%%%%%%%%
bad_tag = test(ada_class == 1,1);

% Show classification
subplot(3,1,3), hold on, grid on;
if ~isempty(bad_tag)
    line([bad_tag bad_tag],[low_plot up_plot],'Color',[1 0 0.1]);
end
plot(time,features(:,1),'b');
plot(time,features(:,2),'m');
plot(time,features(:,3),'k');
plot(time,features(:,4),'g');
plot(time,features(:,5),'y');
axis tight;
title(ada_string,'FontWeight','bold');