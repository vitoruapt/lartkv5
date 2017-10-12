%test neural network

function [] = test_net(net,test_set)

for number = 1:length(test_set)
    
    test = test_set(number).set;
    name = test_set(number).name;
    
    %prepare the data
    features = test(:,3:end);
    input_val = features';
    target_val = test(:,2)';
    target_val(2,target_val == 0) = 1;
    
    gt_class = test(:,2);    %second column has leader tag
    
    %for plot purposes
    low_plot = min(min(features));
    up_plot = max(max(features));
    time = test(:,1);
    
    % Test the Network
    outputs = net(input_val);
    %errors = gsubtract(y_val',outputs);
    %performance = perform(net,y_val',outputs)
    
    classes = round(outputs(2,:));
    %classes = vec2ind(outputs) - 1;
    
    %%%%% debug plot %%%%%%
    % figure, hold;
    % plot(target_val','o');
    % plot(outputs(1,:),'k');
    % plot(outputs(2,:),'r');
    % plot(classes,'y');
    
    %diferences = abs(test_class - gd_class);
    % differences = abs(target_val(2,:)-classes);
    % error = length(find(differences == 1));
    % error_ratio = error / length(differences);
    % title_string =  ...
    %     sprintf('NeuralNet Classification. error:%.2f',error_ratio);
    
    
    differences = classes - target_val(2,:);
    false_bad = length(find(differences == -1));
    false_good = length(find(differences == 1));
    false_total = false_bad + false_good;
    
    false_bad_ratio = false_bad / length(differences);
    false_good_ratio = false_good / length(differences);
    false_bad_ratio = round(false_bad_ratio*100)/100;
    false_good_ratio = round(false_good_ratio*100)/100;
    error_ratio = false_bad_ratio + false_good_ratio;
    
    title_string = ...
        sprintf('false good:%.2f, false bad:%.2f, total error:%.2f',...
        false_good_ratio, false_bad_ratio, error_ratio);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%% groundtruth plot %%%%%%%%%%
    bad_tag = test(gt_class == 1,1);
    
    % Show groundtruth
    figure;
    subplot(2,1,1), hold on, grid on;
    if ~isempty(bad_tag)
        line([bad_tag bad_tag],[low_plot up_plot],'Color',[1 0 0.1]);
    end
    plot(time,features(:,1),'b.');
    plot(time,features(:,2),'m.');
    plot(time,features(:,3),'k.');
%     plot(time,features(:,4),'g.');
%     plot(time,features(:,5),'y.');
    %legend('vel','vel diff','head diff','angle', 'distance',...
    %      'Location','NorthWest');
    %axis equal; 
    axis tight;
    title(['test:' name '     ' 'ground truth']);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%% classification plot %%%%%%%
    bad_tag = test(classes == 0,1);
    
    % Show classification
    subplot(2,1,2), hold on, grid on;
    if ~isempty(bad_tag)
        line([bad_tag bad_tag],[low_plot up_plot],'Color',[1 0 0.1]);
    end
    plot(time,features(:,1),'b.');
    plot(time,features(:,2),'m.');
    plot(time,features(:,3),'k.');
%     plot(time,features(:,4),'g.');
%     plot(time,features(:,5),'y.');
    
    plot(time,outputs(1,:),'bo');
    
    %legend('vel','vel diff','head diff','angle', 'distance',...
    %      'Location','NorthWest');
    %axis equal; 
    axis tight;
    title(title_string);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end