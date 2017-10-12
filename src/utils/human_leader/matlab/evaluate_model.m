%evaluates adaboost classifier, comparing groundtruth
%with class output, may receive a single dataset or a structure of them,
%create plots comparing ground truth and classification, also prints error
%of false good, false bad and false total

function [] = evaluate_model(model,test_set)

if size(test_set,1)~=1
    temp = test_set;
    clear test_set;
    test_set.set = temp;
    test_set.name = 'single';
end

for number = 1:length(test_set)
    
    test = test_set(number).set;
    name = test_set(number).name;
    
    %prepare the data
    features = test(:,3:end);
    test(test(:,2)==0,2)=-1; %transform from 0 to -1
    gd_class = test(:,2);    %second column has leader tag [-1 good / 1 bad]
    
    %for plot purposes
    low_plot = min(min(features));
    up_plot = max(max(features));
    time = test(:,1);
    
    tic
    % Classify the features with the trained model
    if(isstruct(model))
        %classic adaboost
        test_class=adaboost('apply',features, model);
    else
        %matlab adaboost
        test_class=predict(model, features);
    end
    toc
    
    differences = test_class - gd_class;
    false_bad = length(find(differences == 2));
    false_good = length(find(differences == -2));
    false_total = false_bad + false_good;
    
    false_bad_ratio = false_bad / length(differences);
    false_good_ratio = false_good / length(differences);
    %     error_ratio = false_total / length(differences);
    false_bad_ratio = round(false_bad_ratio*100)/100;
    false_good_ratio = round(false_good_ratio*100)/100;
    error_ratio = false_bad_ratio + false_good_ratio;
%     error_ratio = round(error_ratio*100)/100;
    title_string = ...
        sprintf('false good:%.2f, false bad:%.2f, total error:%.2f',...
        false_good_ratio, false_bad_ratio, error_ratio);
    %title_string =  ['test:' name ' ' title_string];
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%% target plot %%%%%%%%%%%%%%%
    bad_tag = test(gd_class == 1,1);
    
    % Show the data
    H = figure;
    set(H,'defaultlinelinewidth',3);
    set(H,'defaultaxeslinewidth',2);
    set(H,'defaulttextfontsize',12);
    set(H,'defaultaxesfontsize',12);
    
    subplot(2,1,1), hold on, grid on;
    if ~isempty(bad_tag)
        line([bad_tag bad_tag],[low_plot up_plot],'Color',[1 0 0.1]);
    end
    plot(time,features(:,1),'b.');
    plot(time,features(:,2),'c.');
    plot(time,features(:,3),'k.');
%     plot(time,features(:,4),'g.');
%     plot(time,features(:,5),'y.');
    %     plot(time,features(:,6),'m.');
    %     plot(time,features(:,7),'color',[0.5 0 0]);
    
    %legend('vel','vel diff','head diff','angle', 'distance',...
    %      'Location','NorthWest');
    axis tight; %axis equal
    title(['test:' name '     ' 'ground truth']);
    %   large_markers;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Show result
    bad_tag = test(test_class == 1,1);
    
    % Show the data
    subplot(2,1,2), hold on, grid on;
    if ~isempty(bad_tag)
        line([bad_tag bad_tag],[low_plot up_plot],'Color',[1 0 0.1]);
    end
    plot(time,features(:,1),'b.');
    plot(time,features(:,2),'c.');
    plot(time,features(:,3),'k.');
%     plot(time,features(:,4),'g.');
%     plot(time,features(:,5),'y.');
    %     plot(time,features(:,6),'m.');
    %     plot(time,features(:,7),'color',[0.5 0 0]);
    %legend('vel','vel diff','head diff','angle', 'distance',...
    %      'Location','NorthWest');
    axis tight; %axis equal;
    title(title_string);
    %    large_markers;
end
%set(gcf,'position',[200 200 800 400]);

    score(length(test_class)-1)=0;
    for i=1:length(test_class)
        if test_class(i)==-1
            vote = 0.01;
        else
            vote = -0.5;
        end
        score(i+1) = score(i) + vote;
        if score(i+1) > 1
            score(i+1) = 1;
        elseif score(i+1) < -0.1
            score(i+1) = -0.1;
        end
    end
    
    X = figure; hold;
    set(X,'position',[200 400 800 300]);
    set(X,'defaultlinelinewidth',3);
    set(X,'defaultaxeslinewidth',2);
    set(X,'defaulttextfontsize',12);
    set(X,'defaultaxesfontsize',12);
    
    line([bad_tag bad_tag],[-0.1 1],'Color',[1 0 0.1]);
    score = score(1:end-1);
    sc = plot(time,score);
    l1 = line([time(1) time(end)],[0 0],'Color',[1 0 0.1]);
    l2 = line([time(1) time(end)],[0 0],'Color',[0 0 0]);
    
    legend([l1, sc, l2],'bad leader','leader score','threshold',...
        'location','eastoutside');
    axis tight;
    xlabel('time (s)'), ylabel('score');
    
    
end


function large_markers
tmp_l = legend('velocity','lateral displacement',...
    'relative heading','angle','distance',...
    'relative vel. x','relative vel. y',...
    'location','eastOutside');
set(tmp_l,'visible','off')
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
%set(ax2,'position',get(ax1,'position'));
hL = legend(ax2,'velocity','lateral displacement',...
    'relative heading','angle','distance',...
    'relative vel. x','relative vel. y',...
    'location','eastOutside');
set(ax2,'visible','off')
axis(ax1, 'tight');
%set(gcf,'position',[200 200 800 400]);
end




