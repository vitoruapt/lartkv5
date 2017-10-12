%trains an adaboost classifier, input is training set, and iterations are
%the max number of weak classifiers allowed

function [classestimate,model,feat_of_wc] = train_adaboost(data,iterations)

%train adaboost classifier downloaded from internet

%features:
% 3: target velocity
% 4: lateral displacement (former relative velocity)
% 5: heading difference
% 6: angle between robot head and target pos
% 7: distance
% 8: relative velocity x
% 9: relative velocity y

datafeatures = data(:,3:end);

% data classification
data(data(:,2)==0,2)=-1; %transform from 0 to -1
dataclass = data(:,2);   %second column has leader tag

% Use Adaboost to make a classifier
[classestimate,model]=adaboost('train',datafeatures,dataclass,iterations);

% Show the error verus number of weak classifiers
error=zeros(1,length(model));
feat_of_wc = zeros(1,length(model));
cont_ratio = zeros(1,size(datafeatures,2));

for i=1:length(model)
    error(i)=model(i).error;
    feature_id = model(i).dimension;
    feat_of_wc(i) = feature_id;
    cont_ratio(feature_id)=cont_ratio(feature_id)+model(i).alpha;
end
cont_ratio = cont_ratio./sum(cont_ratio);

fh1 = figure;
set(fh1,'defaultlinelinewidth',3);
set(fh1,'defaultaxeslinewidth',2);
set(fh1,'defaulttextfontsize',12);
set(fh1,'defaultaxesfontsize',12);
set(fh1,'position',[480 210 800 400]);
hold, title('Classification error versus number of weak classifiers');
plot(error);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

fh2 = figure;
set(fh2,'defaultlinelinewidth',3);
set(fh2,'defaultaxeslinewidth',2);
set(fh2,'defaulttextfontsize',12);
set(fh2,'defaultaxesfontsize',12);
set(fh2,'position',[ 679   106   600   400]);
hold, title('Features');

% [H,X] = hist(feat_of_wc,1:size(datafeatures,2));
[H,X] = hist(feat_of_wc,1:size(datafeatures,2));
a = [H;X]';
a(:,3) = a(:,1)/length(feat_of_wc);
sortrows(a)

cm = jet(length(X));
for i=1:length(X)
    h=bar(X(i),H(i));
    set(h,'facecolor',cm(i,:));
end

ylabel('weak classifiers');
set(gca,'XTickLabel',[]);
axis tight;
% legend('velocity',...
%     'lateral displacement',...
%     'relative heading',...
%     'angle','distance',...
%     'relative vel. x',...
%     'relative vel. y',...
%     'location','eastOutside');
legend('mean target vel.',...
    'mean lat. displ.',...
    'mean rel. head.',...
    'mean angle',...
    'mean distance',...
    'mean relative vel. x',...
    'mean relative vel. y',...
    'Location','eastOutside')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

fh3 = figure;
set(fh3,'defaultlinelinewidth',3);
set(fh3,'defaultaxeslinewidth',2);
set(fh3,'defaulttextfontsize',12);
set(fh3,'defaultaxesfontsize',12);
set(fh3,'position',[ 679   106   600   400]);
hold,
title({'Features Contribution Ratio';...
    sprintf('(number of weak classifiers = %i)',length(feat_of_wc))})

%X = 1:length(cont_ratio);
H = cont_ratio;
cm = jet(length(X));
for i=1:length(X)
    h2=bar(X(i),H(i));
    set(h2,'facecolor',cm(i,:));
end

ylabel('contribution ratio');
set(gca,'XTick',1:21);
set(gcf,'position',[250 100 1200 500]);
axis tight;
set(gca,'YLim',[0 0.25]);
grid;
legend('1. lateral displ.',...
      '2. rel. heading',...
      '3. angle',...
      '4. distance',...
      '5. stdv distance',...
      'Location','eastOutside');
  
  
  legend('1. target velocity',...
      '2. lateral displ.',...
      '3. rel. heading',...
      '4. angle',...
      '5. distance',...
      '6. relative vel. x',...
      '7. relative vel. y',...
      '8. stdv target vel.',...
      '9. stdv lateral displ.',...
      '10. stdv rel. head.',...
      '11. stdv angle',...
      '12. stdv distance',...
      '13. stdv relative vel. x',...
      'Location','eastOutside');
  

% if (length(H)~=length(X))
%     X = [1 round(X)]
% end
a = [H;X]';
sortrows(a)
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%   legend('1. target velocity',...
%       '2. lateral displ.',...
%       '3. rel. heading',...
%       '4. angle',...
%       '5. distance',...
%       '6. relative vel. x',...
%       '7. relative vel. y',...
%       '8. \Delta target vel.',...
%       '9. \Delta lateral displ.',...
%       '10. \Delta rel. head.',...
%       '11. \Delta angle',...
%       '12. \Delta distance',...
%       '13. \Delta relative vel. x',...
%       '14. \Delta relative vel. y',...
%       '15. stdv target vel.',...
%       '16. stdv lateral displ.',...
%       '17. stdv rel. head.',...
%       '18. stdv angle',...
%       '19. stdv distance',...
%       '20. stdv relative vel. x',...
%       '21. stdv relative vel. y',...
%       'Location','eastOutside');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Training results
% Show results
%   blue=datafeatures(classestimate==-1,:); red=datafeatures(classestimate==1,:);
%   I=zeros(161,161);
%   for i=1:length(model)
%       if(model(i).dimension==1)
%           if(model(i).direction==1), rec=[-80 -80 80+model(i).threshold 160];
%           else rec=[model(i).threshold -80 80-model(i).threshold 160 ];
%           end
%       else
%           if(model(i).direction==1), rec=[-80 -80 160 80+model(i).threshold];
%           else rec=[-80 model(i).threshold 160 80-model(i).threshold];
%           end
%       end
%       rec=round(rec);
%       y=rec(1)+81:rec(1)+81+rec(3); x=rec(2)+81:rec(2)+81+rec(4);
%       I=I-model(i).alpha; I(x,y)=I(x,y)+2*model(i).alpha;
%   end
%  subplot(2,2,2), imshow(I,[]); colorbar; axis xy;
%  colormap('jet'), hold on
%  plot(blue(:,1)+81,blue(:,2)+81,'bo');
%  plot(red(:,1)+81,red(:,2)+81,'ro');
%  title('Training Data classified with adaboost model');



