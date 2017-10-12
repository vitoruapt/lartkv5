%create linear variation of features to find what are the thresholds
%used by the classifier on each feature, receive as argument the classifier
%and the dimension to evaluate


function []=show_thresholds(model,dim)

clear new_model;
clear j; j=1; clear test_thresh;

% test_thresh = zeros(401,9);

test_thresh(:,1) = 1:401;
% second column is tag, does not matter here.
test_thresh(:,3) = -10:0.05:10; %1 velocity
% test_thresh(:,4) = -10:0.05:10; %2 lat disp.
% test_thresh(:,5) = -pi:0.0157:pi; %3 rel head.
% test_thresh(:,6) = -pi:0.0157:pi; %4 angle
% test_thresh(:,7) = -10:0.05:10; %5 dist
% test_thresh(:,8) = -10:0.05:10; %6 rel vx
% test_thresh(:,9) = -10:0.05:10; %7 rel vy
% 
% test_thresh(:,10:23) = 0;
% for ii=10:23
%     test_thresh(:,ii) = -10:0.05:10;
% end

for i=1:length(model)
    if model(i).dimension == dim
        new_model(j)=model(i);
        j=j+1;
    end
end

evaluate_model_single(new_model,test_thresh);
end
