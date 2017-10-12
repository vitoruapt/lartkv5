%compute new features based on existing ones
%
% derivatives from position 10 to 16
% standard deviation, based on winsize, position 17 to 23
% mean, based on winsize, position 24 to 30
% 
% in final part, must uncomment the desired set, using only portions of
% the features computed, eliminating features with reduced contribution
% for example
%
%
% 3 velocity
% 4 lateral displacement
% 5 relative heading
% 6 angle 2 robot
% 7 distance
% 8 relative velocity x
% 9 relative velocity y

function new_features = enhance_features(input_features)

new_features = input_features;

% diff part
new_features(2:end,10) = diff(input_features(:,3));
new_features(2:end,11) = diff(input_features(:,4));
new_features(2:end,12) = diff(input_features(:,5));
new_features(2:end,13) = diff(input_features(:,6));
new_features(2:end,14) = diff(input_features(:,7));
new_features(2:end,15) = diff(input_features(:,8));
new_features(2:end,16) = diff(input_features(:,9));

new_features(1,10) = new_features(2,10);
new_features(1,11) = new_features(2,11);
new_features(1,12) = new_features(2,12);
new_features(1,13) = new_features(2,13);
new_features(1,14) = new_features(2,14);
new_features(1,15) = new_features(2,15);
new_features(1,16) = new_features(2,16);

% mean and stdv part
for i = 1:length(input_features)
    winsize = 30;
    while(i-winsize <= 0) %not enough past info
        winsize = winsize - 1;
    end
    
    % stdv part
    new_features(i,17) = std(input_features(i-winsize:i,3)); %std vel
    new_features(i,18) = std(input_features(i-winsize:i,4)); %std lat. disp.
    new_features(i,19) = std(input_features(i-winsize:i,5)); %std rel. head.
    new_features(i,20) = std(input_features(i-winsize:i,6)); %std angle 2 robot
    new_features(i,21) = std(input_features(i-winsize:i,7)); %std dist
    new_features(i,22) = std(input_features(i-winsize:i,8)); %std rel v x
    new_features(i,23) = std(input_features(i-winsize:i,8)); %std rel v y
    
    % mean part
    new_features(i,24) = mean(input_features(i-winsize:i,3)); %mean vel
    new_features(i,25) = mean(input_features(i-winsize:i,4)); %mean lat. disp.
    new_features(i,26) = mean(input_features(i-winsize:i,5)); %mean rel. head
    new_features(i,27) = mean(input_features(i-winsize:i,6)); %mean angle 2 robot
    new_features(i,28) = mean(input_features(i-winsize:i,7)); %mean dist
    new_features(i,29) = mean(input_features(i-winsize:i,8)); %mean rel v x
    new_features(i,30) = mean(input_features(i-winsize:i,9)); %mean rel v y
end

% %test single feature thresholds
% new_features = [new_features(:,1:2)...
%      new_features(:,7)];

%test integration with opencv adaboost
% new_features = [new_features(:,1:2)...
%     new_features(:,3:7)];

% %test vel, lat dist., dist
% new_features = [new_features(:,1:2)...
%     new_features(:,3)...
%     new_features(:,4)...
%     new_features(:,7)...
%     ];


% %reduced4 (lat. displ., dist., stdv distance)
% new_features = [new_features(:,1:2)...
%     new_features(:,4)...
%     new_features(:,7)...
%     new_features(:,21)...
%     ];

% %reduced3 ODNM (minus relvx, relvy, stdv ld, stdv dist)
% new_features = [new_features(:,1:2)...
%     new_features(:,3:7)...
%     ];

% %reduced3 (lat. displ., rel. heading, angle, dist. stdv distance)
% new_features = [new_features(:,1:2)...
%     new_features(:,4:7)...
%     new_features(:,21)...
%     ];

%reduced2 == ODNM
% new_features = [new_features(:,1:2)...
%     new_features(:,3:9)...
%     new_features(:,17)...
%     new_features(:,21)...
%     ];

% %reduced1 (no diff, no stdv rel_v_y)
% new_features = [new_features(:,1:2)...
%     new_features(:,3:9)...
%     new_features(:,17:22)...
%     ];

% %train complete
 new_features = [new_features(:,1:2)...
    new_features(:,3:9)...
    new_features(:,10:16)...
    new_features(:,17:23)...
    ];

% %train no diff
% new_features = [new_features(:,1:2)...
%     new_features(:,3:9)...
%     new_features(:,17:23)...
%     ];

%with mean
% new_features = [new_features(:,1:2)...
%     new_features(:,3:9)... %kf output
%     new_features(:,10:16)... %diff
%     new_features(:,17:23)... %stdv
%     new_features(:,24:30)... %mean
%     ];

%only one
% new_features = [new_features(:,1:2)...
%     new_features(:,3)];

