%interface with ros, receives a feature message and
%apply classifier, then outputs a new message with
%the classification

function []=ros_subscriber(model)

% create a publisher for a geometry_msgs/Pose message
pub_process = geometry_msgs_Pose(...
    'connect','publisher','subscribe_to_matlab','pose');

% create a subscriber for a geometry_msgs/Twist message
sub_process = geometry_msgs_PoseWithCovariance(...
    'connect','subscriber','publish_to_matlab','PoseWithCovariance');

% create an empty messages structure
out_msg = geometry_msgs_Pose('empty');
in_msg = geometry_msgs_PoseWithCovariance('empty');

% read a message and print to screen
while (1)
      
    in_msg = geometry_msgs_PoseWithCovariance('read', sub_process, 100);
    
    if ~isempty(in_msg)
        tic
        %info is being passed in the covariance struct
        x_pos = in_msg.pose.position.x;
        y_pos = in_msg.pose.position.y;
        id = in_msg.pose.position.z;
        sample = [in_msg.covariance(1) ...
            in_msg.covariance(2) ...
            in_msg.covariance(3) ...
            in_msg.covariance(4) ...
            in_msg.covariance(5)];
        
        if(isstruct(model))
            %to be used with classic adaboost
            %quality = ada_classify(sample,model);
            quality = adaboost('apply', sample, model);
        else  
            %to be used with adaboost from matlab
            quality = predict(model,sample);
        end
               
        out_msg.position.x = x_pos;
        out_msg.position.y = y_pos;
        out_msg.position.z = quality;
        out_msg.orientation.x = id; 
        geometry_msgs_Pose('send', pub_process, out_msg);
        toc
    end
end
