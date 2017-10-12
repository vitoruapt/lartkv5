%train neural network

% function [net] = train_net(inputs,targets)
function [net] = train_net(train_set,n_neurons)

inputs = train_set(:,3:end);
targets = train_set(:,2);
targets(targets==0,2)=1;

% Solve a Pattern Recognition Problem with a Neural Network
inputs = inputs';
targets = targets';

% Create a Pattern Recognition Network

hiddenLayerSize = n_neurons;%[10,10];
net = patternnet(hiddenLayerSize);
%net.performParam.regularization = 0.1;

% net.trainFcn = 'trainbr';
net.trainParam.max_fail = 100;

% Setup Division of Data for Training, Validation, Testing
%net.divideFcn = 'dividetrain'

net.divideFcn = 'divideind';
net.divideParam.testInd = [];
net.divideParam.trainInd = 1:8504;
net.divideParam.valInd = 8505:10196;


% net.divideparam.trainratio = 85/100;
% net.divideparam.valratio = 10/100;
% net.divideparam.testratio = 5/100;

% Train the Network
[net,tr] = train(net,inputs,targets);
