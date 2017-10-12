%compute relative velocities, decomposing original feature that was scalar
%only, requires target_velocity, relative_velocity and relative_heading
function [rel_vx rel_vy] = relative_velocities(dataset)

tgt_v = dataset(:,3);
rel_v = dataset(:,4);
rel_h = dataset(:,5);

%recompute robot velocity
robot_v = rel_v  + tgt_v;
%sin an cos of relative angle (heading)
sin_v = sin(rel_h);
cos_v = cos(rel_h);
%compute target vel wrt robot frame
tgt_vx = tgt_v.*cos_v;
tgt_vy = tgt_v.*sin_v;
%compute relative vel wrt robot frame
rel_vx = tgt_vx - robot_v;
rel_vy = tgt_vy;
%rel_vx = tgt_v.*cos(rel_h) - (rel_v  + tgt_v);

%for comparison with rel_v from dataset
%rel_v = sqrt(((robotv-tgt_vx).^2)+(tgt_vy).^2);

% figure,hold
% plot(rel_v,'b')
% plot(abs(dataset(:,4)),'r')
% plot(tgt_vx,'m')
% plot(tgt_vy,'c')
% plot(sin_v,'k')
% plot(cos_v,'y')
% 
% delta = abs(dataset(:,4))-rel_v;
%plot(delta,'go')
end