%back tagging, never used

function out = do_transition(in)

out = in;
tag_i = find(diff(in(:,2))==1);
timestep = mean(diff(in(:,1)));
window = 1;
steps = round(window/timestep);
trans_i = tag_i - steps;
out(:,2)=0;
out(trans_i:tag_i,2)=1;

end
