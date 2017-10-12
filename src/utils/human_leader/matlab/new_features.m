%decompose relative velocity in x and y, compute lateral displacement

function [out]=new_features(in)

out = in;
[rel_vel_x rel_vel_y] = relative_velocities(in);
out(:,8) = rel_vel_x;
out(:,9) = rel_vel_y;
out(:,4) = sin(in(:,6)).*in(:,7); %put ld in place of rv