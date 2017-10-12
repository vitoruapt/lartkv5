function [out_var] = new_tag(in_var, n_tag)

%new tag position on matrix
[~, pos] = min(abs(in_var(:,1) - n_tag));

%new tag vector
tag_vec = zeros(length(in_var),1);
tag_vec(pos:end) = 1;

%in the case of no tag
if(n_tag==99)
    tag_vec(end)=0;
end

%new var
out_var = [in_var(:,1) tag_vec in_var(:,5:9)];

plot_test(out_var);

%%%%%%%%%%%%%%%%%%%%

% st01 = new_tag(x01,27.97);
% fr01 = new_tag(x02,9.08);
% gd02 = new_tag(x05,99);
% gd01 = new_tag(x04,99);
% as01 = new_tag(x06,12.02);
% fr02 = new_tag(x07,12.78);
% gd03 = new_tag(x08,99);
% gd04 = new_tag(x09,99);
% as02 = new_tag(x10,18.24);
% fr03 = new_tag(x12,9.14);
% as03 = new_tag(x14,12.71);
% st02 = new_tag(x15,17.17);
% gd05 = new_tag(x16,99);
% gd06 = new_tag(x17,99);
% st03 = new_tag(x18,30.24);
% fr04 = new_tag(x20,14.01);
% st04 = new_tag(x21a,9.96);
% as04 = new_tag(x21b,6.64);
% gd07 = new_tag(x23,99);
% st05 = new_tag(x24,18.61);
% as05 = new_tag(x25,18.91);
% st06 = new_tag(x26,10.86);
% gd08 = new_tag(x27,99);
% st07 = new_tag(x28,14.64);
% st08 = new_tag(x30,11.92);
% gd09 = new_tag(x31a,99);
% gd10 = new_tag(x31b,99);
% gd11 = new_tag(x33,99);
% fr05 = new_tag(x34,14.67);
% st09 = new_tag(xi01,15.6);
% as06 = new_tag(xi03,18.96);
% as07 = new_tag(xi05,12.28);