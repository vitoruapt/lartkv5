%new tag and shift time
%as tests have different initial times, this function put all of them
%in the same reference frame. the offsets are computed based on the 
%recorded images, because they have the correct time
%first the tags are obatined using rxbag, then the first image of each
%bag is compared wrt their clock stamp, the difference is the offset
%normally the problem that created this differences has been solved,
%and this would not be required for new bags and tests

function [new_var] = do_tag(input_var, pro_tag, rich_tag, jor_tag,offset,off_j)

%shift
i = diff(input_var(:,2))==1;
old_time = input_var(i,1);
if(isempty(old_time))
    time_diff = pro_tag;
else
    time_diff = pro_tag - old_time;
end
new_var = input_var;
new_var(:,1) = new_var(:,1) + time_diff;

%new tag rich
[val pos] = min(abs(new_var(:,1) - (rich_tag -offset)));
class_rich = zeros(length(input_var),1);
class_rich(pos:end)=1;
if(rich_tag==99)
    class_rich(end)=0;
end

%new tag jor
[val pos] = min(abs(new_var(:,1) - (jor_tag -offset-off_j)));
class_jor = zeros(length(input_var),1);
class_jor(pos:end)=1;
if(rich_tag==99) 
  class_jor(end)=0;
end

new_var = [new_var(:,1:2) class_rich class_jor new_var(:,3:end)];

plot_test2(new_var);