function out = do_normalize(in)

out = in;
for i = 5:9;
    range = max(in(:,i))-min(in(:,i));
    tmp = (in(:,i)-min(in(:,i)))/range;
    tmp = (tmp*2)-1;
    out(:,i)=tmp;
end

% all = [st01c;st02c;st03c;st04c;st05c;st06c;st07c;st08c;st09c;...
%     gd01c;gd02c;gd03c;gd04c;gd05c;gd06c;gd07c;gd08c;gd09c;...
%     as01c;as02c;as03c;as04c;as05c;as06c;as07c;as08c;as09c;...
%     fr01c;fr02c;fr03c;fr04c;fr05c;fr06c;fr07c;fr08c;fr09c];
% 
% for i = 3:7
%     minmax(i-2).min = min(all(:,i));
%     minmax(i-2).max = max(all(:,i));
%     minmax(i-2).range = minmax(i-2).max - minmax(i-2).min;
% end
% 
% for name=1:9
%     temp = eval(sprintf('st0%dc',name));
%     for i = 3:7
%         temp(:,i) = (((temp(:,i)-minmax(i-2).min)./minmax(i-2).range) -0.5)*2;
%     end
%     eval(sprintf('st0%dc=temp;',name));
% end
%    
% 
% for name=1:9
%     temp = eval(sprintf('gd0%dc',name));
%     for i = 3:7
%         temp(:,i) = (((temp(:,i)-minmax(i-2).min)./minmax(i-2).range) -0.5)*2;
%     end
%     eval(sprintf('gd0%dc=temp;',name));
% end
% 
% for name=1:9
%     temp = eval(sprintf('as0%dc',name));
%     for i = 3:7
%         temp(:,i) = (((temp(:,i)-minmax(i-2).min)./minmax(i-2).range) -0.5)*2;
%     end
%     eval(sprintf('as0%dc=temp;',name));
% end
% 
% for name=1:9
%     temp = eval(sprintf('fr0%dc',name));
%     for i = 3:7
%         temp(:,i) = (((temp(:,i)-minmax(i-2).min)./minmax(i-2).range) -0.5)*2;
%     end
%     eval(sprintf('fr0%dc=temp;',name));
% end
% 
% 
% ts_st_c = [st01c;st02c;st03c;st05c;st06c;st07c;st09c];
% ts_gd_c = [gd01c;gd03c;gd04c;gd05c;gd07c;gd08c;gd09c];
% ts_as_c = [as01c;as02c;as04c;as05c;as06c;as08c;as09c];
% ts_fr_c = [fr01c;fr02c;fr03c;fr04c;fr05c;fr06c;fr09c];
% train_set_complete = [ts_st_c;ts_gd_c;ts_as_c;ts_fr_c];
% 
% j = 0;
% j=j+1;validate_complete(j).name = 'st08';validate_complete(j).set = st08c;
% j=j+1;validate_complete(j).name = 'st04';validate_complete(j).set = st04c;
% j=j+1;validate_complete(j).name = 'gd06';validate_complete(j).set = gd06c;
% j=j+1;validate_complete(j).name = 'gd02';validate_complete(j).set = gd02c;
% j=j+1;validate_complete(j).name = 'as07';validate_complete(j).set = as07c;
% j=j+1;validate_complete(j).name = 'as03';validate_complete(j).set = as03c;
% j=j+1;validate_complete(j).name = 'fr04';validate_complete(j).set = fr04c;
% j=j+1;validate_complete(j).name = 'fr05';validate_complete(j).set = fr05c;
% 
% validate_complete_total.set = [st08c; st04c; gd06c; gd02c; as07c; as03c; fr04c; fr05c];
% validate_complete_total.name = 'complete set total';
% 
% 
% %%%%%%%%%%%%% cropped %%%%%%%%%%%%%%%%%%%
% 
% for name=1:9
%     temp = eval(sprintf('st0%d',name));
%     for i = 3:7
%         temp(:,i) = (((temp(:,i)-minmax(i-2).min)./minmax(i-2).range) -0.5)*2;
%     end
%     eval(sprintf('st0%d=temp;',name));
% end
%    
% 
% for name=1:9
%     temp = eval(sprintf('gd0%d',name));
%     for i = 3:7
%         temp(:,i) = (((temp(:,i)-minmax(i-2).min)./minmax(i-2).range) -0.5)*2;
%     end
%     eval(sprintf('gd0%d=temp;',name));
% end
% 
% for name=1:9
%     temp = eval(sprintf('as0%d',name));
%     for i = 3:7
%         temp(:,i) = (((temp(:,i)-minmax(i-2).min)./minmax(i-2).range) -0.5)*2;
%     end
%     eval(sprintf('as0%d=temp;',name));
% end
% 
% for name=1:9
%     temp = eval(sprintf('fr0%d',name));
%     for i = 3:7
%         temp(:,i) = (((temp(:,i)-minmax(i-2).min)./minmax(i-2).range) -0.5)*2;
%     end
%     eval(sprintf('fr0%d=temp;',name));
% end
% 
% 
% ts_st = [st01;st02;st03;st05;st06;st07;st09];
% ts_gd = [gd01;gd03;gd04;gd05;gd07;gd08;gd09];
% ts_as = [as01;as02;as04;as05;as06;as08;as09];
% ts_fr = [fr01;fr02;fr03;fr04;fr05;fr06;fr09];
% train_set_cropped = [ts_st;ts_gd;ts_as;ts_fr];
% 
% j = 0;
% j=j+1;validate_cropped(j).name = 'st08';validate_cropped(j).set = st08;
% j=j+1;validate_cropped(j).name = 'st04';validate_cropped(j).set = st04;
% j=j+1;validate_cropped(j).name = 'gd06';validate_cropped(j).set = gd06;
% j=j+1;validate_cropped(j).name = 'gd02';validate_cropped(j).set = gd02;
% j=j+1;validate_cropped(j).name = 'as07';validate_cropped(j).set = as07;
% j=j+1;validate_cropped(j).name = 'as03';validate_cropped(j).set = as03;
% j=j+1;validate_cropped(j).name = 'fr04';validate_cropped(j).set = fr04;
% j=j+1;validate_cropped(j).name = 'fr05';validate_cropped(j).set = fr05;
% 
% validate_cropped_total.set = [st08; st04; gd06; gd02; as07; as03; fr04; fr05];
% validate_cropped_total.name = 'complete set total';
% 
% all = [st01c;st02c;st03c;st04c;st05c;st06c;st07c;st08c;st09c;...
%     gd01c;gd02c;gd03c;gd04c;gd05c;gd06c;gd07c;gd08c;gd09c;...
%     as01c;as02c;as03c;as04c;as05c;as06c;as07c;as08c;as09c;...
%     fr01c;fr02c;fr03c;fr04c;fr05c;fr06c;fr07c;fr08c;fr09c];