%apply enhance_features to vars, to compute extra feature, clear all the
%previous sets, create sets for each situation, create training set, etc.

%%%%%%%%%%%% enhance vars based on the improved version of them %%%%%%%%
%%%%%%%%%%%% which contains the lateral displ, rvx and rvy      %%%%%%%%
for name=1:9
    eval(sprintf('st%02d_enh=enhance_features(st%02d_ld);',name,name));
end

for name=1:9
    eval(sprintf('gd%02d_enh=enhance_features(gd%02d_ld);',name,name));
end

for name=1:7
    eval(sprintf('as%02d_enh=enhance_features(as%02d_ld);',name,name));
end

for name=1:5
    eval(sprintf('fr%02d_enh=enhance_features(fr%02d_ld);',name,name));
end

for name=1:11
    eval(sprintf('od%02d_enh=enhance_features(od%02d_ld);',name,name));
end

for name=1:7
    eval(sprintf('nm%02d_enh=enhance_features(nm%02d_ld);',name,name));
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%clear all previously created enhanced vars
clear ts_st_enh ts_gd_enh ts_as_enh ts_fr_enh ts_od_enh ts_nm_enh;
clear validation_enh train_set_enh;
clear test_enh test_enh_total test_enh_fk test_enh_fk_total all_sets;

%create sets for each situation (except od and nm)
ts_st_enh = [st02_enh;st03_enh;st05_enh;st06_enh;st07_enh;st09_enh];
ts_gd_enh = [gd01_enh;gd03_enh;gd04_enh;gd05_enh;gd07_enh;gd08_enh;gd09_enh];
ts_as_enh = [as01_enh;as04_enh;as06_enh;as07_enh];
ts_fr_enh = [fr01_enh;fr03_enh;fr05_enh];

use_odnm = true;
prepare_for_ann = false;

if(use_odnm)
    %create sets for od and nm situations
    ts_od_enh = [od02_enh;od03_enh;od04_enh;od05_enh;od06_enh;od07_enh;od08_enh];
    ts_nm_enh = [nm01_enh;nm02_enh;nm04_enh;nm06_enh];
    %validation used for ANN only
    if(prepare_for_ann)
        validation_enh = [st01_enh;as05_enh;fr02_enh;od09_enh;nm07_enh];
    else
        validation_enh = [];
    end
    %create training dataset
    train_set_enh = [...
        ts_st_enh; ts_gd_enh;...
        ts_as_enh; ts_fr_enh;...
        ts_od_enh; ts_nm_enh;...
        validation_enh];
    
    %create dataset containing all enhanced vars (for debug only)
    all_sets = [st01_enh;st02_enh;st03_enh;st04_enh;st05_enh;...
        st06_enh;st07_enh;st08_enh;st09_enh;...
        gd01_enh;gd02_enh;gd03_enh;gd04_enh;gd05_enh;...
        gd06_enh;gd07_enh;gd08_enh;gd09_enh;...
        as01_enh;as02_enh;as03_enh;as04_enh;as05_enh;...
        as06_enh;as07_enh;...
        fr01_enh;fr02_enh;fr03_enh;fr04_enh;fr05_enh;...
        od01_enh;od02_enh;od03_enh;od04_enh;od05_enh;...
        od06_enh;od07_enh;od08_enh;od09_enh;od10_enh;...
        nm01_enh;nm02_enh;nm03_enh;nm04_enh;nm05_enh;...
        nm06_enh;nm07_enh];
else
    if(prepare_for_ann)
        validation_enh = [st01_enh;as05_enh;fr02_enh]; 
    else
        validation_enh = [];
    end
    %create training dataset
    train_set_enh = [ts_st_enh; ts_gd_enh;...
        ts_as_enh; ts_fr_enh;...
        validation_enh];
end


%%% create test dataset
j = 0;
j=j+1;test_enh(j).name = 'st04';test_enh(j).set = st04_enh;
j=j+1;test_enh(j).name = 'st08';test_enh(j).set = st08_enh;
j=j+1;test_enh(j).name = 'gd02';test_enh(j).set = gd02_enh;
j=j+1;test_enh(j).name = 'gd06';test_enh(j).set = gd06_enh;
j=j+1;test_enh(j).name = 'as02';test_enh(j).set = as02_enh;
j=j+1;test_enh(j).name = 'as03';test_enh(j).set = as03_enh;
j=j+1;test_enh(j).name = 'fr04';test_enh(j).set = fr04_enh;
if(use_odnm)
    j=j+1;test_enh(j).name = 'nm05';test_enh(j).set = nm05_enh;
    j=j+1;test_enh(j).name = 'nm03';test_enh(j).set = nm03_enh;
    j=j+1;test_enh(j).name = 'od01';test_enh(j).set = od01_enh;
    j=j+1;test_enh(j).name = 'od10';test_enh(j).set = od10_enh;
end

%%% create grouped test dataset 
if(use_odnm)
    test_enh_total.set = [st04_enh; st08_enh; gd02_enh;...
        gd06_enh; as02_enh; as03_enh;...
        fr04_enh; nm05_enh; nm03_enh;...
        od01_enh; od10_enh];
else
    test_enh_total.set = [st04_enh; st08_enh; gd02_enh;...
        gd06_enh; as02_enh; as03_enh;...
        fr04_enh];
end
test_enh_total.name = 'complete set total';

%%%% create dataset of vars used in training, to test training error
j = 0;
j=j+1;test_enh_fk(j).name = 'st03';test_enh_fk(j).set = st03_enh;
j=j+1;test_enh_fk(j).name = 'st05';test_enh_fk(j).set = st05_enh;
j=j+1;test_enh_fk(j).name = 'gd03';test_enh_fk(j).set = gd03_enh;
j=j+1;test_enh_fk(j).name = 'gd04';test_enh_fk(j).set = gd04_enh;
j=j+1;test_enh_fk(j).name = 'as05';test_enh_fk(j).set = as05_enh;
j=j+1;test_enh_fk(j).name = 'as06';test_enh_fk(j).set = as06_enh;
j=j+1;test_enh_fk(j).name = 'fr05';test_enh_fk(j).set = fr05_enh;
if(use_odnm)
    j=j+1;test_enh_fk(j).name = 'od02';test_enh_fk(j).set = od02_enh;
    j=j+1;test_enh_fk(j).name = 'od04';test_enh_fk(j).set = od04_enh;
    j=j+1;test_enh_fk(j).name = 'nm02';test_enh_fk(j).set = nm02_enh;
    j=j+1;test_enh_fk(j).name = 'nm04';test_enh_fk(j).set = nm04_enh;
end

test_enh_fk_total.set = train_set_enh;
test_enh_fk_total.name = 'training set total';

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%%%%%%%%%%% new sets with lateral distance and rel vel %%%%%%%%%%%%%%%%%
% 
% for name=1:9
%     temp = eval(sprintf('st%02d',name));
%     temp = new_features(temp);
%     eval(sprintf('st%02d_ld=temp;',name));
% end
% 
% for name=1:9
%     temp = eval(sprintf('gd%02d',name));
%     temp = new_features(temp);
%     eval(sprintf('gd%02d_ld=temp;',name));
% end
% 
% for name=1:7
%     temp = eval(sprintf('as%02d',name));
%     temp = new_features(temp);
%     eval(sprintf('as%02d_ld=temp;',name));
% end
% 
% for name=1:5
%     temp = eval(sprintf('fr%02d',name));
%     temp = new_features(temp);
%     eval(sprintf('fr%02d_ld=temp;',name));
% end
% 
% ts_st_ld = [st01_ld;st03_ld;st05_ld;st06_ld;st07_ld;st09_ld];
% ts_gd_ld = [gd01_ld;gd03_ld;gd04_ld;gd05_ld;gd07_ld;gd08_ld;gd09_ld];
% ts_as_ld = [as01_ld;as05_ld;as06_ld;as07_ld];
% ts_fr_ld = [fr01_ld;fr02_ld;fr05_ld];
% ts_od_ld = [od02_ld;od03_ld;od04_ld;od05_ld;od06_ld;od07_ld;od08_ld];
% ts_nm_ld = [nm01_ld;nm02_ld;nm04_ld;nm06_ld];
% validation_ld = [st02_ld;as04_ld;fr03_ld;od09_ld;nm07_ld];
% train_set_ld = [ts_st_ld; ts_gd_ld;...
%     ts_as_ld; ts_fr_ld;...
%     ts_od_ld; ts_nm_ld;...
%     validation_ld;];
% 
% j = 0;
% j=j+1;test_ld(j).name = 'st04';test_ld(j).set = st04_ld;
% j=j+1;test_ld(j).name = 'st08';test_ld(j).set = st08_ld;
% j=j+1;test_ld(j).name = 'gd02';test_ld(j).set = gd02_ld;
% j=j+1;test_ld(j).name = 'gd06';test_ld(j).set = gd06_ld;
% j=j+1;test_ld(j).name = 'as02';test_ld(j).set = as02_ld;
% j=j+1;test_ld(j).name = 'as03';test_ld(j).set = as03_ld;
% j=j+1;test_ld(j).name = 'fr04';test_ld(j).set = fr04_ld;
% j=j+1;test_ld(j).name = 'nm05';test_ld(j).set = nm05_ld;
% j=j+1;test_ld(j).name = 'nm03';test_ld(j).set = nm03_ld;
% j=j+1;test_ld(j).name = 'od01';test_ld(j).set = od01_ld;
% j=j+1;test_ld(j).name = 'od10';test_ld(j).set = od10_ld;
% 
% test_ld_total.set = [st04_ld; st08_ld; gd02_ld;...
%     gd06_ld; as02_ld; as03_ld;...
%     fr04_ld; nm05_ld; nm03_ld;...
%     od01_ld; od10_ld];
% test_ld_total.name = 'complete set total';
% 
% %%%%%%%%%%%% new sets %%%%%%%%%%%%%%%%%%%%
% 
% ts_st = [st01;st02;st03;st05;st06;st07;st09];
% ts_gd = [gd01;gd03;gd04;gd05;gd07;gd08;gd09];
% ts_as = [as01;as02;as04;as05;as06];
% ts_fr = [fr01;fr02;fr03;fr05];
% train_set = [ts_st;ts_gd;ts_as;ts_fr];
% 
% j = 0;
% j=j+1;test(j).name = 'st04';test(j).set = st04;
% j=j+1;test(j).name = 'st08';test(j).set = st08;
% j=j+1;test(j).name = 'gd02';test(j).set = gd02;
% j=j+1;test(j).name = 'gd06';test(j).set = gd06;
% j=j+1;test(j).name = 'as02';test(j).set = as02;
% j=j+1;test(j).name = 'as03';test(j).set = as03;
% j=j+1;test(j).name = 'fr04';test(j).set = fr04;
% 
% test_total.set = [st04; st08; gd02; gd06; as02; as03; fr04];
% test_total.name = 'complete set total';
% 
% 
% %%%%%%%%%%%%%%%%%%%% clear sets %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% for name=1:9
%     eval(sprintf('clear st%02d_ld;',name));
%     eval(sprintf('clear st%02d;',name));
% end
% 
% for name=1:9
%     eval(sprintf('clear gd%02d_ld;',name));
%     eval(sprintf('clear gd%02d;',name));
% end
% 
% for name=1:7
%     eval(sprintf('clear as%02d_ld;',name));
%     eval(sprintf('clear as%02d;',name));
% end
% 
% for name=1:5
%     eval(sprintf('clear fr%02d_ld;',name));
%     eval(sprintf('clear fr%02d;',name));
% end
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%