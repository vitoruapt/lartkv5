%%%%%%%%%%%% raw with derivative in vel and dist %%%%%%%%

for name=1:9
    eval(sprintf('st%02d_ntr=do_transition(st%02d_enh);',name,name));
end

for name=1:9
    eval(sprintf('gd%02d_ntr=do_transition(gd%02d_enh);',name,name));
end

for name=1:7
    eval(sprintf('as%02d_ntr=do_transition(as%02d_enh);',name,name));
end

for name=1:5
    eval(sprintf('fr%02d_ntr=do_transition(fr%02d_enh);',name,name));
end

ts_st_ntr = [st01_ntr;st03_ntr;st05_ntr;st06_ntr;st07_ntr;st09_ntr];
ts_gd_ntr = [gd01_ntr;gd03_ntr;gd04_ntr;gd05_ntr;gd07_ntr;gd08_ntr;gd09_ntr];
ts_as_ntr = [as01_ntr;as05_ntr;as06_ntr;as07_ntr];
ts_fr_ntr = [fr01_ntr;fr02_ntr;fr05_ntr];
validation_ntr = [st02_ntr;as04_ntr;fr03_ntr];
train_set_ntr = [ts_st_ntr; ts_gd_ntr;...
                ts_as_ntr; ts_fr_ntr;...
                validation_ntr;];
%%%


j = 0;
j=j+1;test_ntr(j).name = 'st04';test_ntr(j).set = st04_ntr;
j=j+1;test_ntr(j).name = 'st08';test_ntr(j).set = st08_ntr;
j=j+1;test_ntr(j).name = 'gd02';test_ntr(j).set = gd02_ntr;
j=j+1;test_ntr(j).name = 'gd06';test_ntr(j).set = gd06_ntr;
j=j+1;test_ntr(j).name = 'as02';test_ntr(j).set = as02_ntr;
j=j+1;test_ntr(j).name = 'as03';test_ntr(j).set = as03_ntr;
j=j+1;test_ntr(j).name = 'fr04';test_ntr(j).set = fr04_ntr;

test_ntr_total.set = [st04_ntr; st08_ntr; gd02_ntr;...
                     gd06_ntr; as02_ntr; as03_ntr;...
                     fr04_ntr];
test_ntr_total.name = 'complete set total';

%%%% used for training, so i can test training error

j = 0;
j=j+1;test_ntr_fk(j).name = 'st03';test_ntr_fk(j).set = st03_ntr;
j=j+1;test_ntr_fk(j).name = 'st05';test_ntr_fk(j).set = st05_ntr;
j=j+1;test_ntr_fk(j).name = 'gd03';test_ntr_fk(j).set = gd03_ntr;
j=j+1;test_ntr_fk(j).name = 'gd04';test_ntr_fk(j).set = gd04_ntr;
j=j+1;test_ntr_fk(j).name = 'as05';test_ntr_fk(j).set = as05_ntr;
j=j+1;test_ntr_fk(j).name = 'as06';test_ntr_fk(j).set = as06_ntr;
j=j+1;test_ntr_fk(j).name = 'fr05';test_ntr_fk(j).set = fr05_ntr;

test_ntr_fk_total.set = train_set_ntr;
test_ntr_fk_total.name = 'complete set total';