%%%%%%%%%%% ST %%%%%%%%%%%%%%%

st01eco = enhance_features(st01c);
st02eco = enhance_features(st02c);
st03eco = enhance_features(st03c);
st04eco = enhance_features(st04c);
st05eco = enhance_features(st05c);
st06eco = enhance_features(st06c);
st07eco = enhance_features(st07c);
st08eco = enhance_features(st08c);
st09eco = enhance_features(st09c);
st10eco = enhance_features(st10c);

st01ecr = enhance_features(st01);
st02ecr = enhance_features(st02);
st03ecr = enhance_features(st03);
st04ecr = enhance_features(st04);
st05ecr = enhance_features(st05);
st06ecr = enhance_features(st06);
st07ecr = enhance_features(st07);
st08ecr = enhance_features(st08);
st09ecr = enhance_features(st09);
st10ecr = enhance_features(st10);

%%%%%%%%%% GD %%%%%%%%%%%%%%%%%%%

gd01eco = enhance_features(gd01c);
gd02eco = enhance_features(gd02c);
gd03eco = enhance_features(gd03c);
gd04eco = enhance_features(gd04c);
gd05eco = enhance_features(gd05c);
gd06eco = enhance_features(gd06c);
gd07eco = enhance_features(gd07c);
gd08eco = enhance_features(gd08c);
gd09eco = enhance_features(gd09c);
gd10eco = enhance_features(gd10c);

gd01ecr = enhance_features(gd01);
gd02ecr = enhance_features(gd02);
gd03ecr = enhance_features(gd03);
gd04ecr = enhance_features(gd04);
gd05ecr = enhance_features(gd05);
gd06ecr = enhance_features(gd06);
gd07ecr = enhance_features(gd07);
gd08ecr = enhance_features(gd08);
gd09ecr = enhance_features(gd09);
gd10ecr = enhance_features(gd10);

%%%%%%%%%% AS %%%%%%%%%%%%%%%%%%%

as01eco = enhance_features(as01c);
as02eco = enhance_features(as02c);
as03eco = enhance_features(as03c);
as04eco = enhance_features(as04c);
as05eco = enhance_features(as05c);
as06eco = enhance_features(as06c);
as07eco = enhance_features(as07c);
as08eco = enhance_features(as08c);
as09eco = enhance_features(as09c);
as10eco = enhance_features(as10c);

as01ecr = enhance_features(as01);
as02ecr = enhance_features(as02);
as03ecr = enhance_features(as03);
as04ecr = enhance_features(as04);
as05ecr = enhance_features(as05);
as06ecr = enhance_features(as06);
as07ecr = enhance_features(as07);
as08ecr = enhance_features(as08);
as09ecr = enhance_features(as09);
as10ecr = enhance_features(as10);

%%%%%%%%%% FR %%%%%%%%%%%%%%%%%%%

fr01eco = enhance_features(fr01c);
fr02eco = enhance_features(fr02c);
fr03eco = enhance_features(fr03c);
fr04eco = enhance_features(fr04c);
fr05eco = enhance_features(fr05c);
fr06eco = enhance_features(fr06c);
fr07eco = enhance_features(fr07c);
fr08eco = enhance_features(fr08c);
fr09eco = enhance_features(fr09c);
fr10eco = enhance_features(fr10c);

fr01ecr = enhance_features(fr01);
fr02ecr = enhance_features(fr02);
fr03ecr = enhance_features(fr03);
fr04ecr = enhance_features(fr04);
fr05ecr = enhance_features(fr05);
fr06ecr = enhance_features(fr06);
fr07ecr = enhance_features(fr07);
fr08ecr = enhance_features(fr08);
fr09ecr = enhance_features(fr09);
fr10ecr = enhance_features(fr10);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ts_as_eco = [as01eco;as02eco;as04eco;as05eco;as06eco;as08eco;as09eco];
ts_st_eco = [st01eco;st02eco;st03eco;st05eco;st06eco;st07eco;st09eco];
ts_gd_eco = [gd01eco;gd03eco;gd04eco;gd05eco;gd07eco;gd08eco;gd09eco];
ts_fr_eco = [fr01eco;fr02eco;fr03eco;fr04eco;fr05eco;fr06eco;fr09eco];
ts_as_ecr = [as01ecr;as02ecr;as04ecr;as05ecr;as06ecr;as08ecr;as09ecr];
ts_gd_ecr = [gd01ecr;gd03ecr;gd04ecr;gd05ecr;gd07ecr;gd08ecr;gd09ecr];
ts_st_ecr = [st01ecr;st02ecr;st03ecr;st05ecr;st06ecr;st07ecr;st09ecr];
ts_fr_ecr = [fr01ecr;fr02ecr;fr03ecr;fr04ecr;fr05ecr;fr06ecr;fr09ecr];

train_set_eco = [ts_st_eco;ts_gd_eco;ts_as_eco;ts_fr_eco];
train_set_ecr = [ts_st_ecr;ts_gd_ecr;ts_as_ecr;ts_fr_ecr];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

validate_eco(1).name = 'st08';
validate_eco(2).name = 'st04';
validate_eco(3).name = 'gd06';
validate_eco(4).name = 'gd02';
validate_eco(5).name = 'as07';
validate_eco(6).name = 'as03';
validate_eco(7).name = 'fr07';
validate_eco(8).name = 'fr08';ts_as_eco = [as01eco;as02eco;as04eco;as05eco;as06eco;as08eco;as09eco];
ts_st_eco = [st01eco;st02eco;st03eco;st05eco;st06eco;st07eco;st09eco];
ts_gd_eco = [gd01eco;gd03eco;gd04eco;gd05eco;gd07eco;gd08eco;gd09eco];
ts_fr_eco = [fr01eco;fr02eco;fr03eco;fr04eco;fr05eco;fr06eco;fr09eco];
ts_as_ecr = [as01ecr;as02ecr;as04ecr;as05ecr;as06ecr;as08ecr;as09ecr];
ts_gd_ecr = [gd01ecr;gd03ecr;gd04ecr;gd05ecr;gd07ecr;gd08ecr;gd09ecr];
ts_st_ecr = [st01ecr;st02ecr;st03ecr;st05ecr;st06ecr;st07ecr;st09ecr];
ts_fr_ecr = [fr01ecr;fr02ecr;fr03ecr;fr04ecr;fr05ecr;fr06ecr;fr09ecr];

train_set_eco = [ts_st_eco;ts_gd_eco;ts_as_eco;ts_fr_eco];
train_set_ecr = [ts_st_ecr;ts_gd_ecr;ts_as_ecr;ts_fr_ecr];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

validate_eco(1).name = 'st08';
validate_eco(2).name = 'st04';
validate_eco(3).name = 'gd06';
validate_eco(4).name = 'gd02';
validate_eco(5).name = 'as07';
validate_eco(6).name = 'as03';
validate_eco(7).name = 'fr07';
validate_eco(8).name = 'fr08';

validate_eco(1).set = st08eco;
validate_eco(2).set = st04eco;
validate_eco(3).set = gd06eco;
validate_eco(4).set = gd02eco;
validate_eco(5).set = as07eco;
validate_eco(6).set = as03eco;
validate_eco(7).set = fr07eco;
validate_eco(8).set = fr08eco;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

validate_ecr(1).name = 'st08';
validate_ecr(2).name = 'st04';
validate_ecr(3).name = 'gd06';
validate_ecr(4).name = 'gd02';
validate_ecr(5).name = 'as07';
validate_ecr(6).name = 'as03';
validate_ecr(7).name = 'fr07';
validate_ecr(8).name = 'fr08';

validate_ecr(1).set = st08ecr;
validate_ecr(2).set = st04ecr;
validate_ecr(3).set = gd06ecr;
validate_ecr(4).set = gd02ecr;
validate_ecr(5).set = as07ecr;
validate_ecr(6).set = as03ecr;
validate_ecr(7).set = fr07ecr;
validate_ecr(8).set = fr08ecr;


validate_eco(1).set = st08eco;
validate_eco(2).set = st04eco;
validate_eco(3).set = gd06eco;
validate_eco(4).set = gd02eco;
validate_eco(5).set = as07eco;
validate_eco(6).set = as03eco;
validate_eco(7).set = fr07eco;
validate_eco(8).set = fr08eco;

validate_eco_total.set = ...
    [st08eco; st04eco; gd06eco; gd02eco;...
    as07eco; as03eco; fr07eco; fr08eco];
validate_eco_total.name = 'eco total'

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

validate_ecr(1).name = 'st08';
validate_ecr(2).name = 'st04';
validate_ecr(3).name = 'gd06';
validate_ecr(4).name = 'gd02';
validate_ecr(5).name = 'as07';
validate_ecr(6).name = 'as03';
validate_ecr(7).name = 'fr07';
validate_ecr(8).name = 'fr08';

validate_ecr(1).set = st08ecr;
validate_ecr(2).set = st04ecr;
validate_ecr(3).set = gd06ecr;
validate_ecr(4).set = gd02ecr;
validate_ecr(5).set = as07ecr;
validate_ecr(6).set = as03ecr;
validate_ecr(7).set = fr07ecr;
validate_ecr(8).set = fr08ecr;

validate_ecr_total.set = ...
    [st08ecr; st04ecr; gd06ecr; gd02ecr;...
    as07ecr; as03ecr; fr07ecr; fr08ecr];
validate_ecr_total.name = 'ecr total'
