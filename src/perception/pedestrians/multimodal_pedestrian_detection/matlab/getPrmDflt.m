 function varargout = getPrmDflt( prm, dfs, checkExtra )
% 00002 % Helper to set default values (if not already set) of parameter struct.
% 00003 %
% 00004 % Takes input parameters and a list of 'name'/default pairs, and for each
% 00005 % 'name' for which prm has no value (prm.(name) is not a field or 'name'
% 00006 % does not appear in prm list), getPrmDflt assigns the given default
% 00007 % value. If default value for variable 'name' is 'REQ', and value for
% 00008 % 'name' is not given, an error is thrown. See below for usage details.
% 00009 %
% 00010 % USAGE (nargout==1)
% 00011 %  prm = getPrmDflt( prm, dfs, [checkExtra] )
% 00012 %
% 00013 % USAGE (nargout>1)
% 00014 %  [ param1 ... paramN ] = getPrmDflt( prm, dfs, [checkExtra] )
% 00015 %
% 00016 % INPUTS
% 00017 %  prm          - param struct or cell of form {'name1' v1 'name2' v2 ...}
% 00018 %  dfs          - cell of form {'name1' def1 'name2' def2 ...}
% 00019 %  checkExtra   - [0] if 1 throw error if prm contains params not in dfs
% 00020 %
% 00021 % OUTPUTS (nargout==1)
% 00022 %  prm    - parameter struct with fields 'name1' through 'nameN' assigned
% 00023 %
% 00024 % OUTPUTS (nargout>1)
% 00025 %  param1 - value assigned to parameter with 'name1'
% 00026 %   ...
% 00027 %  paramN - value assigned to parameter with 'nameN'
% 00028 %
% 00029 % EXAMPLE
% 00030 %  dfs = { 'x','REQ', 'y',0, 'z',[], 'eps',1e-3 };
% 00031 %  prm = getPrmDflt( struct('x',1,'y',1), dfs )
% 00032 %  [ x y z eps ] = getPrmDflt( {'x',2,'y',1}, dfs )
% 00033 %
% 00034 % See also INPUTPARSER
 %
 % Piotr's Image&Video Toolbox      Version 2.42
 % Copyright 2010 Piotr Dollar.  [pdollar-at-caltech.edu]
 % Please email me if you find bugs, or have suggestions or questions!
 % Licensed under the Lesser GPL [see external/lgpl.txt]
 
 if( mod(length(dfs),2) ), error('odd number of default parameters'); end
 if nargin<=2, checkExtra = 0; end
 
 % get the input parameters as two cell arrays: prmVal and prmField
 if iscell(prm) && length(prm)==1, prm=prm{1}; end
 if iscell(prm)
   if(mod(length(prm),2)), error('odd number of parameters in prm'); end
   prmField = prm(1:2:end); prmVal = prm(2:2:end);
 else
   if(~isstruct(prm)), error('prm must be a struct or a cell'); end
   prmVal = struct2cell(prm); prmField = fieldnames(prm);
 end
 
 % get and update default values using quick for loop
 dfsField = dfs(1:2:end); dfsVal = dfs(2:2:end);
 if checkExtra
   for i=1:length(prmField)
     j = find(strcmp(prmField{i},dfsField));
     if isempty(j), error('parameter %s is not valid', prmField{i}); end
     dfsVal(j) = prmVal(i);
   end
 else
   for i=1:length(prmField)
     dfsVal(strcmp(prmField{i},dfsField)) = prmVal(i);
   end
 end
 
 % check for missing values
 if any(strcmp('REQ',dfsVal))
   cmpArray = find(strcmp('REQ',dfsVal));
   error(['Required field ''' dfsField(cmpArray(1)) ''' not specified.'] );
 end
 
 % set output
 if nargout==1
   varargout{1} = cell2struct( dfsVal, dfsField, 2 );
 else
   varargout = dfsVal;
end
