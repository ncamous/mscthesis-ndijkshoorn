function YamlStruct = ReadYaml(yaml_file)
% This function reads Yaml file into struct
% Example
% >> yaml_file = 'EnaspolMain.yaml';
% >> YamlStruct = ReadYaml(yaml_file)
%
%  %======================================================================
%{
		Copyright (c) 2011
		This program is a result of a joined cooperation of Energocentrum
		PLUS, s.r.o. and Czech Technical University (CTU) in Prague.
        The program is maintained by Energocentrum PLUS, s.r.o. and
        licensed under the terms of MIT license. Full text of the license
        is included in the program release.
		
        Author(s):
		Jiri Cigler, Dept. of Control Engineering, CTU Prague 
		Jan  Siroky, Energocentrum PLUS s.r.o.
		
        Implementation and Revisions:

        Auth  Date        Description of change
        ----  ---------   -------------------------------------------------
        jc    01-Mar-11   First implementation
        jc    02-Mar-11   .jar package initialization moved to external fun
%}
%======================================================================

InitYaml();

import('org.yaml.snakeyaml.Yaml');

yamlreader = Yaml();
yml = fileread(yaml_file);
jymlobj = yamlreader.loadAll(yml);

iterator = jymlobj.iterator();

while (iterator.hasNext())
    field = iterator.next();

    Data = Hash2Struct(field);
end

end % end of function


