function [data_alt, data_or, data_accel, data_vel] = loadDataset(filename, usarsim_compatibility)

    % vars to hold plot data
    data_alt     = [];
    data_or      = [];
    data_accel   = [];
    data_vel     = [];
    
    if (usarsim_compatibility == 1)
        data_vel = zeros(1,4);
    end
  
    % read YAML
    addpath(genpath('yaml'));
    yaml_file = filename;

    InitYaml();

    import('org.yaml.snakeyaml.Yaml');

    yamlreader = Yaml();
    yml = fileread(yaml_file);
    jymlobj = yamlreader.loadAll(yml);

    iterator = jymlobj.iterator();

    while (iterator.hasNext())
        field = iterator.next();
        Data = Hash2Struct(field);

        % document is measurement
        if (isfield(Data, 'e') && Data.e == 1)

            % sonar
            if (isfield(Data, 'alt') && (usarsim_compatibility == 0 || Data.s == 3))
                data_alt = [data_alt; Data.t Data.alt];
            end

            % IMU (or)
            if (isfield(Data, 'or') && (usarsim_compatibility == 0 || Data.s == 2))
                data_or = [data_or; Data.t Data.or];
            end

            % accel
            if (isfield(Data, 'accel') && (usarsim_compatibility == 0 || Data.s == 4))
                data_accel = [data_accel; Data.t Data.accel];
            end

            % vel
            if (isfield(Data, 'vel') && usarsim_compatibility == 0)
                data_vel = [data_vel; Data.t Data.vel];
            end
        end
    end
end