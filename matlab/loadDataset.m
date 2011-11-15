function [data_alt, data_or, data_accel, data_vel, navdata_euler_angles, gyros_offsets, phys_gyro_temp, phys_gyros, raw_gyros, raw_gyros_110] = loadDataset(filename)

    % vars to hold plot data
    data_alt     = [];
    data_or      = [];
    data_accel   = [];
    data_vel     = [];
    
    navdata_euler_angles	= [];
    gyros_offsets           = [];
    phys_gyro_temp          = [];
    phys_gyros              = [];
    raw_gyros               = [];
    raw_gyros_110           = [];
    
  
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
            if (isfield(Data, 'alt'))
                data_alt = [data_alt; Data.t Data.alt];
            end

            % IMU (or)
            if (isfield(Data, 'or'))
                data_or = [data_or; Data.t Data.or];
            end

            % accel
            if (isfield(Data, 'accel'))
                data_accel = [data_accel; Data.t Data.accel];
            end

            % vel
            if (isfield(Data, 'vel'))
                data_vel = [data_vel; Data.t Data.vel];
            end
            
            
            
            % navdata_euler_angles
            if (isfield(Data, 'navdata_euler_angles'))
                navdata_euler_angles = [navdata_euler_angles; Data.t Data.navdata_euler_angles];
            end
            
            % gyros_offsets
            if (isfield(Data, 'gyros_offsets'))
                gyros_offsets = [gyros_offsets; Data.t Data.gyros_offsets];
            end
            
            % phys_gyro_temp
            if (isfield(Data, 'phys_gyro_temp'))
                phys_gyro_temp = [phys_gyro_temp; Data.t Data.phys_gyro_temp];
            end
            
            % phys_gyros
            if (isfield(Data, 'phys_gyros'))
                phys_gyros = [phys_gyros; Data.t Data.phys_gyros];
            end
            
            % raw_gyros
            if (isfield(Data, 'raw_gyros'))
                raw_gyros = [raw_gyros; Data.t Data.raw_gyros];
            end
            
            % vel
            if (isfield(Data, 'raw_gyros_110'))
                raw_gyros_110 = [raw_gyros_110; Data.t Data.raw_gyros_110];
            end
        end
    end
end