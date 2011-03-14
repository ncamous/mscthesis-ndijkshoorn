filename = 'C:/Users/Nick/Documents/Thesis/code/cpp/dataset_collector/dataset_collector/dataset/001/output.yaml';

% vars to hold plot data
plot_alt     = [];
plot_or      = [];
plot_accel   = [];
plot_vel     = [];


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
        if (Data.s == 3)
            plot_alt = [plot_alt; Data.alt];
        end
        
        % IMU (or)
        if (Data.s == 2)
            plot_or = [plot_or; Data.or*0.1];
        end
        
        % accel
        if (Data.s == 4)
            plot_accel = [plot_accel; Data.accel];
        end
    end
end



% ALT
figure();
plot(plot_alt);

% OR
yrange = [-10000, 360000];
figure();

subplot(3,1,1), plot(plot_or(:,1));
ylim(yrange);
title('X orientation');

% y
subplot(3,1,2), plot(plot_or(:,2));
ylim(yrange);
title('Y orientation');

% z
subplot(3,1,3), plot(plot_or(:,3));
ylim(yrange);
title('Z orientation');


% ACCEL
yrange = [-150, 150];
figure();

subplot(3,1,1), plot(plot_accel(:,1));
ylim(yrange);
title('X acceleration');

% y
subplot(3,1,2), plot(plot_accel(:,2));
ylim(yrange);
title('Y acceleration');

% z
subplot(3,1,3), plot(plot_accel(:,3));
ylim(yrange);
title('Z acceleration');
