width=176; % 176
height=38; % 144, but i only stored a part of the buffer at the moment (friday)
N=width*height;
DRONE_VIDEO_MAX_WIDTH=640;
DRONE_VIDEO_MAX_HEIGHT=480;

for i=1:182
    id = fopen(sprintf('C:/Users/Nick/Documents/Thesis/code/cpp/dataset_collector/dataset_collector/dataset/002/%06d.jpg', i), 'r');
    x = fread(id,inf,'uint8')';
    x=x/255;
    fclose(id);

    C=zeros(height,width,3);
	index = 0;

	for j=1:height
        index = j*DRONE_VIDEO_MAX_WIDTH*3;
        for k=1:DRONE_VIDEO_MAX_WIDTH
            C(j,k,1) = x(index);
            C(j,k,2) = x(index+1);
            C(j,k,3) = x(index+2);
            index = index + 3;
        end
    end

    figure(1);clf;
    imshow(C, []);
end