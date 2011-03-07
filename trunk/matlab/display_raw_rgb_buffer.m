folder = 'C:/Users/Nick/Documents/Thesis/code/cpp/dataset_collector/dataset_collector/dataset/004';
nr_images = 53;

%width=176; % 176
%height=38; % 144, but i only stored a part of the buffer at the moment (friday)
N=width*height;
%DRONE_VIDEO_MAX_WIDTH=640;
%DRONE_VIDEO_MAX_HEIGHT=480;

for i = 1:nr_images
    
    id = fopen(sprintf('%s/%06d.raw', folder, i), 'r');
    x = fread(id,inf,'uint8')';

    width = 0;
    width = bitshift(uint32(x(1)), 8);
    width = width + uint32(x(2));
  
    height = 0;
    height = bitshift(uint32(x(3)), 8);
    height = height + uint32(x(4)); 

    byteoffset = 5;

    x=x/255;
    fclose(id);

    C=zeros(height,width,3);
	index = 0;

	for j=1:height
        %index = j*DRONE_VIDEO_MAX_WIDTH*3;
        index = byteoffset + (j-1)*width*3;
        %for k=1:DRONE_VIDEO_MAX_WIDTH
        for k=1:width
            C(j,k,1) = x(index+2);
            C(j,k,2) = x(index+1);
            C(j,k,3) = x(index);
            index = index + 3;
        end
	end

    figure(1);clf;
    imshow(C, []);
end