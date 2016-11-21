% The robot drives along the x-axis, hence the very long image. The pixels
% of value 1 represent where the robot can drive. Note that the robot will
% not actually drive within the columns of 1's, but the laser scanner will read
% empty space in the columns (called ranges).

% conversion factor from meters to decimeters (all pixels in decimeters)
m_to_dec=10;

%----ADJUSTABLE PARAMS-----
num_rows = 2;
row_width = 0.4*m_to_dec; %units in tens of cm (ie 40 cm row width)
num_ranges = 5;

%----FIXED PARAMS-----

total_width = 0.8*m_to_dec; %30 inches * 0.0254 m/in
intra_row_width = total_width-row_width;
range_width = 2*m_to_dec; %2m range width
intra_range_width = 3*m_to_dec; 
num_intra_ranges = num_ranges+1;
num_intra_rows = num_rows-1;

%------MATRIX SIZE-----
map_x = (num_ranges*range_width+num_intra_ranges*intra_range_width);
map_y = (num_rows*row_width+num_intra_rows*intra_row_width);

map = zeros(map_y, map_x);

for i=intra_range_width:(intra_range_width+range_width):(map_x-range_width)
    map(:,i+1:i+range_width)=1;
end

for i=row_width:total_width:(map_y-intra_row_width)
    map(i+1:i+intra_row_width,:)=1;
end

% Visualize the map
spy(map)

% Write to the dat file
garbage = ['----hey sameer-----'; '----hey karthik----'; '--this is merritt--'; 'just using up space'];
dlmwrite('sorghum_field.dat',garbage, ' ')
dlmwrite('sorghum_field.dat',map,'-append',...
'delimiter',' ','roffset',3)

