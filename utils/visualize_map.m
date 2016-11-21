map_filename = '../data/map/sorghum_field.dat';
map = dlmread(map_filename, ' ', 7, 0);
imagesc(map);