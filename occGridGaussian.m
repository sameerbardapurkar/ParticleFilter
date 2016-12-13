% conversion factor from meters to decimeters (all pixels in decimeters)
m_to_dec=100;

%----ADJUSTABLE PARAMS-----
num_rows = 6;
row_width = 0.02*m_to_dec; %units in tens of cm (ie 40 cm row width)
num_ranges = 2;

%----FIXED PARAMS-----

total_width = 0.76*m_to_dec; %30 inches * 0.0254 m/in
intra_row_width = total_width-row_width;
range_width = 1.524*m_to_dec; %2m range width
intra_range_width = 3.048*m_to_dec; 
num_intra_ranges = num_ranges+1;
num_intra_rows = num_rows-1;

tic
gauss_numbers=20;
mu = 0;
sigma = total_width/6;

image_width = total_width*6;
image_length = 2*int64(intra_range_width)+int64(range_width);

rowstarts = [total_width/2 total_width/2+total_width total_width/2+total_width*2 total_width/2+total_width*3 total_width/2+total_width*4 total_width/2+total_width*5];

im=ones(image_width, image_length);
for imsize = 1:int64(intra_range_width)
    randnums = int64(normrnd(0,sigma,[1 gauss_numbers]));
    for i=1:image_width
        for j=1:length(rowstarts)
            for k=1:gauss_numbers
                if i==(rowstarts(j)+randnums(k))
                    im(i,imsize)=0;
                end
            end
        end
    end
end
toc

im(:,int64(intra_range_width)+int64(range_width):image_length)=im(:,1:int64(intra_range_width)+1);

a = [im,ones(size(im,1),int64(range_width)),im];

a = int64(a);


%{
for imsize = (int64(intra_range_width)+int64(range_width)):image_length
    randnums = int64(normrnd(0,sigma,[1 10]));
    for i=1:image_width
        for j=1:length(rowstarts)
            for k=1:gauss_numbers
                if i==(rowstarts(j)+randnums(k))
                    im(i,imsize)=1;
                end
            end
        end
    end
end
toc
%}

garbage = ['----hey sameer-----'; '----hey karthik----'; '--this is merritt--'; 'just using up space'];

dlmwrite('sorghum_field.dat',garbage, ' ')
dlmwrite('sorghum_field.dat',a,'-append',...
'delimiter',' ','roffset',3)
