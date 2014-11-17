%%
[orgb map alpha] = imread('planetracker.png');
nrgb = orgb(:,:,[2 1 3]);
image(orgb)
image(nrgb)
imwrite(nrgb,'otherplane.png','Alpha',alpha);

%%
[a,b,c] = imread('apm2.ico','ICO'); 
%%
% Augment colormap for background color (white).
b2 = [b; 1 1 1]; 
% Create new image for display. 
d = ones(size(a)) * (length(b2) - 1); 
% Use the AND mask to mix the background and
% foreground data on the new image
d(c == 0) = a(c == 0); 
% Display new image
image(uint8(d)), colormap(b2)

%%
nrgb = orgb(:,:,[2 1 3]);
image(orgb)
image(nrgb)
%imwrite(nrgb,'otherplane.png','Alpha',alpha);

