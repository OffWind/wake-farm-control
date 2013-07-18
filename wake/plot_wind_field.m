%% This file reads the wind flow file from the disk and plots the wind field
%It is a huge file, so it can take some time
% Created 18/7-13 by MS

close all


%% Initializations
if(~exist('customFigPos'))
    addpath([pwd '../../../matlab/plottools'])
end


%% Load flow data
if(~exist('wind_data'))
    disp('Reading wind file. IT can take a loooong time (around 1 min). Default wind file size is 75MB')
    wind_data = dlmread('simple_flow.dat');
else
    disp('Be aware that the wind file is not reloaded unless you clear the wind_data variable')    
end


%% Process and plot wind flow
%Recreate x and y coordinates from data
xx = unique(wind_data(:,1));
yy = unique(wind_data(:,2));

%Reshape the wind flow data from a vector to an image matrix
wind_flow = reshape(wind_data(:,3),length(xx),length(yy))'; %Transpose to make the wind come from left instead of below
wind_flow = wind_flow(end:-1:1,:); %Must rearrange like this because of the way the surf plots the matrix

%Plot
tic
surf(xx,yy,wind_flow,'Linestyle','none')
view(gca,[-0.5 90]);
xlim([xx(1) xx(end)])
ylim([yy(1) yy(end)])
t = toc;
title(['surf  t=' num2str(t)])


return

%This produces the same figure as above. It is simpler, but much slower for
%some reason
figure
%contourf(xx,yy,wind_flow,100,'Linestyle','none')
tic
contourf(xx,yy,wind_flow,'Linestyle','none')
t=toc
title(['contourf  t=' num2str(t)])

%%New one
figure
tic
pcolor(xx,yy,wind_flow)
t= toc
title(['pcolor  t=' num2str(t)])
shading flat

figure
tic
imagesc(xx,yy,wind_flow)
t= toc
title(['imagesc t=' num2str(t)])

figure
tic
contour(xx,yy,wind_flow)
t=toc
title(['contour  t=' num2str(t)])

customFigPos

