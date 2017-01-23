%
%  A demonstration file for gridLegend
%
% Adrian Cherry
% 2/11/2010

% Lets generate lots of data and plot it
y=[0:0.1:3.0]'*sin(-pi:0.1:pi);
hdlY=plot(y');

% now the standard legend function will probably flow off the figure
% although if you maximise it then you might see all of the legend,
% however the print preview is probably truncated.
legend(hdlY);
pause

% plot again this time using gridLegend to print the legend in 4 columns
hdlY=plot(y');
gridLegend(hdlY,4);
pause

% As standard the legend flows down filling the first column before
% moing onto the next. We can change this by using the Orientation
% horizontal option to fill across the rows before moving down a row.
gridLegend(hdlY,4,'Orientation','Horizontal');
pause 

% to use some options the standard legend function needs a key
for i=1:31,
    key{i}=sprintf('trace %d',i);
end

% here we place legend on the lefthand side and reduce the fontsize
hdlY=plot(y');
gridLegend(hdlY,2,key,'location','westoutside','Fontsize',8,'Box','off');
pause

% try with subplots - set figure to full screen because it is crowded.
figure('units','normalized','outerposition',[0 0 1 1]);
sb1=subplot(2,2,1);
hdlY1=plot(y');
sub2=subplot(2,2,2);
hdlY2=plot(y');
sub3=subplot(2,2,3);
hdlY3=plot(y');
sub4=subplot(2,2,4);
hdlY4=plot(y');
gridLegend(hdlY1,4,key,'location','north','Fontsize',8,'Box','off');
gridLegend(hdlY2,3,key,'location','eastoutside','Fontsize',8,'Box','on');
gridLegend(hdlY3,2,key,'location','westoutside','Fontsize',8,'Box','off');
gridLegend(hdlY4,5,key,'location','southoutside','Fontsize',8,'Box','off');



