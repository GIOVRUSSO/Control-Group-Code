folderpath = '/Users/tomstanton 1/Documents/College/Stage 5/ME Project/Data/Cycles120/';
savepath = '/Users/tomstanton 1/Documents/College/Stage 5/ME Project/Data/Summary/Cycles/120/Cycle';
csvfiles = dir(strcat(folderpath,'/*.csv')); 
N = length(csvfiles) ; 
A = cell(N,1) ;
for i = 1:N
    figure
    hold on;
    filepath=strcat(folderpath, csvfiles(i).name);
    A{i} = csvread(filepath,1,1);
    yyaxis left;
    xLength = length(A{i}(:,1))-1;
    plot(0:xLength,A{i}(:,1),'-o');
    plot(0:xLength,A{i}(:,2),'-og');
    plot([0 xLength],[140 140],'-r');
    ylim([0 200])
    yyaxis right;
    stairs(0:xLength,A{i}(:,3),'-');
    ylim([0 1])
    legend('boxoff')
    legend('Heart Rate (bpm)','Human Power (W)','BPM max','Assistance Level')
    yyaxis left;
    xlabel('Time (mins)');
    title(strcat('Cycle:'," ",string(i)));
    xlim([0 xLength]);
    %saveas(gcf,strcat(filepath(1:length(filepath)-4),'.png'));
    saveas(gcf,strcat(savepath," ",string(i),'/Recordings Plot.png'));
end
%close all;

