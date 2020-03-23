clear all;
folderpath = '/Users/tomstanton 1/Documents/College/Stage 5/ME Project/Data/Cycles140/';
folderpathMPC = '/Users/tomstanton 1/Documents/College/Stage 5/ME Project/Data/MPC_Predictions/140/Cycle ';
savepath = '/Users/tomstanton 1/Documents/College/Stage 5/ME Project/Data/Summary/Cycles/140/Cycle';
csvfiles = dir(strcat(folderpath,'/*.csv'));
N = length(csvfiles);
A = cell(N,1) ;
error = zeros(N,1);
cycleduration = zeros(N,1);
starts = [5 5 4 4 4 4 5 4 3 4 7 4 4 5 4 4 4];
%starts = [4];
k=0;

for i=1:N
    cyclepath=strcat(folderpath, csvfiles(i).name);
    currentCycle = csvread(cyclepath,1,1);
    x1k = currentCycle(:,1); 
    A{i} = x1k;
    x2k = currentCycle(:,2);
    uk = currentCycle(:,3);
    cycleduration(i)=length(x1k)-1;
    currentMPCfolder = strcat(folderpathMPC," ",string(i));
    mpcfiles = dir(strcat(currentMPCfolder,'/*.csv'));
    x1k1 = zeros(length(mpcfiles),1);
    
    for j = 1:length(x1k1)
        mpcpath=strcat(currentMPCfolder,"/",mpcfiles(j).name);
        currPred = csvread(mpcpath,1,0);
        x1k1(j) = currPred(2,2);
        
        if j>=starts(i)  
            k=k+1;
            errork(k) = (x1k1(j-1)-x1k(j));
            hrk(k) = x1k(j);
            pred(k) = x1k1(j);
            hpk(k) = x2k(j);
            alk(k) = uk(j);
        end
    end
    figure
    plot((0:length(x1k)-1),x1k,'-o');
    hold on;
    plot((starts(i)-1:length(x1k1)),x1k1(starts(i)-1:length(x1k1)),'o');
    ylim([0 150]);
    xlim([0 length(x1k)-1]);
    plot([0 length(x1k)-1],[140 140],'-r');
    xlabel('Time (mins)');
    ylabel('Heart Rate (bpm)');
    legend('Recorded','Predicted','Reference','Location','southeast');
    title(strcat('Test'," ",string(i),', HR_{Ref} = 140 bpm'));
    grid on;
    %saveas(gcf,strcat(savepath," ",string(i),'/Predicted v Recorded HR Plot.png'));
    error(i) = sum(abs(x1k(starts(i):length(x1k))-x1k1(starts(i)-1:length(x1k1))))/length(x1k);
    
end
%close all
errorpct = 100*(errork./hrk);
l2 = find(abs(errorpct)<=2);
e2 = errorpct(l2);
hr2 = hrk(l2);
hp2 = hpk(l2);
al2 = alk(l2);
l5 = find(abs(errorpct)<=5 & abs(errorpct)>2);
e5 = errorpct(l5);
hr5 = hrk(l5);
hp5 = hpk(l5);
al5 = alk(l5);
l10 = find(abs(errorpct)<=10 & abs(errorpct)>5);
e10 = errorpct(l10);
hr10 = hrk(l10);
hp10 = hpk(l10);
al10 = alk(l10);
l20 = find(abs(errorpct)>10);
e20 = errorpct(l20);
hr20 = hrk(l20);
hp20 = hpk(l20);
al20 = alk(l20);

ehr100 = errorpct(find(hrk<110));
ehr110 = errorpct(find(hrk>=110 & hrk<120));
ehr120 = errorpct(find(hrk>=120 & hrk<130));
ehr130 = errorpct(find(hrk>=130));
[mean(abs(ehr100)); mean(abs(ehr110)); mean(abs(ehr120)); mean(abs(ehr130))]

%ehr100 = errorpct(find(hrk<105));
%ehr105 = errorpct(find(hrk>=105 & hrk<110));
%ehr110 = errorpct(find(hrk>=110));
%[mean(abs(ehr100)); mean(abs(ehr105)); mean(abs(ehr110))]


figure
sz=10;
plot(l2,e2,'.g','MarkerSize',sz);
hold on
plot(l5,e5,'.y','MarkerSize',sz);
gca.ColorOrderIndex = 2;
plot(l10,e10,'.','MarkerSize',sz);
plot(l20,e20,'.r','MarkerSize',sz);
ylim([-20 20]);
xlabel('Sample Number');
ylabel('Error (%)');
title('Error of Each Recorded Sample');

figure
sz=10;
plot(hr2,e2,'.g','MarkerSize',sz);
hold on
plot(hr5,e5,'.y','MarkerSize',sz);
gca.ColorOrderIndex = 2;
plot(hr10,e10,'.','MarkerSize',sz);
plot(hr20,e20,'.r','MarkerSize',sz);
ylim([-20 20]);
xlabel('Heart Rate (bpm)');
ylabel('Error (%)');
title('Error at Different Heart Rates');

figure
sz=10;
plot(hp2,e2,'.g','MarkerSize',sz);
hold on
plot(hp5,e5,'.y','MarkerSize',sz);
gca.ColorOrderIndex = 2;
plot(hp10,e10,'.','MarkerSize',sz);
plot(hp20,e20,'.r','MarkerSize',sz);
ylim([-20 20]);
xlabel('Human Power (W)');
ylabel('Error of Subsequenct Heart Rate (%)');
title('Error following Human Power');

figure
sz=10;
plot(al2,e2,'.g','MarkerSize',sz);
hold on
plot(al5,e5,'.y','MarkerSize',sz);
gca.ColorOrderIndex = 2;
plot(al10,e10,'.','MarkerSize',sz);
plot(al20,e20,'.r','MarkerSize',sz);
ylim([-20 20]);
xlabel('Assistance Level');
ylabel('Error of Subsequenct Heart Rate (%)');
title('Error following Assistance Level');

figure
for i=1:N
    plot([0:length(A{i})-1],A{i},'-');
    hold on
    xlabel('Time (mins)');
    ylabel('Heart Rate (bpm)')
    plot([0 15],[140 140],'.-r');
    ylim([0 150]);
    xlim([0 15]);
    title('Tests with HR_{Ref} = 140 bpm');
end


