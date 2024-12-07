function [] = trace_all(tracelist,activatedmethod,actin,datasetname )
%trace_all Draw the tracelist of all the methods.


figure1 = figure('Name','Figure','Color',[1 1 1]);
colorlist=[141,47,37;78,25,69;203,148,117;140,191,135;62,96,141    ]/256;






for j=1:actin
    subplot(2,actin, j);
    trace_now=tracelist(j).tracelist;
    methodname=activatedmethod(j).methodname;
    len=length(trace_now(1).P);
    step = 1:len-50;


    subplot(2,actin,j+actin    );
    plot(step, trace_now(2).P(1,51:end), "DisplayName", 'Robot 2','Color',[colorlist(2,1),colorlist(2,2),colorlist(2,3)]);hold on
    plot(step, trace_now(3).P(1,51:end), "DisplayName", 'Robot 3','Color',[colorlist(3,1),colorlist(3,2),colorlist(3,3)]);hold on
    plot(step, trace_now(4).P(1,51:end), "DisplayName", 'Robot 4','Color',[colorlist(4,1),colorlist(4,2),colorlist(4,3)]);hold on
    plot(step, trace_now(5).P(1,51:end), "DisplayName", 'Robot 5','Color',[colorlist(5,1),colorlist(5,2),colorlist(5,3)]);hold on
    xlabel('Steps');
    ylabel('Trace of P(1:9,1:9)');


end
for j=1:actin
    subplot(2,actin, j);
    trace_now=tracelist(j).tracelist;
    methodname=activatedmethod(j).methodname;
    len=length(trace_now(1).P);
    step = 1:len-1;
    plot(step, trace_now(1).P(1,2:end), "DisplayName", 'Robot 1','Color',[colorlist(1,1),colorlist(1,2),colorlist(1,3)]);hold on
    plot(step, trace_now(2).P(1,2:end), "DisplayName", 'Robot 2','Color',[colorlist(2,1),colorlist(2,2),colorlist(2,3)]);hold on
    plot(step, trace_now(3).P(1,2:end), "DisplayName", 'Robot 3','Color',[colorlist(3,1),colorlist(3,2),colorlist(3,3)]);hold on
    plot(step, trace_now(4).P(1,2:end), "DisplayName", 'Robot 4','Color',[colorlist(4,1),colorlist(4,2),colorlist(4,3)]);hold on
    plot(step, trace_now(5).P(1,2:end), "DisplayName", 'Robot 5','Color',[colorlist(5,1),colorlist(5,2),colorlist(5,3)]);hold on
    
    xlabel('Steps','FontSize',9);
    ylabel('Trace of P(1:9,1:9)','FontSize',9);
    subtitle(methodname);


end

legend( );


f1=gcf;

folder=['TestResults/',datasetname,'/']; 
name = [datasetname,'trace.pdf'];
savepath1 = [folder,name];
if exist(folder)==0 
    mkdir(folder); 
end
exportgraphics(f1,savepath1)

end