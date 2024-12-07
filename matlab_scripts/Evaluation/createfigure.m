function createfigure(X1, YMatrix1, X2, YMatrix2, YMatrix3, YMatrix4, YMatrix5, YMatrix6, YMatrix7, YMatrix8, YMatrix9)
%CREATEFIGURE(X1, YMatrix1, X2, YMatrix2, YMatrix3, YMatrix4, YMatrix5, YMatrix6, YMatrix7, YMatrix8, YMatrix9)
%  X1:  plot x 数据的向量
%  YMATRIX1:  plot y 数据的矩阵
%  X2:  plot x 数据的向量
%  YMATRIX2:  plot y 数据的矩阵
%  YMATRIX3:  plot y 数据的矩阵
%  YMATRIX4:  plot y 数据的矩阵
%  YMATRIX5:  plot y 数据的矩阵
%  YMATRIX6:  plot y 数据的矩阵
%  YMATRIX7:  plot y 数据的矩阵
%  YMATRIX8:  plot y 数据的矩阵
%  YMATRIX9:  plot y 数据的矩阵

%  由 MATLAB 于 12-Aug-2024 18:07:04 自动生成

% 创建 figure
figure1 = figure('Name','Figure','Color',[1 1 1]);

% 创建 subplot
subplot1 = subplot(2,5,1,'Parent',figure1);
hold(subplot1,'on');

% 使用 plot 的矩阵输入创建多个 line 对象
plot1 = plot(X1,YMatrix1);
set(plot1(1),'DisplayName','Robot 1',...
    'Color',[0.55078125 0.18359375 0.14453125]);
set(plot1(2),'DisplayName','Robot 2',...
    'Color',[0.3046875 0.09765625 0.26953125]);
set(plot1(3),'DisplayName','Robot 3',...
    'Color',[0.79296875 0.578125 0.45703125]);
set(plot1(4),'DisplayName','Robot 4',...
    'Color',[0.546875 0.74609375 0.52734375]);
set(plot1(5),'DisplayName','Robot 5','Color',[0.2421875 0.375 0.55078125]);

% 创建 ylabel
ylabel('Trace of P(1:9,1:9)');

% 创建 xlabel
xlabel('Steps');

box(subplot1,'on');
hold(subplot1,'off');
% 创建 subplot
subplot2 = subplot(2,5,6,'Parent',figure1);
hold(subplot2,'on');

% 使用 plot 的矩阵输入创建多个 line 对象
plot2 = plot(X2,YMatrix2);
set(plot2(1),'DisplayName','Robot 2',...
    'Color',[0.3046875 0.09765625 0.26953125]);
set(plot2(2),'DisplayName','Robot 3',...
    'Color',[0.79296875 0.578125 0.45703125]);
set(plot2(3),'DisplayName','Robot 4',...
    'Color',[0.546875 0.74609375 0.52734375]);
set(plot2(4),'DisplayName','Robot 5','Color',[0.2421875 0.375 0.55078125]);

% 创建 ylabel
ylabel('Trace of P(1:9,1:9)');

% 创建 xlabel
xlabel('Steps');

box(subplot2,'on');
hold(subplot2,'off');
% 创建 subplot
subplot3 = subplot(2,5,2,'Parent',figure1);
hold(subplot3,'on');

% 使用 plot 的矩阵输入创建多个 line 对象
plot3 = plot(X1,YMatrix3);
set(plot3(1),'DisplayName','Robot 1',...
    'Color',[0.55078125 0.18359375 0.14453125]);
set(plot3(2),'DisplayName','Robot 2',...
    'Color',[0.3046875 0.09765625 0.26953125]);
set(plot3(3),'DisplayName','Robot 3',...
    'Color',[0.79296875 0.578125 0.45703125]);
set(plot3(4),'DisplayName','Robot 4',...
    'Color',[0.546875 0.74609375 0.52734375]);
set(plot3(5),'DisplayName','Robot 5','Color',[0.2421875 0.375 0.55078125]);

% 创建 ylabel
ylabel('Trace of P(1:9,1:9)');

% 创建 xlabel
xlabel('Steps');

box(subplot3,'on');
hold(subplot3,'off');
% 创建 subplot
subplot4 = subplot(2,5,7,'Parent',figure1);
hold(subplot4,'on');

% 使用 plot 的矩阵输入创建多个 line 对象
plot4 = plot(X2,YMatrix4);
set(plot4(1),'DisplayName','Robot 2',...
    'Color',[0.3046875 0.09765625 0.26953125]);
set(plot4(2),'DisplayName','Robot 3',...
    'Color',[0.79296875 0.578125 0.45703125]);
set(plot4(3),'DisplayName','Robot 4',...
    'Color',[0.546875 0.74609375 0.52734375]);
set(plot4(4),'DisplayName','Robot 5','Color',[0.2421875 0.375 0.55078125]);

% 创建 ylabel
ylabel('Trace of P(1:9,1:9)');

% 创建 xlabel
xlabel('Steps');

box(subplot4,'on');
hold(subplot4,'off');
% 创建 subplot
subplot5 = subplot(2,5,3,'Parent',figure1);
hold(subplot5,'on');

% 使用 plot 的矩阵输入创建多个 line 对象
plot5 = plot(X1,YMatrix5);
set(plot5(1),'DisplayName','Robot 1',...
    'Color',[0.55078125 0.18359375 0.14453125]);
set(plot5(2),'DisplayName','Robot 2',...
    'Color',[0.3046875 0.09765625 0.26953125]);
set(plot5(3),'DisplayName','Robot 3',...
    'Color',[0.79296875 0.578125 0.45703125]);
set(plot5(4),'DisplayName','Robot 4',...
    'Color',[0.546875 0.74609375 0.52734375]);
set(plot5(5),'DisplayName','Robot 5','Color',[0.2421875 0.375 0.55078125]);

% 创建 ylabel
ylabel('Trace of P(1:9,1:9)');

% 创建 xlabel
xlabel('Steps');

box(subplot5,'on');
hold(subplot5,'off');
% 创建 subplot
subplot6 = subplot(2,5,8,'Parent',figure1);
hold(subplot6,'on');

% 使用 plot 的矩阵输入创建多个 line 对象
plot6 = plot(X2,YMatrix6);
set(plot6(1),'DisplayName','Robot 2',...
    'Color',[0.3046875 0.09765625 0.26953125]);
set(plot6(2),'DisplayName','Robot 3',...
    'Color',[0.79296875 0.578125 0.45703125]);
set(plot6(3),'DisplayName','Robot 4',...
    'Color',[0.546875 0.74609375 0.52734375]);
set(plot6(4),'DisplayName','Robot 5','Color',[0.2421875 0.375 0.55078125]);

% 创建 ylabel
ylabel('Trace of P(1:9,1:9)');

% 创建 xlabel
xlabel('Steps');

box(subplot6,'on');
hold(subplot6,'off');
% 创建 subplot
subplot7 = subplot(2,5,4,'Parent',figure1);
hold(subplot7,'on');

% 使用 plot 的矩阵输入创建多个 line 对象
plot7 = plot(X1,YMatrix7);
set(plot7(1),'DisplayName','Robot 1',...
    'Color',[0.55078125 0.18359375 0.14453125]);
set(plot7(2),'DisplayName','Robot 2',...
    'Color',[0.3046875 0.09765625 0.26953125]);
set(plot7(3),'DisplayName','Robot 3',...
    'Color',[0.79296875 0.578125 0.45703125]);
set(plot7(4),'DisplayName','Robot 4',...
    'Color',[0.546875 0.74609375 0.52734375]);
set(plot7(5),'DisplayName','Robot 5','Color',[0.2421875 0.375 0.55078125]);

% 创建 ylabel
ylabel('Trace of P(1:9,1:9)');

% 创建 xlabel
xlabel('Steps');

box(subplot7,'on');
hold(subplot7,'off');
% 创建 subplot
subplot8 = subplot(2,5,9,'Parent',figure1);
hold(subplot8,'on');

% 使用 plot 的矩阵输入创建多个 line 对象
plot8 = plot(X2,YMatrix8);
set(plot8(1),'DisplayName','Robot 2',...
    'Color',[0.3046875 0.09765625 0.26953125]);
set(plot8(2),'DisplayName','Robot 3',...
    'Color',[0.79296875 0.578125 0.45703125]);
set(plot8(3),'DisplayName','Robot 4',...
    'Color',[0.546875 0.74609375 0.52734375]);
set(plot8(4),'DisplayName','Robot 5','Color',[0.2421875 0.375 0.55078125]);

% 创建 ylabel
ylabel('Trace of P(1:9,1:9)');

% 创建 xlabel
xlabel('Steps');

box(subplot8,'on');
hold(subplot8,'off');
% 创建 subplot
subplot9 = subplot(2,5,5,'Parent',figure1);
hold(subplot9,'on');

% 使用 plot 的矩阵输入创建多个 line 对象
plot9 = plot(X1,YMatrix9);
set(plot9(1),'DisplayName','Robot 1',...
    'Color',[0.55078125 0.18359375 0.14453125]);
set(plot9(2),'DisplayName','Robot 2',...
    'Color',[0.3046875 0.09765625 0.26953125]);
set(plot9(3),'DisplayName','Robot 3',...
    'Color',[0.79296875 0.578125 0.45703125]);
set(plot9(4),'DisplayName','Robot 4',...
    'Color',[0.546875 0.74609375 0.52734375]);
set(plot9(5),'DisplayName','Robot 5','Color',[0.2421875 0.375 0.55078125]);

% 创建 ylabel
ylabel('Trace of P(1:9,1:9)');

% 创建 xlabel
xlabel('Steps');

box(subplot9,'on');
hold(subplot9,'off');
% 创建 legend
legend1 = legend(subplot9,'show');
set(legend1,...
    'Position',[0.803359054212856 0.234664689834437 0.0646327363472577 0.183896624307272]);

% 创建 textbox
annotation(figure1,'textbox',...
    [0.058606124604014 0.276100620984256 0.030095036958817 0.0951704559678381],...
    'String',{'zoom','out'},...
    'FitBoxToText','off',...
    'EdgeColor',[1 1 1]);

