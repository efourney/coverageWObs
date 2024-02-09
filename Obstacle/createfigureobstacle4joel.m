function createfigure(polyshape1, polyshape2, X1, XData1, YData1, XData2, YData2, X2, XData3, YData3, XData4, YData4, X3, YMatrix1, XData5, YData5, XData6, YData6, X4, Y1, XData7, YData7, XData8, YData8, XData9, YData9, XData10, YData10, X5, XData11, YData11, XData12, YData12, X6, XData13, YData13, XData14, YData14, X7, YMatrix2, XData15, YData15, XData16, YData16, XData17, YData17, XData18, YData18, X8, XData19, YData19, XData20, YData20, X9, Y2, X10, YMatrix3, X11, Y3, X12, X13, Y4, X14, Y5, X15, Y6, X16, X17, X18, Y7, X19, Y8, X20, X21, X22, X23, Y9, X24, X25, YMatrix4, X26, Y10, X27, YMatrix5, X28, Y11, X29, Y12, X30, Y13, X31, Y14, X32, YMatrix6, X33, Y15, X34, Y16, X35, YMatrix7, X36, Y17, X37, Y18, X38, Y19, X39, Y20, X40, Y21, X41, Y22, X42, Y23, X43, Y24, X44, X45, YMatrix8, X46, Y25, X47, Y26, X48, Y27, X49, Y28, X50, Y29, X51, Y30, X52, Y31, X53, Y32, X54, Y33, X55, Y34, X56, Y35, X57, Y36, X58, Y37, X59, Y38)
%CREATEFIGURE(polyshape1, polyshape2, X1, XData1, YData1, XData2, YData2, X2, XData3, YData3, XData4, YData4, X3, YMatrix1, XData5, YData5, XData6, YData6, X4, Y1, XData7, YData7, XData8, YData8, XData9, YData9, XData10, YData10, X5, XData11, YData11, XData12, YData12, X6, XData13, YData13, XData14, YData14, X7, YMatrix2, XData15, YData15, XData16, YData16, XData17, YData17, XData18, YData18, X8, XData19, YData19, XData20, YData20, X9, Y2, X10, YMatrix3, X11, Y3, X12, X13, Y4, X14, Y5, X15, Y6, X16, X17, X18, Y7, X19, Y8, X20, X21, X22, X23, Y9, X24, X25, YMatrix4, X26, Y10, X27, YMatrix5, X28, Y11, X29, Y12, X30, Y13, X31, Y14, X32, YMatrix6, X33, Y15, X34, Y16, X35, YMatrix7, X36, Y17, X37, Y18, X38, Y19, X39, Y20, X40, Y21, X41, Y22, X42, Y23, X43, Y24, X44, X45, YMatrix8, X46, Y25, X47, Y26, X48, Y27, X49, Y28, X50, Y29, X51, Y30, X52, Y31, X53, Y32, X54, Y33, X55, Y34, X56, Y35, X57, Y36, X58, Y37, X59, Y38)
%  POLYSHAPE1:  polyshape object
%  POLYSHAPE2:  polyshape object
%  X1:  vector of x data
%  XDATA1:  line xdata
%  YDATA1:  line ydata
%  XDATA2:  line xdata
%  YDATA2:  line ydata
%  X2:  vector of x data
%  XDATA3:  line xdata
%  YDATA3:  line ydata
%  XDATA4:  line xdata
%  YDATA4:  line ydata
%  X3:  vector of x data
%  YMATRIX1:  matrix of y data
%  XDATA5:  line xdata
%  YDATA5:  line ydata
%  XDATA6:  line xdata
%  YDATA6:  line ydata
%  X4:  vector of x data
%  Y1:  vector of y data
%  XDATA7:  line xdata
%  YDATA7:  line ydata
%  XDATA8:  line xdata
%  YDATA8:  line ydata
%  XDATA9:  line xdata
%  YDATA9:  line ydata
%  XDATA10:  line xdata
%  YDATA10:  line ydata
%  X5:  vector of x data
%  XDATA11:  line xdata
%  YDATA11:  line ydata
%  XDATA12:  line xdata
%  YDATA12:  line ydata
%  X6:  vector of x data
%  XDATA13:  line xdata
%  YDATA13:  line ydata
%  XDATA14:  line xdata
%  YDATA14:  line ydata
%  X7:  vector of x data
%  YMATRIX2:  matrix of y data
%  XDATA15:  line xdata
%  YDATA15:  line ydata
%  XDATA16:  line xdata
%  YDATA16:  line ydata
%  XDATA17:  line xdata
%  YDATA17:  line ydata
%  XDATA18:  line xdata
%  YDATA18:  line ydata
%  X8:  vector of x data
%  XDATA19:  line xdata
%  YDATA19:  line ydata
%  XDATA20:  line xdata
%  YDATA20:  line ydata
%  X9:  vector of x data
%  Y2:  vector of y data
%  X10:  vector of x data
%  YMATRIX3:  matrix of y data
%  X11:  vector of x data
%  Y3:  vector of y data
%  X12:  vector of x data
%  X13:  vector of x data
%  Y4:  vector of y data
%  X14:  vector of x data
%  Y5:  vector of y data
%  X15:  vector of x data
%  Y6:  vector of y data
%  X16:  vector of x data
%  X17:  vector of x data
%  X18:  vector of x data
%  Y7:  vector of y data
%  X19:  vector of x data
%  Y8:  vector of y data
%  X20:  vector of x data
%  X21:  vector of x data
%  X22:  vector of x data
%  X23:  vector of x data
%  Y9:  vector of y data
%  X24:  vector of x data
%  X25:  vector of x data
%  YMATRIX4:  matrix of y data
%  X26:  vector of x data
%  Y10:  vector of y data
%  X27:  vector of x data
%  YMATRIX5:  matrix of y data
%  X28:  vector of x data
%  Y11:  vector of y data
%  X29:  vector of x data
%  Y12:  vector of y data
%  X30:  vector of x data
%  Y13:  vector of y data
%  X31:  vector of x data
%  Y14:  vector of y data
%  X32:  vector of x data
%  YMATRIX6:  matrix of y data
%  X33:  vector of x data
%  Y15:  vector of y data
%  X34:  vector of x data
%  Y16:  vector of y data
%  X35:  vector of x data
%  YMATRIX7:  matrix of y data
%  X36:  vector of x data
%  Y17:  vector of y data
%  X37:  vector of x data
%  Y18:  vector of y data
%  X38:  vector of x data
%  Y19:  vector of y data
%  X39:  vector of x data
%  Y20:  vector of y data
%  X40:  vector of x data
%  Y21:  vector of y data
%  X41:  vector of x data
%  Y22:  vector of y data
%  X42:  vector of x data
%  Y23:  vector of y data
%  X43:  vector of x data
%  Y24:  vector of y data
%  X44:  vector of x data
%  X45:  vector of x data
%  YMATRIX8:  matrix of y data
%  X46:  vector of x data
%  Y25:  vector of y data
%  X47:  vector of x data
%  Y26:  vector of y data
%  X48:  vector of x data
%  Y27:  vector of y data
%  X49:  vector of x data
%  Y28:  vector of y data
%  X50:  vector of x data
%  Y29:  vector of y data
%  X51:  vector of x data
%  Y30:  vector of y data
%  X52:  vector of x data
%  Y31:  vector of y data
%  X53:  vector of x data
%  Y32:  vector of y data
%  X54:  vector of x data
%  Y33:  vector of y data
%  X55:  vector of x data
%  Y34:  vector of y data
%  X56:  vector of x data
%  Y35:  vector of y data
%  X57:  vector of x data
%  Y36:  vector of y data
%  X58:  vector of x data
%  Y37:  vector of y data
%  X59:  vector of x data
%  Y38:  vector of y data

%  Auto-generated by MATLAB on 20-Mar-2023 14:05:21

% Create figure
figure1 = figure('InvertHardcopy','off','Color',[1 1 1]);

% Create axes
axes1 = axes('Parent',figure1);
axis off
hold(axes1,'on');

% Create plot
plot(polyshape1,'Parent',axes1,'LineWidth',2,'FaceColor',[0 0.447 0.741],...
    'FaceAlpha',0.35);

% Create plot
plot(polyshape2,'Parent',axes1,'LineWidth',2,'FaceColor',[0.85 0.325 0.098],...
    'FaceAlpha',0.35);

% Create plot
plot(X1,X1,'MarkerSize',20,'Marker','.','LineWidth',2,'LineStyle','none',...
    'Color',[1 0 0]);

% Create text
text('Parent',axes1,'LineWidth',2,'FontWeight','bold','FontSize',14,...
    'String','1',...
    'Position',[-0.95727923627685 -1.13394495412844 0],...
    'Visible','on');

% Create hggroup
hggroup1 = hggroup('Parent',axes1);

% Create line
line(XData1,YData1,'Parent',hggroup1,'LineWidth',2,'Color',[1 1 1]);

% Create line
line(XData2,YData2,'Parent',hggroup1,'LineWidth',2,'LineStyle','--',...
    'Color',[1 0 0]);

% Create plot
plot(X2,X1,'MarkerSize',20,'Marker','.','LineWidth',2,'LineStyle','none',...
    'Color',[1 0 0]);

% Create text
text('Parent',axes1,'LineWidth',2,'FontWeight','bold','FontSize',14,...
    'String','2',...
    'Position',[1.11909307875895 -1.09266055045872 0],...
    'Visible','on');

% Create hggroup
hggroup2 = hggroup('Parent',axes1);

% Create line
line(XData3,YData3,'Parent',hggroup2,'LineWidth',2,'Color',[1 1 1]);

% Create line
line(XData4,YData4,'Parent',hggroup2,'LineWidth',2,'LineStyle','--',...
    'Color',[1 0 0]);

% Create multiple lines using matrix input to plot
plot(X3,YMatrix1,'MarkerSize',20,'Marker','.','LineWidth',2,...
    'LineStyle','none',...
    'Color',[1 0 0]);

% Create text
text('Parent',axes1,'LineWidth',2,'FontWeight','bold','FontSize',14,...
    'String','3',...
    'Position',[3.92816229116945 -0.147706422018348 0],...
    'Visible','on');

% Create hggroup
hggroup3 = hggroup('Parent',axes1);

% Create line
line(XData5,YData5,'Parent',hggroup3,'LineWidth',2,'Color',[1 1 1]);

% Create line
line(XData6,YData6,'Parent',hggroup3,'LineWidth',2,'LineStyle','--',...
    'Color',[1 0 0]);

% Create plot
plot(X4,Y1,'MarkerSize',20,'Marker','.','LineWidth',2,'LineStyle','none',...
    'Color',[1 0 0]);

% Create text
text('Parent',axes1,'LineWidth',2,'FontWeight','bold','FontSize',14,...
    'String','4',...
    'Position',[5.23365155131265 2.18256880733945 0],...
    'Visible','on');

% Create hggroup
hggroup4 = hggroup('Parent',axes1);

% Create line
line(XData7,YData7,'Parent',hggroup4,'LineWidth',2,'Color',[1 1 1]);

% Create line
line(XData8,YData8,'Parent',hggroup4,'LineWidth',2,'LineStyle','--',...
    'Color',[1 0 0]);

% Create text
text('Parent',axes1,'LineWidth',2,'FontWeight','bold','FontSize',14,...
    'String','5',...
    'Position',[3.7563245823389 3.64128440366973 0],...
    'Visible','on');

% Create hggroup
hggroup5 = hggroup('Parent',axes1);

% Create line
line(XData9,YData9,'Parent',hggroup5,'LineWidth',2,'Color',[1 1 1]);

% Create line
line(XData10,YData10,'Parent',hggroup5,'LineWidth',2,'LineStyle','--',...
    'Color',[1 0 0]);

% Create plot
plot(X5,X4,'MarkerSize',20,'Marker','.','LineWidth',2,'LineStyle','none',...
    'Color',[1 0 0]);

% Create text
text('Parent',axes1,'LineWidth',2,'FontWeight','bold','FontSize',14,...
    'String','6',...
    'Position',[-0.205489260143197 5.1 0],...
    'Visible','on');

% Create hggroup
hggroup6 = hggroup('Parent',axes1);

% Create line
line(XData11,YData11,'Parent',hggroup6,'LineWidth',2,'Color',[1 1 1]);

% Create line
line(XData12,YData12,'Parent',hggroup6,'LineWidth',2,'LineStyle','--',...
    'Color',[1 0 0]);

% Create plot
plot(X6,X6,'MarkerSize',20,'Marker','.','LineWidth',2,'LineStyle','none',...
    'Color',[1 0 0]);

% Create text
text('Parent',axes1,'LineWidth',2,'FontWeight','bold','FontSize',14,...
    'String','7',...
    'Position',[6.1 6.1 0],...
    'Visible','on');

% Create hggroup
hggroup7 = hggroup('Parent',axes1);

% Create line
line(XData13,YData13,'Parent',hggroup7,'LineWidth',2,'Color',[1 1 1]);

% Create line
line(XData14,YData14,'Parent',hggroup7,'LineWidth',2,'LineStyle','--',...
    'Color',[1 0 0]);

% Create multiple lines using matrix input to plot
plot(X7,YMatrix2,'MarkerSize',20,'Marker','.','LineWidth',2,...
    'LineStyle','none',...
    'Color',[1 0 0]);

% Create text
text('Parent',axes1,'LineWidth',2,'FontWeight','bold','FontSize',14,...
    'String','8',...
    'Position',[8.1 3.1 0],...
    'Visible','on');

% Create hggroup
hggroup8 = hggroup('Parent',axes1);

% Create line
line(XData15,YData15,'Parent',hggroup8,'LineWidth',2,'Color',[1 1 1]);

% Create line
line(XData16,YData16,'Parent',hggroup8,'LineWidth',2,'LineStyle','--',...
    'Color',[1 0 0]);

% Create text
text('Parent',axes1,'LineWidth',2,'FontWeight','bold','FontSize',14,...
    'String','9',...
    'Position',[8.1 5.1 0],...
    'Visible','on');

% Create hggroup
hggroup9 = hggroup('Parent',axes1);

% Create line
line(XData17,YData17,'Parent',hggroup9,'LineWidth',2,'Color',[1 1 1]);

% Create line
line(XData18,YData18,'Parent',hggroup9,'LineWidth',2,'LineStyle','--',...
    'Color',[1 0 0]);

% Create plot
plot(X8,X5,'MarkerSize',20,'Marker','.','LineWidth',2,'LineStyle','none',...
    'Color',[1 0 0]);

% Create text
text('Parent',axes1,'LineWidth',2,'FontWeight','bold','FontSize',14,...
    'String','10',...
    'Position',[7.1 0.1 0],...
    'Visible','on');

% Create hggroup
hggroup10 = hggroup('Parent',axes1);

% Create line
line(XData19,YData19,'Parent',hggroup10,'LineWidth',2,'Color',[1 1 1]);

% Create line
line(XData20,YData20,'Parent',hggroup10,'LineWidth',2,'LineStyle','--',...
    'Color',[1 0 0]);

% Create plot
plot(X9,Y2,'LineWidth',2,'Color',[0 0 0]);

% Create multiple lines using matrix input to plot
plot(X10,YMatrix3,'LineWidth',2,'Color',[0 0 0]);

% Create plot
plot(X11,Y3,'LineWidth',2,'Color',[0 0 0]);

% Create plot
plot(X12,X11,'LineWidth',2,'Color',[0 0 0]);

% Create plot
plot(X13,Y4,'LineWidth',2,'Color',[0 0 0]);

% Create plot
plot(X14,Y5,'LineWidth',2,'Color',[0 0 0]);

% Create plot
plot(X15,Y6,'LineWidth',2,'Color',[0 0 0]);

% Create multiple lines using matrix input to plot
plot(X16,YMatrix3,'LineWidth',2,'Color',[0 0 0]);

% Create plot
plot(X17,Y3,'LineWidth',2,'Color',[0 0 0]);

% Create plot
plot(X18,Y7,'LineWidth',2,'Color',[0 0 0]);

% Create plot
plot(X19,Y8,'LineWidth',2,'Color',[0 0 0]);

% Create plot
plot(X20,Y5,'LineWidth',2,'Color',[0 0 0]);

% Create plot
plot(X21,Y6,'LineWidth',2,'Color',[0 0 0]);

% Create plot
plot(X22,X12,'LineWidth',2,'Color',[0 0 0]);

% Create plot
plot(X23,Y9,'LineWidth',2,'Color',[0 0 0]);

% Create plot
plot(X24,YMatrix1,'LineWidth',2,'Color',[0 0 0]);

% Create multiple lines using matrix input to plot
plot(X25,YMatrix4,'LineWidth',2,'Color',[0 0 0]);

% Create plot
plot(X26,Y10,'LineWidth',2,'Color',[0 0 0]);

% Create multiple lines using matrix input to plot
plot(X27,YMatrix5,'LineWidth',2,'Color',[0 0 0]);

% Create plot
plot(X28,Y11,'LineWidth',2,'Color',[0 0 0]);

% Create plot
plot(X29,Y12,'LineWidth',2,'Color',[0 0 0]);

% Create plot
plot(X30,Y13,'LineWidth',2,'Color',[0 0 0]);

% Create plot
plot(X31,Y14,'LineWidth',2,'Color',[0 0 0]);

% Create multiple lines using matrix input to plot
plot(X32,YMatrix6,'LineWidth',2,'Color',[0 0 0]);

% Create plot
plot(X33,Y15,'LineWidth',2,'Color',[0 0 0]);

% Create plot
plot(X34,Y16,'LineWidth',2,'Color',[0 0 0]);

% Create multiple lines using matrix input to plot
plot(X35,YMatrix7,'LineWidth',2,'Color',[0 0 0]);

% Create plot
plot(X36,Y17,'LineWidth',2,'Color',[0 0 0]);

% Create plot
plot(X37,Y18,'LineWidth',2,'Color',[0 0 0]);

% Create plot
plot(X38,Y19,'LineWidth',2,'Color',[0 0 0]);

% Create plot
plot(X39,Y20,'LineWidth',2,'Color',[0 0 0]);

% Create plot
plot(X40,Y21,'LineWidth',2,'Color',[0 0 0]);

% Create plot
plot(X41,Y22,'LineWidth',2,'Color',[0 0 0]);

% Create plot
plot(X42,Y23,'LineWidth',2,'Color',[0 0 0]);

% Create plot
plot(X43,Y24,'LineWidth',2,'Color',[0 0 0]);

% Create plot
plot(X44,YMatrix2,'LineWidth',2,'Color',[0 0 0]);

% Create multiple lines using matrix input to plot
plot(X45,YMatrix8,'LineWidth',2,'Color',[0 0 0]);

% Create plot
plot(X46,Y25,'LineWidth',6,'Color',[1 0 0]);

% Create plot
plot(X47,Y26,'LineWidth',6,'Color',[1 0 0]);

% Create plot
plot(X48,Y27,'LineWidth',6,'Color',[1 0 0]);

% Create plot
plot(X49,Y28,'LineWidth',6,'Color',[1 0 0]);

% Create plot
plot(X50,Y29,'LineWidth',6,'Color',[1 0 0]);

% Create plot
plot(X51,Y30,'LineWidth',6,'Color',[1 0 0]);

% Create plot
plot(X52,Y31,'LineWidth',6,'Color',[1 0 0]);

% Create plot
plot(X53,Y32,'LineWidth',6,'Color',[1 0 0]);

% Create plot
plot(X54,Y33,'LineWidth',6,'Color',[1 0 0]);

% Create plot
plot(X55,Y34,'LineWidth',6,'Color',[1 0 0]);

% Create plot
plot(X56,Y35,'LineWidth',6,'Color',[1 0 0]);

% Create plot
plot(X57,Y36,'LineWidth',6,'Color',[1 0 0]);

% Create plot
plot(X58,Y37,'LineWidth',6,'Color',[1 0 0]);

% Create plot
plot(X59,Y38,'LineWidth',6,'Color',[1 0 0]);

% Uncomment the following line to preserve the X-limits of the axes
% xlim(axes1,[-2.09457861847661 9.90542138152339]);
% Uncomment the following line to preserve the Y-limits of the axes
% ylim(axes1,[-1.92348998994035 7.07651001005965]);
% Uncomment the following line to preserve the Z-limits of the axes
% zlim(axes1,[-1 1]);
hold(axes1,'off');
% Set the remaining axes properties
set(axes1,'FontSize',14,'FontWeight','bold','LineWidth',2);
