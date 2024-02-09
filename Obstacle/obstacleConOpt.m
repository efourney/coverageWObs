function x = obstacleConOpt(ACELL,BCELL,targView,targ2view,TargetNodes,r)
%reformulate into a cell, nx3
%column1 is A, collumn2 is B, column 3 is vector for x(i)-vec <=r 
masterCel = {};
bigM = [];
bigMid = {};
for i = 1:length(ACELL)
    curCel = ACELL{i};
    tempc = cell(length(curCel),3);
    curTarg = targ2view(i);
    if ~isnan(curTarg)
        listOfOpt = targView{i};
        %NO BIG m YET
        for k = 1:length(listOfOpt)
        tempc{listOfOpt(k),3} = TargetNodes(curTarg,:);
        end
        if length(listOfOpt)>1
        bigM = [bigM,listOfOpt+length(masterCel(:,1))];
        bigMid{end+1} = listOfOpt+length(masterCel(:,1));
        end
    end
    tempc(:,1) = curCel;
    tempc(:,2) = BCELL{i};
    masterCel = [masterCel;tempc];
end

%now we just need to write an optimization problem using masterCel
r = num2str(r);
%fileID = fopen('emCvxOptScript.m','w');
fileID = fopen('/Users/emilyfourney/Desktop/coverageobstacles/Obstacle/Obstacle/emCvxOptScriptOBS.m','w');
fprintf(fileID,'cvx_begin');
fprintf(fileID,'\n');
hig = num2str(length(masterCel(:,1))+length(bigM));
fprintf(fileID,['    variable x(' hig ',2)']);
fprintf(fileID,'\n');
fprintf(fileID,['    minimize(']);
for i = 1:length(masterCel(:,1))-1
    fprintf(fileID,['norm(x(' num2str(i+1) ',:)-x(' num2str(i) ',:))']);
    if i~=length(masterCel(:,1))-1
        fprintf(fileID,'+');
    end
end
fprintf(fileID,[')\n']);
fprintf(fileID,['    subject to']);
fprintf(fileID,'\n');
% fprintf(fileID,['        x(1,1) == v(1,1)']);
% fprintf(fileID,'\n');
% fprintf(fileID,['        x(1,2) == v(1,2)']);
% fprintf(fileID,'\n');
for i = 1:length(masterCel(:,1))
    if ~any(i == bigM)
    fprintf(fileID,['        masterCel{' num2str(i) ',1}*x(' num2str(i) ',:)'' <= masterCel{' num2str(i) ',2} \n']);
    if ~isempty(masterCel{i,3})
        fprintf(fileID,['        norm(x(' num2str(i) ',:)-masterCel{' num2str(i) ',3}) <= ' r '\n']);
    end
    else %this is a big M situation
    ind4e = find((i == bigM)) + length(masterCel(:,1));
    fprintf(fileID,['        masterCel{' num2str(i) ',1}*x(' num2str(i) ',:)'' <= masterCel{' num2str(i) ',2} + 1*x(' num2str(ind4e) ',1) \n']);
    fprintf(fileID,['        0 <= x(' num2str(ind4e) ',1) <= 1 \n']);
    if ~isempty(masterCel{i,3})
        fprintf(fileID,['        norm(x(' num2str(i) ',:)-masterCel{' num2str(i) ',3}) <= ' r '+ 1*x(' num2str(ind4e) ',1) \n']);
    end
    end
end
%only true if doing big m
%fprintf(fileID,['        norm(x(' num2str(i+1) ',:)-masterCel{' num2str(i+1) ',3}) <= ' r '\n'])


count = 0;
for j = 1:length(bigMid)
    fprintf(fileID,['        '])
    curGru = bigMid{j};
    for i = 1:length(curGru)
        count = count+1;
        fprintf(fileID,['x(' num2str(count+length(masterCel(:,1))) ',1)']);
        if i~=length(curGru)
            fprintf(fileID,'+');
        end
    end
    fprintf(fileID,['<=' num2str(i-1) '\n']);
end


fprintf(fileID,'cvx_end');
fclose(fileID);

% tic
emCvxOptScriptOBS
% 
% %now we pic the index for the tight big M
% ind1 = length(masterCel(:,1))+1;
% bigMtight = [];
% for i = 1:length(bigMid)
%     range = ind1:ind1+length(bigMid{i})-1;
%     [~,indOMin] = min(x(range,1));
%     bigMtight = [bigMtight,bigMid{i}(indOMin)];
%     ind1 = range(end)+1;
% end
% 
% fileID = fopen('/Users/emilyfourney/Desktop/coverageobstacles/Obstacle/Obstacle/emCvxOptScriptOBS2.m','w');
% fprintf(fileID,'cvx_begin');
% fprintf(fileID,'\n');
% hig = num2str(length(masterCel(:,1))+length(bigM));
% fprintf(fileID,['    variable x(' hig ',2)']);
% fprintf(fileID,'\n');
% fprintf(fileID,['    minimize(']);
% for i = 1:length(masterCel(:,1))-1
%     fprintf(fileID,['norm(x(' num2str(i+1) ',:)-x(' num2str(i) ',:))']);
%     if i~=length(masterCel(:,1))-1
%         fprintf(fileID,'+');
%     end
% end
% fprintf(fileID,[')\n']);
% fprintf(fileID,['    subject to']);
% fprintf(fileID,'\n');
% % fprintf(fileID,['        x(1,1) == v(1,1)']);
% % fprintf(fileID,'\n');
% % fprintf(fileID,['        x(1,2) == v(1,2)']);
% % fprintf(fileID,'\n');
% for i = 1:length(masterCel(:,1))-1
%     if ~any(i == bigM)
%     fprintf(fileID,['        masterCel{' num2str(i) ',1}*x(' num2str(i) ',:)'' <= masterCel{' num2str(i) ',2} \n'])
%     if ~isempty(masterCel{i,3})
%         fprintf(fileID,['        norm(x(' num2str(i) ',:)-masterCel{' num2str(i) ',3}) <= ' r '\n'])
%     end
%     else %this is a big M situation
%     if any(i == bigMtight)
%     fprintf(fileID,['        masterCel{' num2str(i) ',1}*x(' num2str(i) ',:)'' <= masterCel{' num2str(i) ',2} \n'])
%     if ~isempty(masterCel{i,3})
%         fprintf(fileID,['        norm(x(' num2str(i) ',:)-masterCel{' num2str(i) ',3}) <= ' r '\n'])
%     end
%     end %if not the variable is free will always be in bounds
%     end
% end
% %only true if doing big m
% %fprintf(fileID,['        norm(x(' num2str(i+1) ',:)-masterCel{' num2str(i+1) ',3}) <= ' r '\n'])
% 
% 
% % count = 0;
% % for j = 1:length(bigMid)
% %     fprintf(fileID,['        '])
% %     curGru = bigMid{j};
% %     for i = 1:length(curGru)
% %         count = count+1;
% %         fprintf(fileID,['x(' num2str(count+length(masterCel(:,1))) ',1)']);
% %         if i~=length(curGru)
% %             fprintf(fileID,'+');
% %         end
% %     end
% %     fprintf(fileID,['<=' num2str(i-1) '\n']);
% % end
% 
% 
% fprintf(fileID,'cvx_end');
% fclose(fileID);
% 
% % tic
% emCvxOptScriptOBS2