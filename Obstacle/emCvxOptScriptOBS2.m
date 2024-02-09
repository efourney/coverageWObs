cvx_begin
    variable x(21,2)
    minimize(norm(x(2,:)-x(1,:))+norm(x(3,:)-x(2,:))+norm(x(4,:)-x(3,:))+norm(x(5,:)-x(4,:))+norm(x(6,:)-x(5,:))+norm(x(7,:)-x(6,:))+norm(x(8,:)-x(7,:))+norm(x(9,:)-x(8,:))+norm(x(10,:)-x(9,:))+norm(x(11,:)-x(10,:))+norm(x(12,:)-x(11,:))+norm(x(13,:)-x(12,:))+norm(x(14,:)-x(13,:))+norm(x(15,:)-x(14,:))+norm(x(16,:)-x(15,:))+norm(x(17,:)-x(16,:))+norm(x(18,:)-x(17,:))+norm(x(19,:)-x(18,:))+norm(x(20,:)-x(19,:))+norm(x(21,:)-x(20,:)))
    subject to
        masterCel{1,1}*x(1,:)' <= masterCel{1,2} 
        masterCel{2,1}*x(2,:)' <= masterCel{2,2} 
        masterCel{3,1}*x(3,:)' <= masterCel{3,2} 
        masterCel{4,1}*x(4,:)' <= masterCel{4,2} 
        masterCel{5,1}*x(5,:)' <= masterCel{5,2} 
        masterCel{6,1}*x(6,:)' <= masterCel{6,2} 
        masterCel{7,1}*x(7,:)' <= masterCel{7,2} 
        masterCel{8,1}*x(8,:)' <= masterCel{8,2} 
        masterCel{9,1}*x(9,:)' <= masterCel{9,2} 
        masterCel{10,1}*x(10,:)' <= masterCel{10,2} 
        masterCel{11,1}*x(11,:)' <= masterCel{11,2} 
        masterCel{12,1}*x(12,:)' <= masterCel{12,2} 
        masterCel{13,1}*x(13,:)' <= masterCel{13,2} 
        masterCel{14,1}*x(14,:)' <= masterCel{14,2} 
        masterCel{15,1}*x(15,:)' <= masterCel{15,2} 
        masterCel{16,1}*x(16,:)' <= masterCel{16,2} 
        masterCel{17,1}*x(17,:)' <= masterCel{17,2} 
        masterCel{18,1}*x(18,:)' <= masterCel{18,2} 
        masterCel{19,1}*x(19,:)' <= masterCel{19,2} 
        masterCel{20,1}*x(20,:)' <= masterCel{20,2} 
cvx_end