

function [fout, vout, cout] = stlmat(filename)


fid=fopen(filename,'r'); %打开文件，需要是STL ASCII格式。
if fid == -1 
    error('无法打开文件，请检查名称或路径')
end

  
vnum=0;       %顶点数计数器
report_num=0; %报告正在进行的状态
VColor = 0;
%
while feof(fid) == 0                    %测试文件的结束
    tline = fgetl(fid);                 %从文件中读取一行数据。
    fword = sscanf(tline, '%s ');       %将该行设置为字符串

    if strncmpi(fword, 'c',1) == 1    % 检查是否有“C”色行，因为“C”是第1个字符。
       VColor = sscanf(tline, '%*s %f %f %f'); % 如果是C，则获取表面的RGB颜色数据。
    end                                %保持这个颜色，直到使用下一个颜色。
    if strncmpi(fword, 'v',1) == 1    % 检查一个“V”行，因为“V”是第1个字符
       vnum = vnum + 1;                %如果一个V，我们计算V的个数
       report_num = report_num + 1;    % 报告计数器，以便长文件显示状态
       if report_num > 249
           fprintf('Reading vertix num: %d.\n',vnum);
           report_num = 0;
       end
       v(:,vnum) = sscanf(tline, '%*s %f %f %f'); % 得到它的XYZ数据。
       c(:,vnum) = VColor;              % 为每个顶点指定一种颜色，它将为面着色。
    end                                 % 我们“*s”跳过名称“color”并获取数据。                                        
end
%   构建面列表;顶点是有序的，所以给它们编号。
%
fnum = vnum/3;      %面数，vnum是顶点数，STL是三角形
flist = 1:vnum;     %面列表的顶点，都按顺序排列
F = reshape(flist, 3,fnum); %用“3×fnum”矩阵表示表面列表数据。
%
%   返回面和顶点
%
fout = F';  %将数组定向到patch中直接使用
vout = v';  
cout = c';
%
fclose(fid);