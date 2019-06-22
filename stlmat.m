

function [fout, vout, cout] = stlmat(filename)


fid=fopen(filename,'r'); %���ļ�����Ҫ��STL ASCII��ʽ��
if fid == -1 
    error('�޷����ļ����������ƻ�·��')
end

  
vnum=0;       %������������
report_num=0; %�������ڽ��е�״̬
VColor = 0;
%
while feof(fid) == 0                    %�����ļ��Ľ���
    tline = fgetl(fid);                 %���ļ��ж�ȡһ�����ݡ�
    fword = sscanf(tline, '%s ');       %����������Ϊ�ַ���

    if strncmpi(fword, 'c',1) == 1    % ����Ƿ��С�C��ɫ�У���Ϊ��C���ǵ�1���ַ���
       VColor = sscanf(tline, '%*s %f %f %f'); % �����C�����ȡ�����RGB��ɫ���ݡ�
    end                                %���������ɫ��ֱ��ʹ����һ����ɫ��
    if strncmpi(fword, 'v',1) == 1    % ���һ����V���У���Ϊ��V���ǵ�1���ַ�
       vnum = vnum + 1;                %���һ��V�����Ǽ���V�ĸ���
       report_num = report_num + 1;    % ������������Ա㳤�ļ���ʾ״̬
       if report_num > 249
           fprintf('Reading vertix num: %d.\n',vnum);
           report_num = 0;
       end
       v(:,vnum) = sscanf(tline, '%*s %f %f %f'); % �õ�����XYZ���ݡ�
       c(:,vnum) = VColor;              % Ϊÿ������ָ��һ����ɫ������Ϊ����ɫ��
    end                                 % ���ǡ�*s���������ơ�color������ȡ���ݡ�                                        
end
%   �������б�;����������ģ����Ը����Ǳ�š�
%
fnum = vnum/3;      %������vnum�Ƕ�������STL��������
flist = 1:vnum;     %���б�Ķ��㣬����˳������
F = reshape(flist, 3,fnum); %�á�3��fnum�������ʾ�����б����ݡ�
%
%   ������Ͷ���
%
fout = F';  %�����鶨��patch��ֱ��ʹ��
vout = v';  
cout = c';
%
fclose(fid);