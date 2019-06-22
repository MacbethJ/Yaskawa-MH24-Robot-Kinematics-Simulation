
%%                                               参考文献
%Puma Robot Simulation(https://ww2.mathworks.cn/matlabcentral/fileexchange/59663-puma-robot-simulation)
%RoboticsToobox (http://petercorke.com/wordpress/toolboxes/robotics-toolbox)
%%                                                机械臂GUI                                              
function MH24
% MH24机器人的GUI运动学演示
% 机械臂几何在Matlab文件交换中使用CAD2MATDEMO代码
obj=findobj('Tag','fig_1');
if ~isempty(obj)
%    close(obj);
end
% 为GUI创建一个新图形
set(0,'Units','pixels')
dim = get(0,'ScreenSize');
fig_1 = figure('Position',[0,35,dim(3)-200,dim(4)-110],...
    'Name',' MH24机械臂3D图形演示',...
    'NumberTitle','off','CloseRequestFcn',@del_app,...
    'Tag','fig_1','menubar','none');
hold on;

light;                             % 添加默认灯光
daspect([1 1 1]);                    % 设置纵横比
view(135,25);
xlabel('X','hittest','off'),ylabel('Y','hittest','off'),zlabel('Z','hittest','off');
title('MH24 机械臂运动学仿真');
axis([-2000 2000 -2000 2000 -2000 2000]);
grid on;
box on;

Initialize;

patch([-200 -200 200 200],[200 -200 -200 200],[0 0 0 0],'y',...
    'hittest','off');
patch([-200 -200 -200 -200],[200 200 -200 -200],[0 -200 -200 0],'y',...
    'hittest','off');
patch([-200 -200 200 200],[-200 -200 -200 -200],[0 -200 -200 0],'y',...
    'hittest','off');
patch([200 200 200 200],[200 200 -200 -200],[0 -200 -200 0],'y',...
    'hittest','off');
patch([-200 -200 200 200],[200 200 200 200],[0 -200 -200 0],'y',...
    'hittest','off');
patch([-1000 -1000 1000 1000],[1000 -1000 -1000 1000],...
    [-200 -200 -200 -200],[0,0.5,0.1],'facealpha',0.7,'hittest','off');
%UI框架
C_p = uipanel(fig_1,...
    'units','pixels',...
    'Position',[20 300 180 220],...
    'Title','控制模块','FontSize',11);
%
% Create the push buttons: pos is: [left bottom width height]
%MOV
uicontrol(C_p,'String','轨迹规划','callback',@demo_button_press,...
    'Position',[30 160 110 30],...
    'Interruptible','off','FontSize',10);
%RANDOM MOVE
uicontrol(C_p,'String','随机运动','callback',@rnd_demo_button_press,...
    'Position',[30 60 110 30],...
    'Interruptible','off','FontSize',10);
%CLR TRAIL
uicontrol(C_p,'String','轨迹清除','callback',@clr_trail_button_press,...
    'Position',[30 10 110 30],...
    'Interruptible','off','FontSize',10);
%HOME
uicontrol(C_p,'String','原始位置','callback',@home_button_press,...
    'Position',[30 110 110 30],...
    'Interruptible','off','FontSize',10);
% 运动学面板

K_p = uipanel(fig_1,...
    'units','pixels',...
    'Position',[20 45 300 250],...
    'Title','运动学','FontSize',11);

uicontrol(K_p,'style','popupmenu',...
    'string',{'关节控制'},...
    'value',1,...
    'interruptible','off',...
    'callback',@popup_press,...
    'units','pixels',...
    'position',[70 190 160 30],...
    'FontSize',10);
jointspace_flag=true;
%
%     Angle    Range                Default Name
%     Theta 1: 360 (-180 to 180)    90       Waist Joint
%     Theta 2: 261 (-106 to 155)   -90       Shoulder Joint
%     Theta 3: 410 (-170 to 240)   -90       Elbow Joint
%     Theta 4: 400 (-200 to 200)     0       Wrist Roll
%     Theta 5: 300 (-150 to 150)     0       Wrist Bend
%     Theta 6: 910 (-455 to 455)     0       Wrist Swivel

LD = 105; % 用于设置GUI
HT = 18;  % 高度

%% 6个旋转角度按钮的设置
%%  GUI buttons for Theta 1.  
BT = 156; % 按钮
t1_slider = uicontrol(K_p,'style','slider',...
    'Max',180,'Min',-180,'Value',0,...
    'SliderStep',[0.01 0.2],...
    'callback',@slider_button_press,...
    'Position',[LD+20 BT 120 HT],...
    'Interruptible','off');
%最小值
min1=uicontrol(K_p,'style','text',...
    'String','-180',...
    'Position',[LD-10 BT+1 25 HT-4]); 
%最大值
max1=uicontrol(K_p,'style','text',...
    'String','+180',...
    'Position',[LD+145 BT+1 30 HT-4]); % L, B, W, H
%标签
info1=uicontrol(K_p,'style','text',...  % Nice program Doug. Need this
    'String','θ1',...                % due to no TeX in uicontrols.
    'Position',[LD-100 BT 20 HT]); % L, B, W, H
t1_edit = uicontrol(K_p,'style','edit',...
    'String',0,...
    'callback',@edit_button_press,...
    'Position',[LD-75 BT 50 HT]); % L, B, W, H
%
%%  GUI buttons for Theta 2.
BT = 126;   
t2_slider = uicontrol(K_p,'style','slider',...
    'Max',155,'Min',-106,'Value',0,...        % Mech. stop limits !
    'SliderStep',[0.01 0.2],...
    'callback',@slider_button_press,...
    'Position',[LD+20 BT 120 HT],...
    'Interruptible','off');

min2=uicontrol(K_p,'style','text',...
    'String','-106',...
    'Position',[LD-10 BT+1 25 HT-4]); % L, from bottom, W, H

max2=uicontrol(K_p,'style','text',...
    'String','+155',...
    'Position',[LD+145 BT+1 30 HT-4]); % L, B, W, H

info2=uicontrol(K_p,'style','text',...
    'String','θ2',...
    'Position',[LD-100 BT 20 HT]); % L, B, W, H
t2_edit = uicontrol(K_p,'style','edit',...
    'String',0,...
    'callback',@edit_button_press,...
    'Position',[LD-75 BT 50 HT]); % L, B, W, H
%
%%  GUI buttons for Theta 3.
BT = 96;  
t3_slider = uicontrol(K_p,'style','slider',...
    'Max',240,'Min',-170,'Value',0,...
    'SliderStep',[0.01 0.2],...
    'callback',@slider_button_press,...
    'Position',[LD+20 BT 120 HT],...
    'Interruptible','off');

min3=uicontrol(K_p,'style','text',...
    'String','-170',...
    'Position',[LD-10 BT+1 25 HT-4]); % L, from bottom, W, H

max3=uicontrol(K_p,'style','text',...
    'String','+240',...
    'Position',[LD+145 BT+1 30 HT-4]); % L, B, W, H

info3=uicontrol(K_p,'style','text',...
    'String','θ3',...
    'Position',[LD-100 BT 20 HT]); % L, B, W, H
t3_edit = uicontrol(K_p,'style','edit',...
    'String',0,...
    'callback',@edit_button_press,...
    'Position',[LD-75 BT 50 HT]); % L, B, W, H
%
%%  GUI buttons for Theta 4.
BT = 66;   
t4_slider = uicontrol(K_p,'style','slider',...
    'Max',200,'Min',-200,'Value',0,...
    'SliderStep',[0.01 0.2],...
    'callback',@slider_button_press,...
    'Position',[LD+20 BT 120 HT],...
    'Interruptible','off');

min4=uicontrol(K_p,'style','text',...
    'String','-200',...
    'Position',[LD-10 BT+1 25 HT-4]); % L, from bottom, W, H

max4=uicontrol(K_p,'style','text',...
    'String','+200',...
    'Position',[LD+145 BT+1 30 HT-4]); % L, B, W, H

info4=uicontrol(K_p,'style','text',...
    'String','θ4',...
    'Position',[LD-100 BT 20 HT]); % L, B, W, H
t4_edit = uicontrol(K_p,'style','edit',...
    'String',0,...
    'callback',@edit_button_press,...
    'Position',[LD-75 BT 50 HT]); % L, B, W, H
%
%%  GUI buttons for Theta 5.
BT = 36;  
t5_slider = uicontrol(K_p,'style','slider',...
    'Max',150,'Min',-150,'Value',0,...
    'SliderStep',[0.01 0.2],...
    'callback',@slider_button_press,...
    'Position',[LD+20 BT 120 HT],...
    'Interruptible','off');

min5=uicontrol(K_p,'style','text',...
    'String','-150',...
    'Position',[LD-10 BT+1 25 HT-4]); % L, from bottom, W, H

max5=uicontrol(K_p,'style','text',...
    'String','+150',...
    'Position',[LD+145 BT+1 30 HT-4]); % L, B, W, H

info5=uicontrol(K_p,'style','text',...
    'String','θ5',...
    'Position',[LD-100 BT 20 HT]); % L, B, W, H
t5_edit = uicontrol(K_p,'style','edit',...
    'String',0,...
    'callback',@edit_button_press,...
    'Position',[LD-75 BT 50 HT]); % L, B, W, H
%
%%  GUI buttons for Theta 6.
BT = 6;   
t6_slider = uicontrol(K_p,'style','slider',...
    'Max',455,'Min',-455,'Value',0,...
    'SliderStep',[0.01 0.2],...
    'callback',@slider_button_press,...
    'Position',[LD+20 BT 120 HT],...
    'Interruptible','off');

min6=uicontrol(K_p,'style','text',...
    'String','-455',...
    'Position',[LD-10 BT+1 25 HT-4]); % L, from bottom, W, H

max6=uicontrol(K_p,'style','text',...
    'String','+455',...
    'Position',[LD+145 BT+1 30 HT-4]); % L, B, W, H

info6=uicontrol(K_p,'style','text',...
    'String','θ6',...
    'Position',[LD-100 BT 20 HT]); % L, B, W, H
t6_edit = uicontrol(K_p,'style','edit',...
    'String',0,...
    'callback',@edit_button_press,...
    'Position',[LD-75 BT 50 HT]); % L, B, W, H
rotate3d on;

min_group=[min1,min2,min3,min4,min5,min6];
max_group=[max1,max2,max3,max4,max5,max6];
info_group=[info1,info2,info3,info4,info5,info6];
slider_group=[t1_slider,t2_slider,t3_slider,t4_slider,t5_slider,t6_slider];
edit_group=[t1_edit,t2_edit,t3_edit,t4_edit,t5_edit,t6_edit];

setappdata(fig_1,'min_group',min_group);
setappdata(fig_1,'max_group',max_group);
setappdata(fig_1,'info_group',info_group);
setappdata(fig_1,'slider_group',slider_group);
setappdata(fig_1,'edit_group',edit_group);
setappdata(fig_1,'jointspace_flag',jointspace_flag);
setappdata(fig_1,'resolution',0.5);
end
%主函数初始化
function Initialize

[linkdata]=load('FoxbotLinksData.mat','s1','s2', 's3','s4','s5','s6','s7');
setappdata(gcf,'linkdata',linkdata);

% 原始坐标
setappdata(gcf,'ThetaOld',[0,-pi/2,0,0,0,0]);%initial pose

T_01 = tmat(pi/2, 240, 620, 0);
T_12 = tmat(pi, 820, 0, 0);
T_23 = tmat(-pi/2,120,80, 0);
T_34 = tmat(0,0, 825, 0);
T_45 = tmat(0, 0, 100, pi);
T_56 = tmat(0, 0, 0, 0);


% 坐标变换
T_02 = T_01*T_12;
T_03 = T_02*T_23;
T_04 = T_03*T_34;
T_05 = T_04*T_45;
T_06 = T_05*T_56;

% 机器人连杆的实际顶点数据
Link1 = linkdata.s1.V1;
Link2 = (T_01*linkdata.s2.V2')';
Link3 = (T_02*linkdata.s3.V3')';
Link4 = (T_03*linkdata.s4.V4')';
Link5 = (T_04*linkdata.s5.V5')';
Link6 = (T_05*linkdata.s6.V6')';
Link7 = (T_06*linkdata.s7.V7')';
linkdata.workspace.V = [Link2;Link3;Link4;Link5;Link6;Link7]*[1 0 0 0;0 1 0 0;0 0 0 0;0 0 0 1];
linkdata.workspace.F = [linkdata.s2.F2;linkdata.s3.F3;linkdata.s4.F4;linkdata.s5.F5;linkdata.s6.F6;linkdata.s7.F7];
workspace=([eye(3) [75 0 360]';[0 0 0 1]]*(linkdata.workspace.V)')';


line([0 200],[0 0],[0 0],'color','r','linewidth',3);
line([0 0],[0 200],[0 0],'color','g','linewidth',3);
line([0 0],[0 0],[0 200],'color','b','linewidth',3);

%使点云显示为3d模型。
patch('faces',linkdata.workspace.F,'vertices',workspace(:,1:3),...
    'facec',[0 0 0.5],'edgecolor','none','facealpha',0.07,'hittest','off');
L1 = patch('faces', linkdata.s1.F1, 'vertices' ,Link1(:,1:3),...
    'facec', [0.717,0.116,0.123],'EdgeColor','none','facealpha',0.7,'hittest','off');
L2 = patch('faces', linkdata.s2.F2, 'vertices' ,Link2(:,1:3),...
    'facec', [0.216,1,.583],'EdgeColor','none','facealpha',1,'hittest','off');
L3 = patch('faces', linkdata.s3.F3, 'vertices' ,Link3(:,1:3),...
    'facec', [0.306,0.733,1],'EdgeColor','none','facealpha',1,'hittest','off');
L4 = patch('faces', linkdata.s4.F4, 'vertices' ,Link4(:,1:3),...
    'facec', [1,0.542,0.493],'EdgeColor','none','facealpha',1,'hittest','off');
L5 = patch('faces', linkdata.s5.F5, 'vertices' ,Link5(:,1:3),...
    'facec', [0.216,1,.583],'EdgeColor','none','facealpha',0.5,'hittest','off');
L6 = patch('faces', linkdata.s6.F6, 'vertices' ,Link6(:,1:3),...
    'facec', [1,1,0.255],'EdgeColor','none','facealpha',0.5,'hittest','off');
L7 = patch('faces', linkdata.s7.F7, 'vertices' ,Link7(:,1:3),...
    'facec', [0.306,0.733,1],'EdgeColor','none','facealpha',0.5,'hittest','off');
Tr = plot3(0,0,0,'b.');
% 小径支架
coord(1)=line([0 1],[0 0],[0 0],'color','r','linewidth',2);
coord(2)=line([0 0],[0 1],[0 0],'color','g','linewidth',2);
coord(3)=line([0 0],[0 0],[0 1],'color','b','linewidth',2);
xyz=T_06*[200*eye(3) zeros(3,1);ones(1,4)];
set(coord,{'xdata','ydata','zdata'},...
    {[xyz(1,4) xyz(1,1)],[xyz(2,4) xyz(2,1)],[xyz(3,4) xyz(3,1)];...
    [xyz(1,4) xyz(1,2)],[xyz(2,4) xyz(2,2)],[xyz(3,4) xyz(3,2)];...
    [xyz(1,4) xyz(1,3)],[xyz(2,4) xyz(2,3)],[xyz(3,4) xyz(3,3)]});

setappdata(gcf,'patch_h',[L1,L2,L3,L4,L5,L6,L7,Tr,coord]);

fox.a=[0.075 0.27 0.11 0 0 0];
fox.alpha=[-pi/2 0 -pi/2 pi/2 -pi/2 0];
fox.d=[0.36 0 0 0.306 0 0];
fox.I=cat(3,diag([0, 0.35, 0]),...
    diag([0.13, 0.524, 0.539]),...
    diag([0.066, 0.086, 0.0625]),...
    diag([1.8e-2, 0.8e-2, 1.8e-2]),...
    diag([0.3e-3, 0.4e-3, 0.3e-3]),...
    diag([0.15e-3, 0.15e-3, 0.04e-3]));
fox.r=[0, 0, 0;-0.151, 0.006, -0.102;-0.0203, -0.0141, 0.010;...
   0, -0.09, 0;0, 0, 0.02;0, 0, 0.082];
fox.m=[0 10.4 6.8 4.6 0.34 0.18];
fox.Jm=[200e-6 200e-6 200e-6 30e-6 30e-6 30e-6];
fox.G=[120 120 120 80 80 50];
fox.B=[1.48e-3 .817e-3 1.38e-3 71.2e-6 82.6e-6 36.7e-6];
fox.Tc=[0.41 -0.41;0.11 -0.11;0.17, -0.17;0.06, -0.06;...
    0.05, -0.05;0.03, -0.03];
fox.qlim=[-160 160;-145 25;-115 60;-150 150;-110 110;-360 360]*pi/180;
fox.taulim=[240 240 240 180 80 80];
setappdata(gcf,'fox',fox);
end
% 回调函数
function del_app(varargin)
rmappdata(gcf,'linkdata');
rmappdata(gcf,'ThetaOld');
rmappdata(gcf,'patch_h');
rmappdata(gcf,'min_group');
rmappdata(gcf,'max_group');
rmappdata(gcf,'info_group');
rmappdata(gcf,'slider_group');
rmappdata(gcf,'edit_group');
rmappdata(gcf,'jointspace_flag');
rmappdata(gcf,'resolution');
rmappdata(gcf,'fox');

delete(gcf);
end
%%                                            UI各功能
% 滑块
function slider_button_press(h,~)
slider_group=getappdata(gcf,'slider_group');
axis_num=find(slider_group==h);
T_Old = getappdata(gcf,'ThetaOld');
if getappdata(gcf,'jointspace_flag')
    T_Old(axis_num) = get(h,'Value')-(axis_num==2||axis_num==3)*90;
    foxani(T_Old,getappdata(gcf,'resolution'),false);
else
    T=foxfkine(T_Old);
    if axis_num<4
        p_old=T(axis_num,4);
        p_new=get(h,'Value');
        p=linspace(p_old,p_new,ceil(abs(p_new-p_old))+1);
        T=repmat(T,1,1,length(p));
        T(axis_num,4,:)=p;
    else
        o_old=0;
        o_new=get(h,'Value')/180*pi;
        o=linspace(o_old,o_new,ceil(abs(o_new-o_old)/getappdata(gcf,'resolution'))+1);
        R0=T(1:3,1:3);
        T=repmat(T,1,1,length(o));
        for i=1:length(o)
            R_axis=(axis_num==4)*rotx(o(i))+(axis_num==5)*roty(o(i))+(axis_num==6)*rotz(o(i));
            T(1:3,1:3,i)=R_axis*R0;
        end
    end
    q=foxikine(T);
    figure(h.Parent.Parent);
    for i=1:size(q,1)
        foxani(q(i,:),getappdata( gcf,'resolution'),true);
    end
end
end
% 编辑窗口
function edit_button_press(h,~)
edit_group=getappdata(gcf,'edit_group');
axis_num=find(edit_group==h);
T_Old = getappdata(gcf,'ThetaOld');


if getappdata(gcf,'jointspace_flag')
    user_entry = check_edit(h,axis_num);
    T_Old(axis_num)=user_entry-(axis_num==2||axis_num==3)*90;
    foxani(T_Old,getappdata(gcf,'resolution'),false);
else
    T=foxfkine(T_Old);
    if axis_num<4
        user_entry = check_edit(h,axis_num);
        p_old=T(axis_num,4);
        p_new=user_entry;
        p=linspace(p_old,p_new,ceil(abs(p_new-p_old))+1);
        T=repmat(T,1,1,length(p));
        T(axis_num,4,:)=p;
    end
    q=foxikine(T);
    figure(h.Parent.Parent);
    for i=1:size(q,1)
        foxani(q(i,:),getappdata( gcf,'resolution'),true);
    end
end



    function user_entry = check_edit(h,axis_num)
        
        slider_group=getappdata(gcf,'slider_group');
        user_entry = str2double(get(h,'String'));
        min_v=get(slider_group(axis_num),'min');
        max_v=get(slider_group(axis_num),'max');
        if isnan(user_entry)
            warning('input should be numeric');
            user_entry=round(get(slider_group(axis_num),'Value'));
            set(h,'String',user_entry);
        elseif user_entry <= min_v
            warning(['input should be no less than ' num2str(min_v)]);
            user_entry = min_v;
            set(h,'String',user_entry);
        elseif user_entry >= max_v
            warning(['input should be no more than ' num2str(max_v)]);
            user_entry = max_v;
            set(h,'String',user_entry);
        end
    end

end
% 弹出
function popup_press(h,~)
min_group=getappdata(gcf,'min_group');
max_group=getappdata(gcf,'max_group');
info_group=getappdata(gcf,'info_group');
slider_group=getappdata(gcf,'slider_group');
edit_group=getappdata(gcf,'edit_group');
theta_old=getappdata(gcf,'ThetaOld');

jointspace_flag=get(h,'value')==1;
setappdata(gcf,'jointspace_flag',jointspace_flag);
if jointspace_flag
%     Angle    Range                Default Name
%     Theta 1: 360 (-180 to 180)    90       Waist Joint
%     Theta 2: 261 (-106 to 155)   -90       Shoulder Joint
%     Theta 3: 410 (-170 to 240)   -90       Elbow Joint
%     Theta 4: 400 (-200 to 200)     0       Wrist Roll
%     Theta 5: 300 (-150 to 150)     0       Wrist Bend
%     Theta 6: 910 (-455 to 455)     0       Wrist Swivel
    set(min_group,{'string'},{'-180';'-106';'-170';'-200';'-150';'-455'});
    set(max_group,{'string'},{'180';'155';'240';'200';'150';'455'});
    set(info_group,{'string'},{'θ1';'θ2';'θ3';'θ4';'θ5';'θ6'});
    set(slider_group,{'min','max','value'},...
        {-180,180,theta_old(1);...
        -106,155,theta_old(2)+90;...
        -170,240,theta_old(3)+90;...
        -200,200,theta_old(4);...
        -150,150,theta_old(5);...
        -455,455,theta_old(6)});
    set(edit_group,{'string'},...
        {theta_old(1);theta_old(2)+90;theta_old(3)+90;...
        theta_old(4);theta_old(5);theta_old(6)});
    set(edit_group(4:6),'style','edit');
else
    T=foxfkine(theta_old);
  
    set(min_group,{'string'},{'-629';'-670';'-79';'-180';'-180';'-180'});
    set(max_group,{'string'},{'670';'670';'955';'180';'180';'180'});
    set(info_group,{'string'},{'x';'y';'z';'rx';'ry';'rz'});
    set(slider_group,{'min','max','value'},...
        {-629,670,T(1,4);...
        -670,670,T(2,4);...
        -79,955,T(3,4);...
        -180,180,0;...
        -180,180,0;...
        -180,180,0});
    set(edit_group,{'string'},...
        {T(1,4);T(2,4);T(3,4);...
        T(1,3);T(2,3);T(3,3)});
    set(edit_group(4:6),'style','text');
end


end
% 轨迹规划演示，可以修改参数
function demo_button_press(~,~)

t = 0:.01:1;
o=zeros(6,length(t));
o(1,:) = 350*(1-t).*cos(10*pi*t)+350;
o(2,:) = 350*(1-t).* sin(10*pi*t);
o(3,:) = 350+150*t;
o(4,:) = 0;
o(5,:) = 0;
o(6,:) = -1;
T=o2h(o);
q=foxikine(T);
foxani(q(1,:),getappdata( gcf,'resolution'),false);
for i=1:size(q,1)
    foxani(q(i,:),getappdata( gcf,'resolution'),true);
end

foxani([0,-90,0,0,0,0],getappdata(gcf,'resolution'),false);
end
% 原始位置
function home_button_press(~,~)
foxani([0,-90,0,0,0,0],getappdata(gcf,'resolution'),false); % show it animate home
end
% 清除跟踪按钮
function clr_trail_button_press(~,~)
%disp('pushed clear trail bottom');
handles = getappdata(gcf,'patch_h');           %
set( handles(8),'xdata',0,'ydata',0,'zdata',0);
% assignin('base','J',jacobe0(getappdata(gcf,'ThetaOld')));
end
%随机运动
function rnd_demo_button_press(~, ~)

%     Angle    Range                Default Name
%     Theta 1: 360 (-180 to 180)    90       Waist Joint
%     Theta 2: 261 (-106 to 155)   -90       Shoulder Joint
%     Theta 3: 410 (-170 to 240)   -90       Elbow Joint
%     Theta 4: 400 (-200 to 200)     0       Wrist Roll
%     Theta 5: 300 (-150 to 150)     0       Wrist Bend
%     Theta 6: 910 (-455 to 455)     0       Wrist Swivel

theta1 = -180 + 360*rand(1); 
theta2 = -106 + 261*rand(1)-90; 
theta3 = -170 + 410*rand(1)-90;
theta4 = -200 + 400*rand(1);
theta5 = -150 + 300*rand(1);
theta6 = -455 + 910*rand(1);
foxani([theta1,theta2,theta3,theta4,theta5,theta6],getappdata(gcf,'resolution'),true)
end
%%                                                     动画
%机械臂运动
function foxani(theta,resolution,ltrail)

ThetaOld = getappdata(gcf,'ThetaOld');
n=ceil(max(abs(ThetaOld-theta)./[360 261 410 400 300 910]*170)/resolution)+1;
t1 = linspace(ThetaOld(1),theta(1),n);
t2 = linspace(ThetaOld(2),theta(2),n);
t3 = linspace(ThetaOld(3),theta(3),n);
t4 = linspace(ThetaOld(4),theta(4),n);
t5 = linspace(ThetaOld(5),theta(5),n);
t6 = linspace(ThetaOld(6),theta(6),n);

jointspace_flag=getappdata(gcf,'jointspace_flag');
linkdata=getappdata(gcf,'linkdata');
handles = getappdata(gcf,'patch_h');
slider_group=getappdata(gcf,'slider_group');
edit_group=getappdata(gcf,'edit_group');

t_01 = tmat(pi/2, 240, 620, 0+t1*pi/180);
t_12 = tmat(pi, 820, 0, 0-t2*pi/180);
t_23 = tmat(-pi/2,120,80, 0+t3*pi/180);
t_34 = tmat(0,0, 825, 0+t4*pi/180);
t_45 = tmat(0, 0, 100, pi+t5*pi/180);
t_56 = tmat(0, 0, 0, 0+t6*pi/180);

for i = 2:n
    % Forward Kinematics
    T_01=t_01(:,:,i);
    T_02 = T_01*t_12(:,:,i);
    T_03 = T_02*t_23(:,:,i);
    T_04 = T_03*t_34(:,:,i);
    T_05 = T_04*t_45(:,:,i);
    T_06 = T_05*t_56(:,:,i);
    
    Link2 = (T_01*linkdata.s2.V2')';
    Link3 = (T_02*linkdata.s3.V3')';
    Link4 = (T_03*linkdata.s4.V4')';
    Link5 = (T_04*linkdata.s5.V5')';
    Link6 = (T_05*linkdata.s6.V6')';
    Link7 = (T_06*linkdata.s7.V7')';
    xyz=T_06*[200*eye(3) zeros(3,1);ones(1,4)];

    set(handles(2:7),{'vertices'},{Link2(:,1:3);...
        Link3(:,1:3);...
        Link4(:,1:3);...
        Link5(:,1:3);...
        Link6(:,1:3);...
        Link7(:,1:3)});
    set(handles(9:11),{'xdata','ydata','zdata'},...
        {[xyz(1,4) xyz(1,1)],[xyz(2,4) xyz(2,1)],[xyz(3,4) xyz(3,1)];...
        [xyz(1,4) xyz(1,2)],[xyz(2,4) xyz(2,2)],[xyz(3,4) xyz(3,2)];...
        [xyz(1,4) xyz(1,3)],[xyz(2,4) xyz(2,3)],[xyz(3,4) xyz(3,3)]});

    if ltrail
        D=T_06*[1 0 0 0;0 1 0 0;0 0 1 110;0 0 0 1];
        handles(8).XData=[handles(8).XData,D(1,4)];
        handles(8).YData=[handles(8).YData D(2,4)];
        handles(8).ZData=[handles(8).ZData D(3,4)];
    end
    
    if jointspace_flag
        set(edit_group,{'String'},{t1(i);t2(i)+90;t3(i)+90;t4(i);t5(i);t6(i)}); % 更新滑块和文本
        set(slider_group,{'Value'},{t1(i);t2(i)+90;t3(i)+90;t4(i);t5(i);t6(i)});
    else
        set(slider_group,{'value'},{T_06(1,4);T_06(2,4);T_06(3,4);0;0;0});
        set(edit_group,{'string'},{T_06(1,4);T_06(2,4);T_06(3,4);T_06(1,3);T_06(2,3);T_06(3,3)});
    end
    drawnow limitrate;
end

setappdata(gcf,'ThetaOld',theta);
end
%%                                                    运动学部分
%% 逆运动学

%     Angle    Range                Default Name
%     Theta 1: 360 (-180 to 180)    90       Waist Joint
%     Theta 2: 261 (-106 to 155)   -90       Shoulder Joint
%     Theta 3: 410 (-170 to 240)   -90       Elbow Joint
%     Theta 4: 400 (-200 to 200)     0       Wrist Roll
%     Theta 5: 300 (-150 to 150)     0       Wrist Bend
%     Theta 6: 910 (-455 to 455)     0       Wrist Swivel
function q = foxikine(T)
limits=[-180 180;-106 155;-170 240;-200 200;-150 150;-455 455]*pi/180;
a2=270;a3=110;d4=306;
q=zeros(size(T,3),6);
for i=1:size(T,3)
    T_i=T(:,:,i);
    q(i,1) = angle(T_i(1,4)+T_i(2,4)*1i);%离开
    if q(i,1)<limits(1,1)||q(i,1)>limits(1,2)
        q(i,1) = angle(-T_i(1,4)-T_i(2,4)*1i);
        T_01 = tmat(-pi/2, 75, 360, q(i,1));
        T_16=T_01\T_i;
        pt=T_16(1,4)+T_16(2,4)*1i;
        l3=a3+d4*1i;
        if RangeCheck(pt)
            alpha=145/180*pi+angle(pt);
            beta=asin(a2*sin(alpha)/abs(l3));
            r_min=a2*sin(alpha+beta)/sin(beta);
            if abs(pt) > r_min
                %上
                q(i,2)=angle(pt)-acos((pt*pt'+a2*a2-l3*l3')/2/abs(pt)/a2);
                q(i,3)=pi-acos((a2*a2+l3*l3'-pt*pt')/2/a2/abs(l3))-angle(l3);
            else
                %下
                q(i,2)=angle(pt)+acos((pt*pt'+a2*a2-l3*l3')/2/abs(pt)/a2);
                q(i,3)=acos((a2*a2+l3*l3'-pt*pt')/2/a2/abs(l3))-pi-angle(l3);
            end
        else
            h=errordlg('转轴2，3不能达到当前的配置','轨迹规划错误');
            q(i:end,:)=[];
            q=q*180/pi;
            pause(1),if isvalid(h),delete(h);end
            return;
        end
    else
        T_01 = tmat(-pi/2, 75, 360, q(i,1));
        T_16=T_01\T_i;
        pt=T_16(1,4)+T_16(2,4)*1i;
        l3=a3+d4*1i;
        if RangeCheck(pt)
             %下
            q(i,2)=angle(pt)-acos((pt*pt'+a2*a2-l3*l3')/2/abs(pt)/a2);
            q(i,3)=pi-acos((a2*a2+l3*l3'-pt*pt')/2/a2/abs(l3))-angle(l3);
        else
            h=errordlg('转轴2,3不能达到当前的配置','轨迹规划错误');
            q(i:end,:)=[];
            q=q*180/pi;
            pause(1),if isvalid(h),delete(h);end
            return
        end
    end
    
    T_12 = tmat(0, 270, 0, q(i,2));
    T_23 = tmat(-pi/2, 110, 0, q(i,3));
    T_13 = T_12*T_23;
    
    Td4 = [1 0 0 0;0 1 0 0;0 0 1 306;0 0 0 1];
    
    T_46 = Td4\(T_13\T_16);
    
    q_p = rotm2zyz(T_46(1:3,1:3),true);
    q_n = rotm2zyz(T_46(1:3,1:3),false);
    if abs(q_p(1)) > abs(q_n(1))
        q(i,4:6) = q_n;
    else
        q(i,4:6) = q_p;
    end

    if any(q(i,4:6)<limits(4:6,1)'|q(i,4:6)>limits(4:6,2)')
            h=errordlg('转轴4~6不能达到当前的配置','轨迹规划错误');
            q(i:end,:)=[];
            q=q*180/pi;
            pause(1),if isvalid(h),delete(h);end
            return;
    end
end
q=q*180/pi;

    function q=rotm2zyz(R,positive)
        q=zeros(1,3);
        if (1-R(3,3))<eps('single')
            q(1)=0;
            q(2)=0;
            q(3)=angle(R(1,1)+R(2,1)*1i);
        elseif positive
            q(2)=-acos(R(3,3));
            q(1)=angle(R(1,3)+R(2,3)*1i);
            q(3)=angle(-R(3,1)+R(3,2)*1i);
        else
            q(2)=acos(R(3,3));
            q(1)=angle(-R(1,3)-R(2,3)*1i);
            q(3)=angle(R(3,1)-R(3,2)*1i);
        end
    end

    function [l,r_min,r_max]=RangeCheck(pt)
   
        theta=angle(pt);
        r=abs(pt);
        if theta >= angle(-541-100i) && theta <limits(2,1)
            alpha=-145/180*pi-theta;
            beta=asin(a2*sin(alpha)/abs(l3));
            r_max=a2*sin(alpha+beta)/sin(beta);
            r_min=551;
            l=r>=r_min && r<=r_max;
        elseif theta>=limits(2,1) && theta<angle(-278-475i)
            r_min=551;r_max=595;
            l=r>=r_min && r<=r_max;
        elseif theta>=angle(-278-475i) && theta<angle(93.25-237.78i)
            alpha=145/180*pi+theta;
            beta=asin(a2*sin(alpha)/abs(l3));
            r_min=a2*sin(alpha+beta)/sin(beta);
            r_max=595;
            l=r>=r_min && r<=r_max;
        elseif theta>=angle(93.25-237.78i) && theta<limits(2,2)
            r_min=256;r_max=595;
            l=r>=r_min && r<=r_max;
        elseif theta>=limits(2,2) && theta<=angle(-50+250i)
            alpha=theta-25/180*pi;
            beta=asin(a2*sin(alpha)/abs(l3));
            r_max=a2*sin(alpha+beta)/sin(beta);
            r_min=256;
            l=r>=r_min && r<=r_max;
        else
            l=false;r_min=NaN;r_max=NaN;
        end
    end


end
%% 正运动
function T_06=foxfkine(q)
T_06 = tmat(0, 0, 0, q(1)*pi/180)*tmat(-pi/2, 0, 212.02, q(2)*pi/180)*tmat(0,645,0, q(3)*pi/180)*...
    tmat(-pi/2, 150, 716, q(4)*pi/180)*tmat(pi/2, 0, 0, q(5)*pi/180)*tmat(-pi/2, 0, 0, q(6)*pi/180);
end
% 工具定向到齐次变换
function T=o2h(o)
% o的每一列都是一个配置
d6=110;n=[1 0 0];
o(4:6,:)=o(4:6,:)./repmat(sqrt(sum(o(4:6,:).*o(4:6,:))),3,1);
T=zeros(4,4,size(o,2));
T(1:3,4,:)=o(1:3,:)-d6*o(4:6,:);
T(4,4,:)=ones(1,size(o,2));
for i=1:size(o,2)
    if abs(n*o(4:6,i))==1
        T(1:3,1,i)=[1 0 0]'*sign(o(6,i));
        T(1:3,2,i)=[0 1 0]'*sign(o(6,i));
        T(1:3,3,i)=o(4:6,i);
    else
        x=n'-n*o(4:6,i)*n';
        x=x/(x'*x);
        T(1:3,1,i)=x;
        T(1:3,2,i)=axang2rotm([o(4:6,i)' pi/2])*x;
        T(1:3,3,i)=o(4:6,i);
    end
    
end
end
% 齐次变换矩阵计算
function T = tmat(alpha, a, d, theta)
n=length(theta);
theta=reshape(theta,1,1,n);
alpha =repmat(alpha,1,1,n);
a=repmat(a,1,1,n);
d=repmat(d,1,1,n);
c = cos(theta);
s = sin(theta);
ca = cos(alpha);
sa = sin(alpha);
T = [c -s.*ca s.*sa a.*c; s c.*ca -c.*sa a.*s; zeros(1,1,n) sa ca d; ...
    zeros(1,1,n) zeros(1,1,n) zeros(1,1,n) ones(1,1,n)];
end
% 将轴角矢量转换为旋转矩阵
function R = axang2rotm(v)
x=v(1);y=v(2);z=v(3);theta=v(4);
c=cos(theta);s=sin(theta);t=1-c;
R =  [t*x*x + c	  t*x*y - z*s	   t*x*z + y*s
      t*x*y + z*s	  t*y*y + c	       t*y*z - x*s
      t*x*z - y*s	  t*y*z + x*s	   t*z*z + c];
end
function R = rotx(t)  
ct = cos(t);
st = sin(t);
R = [1   0    0
     0   ct  -st
     0   st   ct];
end
function R = roty(t)  
ct = cos(t);
st = sin(t);
R = [ct   0  -st
     0    1   0
     st   0   ct];
end
function R = rotz(t)  
ct = cos(t);
st = sin(t);
R = [ct  -st  0
     st  ct   0
     0   0    1];
end

