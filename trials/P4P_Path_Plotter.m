clear;
clc;

%Start
%-0.5598303039694383,0.39383877607784695,0.3058042748844018
%End
%-0.5708813788615594,-0.37492317400168745,0.3042743954884903

data1 = readtable('NO_OBSTACLE_1_edited.csv','NumHeaderLines',1);  % skips the first row of data
data2 = readtable('STATIC_ARM_TEST_1_edited.csv','NumHeaderLines',1);  % skips the first row of data
data3 = readtable('RA_OBSTACLE_1_edited.csv','NumHeaderLines',1);  % skips the first row of data %Inserted
data4 = readtable('DOUBLE_ARM_OBSTACLE_1_edited.csv','NumHeaderLines',1);  % skips the first row of data %Static
data5 = readtable('DYNAMIC_ARM_1_edited.csv','NumHeaderLines',1);  % skips the first row of data

data6 = readtable('DYNAMIC_2ARM_1_edited.csv','NumHeaderLines',1);
data7 = readtable('DYNAMIC_2ARM_2_edited.csv','NumHeaderLines',1);
data8 = readtable('DYNAMIC_ARM_2_edited.csv','NumHeaderLines',1);
data9 = readtable('DYNAMIC_ARM_3_edited.csv','NumHeaderLines',1);
data10 = readtable('DYNAMIC_ARM_4_edited.csv','NumHeaderLines',1); %Best Dynamic
data11 = readtable('STATIC_ARM_2_edited.csv','NumHeaderLines',1);
data12 = readtable('STATIC_ARM_3_edited.csv','NumHeaderLines',1);
%data13 = readtable('tcp_position.csv','NumHeaderLines',1);

%Convert from table form to array
data1 = data1{:,:};
data2 = data2{:,:};
data3 = data3{:,:};
data4 = data4{:,:};
data5 = data5{:,:};

data6 = data6{:,:};
data7 = data7{:,:};
data8 = data8{:,:};
data9 = data9{:,:};
data10 = data10{:,:};
data11 = data11{:,:};
data12 = data12{:,:};
%data13 = data13{:,:};

set(0,'defaultlinelinewidth',2)
set(gcf,'color','w');
%Figure 1 - No obstacle, Static obstacle, Dynamic obstacle
figure(1)
plot = plot3(data1(:, 1), data1(:, 2), data1(:, 3), '-');
hold on
plot = plot3(data2(:, 1), data2(:, 2), data2(:, 3), '-');
plot = plot3(data10(:, 1), data10(:, 2), data10(:, 3), '-');
%Starting and ending positions
plot = plot3(-0.5598303039694383, 0.39383877607784695, 0.3058042748844018, 'o', 'MarkerEdgeColor', 'green', 'MarkerFaceColor', 'green');
plot = plot3(-0.5508813788615594, -0.38492317400168745, 0.3042743954884903,  'o', 'MarkerEdgeColor', 'red', 'MarkerFaceColor', 'red');
%Static obstacle
%plot = plot3(-0.6, 0.2, 0.43, 'o', 'MarkerEdgeColor', 'red', 'MarkerSize', 30);
plot = plot3(-0.65, 0.2, 0.43, '*', 'MarkerEdgeColor', 'blue', 'MarkerSize', 30);
%Dynamically inserted obstacles
DynamicObstaclex = [-0.57,-0.55];
DynamicObstacley = [0.1,-0.18];
DynamicObstaclez = [0.4,0.4];
% plot = plot3(DynamicObstaclex, DynamicObstacley, DynamicObstaclez, 'o', 'MarkerEdgeColor', '#EDB120', 'MarkerSize', 30);
plot = plot3(DynamicObstaclex, DynamicObstacley, DynamicObstaclez, 'x', 'MarkerEdgeColor', 'blue', 'MarkerSize', 30);
title("Path of TCP for different collision object scenarios", 'FontSize', 25)
xlabel("X axis (m)",'fontweight','bold', 'FontSize', 15)
ylabel("Y axis (m)",'fontweight','bold', 'FontSize', 15)
zlabel("Z axis (m)",'fontweight','bold', 'FontSize', 15)

%axis tight
grid on
axis([-1 0  -0.6 0.6 0.1 0.9])
legend("No Obstacle Path", "Static Obstacle Path", "Inserted Dynamic Obstacle Path", "Starting Position", "Ending Position", "Static Obstacle Position", "Inserted Dynamic Obstacle Position", 'FontSize', 12)
hold off

%Figure 2 - No obstacle, Static obstacle, Dynamic obstacle

figure(2)
set(gcf,'color','w');
plot = plot3(data10(:, 1), data10(:, 2), data10(:, 3), '-', 'Color', '#EDB120');
hold on
%Starting and ending positions
plot = plot3(-0.5598303039694383, 0.39383877607784695, 0.3058042748844018, 'o', 'MarkerEdgeColor', 'green', 'MarkerFaceColor', 'green');
plot = plot3(-0.5508813788615594, -0.38492317400168745, 0.3042743954884903,  'o', 'MarkerEdgeColor', 'red', 'MarkerFaceColor', 'red');
%Dynamically inserted obstacles
DynamicObstaclex = [-0.57,-0.55];
DynamicObstacley = [0.1,-0.18];
DynamicObstaclez = [0.4,0.4];
% plot = plot3(DynamicObstaclex, DynamicObstacley, DynamicObstaclez, 'o', 'MarkerEdgeColor', '#EDB120', 'MarkerSize', 30);
plot = plot3(DynamicObstaclex, DynamicObstacley, DynamicObstaclez, 'x', 'MarkerEdgeColor', 'blue', 'MarkerSize', 30);
title("Path of TCP for different collision object scenarios", 'FontSize', 25)
xlabel("X axis (m)",'fontweight','bold', 'FontSize', 15)
ylabel("Y axis (m)",'fontweight','bold', 'FontSize', 15)
zlabel("Z axis (m)",'fontweight','bold', 'FontSize', 15)

%axis tight
grid on
axis([-1 0  -0.6 0.6 0.1 0.9])
legend("Inserted Dynamic Obstacle Path", "Starting Position", "Ending Position", "Inserted Dynamic Obstacle Position", 'FontSize', 12)
hold off
% %Figure 2
% figure(2)
% plot = plot3(data4(:, 1), data4(:, 2), data4(:, 3), '-');
% hold on
% plot = plot3(data5(:, 1), data5(:, 2), data5(:, 3), '-');
% 
% xlabel("X axis")
% ylabel("Y axis")
% zlabel("Z axis")
% 
% %axis tight
% grid on
% axis([-1 0  -0.6 0.6 0.1 0.9])
% hold off

%Figure 3
%figure(3)
% plot = plot3(data6(:, 1), data6(:, 2), data6(:, 3), '-');
% hold on
% plot = plot3(data7(:, 1), data7(:, 2), data7(:, 3), '-');
% hold on
% plot = plot3(data8(:, 1), data8(:, 2), data8(:, 3), '-');
% hold on
% plot = plot3(data9(:, 1), data9(:, 2), data9(:, 3), '-');
% hold on
%plot = plot3(data10(:, 1), data10(:, 2), data10(:, 3), '-');
% hold on
% plot = plot3(data11(:, 1), data11(:, 2), data11(:, 3), '-');
% hold on
% plot = plot3(data12(:, 1), data12(:, 2), data12(:, 3), '-');

%Messy tcp data
% hold on
% plot = plot3(data13(:, 1), data13(:, 2), data13(:, 3), '-');

% xlabel("X axis")
% ylabel("Y axis")
% zlabel("Z axis")
% 
% %axis tight
% grid on
% axis([-1 0  -0.6 0.6 0.1 0.9])
% hold off