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
%Convert from table form to array
data1 = data1{:,:};
data2 = data2{:,:};
data3 = data3{:,:};
data4 = data4{:,:};
data5 = data5{:,:};

set(0,'defaultlinelinewidth',2)

figure(1)
plot = plot3(data1(:, 1), data1(:, 2), data1(:, 3), '-');
hold on
plot = plot3(data2(:, 1), data2(:, 2), data2(:, 3), '-');
hold on
plot = plot3(data3(:, 1), data3(:, 2), data3(:, 3), '-');

xlabel("X axis")
ylabel("Y axis")
zlabel("Z axis")

%axis tight
grid on
axis([-1 0  -0.6 0.6 0.1 0.9])
hold off

figure(2)
plot = plot3(data4(:, 1), data4(:, 2), data4(:, 3), '-');
hold on
plot = plot3(data5(:, 1), data5(:, 2), data5(:, 3), '-');

xlabel("X axis")
ylabel("Y axis")
zlabel("Z axis")

%axis tight
grid on
axis([-1 0  -0.6 0.6 0.1 0.9])