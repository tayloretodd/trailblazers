clear
fileID = fopen('Test1');
for s =1:20
    tline = fgetl(fileID)
end

    num1 = 1
    num2 = 1
    num3 = 1
    num4 = 1
    num5 = 1
    num6 = 1
    num7 = 1
    num8 = 1
    num9 = 1
    num10 = 1
    num11 = 1
    num12 = 1
    num13 = 1
    num14 = 1
    num15 = 1
    num16  = 1
    
for c =1:6000
  %  tline = fgetl(fileID)
 %while(tline ~= -1)
    tline = fgetl(fileID)
    %Split string into nums
    numbers = strsplit(tline, ', ');
    
    %convert number from strign to num format
    num1 = [num1,str2double(numbers{1})];
    num2 = [num2,str2double(numbers{2})];
    num3 = [num3,str2double(numbers{3})];
    num4 = [num4,str2double(numbers{4})];
    num5 = [num5,str2double(numbers{5})];
    num6 = [num6,str2double(numbers{6})];
    num7 = [num7,str2double(numbers{7})];
    num8 = [num8,str2double(numbers{8})];
    num9 = [num9,str2double(numbers{9})];
    num10 = [num10,str2double(numbers{10})];
    num11 = [num11,str2double(numbers{11})];
    num12 = [num12,str2double(numbers{12})];
    num13 = [num13,str2double(numbers{13})];
    num14 = [num14,str2double(numbers{14})];
    num15 = [num15,str2double(numbers{15})];
    num16  = [num16,str2double(numbers{16})];
end
fclose(fileID)
x_values = 1:6001; % Assuming you want indices as x-values
figure;
plot(x_values, num1);
hold on;
plot(x_values, num2);
plot(x_values, num3);
plot(x_values, num4);%End of Rotation Vector
title("Game Rotation")
legend('Real','i','j','k');
figure;
plot(x_values, num5);
hold on;
plot(x_values, num6);
plot(x_values, num7);%End of Acell
title("Acell")
legend('Z','X','Y');
figure;
plot(x_values, num8);
hold on;
plot(x_values, num9);

plot(x_values, num10);%End of Raw Acell
title("Raw Acell")
legend('Z','X','Y');
figure;
plot(x_values, num11);
hold on;
plot(x_values, num12);

plot(x_values, num13);%End of Gyro
title("Gyro")
legend('Z','X','Y');
figure;
hold on;
plot(x_values, num14);
plot(x_values, num15);
plot(x_values, num16);%End of Raw Gyro
title("Raw Gryo")
legend('Z','X','Y');
