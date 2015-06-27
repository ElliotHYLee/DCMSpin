clc, clear close all;

delete(instrfind);
instrfindall


serialOne=serial('COM3','BaudRate', 115200)

fopen(serialOne)

str=fscanf(serialOne);
sen=str2num(str);
accX(j)=sen(1);
accY(j)=sen(2);
accZ(j)=sen(3);


fclose(serialOne)
delete(serialOne);



