
newData = importdata('加速度.txt', '\t', 2);
a=newData.data;
newData = importdata('角速度.txt', '\t', 2);
w=newData.data;
newData = importdata('角度.txt', '\t', 2);
Angle=newData.data;

subplot(2,2,1);plot(a(:,1),a(:,2:4));grid on;xlabel('时间/s');ylabel('加速度/g');title('加速度曲线');
subplot(2,2,2);plot(w(:,1),w(:,2:4));grid on;xlabel('时间/s');ylabel('角速度/°/s');title('角速度曲线');
subplot(2,2,3);plot(Angle(:,1),Angle(:,2:4));grid on;xlabel('时间/s');ylabel('角度/°');title('角度曲线');
subplot(2,2,4);plot(a(:,1),a(:,5));grid on;xlabel('时间/s');ylabel('温度/°');title('温度曲线');

