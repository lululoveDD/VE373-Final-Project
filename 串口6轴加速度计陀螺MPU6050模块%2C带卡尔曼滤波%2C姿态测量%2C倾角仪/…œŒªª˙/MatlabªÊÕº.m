
newData = importdata('���ٶ�.txt', '\t', 2);
a=newData.data;
newData = importdata('���ٶ�.txt', '\t', 2);
w=newData.data;
newData = importdata('�Ƕ�.txt', '\t', 2);
Angle=newData.data;

subplot(2,2,1);plot(a(:,1),a(:,2:4));grid on;xlabel('ʱ��/s');ylabel('���ٶ�/g');title('���ٶ�����');
subplot(2,2,2);plot(w(:,1),w(:,2:4));grid on;xlabel('ʱ��/s');ylabel('���ٶ�/��/s');title('���ٶ�����');
subplot(2,2,3);plot(Angle(:,1),Angle(:,2:4));grid on;xlabel('ʱ��/s');ylabel('�Ƕ�/��');title('�Ƕ�����');
subplot(2,2,4);plot(a(:,1),a(:,5));grid on;xlabel('ʱ��/s');ylabel('�¶�/��');title('�¶�����');

