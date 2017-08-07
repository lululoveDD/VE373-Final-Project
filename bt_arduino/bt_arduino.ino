#include <SoftwareSerial.h> 

unsigned char Re_buf[11],counter=0;
unsigned char sign=0;
float a[3],w[3],angle[3],T;

// 定義連接藍牙模組的序列埠
SoftwareSerial BT(8, 9); // 接收腳, 傳送腳
char val;  // 儲存接收資料的變數

void setup() {
  Serial.begin(9600);   // 與電腦序列埠連線
  Serial.println("BT is ready!");
  // 設定藍牙模組的連線速率
  // 如果是HC-05，請改成38400
  BT.begin(115200);
}

void loop() {
  // 若收到「序列埠監控視窗」的資料，則送到藍牙模組
  if (Serial.available()) {
    Serial.print('A');
    val = Serial.read();
    BT.print(val);
  }

  // 若收到藍牙模組的資料，則送到「序列埠監控視窗」
  if (BT.available()) {
    
    Re_buf[counter]=(unsigned char)BT.read();
    if(counter==0&&Re_buf[0]!=0x55) return;      //第0号数据不是帧头              
    counter++;       
    if(counter==11)             //接收到11个数据
    {    
       counter=0;               //重新赋值，准备下一帧数据的接收 
       sign=1;
    }  
    /*
    val = BT.read();
    //unsigned char tmp = val;
    Serial.print(val);
    //Serial.print("\n");
    */ 
  }
  if(sign)
  {  
     sign=0;
     if(Re_buf[0]==0x55)      //检查帧头
     {  
	switch(Re_buf [1])
	{
	case 0x51:
		a[0] = (short(Re_buf [3]<<8| Re_buf [2]))/32768.0*16;
		a[1] = (short(Re_buf [5]<<8| Re_buf [4]))/32768.0*16;
		a[2] = (short(Re_buf [7]<<8| Re_buf [6]))/32768.0*16;
		T = (short(Re_buf [9]<<8| Re_buf [8]))/340.0+36.25;
		break;
	case 0x52:
		w[0] = (short(Re_buf [3]<<8| Re_buf [2]))/32768.0*2000;
		w[1] = (short(Re_buf [5]<<8| Re_buf [4]))/32768.0*2000;
		w[2] = (short(Re_buf [7]<<8| Re_buf [6]))/32768.0*2000;
		T = (short(Re_buf [9]<<8| Re_buf [8]))/340.0+36.25;
		break;
	case 0x53:
        	angle[0] = (short(Re_buf [3]<<8| Re_buf [2]))/32768.0*180;
		angle[1] = (short(Re_buf [5]<<8| Re_buf [4]))/32768.0*180;
		angle[2] = (short(Re_buf [7]<<8| Re_buf [6]))/32768.0*180;
		T = (short(Re_buf [9]<<8| Re_buf [8]))/340.0+36.25;
                Serial.print("a:");
                Serial.print(a[0]);Serial.print(" ");
                Serial.print(a[1]);Serial.print(" ");
                Serial.print(a[2]);Serial.print(" ");
                Serial.print("w:");
                Serial.print(w[0]);Serial.print(" ");
                Serial.print(w[1]);Serial.print(" ");
                Serial.print(a[2]);Serial.print(" ");
                Serial.print("angle:");
                Serial.print(angle[0]);Serial.print(" ");
                Serial.print(angle[1]);Serial.print(" ");
                Serial.print(angle[2]);Serial.print(" ");
                Serial.print("T:");
                Serial.println(T);
                break;
	} 
    }
  }
}
