#include <SoftwareSerial.h> 
//#include <Mouse.h>

// 定義連接藍牙模組的序列埠
SoftwareSerial BT(8, 9); // 接收腳, 傳送腳
unsigned char val;  // 儲存接收資料的變數
const float acc[4] = {0, 3, 6, 9};
bool lClick, rClick;
bool pre_lClick = 0, pre_rClick = 0;
int Angle_x, Angle_y, counter = 0;
int x_acc = 0, y_acc = 0;

void decoder(unsigned char val, bool &rClick, bool &lClick, int &Angle_x, int &Angle_y){
    lClick = val & 1; val >>= 1;
    rClick = val & 1; val >>= 1;
    if (val & 1) Angle_y = -1;
    else Angle_y = 1;
    val >>= 1;
    Angle_y *= val % 4;
    val >>= 2;
    if (val & 1) Angle_x = -1;
    else Angle_x = 1;
    val >>= 1;
    Angle_x *= val % 4;
}

void setup() {
  Serial.begin(115200);   // 與電腦序列埠連線
  Serial.println("BT is ready!");
  // 設定藍牙模組的連線速率
  // 如果是HC-05，請改成38400
  BT.begin(9600);
//  Mouse.begin();
}

void loop() {
  //Serial.println("BT is ready!");
  // 若收到「序列埠監控視窗」的資料，則送到藍牙模組
  Serial.println("BT is ready!");
  if (Serial.available()) {
    val = Serial.read();
    BT.print(val);
  }

  if (BT.available()) {
    val = BT.read();
    decoder(val, lClick, rClick, Angle_x, Angle_y);
//    if (!pre_lClick && lClick) Mouse.press(MOUSE_LEFT);
//    if (pre_lClick && !lClick) Mouse.release(MOUSE_LEFT);
//    if (!pre_rClick && rClick) Mouse.press(MOUSE_RIGHT);
//    if (pre_rClick && !rClick) Mouse.release(MOUSE_RIGHT);
    pre_lClick = lClick; pre_rClick = rClick;
    if (counter == 5){
      if (Angle_x >= 0)
        x_acc = -acc[abs(Angle_x)];
      else
        x_acc = acc[abs(Angle_x)];
      if (Angle_y >= 0)
        y_acc = -acc[abs(Angle_y)];
      else
        y_acc = acc[abs(Angle_y)];
//      Mouse.move(x_acc, y_acc, 0);
      counter = 0;
    }
    else{
      counter ++;
    }
    
    Serial.print(lClick);Serial.print('\t');
    Serial.print(rClick);Serial.print('\t');
    Serial.print(Angle_x);Serial.print('\t');
    Serial.print(Angle_y);Serial.print('\n');
    /*
    Serial.print("x_acc: ");
    Serial.println(x_acc);

    Serial.print("y_acc: ");
    Serial.println(y_acc);
    */
  }
}
