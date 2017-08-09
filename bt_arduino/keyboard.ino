#include <SoftwareSerial.h> 
#include <Keyboard.h>

// 定義連接藍牙模組的序列埠
SoftwareSerial BT(8, 9); // 接收腳, 傳送腳
unsigned char val;  // 儲存接收資料的變數
int counter = 0;
const float acc[4] = {0, 3, 6, 9};
bool uPress, dPress, lPress, rPress, sPress;

void decoder(unsigned char val, bool &uPress, bool &dPress, bool &lPress, bool &rPress, bool &sPress){
    bool lClick, rClick;
    int Angle_x, Angle_y;
    lClick = val & 1; val >>= 1;
    rClick = val & 1; val >>= 1;
    if (val & 1) Angle_y = 1;
    else Angle_y = -1;
    val >>= 1;
    Angle_y *= val % 4;
    val >>= 2;
    if (val & 1) Angle_x = -1;
    else Angle_x = 1;
    val >>= 1;
    Angle_x *= val % 4;
    Serial.print(lClick);Serial.print('\t');
    Serial.print(rClick);Serial.print('\t');
    Serial.print(Angle_x);Serial.print('\t');
    Serial.print(Angle_y);Serial.print('\n');
    uPress = lPress = rPress = uPress = dPress = 0;
    sPress = lClick;
    if (Angle_x > 0) lPress = 1;
    if (Angle_x < 0) rPress = 1;
    if (Angle_y > 0) uPress = 1;
    if (Angle_y < 0) dPress = 1;
}

void setup() {
  Serial.begin(9600);   // 與電腦序列埠連線
  Serial.println("BT is ready!");
  // 設定藍牙模組的連線速率
  // 如果是HC-05，請改成38400
  BT.begin(9600);
  Keyboard.begin();
}

void loop() {
  // 若收到「序列埠監控視窗」的資料，則送到藍牙模組
  if (Serial.available()) {
    val = Serial.read();
    BT.print(val);
  }

  if (BT.available()) {
    val = BT.read();
    decoder(val, uPress, dPress, lPress, rPress, sPress);
    if (counter == 5){
      if (uPress) Keyboard.press(KEY_UP_ARROW);
      else Keyboard.release(KEY_UP_ARROW);
      if (dPress) Keyboard.press(KEY_DOWN_ARROW);
      else Keyboard.release(KEY_DOWN_ARROW);
      if (lPress) Keyboard.press(KEY_LEFT_ARROW);
      else Keyboard.release(KEY_LEFT_ARROW);
      if (rPress) Keyboard.press(KEY_RIGHT_ARROW);
      else Keyboard.release(KEY_RIGHT_ARROW);
      if (sPress) Keyboard.press(KEY_LEFT_SHIFT);
      else Keyboard.release(KEY_LEFT_SHIFT);
      counter = 0;
    }else{
      counter ++;
    }
    
  }
}
