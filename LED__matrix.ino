// testcolors demo for Adafruit RGBmatrixPanel library.
// Renders 512 colors on our 16x32 RGB LED matrix:
// http://www.adafruit.com/products/420
// Library supports 4096 colors, but there aren't that many pixels!  :)

// Written by Limor Fried/Ladyada & Phil Burgess/PaintYourDragon
// for Adafruit Industries.
// BSD license, all text above must be included in any redistribution.

#include <RGBmatrixPanel.h>

// Most of the signal pins are configurable, but the CLK pin has some
// special constraints.  On 8-bit AVR boards it must be on PORTB...
// Pin 8 works on the Arduino Uno & compatibles (e.g. Adafruit Metro),
// Pin 11 works on the Arduino Mega.  On 32-bit SAMD boards it must be
// on the same PORT as the RGB data pins (D2-D7)...
// Pin 8 works on the Adafruit Metro M0 or Arduino Zero,
// Pin A4 works on the Adafruit Metro M4 (if using the Adafruit RGB
// Matrix Shield, cut trace between CLK pads and run a wire to A4).

#define CLK  8   // USE THIS ON ARDUINO UNO, ADAFRUIT METRO M0, etc.
//#define CLK A4 // USE THIS ON METRO M4 (not M0)
//#define CLK 11 // USE THIS ON ARDUINO MEGA
#define OE   9
#define LAT 10
#define A   A0
#define B   A1
#define C   A2

#define BIT_MAX  15
#define RED      BIT_MAX,0,0
#define GREEN    0,BIT_MAX,0
#define BLUE     0,0,BIT_MAX
#define BLACK    0,0,0
#define WHITE    BIT_MAX,BIT_MAX,BIT_MAX

#define YELLOW_GREEN 7,BIT_MAX,0
#define YELLOW       BIT_MAX,BIT_MAX,0
#define YELLOW_RED   BIT_MAX,7,0

#define PREAMBLE_HIGH 0xDC
#define PREAMBLE_LOW  0xBA

#define CONNECTED_POS     2
#define BATTERY_POS       3
#define STATUS_POS        4
#define TEMP_POS          5
#define DRIVE_POS         8
#define STEER_POS         11
#define FACE_POS          13
#define CHKSUM_POS        14

int BATTERY[] = {0,0};
int CONNECT[] = {0,1};
int STATUS[]  = {6,0};
int TEMP[]    = {12,0};
int DRIVE_CURRENT[] = {19,0};
int STEER_CURRENT[] = {26,0};

RGBmatrixPanel matrix(A, B, C, CLK, LAT, OE, false);

int data[15];



void setup() {
  matrix.begin();
  int chksum = 0;

  Serial.begin(9600);  
  Serial.println("--- Start Serial Monitor ---");
  Serial.println(); 
  data [0] = 0xDC;
  data [1] = 0xBA;
  data [2] = 0x01;
  data [3] = 0x1F;
  data [4] = 0x00;
  data [5] = 0x01;
  data [6] = 0x11;
  data [7] = 0x11;
  data [8] = 0x22;
  data [9] = 0x22;
  data [10] = 0x22;
  data [11] = 0x33;
  data [12] = 0x33;
  data [13] = 0x01;
  for (int i=2; i < 14; i++){
    chksum += data[i];
  }
  
  int low = chksum & 0x00FF;
  int high = (chksum & 0xFF00) >> 8;
  data [14] = high;
  data [15] = low;
  
  
  matrix.fillScreen(matrix.Color333(0, 0, 0));
}

void loop() {
  // Do nothing -- image doesn't change
  update_screen(data);
  delay(100);
  
}


void update_screen(int message[]){
  long temp = 0;
  long drive_current = 0;
  long steering_current = 0;
  if (preamble_check(message) && chksum_check(message)){
    connected_status(message[CONNECTED_POS]);
    display_status(message[STATUS_POS]);
    display_battery(message[BATTERY_POS]);

    for (int i=TEMP_POS + 2; i>= TEMP_POS; i--){
      temp += (message[i] << i*8);
    }
    
    for (int i=DRIVE_POS + 2; i>= DRIVE_POS; i--){
      drive_current += (message[i] << i*8);
    }
    
    for (int i=STEER_POS + 1; i>= STEER_POS; i--){
      steering_current += (message[i] << i*8);
    }

    display_temp(temp);
    display_currents(drive_current,steering_current);
    //display_face(message[FACE_POS]);
    happy_face();
  }
}


void connected_status(int status){
  if (status) {
    matrix.drawPixel(CONNECT[0], CONNECT[1], matrix.Color444(GREEN));
  }
  else {
    matrix.drawPixel(CONNECT[0], CONNECT[1], matrix.Color444(RED));
  }
}


int preamble_check(int message[]){
  if (message[0] == PREAMBLE_HIGH && message[1] == PREAMBLE_LOW){
    return 1;
  }
  else {
    return 0;
  }
}

int chksum_check(int message[]){
  int chksum = 0;
  for (int i=2; i < 14; i++){
    chksum += message[i];
  }
  int low = chksum & 0x00FF;
  int high = (chksum & 0xFF00) >> 8;
  if ((high == message[CHKSUM_POS]) && (low == message[CHKSUM_POS +1])){
    return 1;
  }
  return 0;
}

void display_battery(int battery_level){
  matrix.drawLine(BATTERY[0],BATTERY[1],BATTERY[0]+4,BATTERY[1],matrix.Color444(BLACK));
  if (battery_level & 0x10){
    matrix.drawPixel(BATTERY[0]+4,BATTERY[1], matrix.Color444(GREEN));
  }
  if (battery_level & 0x08){
    matrix.drawPixel(BATTERY[0]+3,BATTERY[1], matrix.Color444(YELLOW_GREEN));
  }
  if (battery_level & 0x04){
    matrix.drawPixel(BATTERY[0]+2,BATTERY[1], matrix.Color444(YELLOW));
  }
  if (battery_level & 0x02){
    matrix.drawPixel(BATTERY[0]+1,BATTERY[1], matrix.Color444(YELLOW_RED));
  }
  if (battery_level & 0x01){
    matrix.drawPixel(BATTERY[0],BATTERY[1], matrix.Color444(RED));
  }
}


int* get_color(int num){
  int* color = new int[3];
  if (num == 0x04){
    color[0] = BIT_MAX;
    color[1] = 0;
    color[2] = 0;
  }
  else if (num == 0x03){
    color[0] = BIT_MAX;
    color[1] = 7;
    color[2] = 0;
  }
  else if (num == 0x02){
    color[0] = BIT_MAX;
    color[1] = BIT_MAX;
    color[2] = 0;
  }
  else if (num == 0x01){
    color[0] = 7;
    color[1] = BIT_MAX;
    color[2] = 0;
  }
  else {
    color[0] = 0;
    color[1] = BIT_MAX;
    color[2] = 0;
  }
  return color;
}

void display_face(int face){
  if (face == 0x01){
    eight_bit_face();
  }
  else if (face == 0x00){
    happy_face();
  }
}

void display_temp(long temp){
  int* color;
  long mask = 0xF0000;
  for (int i =0; i < 5; i ++){
    color = get_color(((mask >> (i*4)) & temp ) >> (16-(4*i)));
    matrix.drawPixel(TEMP[0]+i,TEMP[1],matrix.Color444(color[0],color[1],color[2]));
  }
}

void display_status(int error_status){
  matrix.drawLine(STATUS[0],STATUS[1],STATUS[0]+4,STATUS[1],matrix.Color444(GREEN));
  if (error_status & 0x01){
    matrix.drawPixel(STATUS[0],STATUS[1],matrix.Color444(RED));
  }
  if (error_status & 0x02){
    matrix.drawPixel(STATUS[0],STATUS[1],matrix.Color444(RED));
  }
  if (error_status & 0x04){
    matrix.drawPixel(STATUS[0],STATUS[1],matrix.Color444(RED));
  }
  if (error_status & 0x08){
    matrix.drawPixel(STATUS[0],STATUS[1],matrix.Color444(RED));
  }
  if (error_status & 0x10){
    matrix.drawPixel(STATUS[0],STATUS[1],matrix.Color444(RED));
  } 
}

void display_currents(long drive_current, long steering_current){
  int* color;
  long drive_mask    = 0xF00000;
  long steering_mask = 0xF000;

  for (int i =0; i < 6; i ++){
    color = get_color(((drive_mask >> (i*4)) & drive_current ) >> (20-(4*i)));
    matrix.drawPixel(DRIVE_CURRENT[0]+i,DRIVE_CURRENT[1],matrix.Color444(color[0],color[1],color[2]));
  }

  for (int i =0; i < 4; i ++){
    color = get_color(((steering_mask >> (i*4)) & steering_current ) >> (12-(4*i)));
    matrix.drawPixel(STEER_CURRENT[0]+i,STEER_CURRENT[1],matrix.Color444(color[0],color[1],color[2]));
  }
}

void happy_eye(int x, int y, int r, int g, int b){
  matrix.drawLine(x-2,y,x,y-2,matrix.Color444(r,g,b));
  matrix.drawLine(x+1,y-2,x+3,y,matrix.Color444(r,g,b));
  
}

void happy_mouth(int x, int y, int r, int g, int b){
  matrix.drawLine(x-4,y,x-2,y+2,matrix.Color444(r,g,b));
  matrix.drawLine(x-1,y+2,x+2,y+2,matrix.Color444(r,g,b));
  matrix.drawLine(x+3,y+2,x+5,y,matrix.Color444(r,g,b));
  matrix.drawPixel(x-4,y-1,matrix.Color444(r,g,b));
  matrix.drawPixel(x+5,y-1,matrix.Color444(r,g,b));
}

void circle_cheek(int x, int y, int r, int g, int b){
  matrix.drawLine(x-1,y,x-1,y+1,matrix.Color444(r,g,b));
  matrix.drawLine(x+2,y,x+2,y+1,matrix.Color444(r,g,b));
  matrix.drawLine(x,y-1,x+1,y-1,matrix.Color444(r,g,b));
  matrix.drawLine(x,y+2,x+1,y+2,matrix.Color444(r,g,b));
}


void cute_mouth(int x, int y, int r, int g, int b){
  matrix.drawLine(x-4,y,x-2,y+2,matrix.Color444(r,g,b));
  matrix.drawLine(x-1,y+2,x,y+1,matrix.Color444(r,g,b));
  matrix.drawLine(x+1,y+2,x+2,y+2,matrix.Color444(r,g,b));
  matrix.drawLine(x+3,y+1,x+4,y,matrix.Color444(r,g,b));
  matrix.drawPixel(x-4,y-1,matrix.Color444(r,g,b));
  matrix.drawPixel(x+4,y-1,matrix.Color444(r,g,b));
  matrix.drawPixel(x,y,matrix.Color444(r,g,b));
  
  //matrix.drawLine(x+3,y+2,x+5,y,matrix.Color444(r,g,b));
  //matrix.drawPixel(x-4,y-1,matrix.Color444(r,g,b));
  //matrix.drawPixel(x+5,y-1,matrix.Color444(r,g,b));
}
void cute_cheeks(int x, int y, int r, int g, int b){
   matrix.drawLine(x-2,y-1,x+1,y-1,matrix.Color444(r,g,b));
   matrix.drawLine(x-2,y,x+1,y,matrix.Color444(r,g,b));
}

void eight_bit_eye(int x, int y){
  matrix.fillCircle(x,y,2,matrix.Color444(0,0,15));
  matrix.drawLine(x-1,y-1,x-1,y+1,matrix.Color444(3,0,15));
  matrix.drawLine(x,y+1,x+1,y+1,matrix.Color444(3,0,15));
  matrix.drawLine(x,y-1,x+1,y-1,matrix.Color444(WHITE));
  matrix.drawLine(x,y,x+1,y,matrix.Color444(WHITE));
}


void eight_bit_face(){
  //clear_face();
  eight_bit_eye(10,5);
  eight_bit_eye(18,5);
  circle_cheek(5,9,RED);
  circle_cheek(23,9,RED);
  happy_mouth(14,11,12,0,15);
}

void happy_face(){
  //clear_face();
  happy_eye(10,5,3,0,15);
  happy_eye(18,5,3,0,15);
  cute_mouth(14,12,3,0,15);
  cute_cheeks(7,9,12,0,15);
  cute_cheeks(22,9,12,0,15);
  
}

void clear_face(){
  matrix.fillRect(0,2,31,15,matrix.Color444(BLACK));
}
