#include <SPI.h>
#include <avr/pgmspace.h>

// Registers
#define REG_Product_ID                           0x00
#define REG_Revision_ID                          0x01
#define REG_Motion                               0x02
#define REG_Delta_X_L                            0x03
#define REG_Delta_X_H                            0x04
#define REG_Delta_Y_L                            0x05
#define REG_Delta_Y_H                            0x06
#define REG_SQUAL                                0x07
#define REG_Pixel_Sum                            0x08
#define REG_Maximum_Pixel                        0x09
#define REG_Minimum_Pixel                        0x0a
#define REG_Shutter_Lower                        0x0b
#define REG_Shutter_Upper                        0x0c
#define REG_Frame_Period_Lower                   0x0d
#define REG_Frame_Period_Upper                   0x0e
#define REG_Configuration_I                      0x0f
#define REG_Configuration_II                     0x10
#define REG_Frame_Capture                        0x12
#define REG_SROM_Enable                          0x13
#define REG_Run_Downshift                        0x14
#define REG_Rest1_Rate                           0x15
#define REG_Rest1_Downshift                      0x16
#define REG_Rest2_Rate                           0x17
#define REG_Rest2_Downshift                      0x18
#define REG_Rest3_Rate                           0x19
#define REG_Frame_Period_Max_Bound_Lower         0x1a
#define REG_Frame_Period_Max_Bound_Upper         0x1b
#define REG_Frame_Period_Min_Bound_Lower         0x1c
#define REG_Frame_Period_Min_Bound_Upper         0x1d
#define REG_Shutter_Max_Bound_Lower              0x1e
#define REG_Shutter_Max_Bound_Upper              0x1f
#define REG_LASER_CTRL0                          0x20
#define REG_Observation                          0x24
#define REG_Data_Out_Lower                       0x25
#define REG_Data_Out_Upper                       0x26
#define REG_SROM_ID                              0x2a
#define REG_Lift_Detection_Thr                   0x2e
#define REG_Configuration_V                      0x2f
#define REG_Configuration_IV                     0x39
#define REG_Power_Up_Reset                       0x3a
#define REG_Shutdown                             0x3b
#define REG_Inverse_Product_ID                   0x3f
#define REG_Motion_Burst                         0x50
#define REG_SROM_Load_Burst                      0x62
#define REG_Pixel_Burst                          0x64

#define adns_com_begin() digitalWrite(ncs, LOW)
#define adns_com_end() digitalWrite(ncs, HIGH)

enum button_state {
   BUTTON_RELEASED     = 0,
   BUTTON_ACK_RELEASED = 1,
   BUTTON_PRESSED      = 2,
   BUTTON_ACK_PRESSED  = 3
};

extern const unsigned short firmware_length;
extern prog_uchar firmware_data[];

volatile boolean mouse_has_data = true;
volatile int scroll_steps = 0;
volatile enum button_state left_state = BUTTON_ACK_RELEASED;

const int ncs = 8;
const int mot = 9;
const int left_pos = 0;
const int left_neg = 1;
// alligned to PORT
const int scroll_phase = 16;
const int scroll_couter = 17;

void setup() {
  Serial.begin(9600);

  // left button
  pinMode(left_pos, INPUT_PULLUP);
  pinMode(left_neg, INPUT_PULLUP);
  attachInterrupt(left_pos, left_press, FALLING);
  attachInterrupt(left_neg, left_release, FALLING);
  
  // rotary encoder
  pinMode(scroll_phase, INPUT_PULLUP);
  pinMode(scroll_couter, INPUT_PULLUP);
  attachInterrupt(scroll_phase, read_encoder, CHANGE);
  attachInterrupt(scroll_couter, read_encoder, CHANGE);
  
  // optical sensor
  pinMode (ncs, OUTPUT);
  pinMode(mot, INPUT);
  attachInterrupt(mot, pointer_moved, FALLING);
  
  SPI.begin();
  SPI.setDataMode(SPI_MODE3);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(64);

  perform_startup();  
  delay(100);
}

void loop() {
  if (scroll_steps) {
    Mouse.scroll(scroll_steps);
    scroll_steps = 0;
  }
  if (!(left_state & 1)) { // not ACK
    left_state = (enum button_state)((int)left_state | 1); // ACK
     Mouse.set_buttons(((left_state & 2) >> 1), 0, 0);
  }
  if (mouse_has_data) {
    mouse_has_data = false;
    adns_com_begin();
  
    byte XLSB = adns_read_reg(REG_Delta_X_L);
    byte XMSB = adns_read_reg(REG_Delta_X_H);
    byte YLSB = adns_read_reg(REG_Delta_Y_L);
    byte YMSB = adns_read_reg(REG_Delta_Y_H);
  
    int dx = (XMSB << 8) | XLSB;
    int dy = (YMSB << 8) | YLSB;

    adns_com_end();

    Mouse.move(dx,dy);
  }
  //Serial.println(mouse_has_data);
  //Serial.println(adns_read_reg(REG_Motion), BIN);
  //Serial.println(digitalRead(mot));
  //Serial.println("--------");
  //delay(100);
}

void read_encoder()
{
  int8_t enc_states[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
  static uint8_t old_AB = 0;
  uint8_t dir;

  old_AB <<= 2;                   //remember previous state
  old_AB |= ( GPIOB_PDIR & 0x03 );  //add current state
  scroll_steps += ( enc_states[( old_AB & 0x0f )]);
}

void left_press() {
  if (left_state == BUTTON_ACK_RELEASED) {
    left_state = BUTTON_PRESSED;
  }
}

void left_release() {
  if (left_state == BUTTON_ACK_PRESSED) {
    left_state = BUTTON_RELEASED;
  }
}

void pointer_moved() {
  mouse_has_data = true;
}

byte adns_read_reg(byte reg_addr){
  adns_com_begin();
  
  // send adress of the register, with MSBit = 0 to indicate it's a read
  SPI.transfer(reg_addr & 0x7f );
  delayMicroseconds(100); // tSRAD
  // read data
  byte data = SPI.transfer(0);
  
  delayMicroseconds(1); // tSCLK-NCS for read operation is 120ns
  adns_com_end();
  delayMicroseconds(19); //  tSRW/tSRR (=20us) minus tSCLK-NCS

  return data;
}

void adns_write_reg(byte reg_addr, byte data){
  adns_com_begin();
  
  //send adress of the register, with MSBit = 1 to indicate it's a write
  SPI.transfer(reg_addr | 0x80 );
  //sent data
  SPI.transfer(data);
  
  delayMicroseconds(20); // tSCLK-NCS for write operation
  adns_com_end();
  delayMicroseconds(100); // tSWW/tSWR (=120us) minus tSCLK-NCS. Could be shortened, but is looks like a safe lower bound 
}

void adns_upload_firmware(){
  // send the firmware to the chip, cf p.18 of the datasheet
  Serial.println("Uploading firmware...");
  // set the configuration_IV register in 3k firmware mode
  adns_write_reg(REG_Configuration_IV, 0x02); // bit 1 = 1 for 3k mode, other bits are reserved 
  
  // write 0x1d in SROM_enable reg for initializing
  adns_write_reg(REG_SROM_Enable, 0x1d); 
  
  // wait for more than one frame period
  delay(10); // assume that the frame rate is as low as 100fps... even if it should never be that low
  
  // write 0x18 to SROM_enable to start SROM download
  adns_write_reg(REG_SROM_Enable, 0x18); 
  
  // write the SROM file (=firmware data) 
  adns_com_begin();
  SPI.transfer(REG_SROM_Load_Burst | 0x80); // write burst destination adress
  delayMicroseconds(15);
  
  // send all bytes of the firmware
  unsigned char c;
  for(int i = 0; i < firmware_length; i++){ 
    c = (unsigned char)pgm_read_byte(firmware_data + i);
    SPI.transfer(c);
    delayMicroseconds(15);
  }
  adns_com_end();
}


void perform_startup(void){
  adns_com_end(); // ensure that the serial port is reset
  adns_com_begin(); // ensure that the serial port is reset
  adns_com_end(); // ensure that the serial port is reset
  adns_write_reg(REG_Power_Up_Reset, 0x5a); // force reset
  delay(50); // wait for it to reboot
  // read registers 0x02 to 0x06 (and discard the data)
  adns_read_reg(REG_Motion);
  adns_read_reg(REG_Delta_X_L);
  adns_read_reg(REG_Delta_X_H);
  adns_read_reg(REG_Delta_Y_L);
  adns_read_reg(REG_Delta_Y_H);
  // upload the firmware
  adns_upload_firmware();
  delay(10);
  //enable laser(bit 0 = 0b), in normal mode (bits 3,2,1 = 000b)
  // reading the actual value of the register is important because the real
  // default value is different from what is said in the datasheet, and if you
  // change the reserved bytes (like by writing 0x00...) it would not work.
  byte laser_ctrl0 = adns_read_reg(REG_LASER_CTRL0);
  adns_write_reg(REG_LASER_CTRL0, laser_ctrl0 & 0xf0 );
  
  delay(1);

  Serial.println("Optical Chip Initialized");
}
