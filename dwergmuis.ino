enum button_state {
   BUTTON_RELEASED     = 0,
   BUTTON_ACK_RELEASED = 1,
   BUTTON_PRESSED      = 2,
   BUTTON_ACK_PRESSED  = 3
};

volatile int scroll_steps = 0;
volatile enum button_state left_state = BUTTON_ACK_RELEASED;

void setup() {

  pinMode(11, INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);
  attachInterrupt(11, left_press, FALLING);
  attachInterrupt(12, left_release, FALLING);
  
  // rotary encoder
  pinMode(16, INPUT_PULLUP);
  pinMode(17, INPUT_PULLUP);
  attachInterrupt(16, read_encoder, CHANGE);
  attachInterrupt(17, read_encoder, CHANGE);
  
  pinMode(13, OUTPUT);       // LED

  Serial.begin(9600);
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
