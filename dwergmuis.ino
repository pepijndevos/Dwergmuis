volatile int scroll_steps = 0;

void setup() {
  pinMode(0, INPUT_PULLUP);
  pinMode(1, INPUT_PULLUP);
  pinMode(13, OUTPUT);
  
  attachInterrupt(0, read_encoder, CHANGE);
  attachInterrupt(1, read_encoder, CHANGE);
  
  Serial.begin(9600);
}

void loop() {
  if (scroll_steps) {
    Mouse.scroll(scroll_steps);
    scroll_steps = 0;
  }
}

void read_encoder()
{
  int8_t enc_states[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
  static uint8_t old_AB = 0;
  uint8_t dir;

  old_AB <<= 2;                   //remember previous state
  old_AB |= ( PIND & 0x03 );  //add current state
  scroll_steps += ( enc_states[( old_AB & 0x0f )]);
}
