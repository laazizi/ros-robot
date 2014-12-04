// Encoder values
long coder[2] = {
  0,0};

// Count the left wheel encoder interrupts
void EncoderTickL()
{
  coder[LEFT] ++;  
}


// Count the right wheel encoder interrupts
void EncoderTickR()
{
  coder[RIGHT] ++; 
}

// Read encoder values
long readEncoder(int i) {
  if (i == LEFT) return coder[LEFT];
  else return coder[RIGHT];
}

// Reset one encoder
void resetEncoder(int i) {
  if (i == LEFT) {
    coder[LEFT] = 0;
  }
  else {
    coder[RIGHT] = 0;
  }
}

// Reset both encoders
void resetEncoders() {
  resetEncoder(LEFT);
  resetEncoder(RIGHT);
}

// Initialize both encoders
void initEncoders()
{
  attachInterrupt(1, EncoderTickL, CHANGE);
  attachInterrupt(0, EncoderTickR, CHANGE); 
}

