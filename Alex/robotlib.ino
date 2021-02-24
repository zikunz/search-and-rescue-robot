// Direction values
typedef enum dir
{
  STOP,
  FORWARD,
  BACKWARD,
  LEFT,
  RIGHT
} dir;

// Motor control pins
#define M1F   5
#define M1R   6
#define M2F   9
#define M2R   10


// Ensure that percent is between 0 and 100
double checkPercent(double percent)
{
  if(percent < 0.0)
    return 0.0;
  else
    if(percent > 100.0)
      return 100.0;
    else
      return percent;
}

// Get PWM value based on percentage
int getPWMValue(double percent)
{
  return (int)(percent/100.0 * 255.0); 
}

void move(double percent, int direction)
{
  int val=getPWMValue(checkPercent(percent));

  // Forward and reverse values for motors 1 and 2
  int M1FVal, M1RVal, M2FVal, M2RVal;

  switch(direction)
  {
    case FORWARD:
      M1FVal=val;
      M2FVal=val;

      M1RVal=0;
      M2RVal=0;
      
    break;

    case BACKWARD:
      M1FVal=0;
      M2FVal=0;

      M1RVal=val;
      M2RVal=val;
      
    break;

    case LEFT:
      M1FVal=0;
      M2FVal=val;
      M1RVal=val;
      M2RVal=0;
    break;

    case RIGHT:
     M1FVal=val;
     M2FVal=0;
     M1RVal=0;
     M2RVal=val;
    break;

    case STOP:
    default:

    M1FVal=0;
    M2FVal=0;
    M1RVal=0;
    M2RVal=0;
  }

  analogWrite(M1F, M1FVal);
  analogWrite(M2F, M2FVal);
  analogWrite(M1R, M1RVal);
  analogWrite(M2R, M2RVal);
}

void forward(double percent)
{
  move(percent, FORWARD);
}

void backward(double percent)
{
  move(percent, BACKWARD);
}

void left(double percent)
{
  move(percent, LEFT);
}

void right(double percent)
{
  move(percent, RIGHT);
}

void stop()
{
  move(0.0, STOP);
}

