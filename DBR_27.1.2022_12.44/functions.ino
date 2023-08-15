//homing is not yet implemented

void motion(float distance, int spd, float acc, boolean hom)
{
  int d;
  acc=INIT_SPEED;
  int st=0;
  static float abs_pos;

  if ((distance>0.0) && (abs_pos+distance)<=TRAVEL && (abs_pos+distance)>=0 )
  {
    d=1;
    abs_pos+=distance; //add the to the abs position the POSITIVE distance
    int maxpulses=distance*PULSES_PER_MM;
    Serial.println(abs_pos);
    for ( int i=0; i<=maxpulses; i++)
    {
      digitalWrite(DIR_PIN,d);
      k=!k;
      digitalWrite(STP_PIN,k);
      if (i>=(maxpulses-st)){
        acc=acc*ACCEL;
      }
      else if(acc>=spd)
      {
        acc=acc*(1/ACCEL);
        st=i;
      }      
      delayMicroseconds(acc); 
    }
    delay(10);
  }
  else if ((distance<0.0) && (abs_pos+distance)>=0 &&(abs_pos+distance)<=TRAVEL )
  {
    d=0;
    abs_pos+=distance; //add the to the abs position the NEGATIVE distance
    distance=distance*-1; // Transform the distance from negative to positive
    Serial.println(distance);
    int maxpulses=distance*PULSES_PER_MM;
    Serial.println(abs_pos);
    for ( int i=0; i<=maxpulses; i++)
    {
      digitalWrite(DIR_PIN,d);
      k=!k;
      digitalWrite(STP_PIN,k);
      if (i>=(maxpulses-st)){
        acc=acc*ACCEL;
      }
      else if(acc>=spd)
      {
        acc=acc*(1/ACCEL);
        st=i;
      }      
      delayMicroseconds(acc); 
    }
    delay(10); 
  }
  else Serial.println("Travel distance exceeded!"); 
} 
