void motion(float distance, int spd, float acc, boolean hom)
{
  int d;
  acc=init_speed;
  int st=0;
  static float abs_pos;

  if ((distance>0.0) && (abs_pos+distance)<=travel && (abs_pos+distance)>=0 )
  {
    d=1;
    abs_pos+=distance; //add the to the abs position the POSITIVE distance
    int maxpulses=distance*pulses_per_mm;
    Serial.println(abs_pos);
    for ( int i=0; i<=maxpulses; i++)
    {
      digitalWrite(pin_dir,d);
      k=!k;
      digitalWrite(stp,k);
      if (i>=(maxpulses-st)){
        acc=acc*a;
      }
      else if(acc>=spd)
      {
        acc=acc*(1/a);
        st=i;
      }      
      delayMicroseconds(acc); 
    }
    delay(10);
  }
  else if ((distance<0.0) && (abs_pos+distance)>=0 &&(abs_pos+distance)<=travel )
  {
    d=0;
    abs_pos+=distance; //add the to the abs position the NEGATIVE distance
    distance=distance*-1; // Transform the distance from negative to positive
    Serial.println(distance);
    int maxpulses=distance*pulses_per_mm;
    Serial.println(abs_pos);
    for ( int i=0; i<=maxpulses; i++)
    {
      digitalWrite(pin_dir,d);
      k=!k;
      digitalWrite(stp,k);
      if (i>=(maxpulses-st)){
        acc=acc*a;
      }
      else if(acc>=spd)
      {
        acc=acc*(1/a);
        st=i;
      }      
      delayMicroseconds(acc); 
    }
    delay(10); 
  }
  else Serial.println("Travel distance exceeded!"); 
} 
