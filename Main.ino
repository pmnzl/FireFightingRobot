enum RunningState {
  START,
  FIND,
  AVOID,
  BLOW, 
  STOP
};

int turret_position = 0;
int last_position = 0;

int lower_range = 40;
//int lower_range = lower_range_reset;
int higher_range = 170;
int mid_range = (higher_range - lower_range) / 2 + lower_range;

int sweep_pos = lower_range+1;

int sweep_direction = 0;

int photo_threshold_reset = 10; //10
int photo_threshold = 10; //50

int left_readings[10] = {0};
int center_readings[10] = {0};
int right_readings[10] = {0};

int index = 0;
int blowCount = 0;

int confirmed = 0;


int highest_value = 0;
int highest_pos = 0;

int left = 0;
int center = 0;
int right = 0;

static RunningState running_state = START;

void TaskMain() {

   //Serial.print("BLOWCOUNT: ");
   //Serial.println(blowCount);

   Serial.print("IR_RR: ");
   Serial.println(IR_RR.getReading());
  
  switch (running_state) {
    case START:
      Start();
      Serial.println("START");
      break;
    case FIND:
      Find(true);
      Serial.println("FIND");
      break;
    case AVOID:
      behave_avoid();
      Serial.println("AVOID");
      break;
    case BLOW:
    Serial.print("BLOWCNT: ");
    //Serial.println(blowCount);
    //Serial.println("BLOW"); 
      stop_r();
      Blow();
      break;
    case STOP: 
      Serial.println("STOP"); 
      stop_r();
      break;

      
  };
}
void readPT() {

  int left_raw = analogRead(Photo_Left);
  int right_raw = analogRead(Photo_Right);
  int center_raw = analogRead(Photo_Center);

  if (left_raw < photo_threshold) left_raw = 0;
  if (center_raw < photo_threshold) center_raw = 0;
  if (right_raw < photo_threshold) right_raw = 0;

  left_readings[index] = left_raw;
  center_readings[index] = center_raw;
  right_readings[index] = right_raw;

  index++;

  if (index == 10) index = 0;

  left = average(left_readings, 10);
  center = average(center_readings, 10);
  right = average(right_readings, 10);

  Serial.print("Left: ");
  Serial.print(left);
  Serial.print(" | Center: ");
  Serial.print(center);
  Serial.print(" | Right: ");
  Serial.println(right);
  
}


void Start() {

  readPT();

  if (center > highest_value){
    highest_value = center;
    highest_pos = sweep_pos;
  }

//  Serial.println(sweep_direction);

    if (!sweep_direction) {
        sweep_pos++;
        if (sweep_pos == higher_range) sweep_direction = 1;
    } else {

//      Serial.println(highest_pos);

      turret.write(highest_pos);
      turret_position = highest_pos;
      delay(1000);
    sweep_direction = 0;
      running_state = FIND;
      return;
    }


    turret.write(sweep_pos);
     delay(15);

  
}


void Blow() {

  Find(true);
  while(ULTRA.getReading() > 100) {
    forward();
  }
  //stop_r();

  Find(false);
  
  digitalWrite(9, HIGH);

  if(analogRead(Photo_Center) < 600 && analogRead(Photo_Left) < 600 && analogRead(Photo_Right) < 600){
    delay(250);
    if(analogRead(Photo_Center) < 600 && analogRead(Photo_Left) < 600 && analogRead(Photo_Right) < 600){
      blowCount++;
      reverse();
      delay(300);
      stop_r();
      digitalWrite(9, LOW);
      
      if(blowCount == 2){running_state = STOP;}
      else{running_state = FIND;}
      
      //NEW UNTESTED - RESET LOWER PT THRESHOLD 
      photo_threshold = photo_threshold_reset;
    }
  }
  
}

float running_error = 0;


void Find(bool move_robot) {
  
  readPT();

  
  int left_bool = left > center || (!center && left);
  int right_bool = right > center || (!center && right);
  
  int center_bool = (right < center && left < center) || (center < left && center < right);
  int sweep_bool = !left && !right && !center;

  if (left_bool) {
    turret_position += 1;
    constrain(turret_position, lower_range,higher_range);
  }

  if (right_bool) {
    turret_position -= 1;
    constrain(turret_position, lower_range,higher_range);
  }

  if (sweep_bool) {

    sweep_pos = turret_position;

    if (!sweep_direction) {
        sweep_pos++;
        if (sweep_pos == higher_range) sweep_direction = 1;
    } else {
      sweep_pos--;
      if (sweep_pos == lower_range) {
        sweep_direction = 0;
        readPT();
        while (center < photo_threshold){
          cw();
          readPT();
        }
        stop_r();
        }
    }
    turret_position = sweep_pos;
  }

  if (last_position != turret_position){
    Serial.println("fIND;");
    Serial.println(turret_position);
    turret.write(turret_position);
    delay(20);
  }
  
  last_position = turret_position;

  if (!move_robot) {
    return;
  }

  // MOTOR CONTROL
  int speed_k = 0;
  int kp = 3;
  
  float ki = 0.1; //0.05

  if (!sweep_bool){
    if (turret_position < mid_range -7 || turret_position > mid_range+7) {
      speed_k = constrain(kp * (mid_range - turret_position) + (ki * running_error), -100, 100);
      if (abs(speed_k) < 90) {
        running_error += (mid_range - turret_position);
      } 
      confirmed = 0;
    } else {
      stop_r();

      confirmed++;

      if (confirmed == 100){
        confirmed = 0;
        running_error = 0;
        running_state = AVOID;
      } 
    }

    left_font_motor.writeMicroseconds(1500 + speed_k);
    left_rear_motor.writeMicroseconds(1500 + speed_k);
    right_rear_motor.writeMicroseconds(1500 + speed_k);
    right_font_motor.writeMicroseconds(1500 + speed_k);

  } else {
    stop_r();
  }
  
}

float average (int * array, int len)  // assuming array is int.
{
  long sum = 0L ;  // sum will be larger than an item, long for safety.
  for (int i = 0 ; i < len ; i++)
    sum += array [i] ;
  return  ((float) sum) / len ;  // average will be fractional, so float may be appropriate.
}

int hasRecentlyStrafed = 0;


void behave_avoid(){ 
   // Read distance sensors 
   float ir_left = IR_FL.getReading();
   float ir_right = IR_FR.getReading(); 
   float us_center = ULTRA.getReading();
   float ir_left_rear = IR_RL.getReading(); 
   float ir_right_rear = IR_RR.getReading();

   //Serial.print("IR_LEFT: "); 
   //Serial.println(ir_left);
   Serial.print("IR_RIGHT: "); 
   Serial.println(ir_right);
   //Serial.print("US_CENTER: "); 
   //Serial.println(us_center);

  int center_raw = analogRead(Photo_Center);
    Serial.print(center_raw);
    Serial.print(" | ");
    Serial.println(us_center);
    if (center_raw > 900 && us_center < 140) {
      running_state = BLOW;
      return;
    }
   // Perform fuzzy logic control
   float dir = fuzzy_controller(ir_left, us_center, ir_right, ir_left_rear, ir_right_rear);

   // Perform correct movement to avoid obstacles
   if(dir == 0){ 
      forward(); 

      if (hasRecentlyStrafed ==1 || (center_raw < photo_threshold_reset-5) ){
        running_state = FIND;
      }
      hasRecentlyStrafed = 0;
   }else if(dir == -1){ 
      strafe_left();
      hasRecentlyStrafed = 1;
   }else{ 
      strafe_right();
      hasRecentlyStrafed = 1;
   }

   return;
}
