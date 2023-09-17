void BT_comm() {                                             // establish bluetooth communication
  while (matlab_ready == false) {
    if (Serial.available() > 0) {
      state = Serial.read();
    }
    if (state == 'r') {
      matlab_ready = true;
      state = 0;
    } else {
      state = 0;
    }
  }
}

void runRM(int speed, boolean rev) {
  if (rev) {
    digitalWrite(RM_DIREC, LOW);     //CCW
  } else {
    digitalWrite(RM_DIREC, HIGH);    //CW
  }
  digitalWrite(RM_BRAKE, LOW);
  analogWrite(RM_PMW, speed);
}

void runLM(int speed, boolean rev) {
  if (rev) {
    digitalWrite(LM_DIREC, LOW);     //CCW
  } else {
    digitalWrite(LM_DIREC, HIGH);    //CW
  }
  digitalWrite(LM_BRAKE, LOW);
  analogWrite(LM_PMW, speed);
}

void runBM(int speed, boolean rev) {
  if (rev) {
    digitalWrite(BM_DIREC, HIGH);    //CCW
  } else {
    digitalWrite(BM_DIREC, LOW);     //CW
  }
  digitalWrite(BM_BRAKE, LOW);
  analogWrite(BM_PMW, speed);
}

void fullstop() {
  digitalWrite(RM_BRAKE, HIGH);
  digitalWrite(LM_BRAKE, HIGH);
  digitalWrite(BM_BRAKE, HIGH);
}

void RMEncoderEvent() {
  if (digitalRead(RM_ENCODER_A) == HIGH) {
    if (digitalRead(RM_ENCODER_B) == LOW) {
      RMCount++;
    } else {
      RMCount--;
    }
  } else {
    if (digitalRead(RM_ENCODER_B) == LOW) {
      RMCount--;
    } else {
      RMCount++;
    }
  }
}

void LMEncoderEvent() {
  if (digitalRead(LM_ENCODER_A) == HIGH) {
    if (digitalRead(LM_ENCODER_B) == LOW) {
      LMCount++;
    } else {
      LMCount--;
    }
  } else {
    if (digitalRead(LM_ENCODER_B) == LOW) {
      LMCount--;
    } else {
      LMCount++;
    }
  }
}

void BMEncoderEvent() {
  if (digitalRead(BM_ENCODER_A) == LOW) {
    if (digitalRead(BM_ENCODER_B) == LOW) {
      BMCount++;
    } else {
      BMCount--;
    }
  } else {
    if (digitalRead(BM_ENCODER_B) == LOW) {
      BMCount--;
    } else {
      BMCount++;
    }
  }
}

void motor_calc() {
  if (RMCount != rm_pulse_prev) {
    rm_speed = (abs(RMCount - rm_pulse_prev) * (717 / 468)) / ((millis() - prev_time) / 1000); //motor speed = 100*inches/sec
    rm_distance += abs(RMCount - rm_pulse_prev);                                              //motor distance = pulses
    rm_pulse_prev = RMCount;
  } else {
    rm_speed = 0;
    rm_distance += 0;
  }
  if (LMCount != lm_pulse_prev) {
    lm_speed = (abs(LMCount - lm_pulse_prev) * (717 / 468)) / ((millis() - prev_time) / 1000);
    lm_distance += abs(LMCount - lm_pulse_prev);
    lm_pulse_prev = LMCount;
  } else {
    lm_speed = 0;
    lm_distance += 0;
  }
  if (BMCount != bm_pulse_prev) {
    bm_speed = (abs(BMCount - bm_pulse_prev) * (717 / 590)) / ((millis() - prev_time) / 1000);
    bm_distance += abs(BMCount - bm_pulse_prev);
    bm_pulse_prev = BMCount;
  } else {
    bm_speed = 0;
    bm_distance += 0;
  }
  prev_time = millis();
}

void ultra_FT() {
  float distance_temp;
  while (distance_FT == 0) {
    digitalWrite(trigPin_FT, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin_FT, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin_FT, LOW);
    duration_FT = pulseIn(echoPin_FT, HIGH);
    distance_temp = (duration_FT * 0.034 / 2 + 7.6) / 2.54;                                    ///tested value 7.6
    if (distance_temp > 150 || duration_FT == 0) {
      distance_FT = distance_FT_prev;
    } else {
      distance_FT = distance_temp;
      distance_FT_prev = distance_temp;
    }
  }
  digitalWrite(trigPin_FT, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin_FT, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin_FT, LOW);
  duration_FT = pulseIn(echoPin_FT, HIGH);
  distance_temp = (duration_FT * 0.034 / 2 + 7.6) / 2.54;                                    ///tested value 7.6
  if (distance_temp > 150 || duration_FT == 0) {
    distance_FT = distance_FT_prev;
  } else {
    distance_FT = distance_temp;
    distance_FT_prev = distance_temp;
  }
}

void ultra_FB() {
  float distance_temp;
  while (distance_FB == 0) {
    digitalWrite(trigPin_FB, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin_FB, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin_FB, LOW);
    duration_FB = pulseIn(echoPin_FB, HIGH);
    distance_temp = (duration_FB * 0.034 / 2 + 6.5) / 2.54;                                 ///tested value 6.5
    if (distance_temp > 150 || duration_FB == 0) {
      distance_FB = distance_FB_prev;
    } else {
      distance_FB = distance_temp;
      distance_FB_prev = distance_temp;
    }
  }
  digitalWrite(trigPin_FB, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin_FB, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin_FB, LOW);
  duration_FB = pulseIn(echoPin_FB, HIGH);
  distance_temp = (duration_FB * 0.034 / 2 + 6.5) / 2.54;                                    ///tested value 6.5
  if (distance_temp > 150 || duration_FB == 0) {
    distance_FB = distance_FB_prev;
  } else {
    distance_FB = distance_temp;
    distance_FB_prev = distance_temp;
  }
}

void ultra_LF() {
  float distance_temp;
  while (distance_LF == 0) {
    digitalWrite(trigPin_LF, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin_LF, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin_LF, LOW);
    duration_LF = pulseIn(echoPin_LF, HIGH);
    distance_temp = (duration_LF * 0.034 / 2 + 8.5) / 2.54;                                     ///tested value 8.6
    if (distance_temp > 150 || duration_LF == 0) {
      distance_LF = distance_LF_prev;
    } else {
      distance_LF = distance_temp;
      distance_LF_prev = distance_temp;
    }
  }
  digitalWrite(trigPin_LF, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin_LF, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin_LF, LOW);
  duration_LF = pulseIn(echoPin_LF, HIGH);
  distance_temp = (duration_LF * 0.034 / 2 + 8.5) / 2.54;                                         ///tested value 8.6
  if (distance_temp > 150 || duration_LF == 0) {
    distance_LF = distance_LF_prev;
  } else {
    distance_LF = distance_temp;
    distance_LF_prev = distance_temp;
  }
}

void ultra_LB() {
  float distance_temp;
  while (distance_LB == 0) {
    digitalWrite(trigPin_LB, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin_LB, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin_LB, LOW);
    duration_LB = pulseIn(echoPin_LB, HIGH);
    distance_temp = (duration_LB * 0.034 / 2 + 9.9) / 2.54;                                     ///tested value 9.9
    if (distance_temp > 150 || duration_LB == 0) {
      distance_LB = distance_LB_prev;
    } else {
      distance_LB = distance_temp;
      distance_LB_prev = distance_temp;
    }
  }
  digitalWrite(trigPin_LB, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin_LB, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin_LB, LOW);
  duration_LB = pulseIn(echoPin_LB, HIGH);
  distance_temp = (duration_LB * 0.034 / 2 + 9.9) / 2.54;                                     ///tested value 9.9
  if (distance_temp > 150 || duration_LB == 0) {
    distance_LB = distance_LB_prev;
  } else {
    distance_LB = distance_temp;
    distance_LB_prev = distance_temp;
  }
}

void ultra_RF() {
  float distance_temp;
  while (distance_RF == 0) {
    digitalWrite(trigPin_RF, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin_RF, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin_RF, LOW);
    duration_RF = pulseIn(echoPin_RF, HIGH);
    distance_temp = (duration_RF * 0.034 / 2 + 8.8) / 2.54;                                       ///tested value 8.8
    if (distance_temp > 150 || duration_RF == 0) {
      distance_RF = distance_RF_prev;
    } else {
      distance_RF = distance_temp;
      distance_RF_prev = distance_temp;
    }
  }
  digitalWrite(trigPin_RF, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin_RF, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin_RF, LOW);
  duration_RF = pulseIn(echoPin_RF, HIGH);
  distance_temp = (duration_RF * 0.034 / 2 + 8.8) / 2.54;                                       ///tested value 8.8
  if (distance_temp > 150 || duration_RF == 0) {
    distance_RF = distance_RF_prev;
  } else {
    distance_RF = distance_temp;
    distance_RF_prev = distance_temp;
  }
}

void ultra_RB() {
  float distance_temp;
  while (distance_RB == 0) {
    digitalWrite(trigPin_RB, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin_RB, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin_RB, LOW);
    duration_RB = pulseIn(echoPin_RB, HIGH);
    distance_temp = (duration_RB * 0.034 / 2 + 9.5) / 2.54;                                ///tested value 9.5
    if (distance_temp > 150 || duration_RB == 0) {
      distance_RB = distance_RB_prev;
    } else {
      distance_RB = distance_temp;
      distance_RB_prev = distance_temp;
    }
  }
  digitalWrite(trigPin_RB, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin_RB, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin_RB, LOW);
  duration_RB = pulseIn(echoPin_RB, HIGH);
  distance_temp = (duration_RB * 0.034 / 2 + 9.5) / 2.54;                                   ///tested value 9.5
  if (distance_temp > 150 || duration_RB == 0) {
    distance_RB = distance_RB_prev;
  } else {
    distance_RB = distance_temp;
    distance_RB_prev = distance_temp;
  }
}

void ultra_BK() {
  float distance_temp;
  while (distance_BK == 0) {
    digitalWrite(trigPin_BK, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin_BK, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin_BK, LOW);
    duration_BK = pulseIn(echoPin_BK, HIGH);
    distance_temp = (duration_BK * 0.034 / 2 + 10.5) / 2.54;                                  ///tested value 10.5
    if (distance_temp > 150 || duration_BK == 0 || distance_temp < 5) {
      distance_BK = distance_BK_prev;
    } else {
      distance_BK = distance_temp;
      distance_BK_prev = distance_temp;
    }
  }
  digitalWrite(trigPin_BK, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin_BK, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin_BK, LOW);
  duration_BK = pulseIn(echoPin_BK, HIGH);
  distance_temp = (duration_BK * 0.034 / 2 + 10.5) / 2.54;                                  ///tested value 10.5
  if (distance_temp > 150 || duration_BK == 0 || distance_temp < 5) {
    distance_BK = distance_BK_prev;
  } else {
    distance_BK = distance_temp;
    distance_BK_prev = distance_temp;
  }
}

// maze variables
void move_forward_minor() {
  float lm_start_count, rm_start_count, total_count;
  lm_start_count = LMCount;
  rm_start_count = RMCount;
  total_count = abs(LMCount - lm_start_count) * 0.866 + abs(RMCount - rm_start_count) * 0.866;
  while (total_count < 60) {
    runRM(255, true);                                               //default 255
    runLM(243, false);                                              //default 244
    runBM(0, true);
    total_count = abs(LMCount - lm_start_count) * 0.866 + abs(RMCount - rm_start_count) * 0.866;
  }
  fullstop();
}

void move_rightforward_minor() {
  float rotation_count;
  rotation_count = LMCount + 30;
  while (LMCount < rotation_count) {
    runLM(255, false);                                              //default 255
    runBM(222, true);                                               //default 242
    runRM(0, true);
  }
  fullstop();
}

void move_leftforward_minor() {
  float rotation_count;
  rotation_count = RMCount - 30;
  while (RMCount > rotation_count) {
    runRM(255, true);                                               //default 255
    runBM(205, false);                                              //default 225
    runLM(0, false);
  }
  fullstop();
}

void move_straight() { 
  move_forward_minor();
  ultra_LF();
  ultra_RF();
  while (distance_LF < side_distance) {
    move_rightforward_minor();
    ultra_LF();
  }
  while (distance_RF < side_distance) {
    move_leftforward_minor();
    ultra_RF();
  }
//   if ((rule == true) && (distance_RF < 9) %% (distance_RF > 6)){
//      while (distance_RF > 6){
//            move_rightforward_minor();
//            ultra_LF();
//            ultra_RF();
//            if (distance_LF < side_distance ){
//              break;
//            }
//      }
//  }else if ((rule == false) && (distance_LF < 9) %% (distance_LF > 6)){
//      while (distance_LF > 6){
//            move_leftforward_minor();
//            ultra_LF();
//            ultra_RF();
//            if (distance_LF < side_distance ){
//              break;
//            }
//      }
//  }
}

void rotate_cw_minor() {
  float rotation_count;
  rotation_count = LMCount + 15; //default 15 
  while (LMCount < rotation_count) {
    runRM(255, false);            //default 255
    runLM(255, false);            //default 255
    runBM(255, false);            //default 255
  }
  fullstop();
}

void rotate_ccw_minor() {
  float rotation_count;
  rotation_count = RMCount - 15;  //default 15
  while (RMCount > rotation_count) {
    runRM(255, true);              //default 255
    runLM(255, true);              //default 255
    runBM(255, true);              //default 255
  }
  fullstop();
}

void adjust_heading() {
     //digitalWrite(LED_R, HIGH);
     maze_sensors();
    if ((distance_LF + distance_LB) < 16) {             //default 14
       while ((distance_LF - distance_LB) > 0.35 ) {     //default 0.5
             rotate_ccw_minor();
             ultra_LF();
             ultra_LB();
       }                
       while ((distance_LB - distance_LF) > 0.35 ) {
             rotate_cw_minor();
             ultra_LF();
             ultra_LB();
       }
       while ((distance_LF - distance_LB) > 0.35 ) {     //default 0.5
             rotate_ccw_minor();
             ultra_LF();
             ultra_LB();
       } 
//       while ((distance_LB - distance_LF) > 0.35 ) {
//             rotate_cw_minor();
//             ultra_LF();
//             ultra_LB();
//       }  
       
    }else if ((distance_RF + distance_RB) < 16) {      //default 14
       while ((distance_RF - distance_RB) > 0.35 ) {    //default 0.5
             rotate_cw_minor();
             ultra_RF();
             ultra_RB();   
       }
       while ((distance_RB - distance_RF) > 0.35 ) {
             rotate_ccw_minor();
             ultra_RF();
             ultra_RB();
       }
       while ((distance_RF - distance_RB) > 0.35 ) {    //default 0.5
             rotate_cw_minor();
             ultra_RF();
             ultra_RB();   
       }
//       while ((distance_RB - distance_RF) > 0.35 ) {
//             rotate_ccw_minor();
//             ultra_RF();
//             ultra_RB();
//       }                                                          
     }else{
       fullstop();
     }
     fullstop();
     //digitalWrite(LED_R, LOW);
}

void move_straight_3in() {
  float lm_start_count, rm_start_count, total_count;
  lm_start_count = LMCount;
  rm_start_count = RMCount;
  total_count = abs(LMCount - lm_start_count) * 0.866 + abs(RMCount - rm_start_count) * 0.866;
  while (total_count < 195) {                                        //default 195
    move_straight();
    total_count = abs(LMCount - lm_start_count) * 0.866 + abs(RMCount - rm_start_count) * 0.866;
  }
  fullstop();
  adjust_heading();
}

void rotate_ccw_90() {
  float rotation_count;
  rotation_count = RMCount - 298;  //default 293
  while (RMCount > rotation_count) {
    runRM(255, true);              //default 255
    runLM(255, true);              //default 255
    runBM(255, true);              //default 255
  }
  fullstop();
  rotation_count = 0; 
}

void rotate_cw_90() {
  float rotation_count;
  rotation_count = LMCount + 288; //default 288
  while (LMCount < rotation_count) {
    runRM(255, false);            //default 255
    runLM(255, false);            //default 255
    runBM(255, false);            //default 255
  }
  fullstop();
  rotation_count = 0;
}

void rotate_cw_180() {
  float rotation_count;
  rotation_count = LMCount + 628; //default 630
  while (LMCount < rotation_count) {
    runRM(255, false);            //default 255
    runLM(255, false);            //default 255
    runBM(255, false);            //default 255
  }
  fullstop();
}

void fake_detect() {
  float rotation_count;
  rotation_count = LMCount + 10; //default 630
  while (LMCount < rotation_count) {
       runRM(255, false);            //default 255
       runLM(255, false);            //default 255
       runBM(255, false);            //default 255
  }
  fullstop();
}

void initialize_orientation() { 
  float max_distance, max_distance_unique;
  float rotation_count;
  bool unique_place = false;
     maze_sensors();
     while ((distance_LF >= 7 || distance_LB >= 7) and (distance_RF >= 7 || distance_RB >= 7)) {   //default 7
           rotate_cw_minor();
           maze_sensors();
//           if (distance_LF > 18 && distance_LB > 18 && distance_RF > 18 && distance_RB > 18 && distance_FT > 18 && distance_BK > 18){
//               unique_place = true;
//               digitalWrite(LED_B, HIGH);
//               break;
//           }
     }
     //digitalWrite(LED_Y, HIGH);
     fullstop();
     adjust_heading();
     maze_sensors();
     max_distance = max(max(max(distance_LF, distance_LB), max(distance_RF, distance_RB)), max(distance_FT, distance_BK));
     if (unique_place == true){
//              max_distance_unique = max_distance;
//              rotation_count = LMCount + 1400;                 //default 1400
//              while (LMCount < rotation_count) {
//                rotate_cw_minor();
//                maze_sensors();
//                max_distance = max(max(max(distance_LF, distance_LB), max(distance_RF, distance_RB)), max(distance_FT, distance_BK));
//                if (max_distance > max_distance_unique){
//                    max_distance_unique = max_distance;
//                }
//              }
//              maze_sensors();
//              while (distance_FT < (max_distance_unique - 0.5)){
//                 rotate_ccw_minor();
//              }         
     }
     else if (max_distance == distance_FT) {
        fullstop();
     }
     else if (max_distance == distance_LF || max_distance == distance_LB) {
        rotate_ccw_90();
     }
     else if (max_distance == distance_RF || max_distance == distance_RB) {
        rotate_cw_90();
     }
     else {
        rotate_cw_180();
     }
     fullstop();
     adjust_heading();
     //digitalWrite(LED_Y, LOW);
     //digitalWrite(LED_B, LOW);
}

int pocket_counter;
void maze_LHR() {
  wall_condition();
  if (distance_LF < 11) {
    if (distance_FT < 7.5) {
      if (distance_RF < 11) {
        movement = 'd';
        record_position();
        rotate_cw_180();
        movement = 'd';
        rule = true;
        pocket_counter += 1;
      } else {
        rotate_cw_90();
        movement = 'd';
      }
    } else {
      move_straight_3in();
      movement = 'w';
    }
  } else {
    while (distance_LB < 11) {
      move_straight_3in();
      movement = 'w';
      adjust_heading();
      wall_condition();
      record_position();
    }
    record_position();
    move_forward_minor();
    fullstop();
    delay(250);
    //move_straight_1in();
    //movement = 'w';
    //adjust_heading();
    //wall_condition();
    //record_position();
    rotate_ccw_90();
    movement = 'a';
    adjust_heading();
    wall_condition();
    record_position();
    while (distance_LF > 11) {
      move_straight_3in();
      movement = 'w';
      adjust_heading();
      wall_condition();
      record_position();
    }
  }
}

void maze_RHR() {
  wall_condition();
  if (distance_RF < 11) {
    if (distance_FT < 7.5) {
      if (distance_LF < 11) {
        movement = 'a';
        record_position();
        rotate_cw_180();
        movement = 'a';
        rule = true;
        pocket_counter += 1;
      } else {
        rotate_ccw_90();
        movement = 'a';
      }
    } else {
      move_straight_3in();
      movement = 'w';
    }
  } else {
    while (distance_RB < 11) {
      move_straight_3in();
      movement = 'w';
      adjust_heading();
      wall_condition();
      record_position();
    }
    record_position();
    move_forward_minor();
    fullstop();
    delay(250);
    //move_straight_1in();
    //movement = 'w';
    //adjust_heading();
    //wall_condition();
    //record_position();
    rotate_cw_90();
    movement = 'd';
    adjust_heading();
    wall_condition();
    record_position();
    while  (distance_RF > 11) {
      move_straight_3in();
      movement = 'w';
      adjust_heading();
      wall_condition();
      record_position();
    }
  }
}

void run_maze () {
  initialize_orientation();
  initialize_orientation();
  wall_condition();
  record_position();
  if (wall_state != 3) {
    while (distance_LF > 11) {
      move_straight_3in();
      movement = 'w';
      adjust_heading();
      wall_condition();
      record_position();
    }
    record_position();
    while (LZ != true) {
      //digitalWrite(LED_R, HIGH);
      maze_LHR();
      adjust_heading();
      wall_condition();
      record_position();
      if (rule == true) {
        break;
      }
    }
    while (LZ != true) {
      //digitalWrite(LED_Y, HIGH);
      maze_RHR();
      adjust_heading();
      wall_condition();
      record_position();
    }
  } else {
    rule = true;
    while (LZ != true) {
      //digitalWrite(LED_B, HIGH);
      maze_RHR();
      adjust_heading();
      wall_condition();
      record_position();
    }
  }
}

void open_grip() {
  SERVO_GRIP.write(150);                                                             //test value
}

void close_grip() {
  SERVO_GRIP.write(75);                                                              //test value
}

void lift_grip() {
  SERVO_ARM.write(150);                                                              //test value
}

void drop_grip() {
  SERVO_ARM.write(32);                                                               //test value
}

void block_detection(){
     ultra_FT();
     while (distance_FT > 10){
            move_forward_minor(); 
            ultra_FT();
     }  
     open_grip();
     delay(1000);
     drop_grip();
     delay(5000);
     move_forward_minor(); 
     close_grip();
     delay(1000);
    open_grip();
    delay(5000);
    close_grip();
    delay(1000);
    lift_grip();
    delay(1000);
}

int end_command = 666;           // 999 when l/z arrived
void record_position() {
  if (Serial.available() > 0) {
    state = Serial.read();
  }
  if (state == 'e') {
    LZ = true;
    end_command = 999;
    state = 0;
  } else {
    state = 0;
  }
  delay(1000);
  Serial.print(millis());
  Serial.print(".");
  Serial.print(wall_state);  //number
  Serial.print(":");
  Serial.print(movement);    //string
  Serial.print(";");
  Serial.print(end_command);    //string
  Serial.println("!");
}
