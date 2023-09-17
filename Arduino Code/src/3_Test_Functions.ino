//////////////////////////////////Test Functions//////////////////////////////////////////
//void move_straight_1in() {
//  float lm_start_count, rm_start_count, total_count;
//  lm_start_count = LMCount;
//  rm_start_count = RMCount;
//  total_count = abs(LMCount - lm_start_count) * 0.866 + abs(RMCount - rm_start_count) * 0.866;
//  while (total_count < 50) {                                        //default 195
//    move_forward_minor();
//    total_count = abs(LMCount - lm_start_count) * 0.866 + abs(RMCount - rm_start_count) * 0.866;
//  }
//  fullstop();
//  adjust_heading();
//}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void unloading_A() {
  // Location: top left corner of LZ
  // Facing EAST
  int i = 1;
  int j = 1;
  int k = 1;
  int a = 1;
  int b = 1;
  ultra_FT();
  while (i < 13 || distance_FT > 8) {
    move_straight_3in();
    delay(500);
    i += 1;
    ultra_FT();
  }
  rotate_cw_90();
  ultra_FT();
  while (j < 5 || distance_FT > 8) {
    move_straight_3in();
    delay(500);
    j += 1;
    ultra_FT();
  }
  rotate_ccw_90();
  while (k < 9) {
    move_straight_3in();
    delay(500);
    k += 1;
  }
  rotate_ccw_90();
  move_straight_3in();
  delay(1000);
  move_straight_3in();
  delay(1000);
  move_straight_3in();
  delay(500);
  drop_grip();
  open_grip();
  fullstop();
}

void unloading_B() {
  // Location: top left corner of LZ
  // Facing EAST
  int i = 1;
  int j = 1;
  int k = 1;
  int a = 1;
  int b = 1;
  ultra_FT();
  while (i < 14 || distance_FT > 8) {
    move_straight_3in();
    delay(500);
    i += 1;
    ultra_FT();
  }
  rotate_cw_90();
  ultra_FT();
  while (j < 5 || distance_FT > 8) {
    move_straight_3in();
    delay(500);
    j += 1;
    ultra_FT();
  }
  rotate_ccw_90();
  while (k < 18 || distance_FT > 9) {
    move_straight_3in();
    delay(500);
    k += 1;
  }
  rotate_ccw_90();
  move_straight_3in();
  delay(1000);
  move_straight_3in();
  drop_grip();
  open_grip();
  fullstop();
}

void unloading_C() {
  // Location: top left corner of LZ
  // Facing EAST
  int i = 1;
  int j = 1;
  int k = 1;
  int a = 1;
  int b = 1;
  ultra_FT();
  while (i < 14 || distance_FT > 8) {
    move_straight_3in();
    delay(500);
    i += 1;
    ultra_FT();
  }
  rotate_cw_90();
  ultra_FT();
  while (j < 5 || distance_FT > 8) {
    move_straight_3in();
    delay(500);
    j += 1;
    ultra_FT();
  }
  rotate_ccw_90();
  ultra_FT();
  while (k < 18 || distance_FT > 9) {
    move_straight_3in();
    delay(500);
    k += 1;
    ultra_FT();
  }
  rotate_cw_90();
  move_straight_3in();
  delay(1000);
  move_straight_3in();
  delay(1000);
  move_straight_3in();
  delay(1000);
  move_straight_3in();
  delay(1000);
  move_straight_3in();
  delay(1000);
  move_straight_3in();
  delay(1000);
  drop_grip();
  open_grip();
  fullstop();
}

void unloading_D() {
  // Location: top left corner of LZ
  // Facing EAST
  int i = 1;
  int j = 1;
  int k = 1;
  int a = 1;
  int b = 1;
  rotate_cw_90();
  ultra_FT();
  while (i < 14) {
    move_straight_3in();
    delay(500);
    i += 1;
    ultra_FT();
  }
  rotate_ccw_90();
  //ultra_FT();
  while (j < 9) {
    move_straight_3in();
    delay(500);
    j += 1;
    //ultra_FT();
  }
  rotate_ccw_90();
  move_straight_3in();
  delay(1000);
  move_straight_3in();
  delay(1000);
  drop_grip();
  delay(1000);
  open_grip();
  delay(1000);
  fullstop();
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void rotate_ccw_45() {
  float rotation_count;
  rotation_count = BMCount - 155;  //default 380                    //test value 135 deg 610
  while (BMCount > rotation_count) {
    rotate_ccw_minor();
  }
  fullstop();
}
void rotate_cw_45() {
  float rotation_count;
  rotation_count = BMCount + 155; //default 371                     //test value 135 deg 610
  while (BMCount < rotation_count) {
    rotate_cw_minor();
  }
  fullstop();
}

//bool block_detected = false;
//void block_detection() {                           // find block until found --> block_deteced = true
//  float rotation_count;
//  ultra_RF();
//  ultra_LF();
//  ultra_LB();
//  ultra_RB();
//  ultra_FT();
//  while (block_detected == false) {
//    maze_sensors();
//    if (distance_LF < 7 && distance_LB < 7 && distance_RF < 20 && distance_RB < 20) {
//      adjust_heading();
//      rotate_cw_45();
//      delay(1000);
//      rotation_count = BMCount - 450;                          // test value rotate 90 de
//      while (BMCount > rotation_count) {
//
//        ultra_FT();
//
//        ultra_FB();
//
//        if ((distance_FT - distance_FB) > 3) {               // test value
//
//          fullstop();
//          block_detected = true;
//          //  block_pickup();
//          break;
//        }
//        Serial.println(distance_FT);
//        Serial.println(distance_FB);
//        rotate_ccw_minor();
//        delay(1000);
//      }
//      rotate_cw_90();
//      delay(1000);
//      rotate_cw_45();
//      delay(1000);
//
//
//
//
//      while (distance_FT > 8) {
//        rotate_ccw_45();
//        delay(1000);
//        rotation_count = BMCount + 450;                          // test value rotate 90 de
//        while (BMCount < rotation_count) {
//
//          ultra_FT();
//
//          ultra_FB();
//
//          if ((distance_FT - distance_FB) > 3) {               // test value
//
//            fullstop();
//            block_detected = true;
//            //  block_pickup();
//            break;
//          }
//          Serial.println(distance_FT);
//          Serial.println(distance_FB);
//          rotate_cw_minor();
//          delay(1000);
//        }
//        rotate_ccw_45();
//        if (distance_FT > 8) {
//          move_straight_3in();
//          move_straight_3in();
//          adjust_heading();
//        }
//      }
//
//      rotate_cw_90();
//      adjust_heading();
//
//      while (distance_FT > 8) {
//        move_straight_3in();
//        move_straight_3in();
//        adjust_heading();
//        rotate_cw_45();
//        rotation_count = BMCount + 450;                          // test value rotate 90 de
//        while (BMCount < rotation_count) {
//
//          ultra_FT();
//
//          ultra_FB();
//
//          if ((distance_FT - distance_FB) > 3) {               // test value
//
//            fullstop();
//            block_detected = true;
//            //  block_pickup();
//            break;
//          }
//          Serial.println(distance_FT);
//          Serial.println(distance_FB);
//          rotate_cw_minor();
//          delay(1000);
//        }
//        rotate_ccw_45();
//        if (distance_FT > 8) {
//          move_straight_3in();
//          move_straight_3in();
//          adjust_heading();
//        }
//      }
//
//      rotate_ccw_90();
//      adjust_heading();
//
//
//      while (distance_FT > 8) {
//        rotate_cw_45();
//        rotation_count = BMCount + 450;                          // test value rotate 90 de
//        while (BMCount < rotation_count) {
//
//          ultra_FT();
//
//          ultra_FB();
//
//          if ((distance_FT - distance_FB) > 3) {               // test value
//
//            fullstop();
//            block_detected = true;
//            break;
//          }
//          Serial.println(distance_FT);
//          Serial.println(distance_FB);
//          rotate_cw_minor();
//          delay(2000);
//        }
//        rotate_ccw_45();
//        if (distance_FT > 8) {
//          move_straight_3in();
//          move_straight_3in();
//        }
//
//      }
//
//
//      rotate_cw_90();
//      adjust_heading();
//      while (distance_FT < 8) {
//        rotate_cw_45();
//        rotation_count = BMCount + 450;                          // test value rotate 90 de
//        while (BMCount < rotation_count) {
//
//          ultra_FT();
//
//          ultra_FB();
//
//          if ((distance_FT - distance_FB) > 3) {               // test value
//
//            fullstop();
//            block_detected = true;
//            break;
//          }
//          Serial.println(distance_FT);
//          Serial.println(distance_FB);
//          rotate_cw_minor();
//          delay(2000);
//        }
//        rotate_ccw_45();
//        if (distance_FT < 8) {
//          move_straight_3in();
//          move_straight_3in();
//        }
//
//      }
//    }
//    else if (distance_LF < 20 && distance_LB < 20 && distance_RF < 7 && distance_RF < 7) {
//      adjust_heading();
//      rotate_cw_45();
//      delay(1000);
//      rotation_count = BMCount - 450;                          // test value rotate 90 de
//      while (BMCount > rotation_count) {
//
//        ultra_FT();
//
//        ultra_FB();
//
//        if ((distance_FT - distance_FB) > 3) {               // test value
//
//          fullstop();
//          block_detected = true;
//          break;
//        }
//        Serial.println(distance_FT);
//        Serial.println(distance_FB);
//        rotate_ccw_minor();
//        delay(1000);
//      }
//      rotate_ccw_45();
//      delay(1000);
//
//
//
//
//      while (distance_FT > 8) {
//        rotate_ccw_45();
//        delay(1000);
//        rotation_count = BMCount + 450;                          // test value rotate 90 de
//        while (BMCount < rotation_count) {
//
//          ultra_FT();
//
//          ultra_FB();
//
//          if ((distance_FT - distance_FB) > 3) {               // test value
//
//            fullstop();
//            block_detected = true;
//            break;
//          }
//          Serial.println(distance_FT);
//          Serial.println(distance_FB);
//          rotate_cw_minor();
//          delay(1000);
//        }
//        rotate_ccw_45();
//        if (distance_FT > 8) {
//          move_straight_3in();
//          move_straight_3in();
//          adjust_heading();
//        }
//      }
//
//      rotate_cw_90();
//      adjust_heading();
//
//      while (distance_FT > 8) {
//        move_straight_3in();
//        move_straight_3in();
//        adjust_heading();
//        rotate_cw_45();
//        rotation_count = BMCount + 450;                          // test value rotate 90 de
//        while (BMCount < rotation_count) {
//
//          ultra_FT();
//
//          ultra_FB();
//
//          if ((distance_FT - distance_FB) > 3) {               // test value
//
//            fullstop();
//            block_detected = true;
//            break;
//          }
//          Serial.println(distance_FT);
//          Serial.println(distance_FB);
//          rotate_cw_minor();
//          delay(1000);
//        }
//        rotate_ccw_45();
//        if (distance_FT > 8) {
//          move_straight_3in();
//          move_straight_3in();
//          adjust_heading();
//        }
//      }
//
//      rotate_ccw_90();
//      adjust_heading();
//
//
//      while (distance_FT > 8) {
//        rotate_cw_45();
//        rotation_count = BMCount + 450;                          // test value rotate 90 de
//        while (BMCount < rotation_count) {
//
//          ultra_FT();
//
//          ultra_FB();
//
//          if ((distance_FT - distance_FB) > 3) {               // test value
//
//            fullstop();
//            block_detected = true;
//            break;
//          }
//          Serial.println(distance_FT);
//          Serial.println(distance_FB);
//          rotate_cw_minor();
//          delay(2000);
//        }
//        rotate_ccw_45();
//        if (distance_FT > 8) {
//          move_straight_3in();
//          move_straight_3in();
//        }
//
//      }
//
//
//      rotate_cw_90();
//      adjust_heading();
//      while (distance_FT < 8) {
//        rotate_cw_45();
//        rotation_count = BMCount + 450;                          // test value rotate 90 de
//        while (BMCount < rotation_count) {
//
//          ultra_FT();
//
//          ultra_FB();
//
//          if ((distance_FT - distance_FB) > 3) {               // test value
//
//            fullstop();
//            block_detected = true;
//            break;
//          }
//          Serial.println(distance_FT);
//          Serial.println(distance_FB);
//          rotate_cw_minor();
//          delay(2000);
//        }
//        rotate_ccw_45();
//        if (distance_FT < 8) {
//          move_straight_3in();
//          move_straight_3in();
//        }
//
//      }
//    }
//  }
//}
//bool block_exist = false;
//bool pickup_ready = false;
//
//void block_pickup () {                                 // no block is in the maze
//
//  float pickup_distance = 8;                                          //test value
//
//  while (pickup_ready == false) {
//
//    while (block_detected == true) {
//
//      ultra_FB();
//
//      ultra_FT();
//
//      if (distance_FB <= pickup_distance) {
//
//        pickup_ready = true;
//
//        break;
//
//      }
//
//      if ((distance_FT - distance_FB) < 4.5) {         //test value
//
//        block_detected = false;
//
//      }
//
//      move_forward_minor();
//
//    }
//
//    block_detection();
//
//  }
//
//  open_grip();
//
//  delay(1000);
//
//  drop_grip();
//
//  delay(1000);
//
//  move_forward_minor();   // test value
//
//  move_forward_minor();
//
//  move_forward_minor();
//
//  close_grip();
//
//  delay(1000);
//
//  lift_grip();
//
//  delay(1000);
//
//  block_exist = false;
//
//}

///////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////LOCALIZATION//////////////////////////////////////////////////////////////////////
float wall_distance = 7.5;                                                               //test value for wall condition, not for maze run
int FT, BK, LF, RF, LB, RB, L, R;
bool left_side, right_side;
void maze_sensors() {
  ultra_FT();
  ultra_LF();
  ultra_LB();
  ultra_RF();
  ultra_RB();
  ultra_BK();
  if (distance_FT < wall_distance) {             // wall detected?
    FT = 1;
  } else {
    FT = 0;
  }
  if (distance_BK < wall_distance) {             // wall detected?
    BK = 1;
  } else {
    BK = 0;
  }
  if (distance_LB < wall_distance) {             // wall detected?
    LB = 1;
  } else {
    LB = 0;
  }
  if (distance_LF < wall_distance) {             // wall detected?
    LF = 1;
  } else {
    LF = 0;
  }
  if (distance_RB < wall_distance) {             // wall detected?
    RB = 1;
  } else {
    RB = 0;
  }
  if (distance_RF < wall_distance) {             // wall detected?
    RF = 1;
  } else {
    RF = 0;
  }

  if (LF == LB) {                                // side equal?
    L = LB;
    left_side = true;                          // full left side
  } else {
    left_side = false;
  }

  if (RF == RB) {
    R = RB;
    right_side = true;                         // full right side
  } else {
    right_side = false;
  }
}
//test wall zone area
void wall_condition() {                                   // analyze wall state
  int sum_side;
  maze_sensors();
  if (FT + BK + LF + LB + RF + RB > 4) {
    wall_state = 3;                                 // 3 blocking (pocket)
  } else if (right_side == true && left_side == true) { // at center position
    sum_side = FT + BK + L + R;
    if (sum_side == 0) {
      wall_state = 0;                               // 0 blocking
    } else if (sum_side == 1) {
      wall_state = 1;                               // 1 blocking
    } else if (sum_side == 3) {
      wall_state = 3;                               // 3 blocking (pocket)
    } else if (sum_side == 2 && FT == BK) {
      wall_state = 5;                               // 2 blocking (opposite)
    } else if (sum_side == 2 && FT != BK) {
      wall_state = 2;                               // 2 blocking (adjacent)
    } else {
      wall_state = wall_state;
    }
  } else if (right_side == false && left_side == false) {
    if (LB == RB) {
      wall_state = 5;                               // 2 blocking (opposite)
    } else {
      wall_state = 2;                               // 2 blocking (adjacent)
    }
  } else {
    if (right_side == true && R == 1) {
      wall_state = 1;                               // 1 blocking
    } else if (left_side == true && L == 1) {
      wall_state = 1;                               // 1 blocking
    } else {
      wall_state = 0;                               // 0 blocking
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
    if (Serial.available() > 0) {
        state = Serial.read();
       }
    if (state == 'l') {
       run_maze ();
       digitalWrite(LED_R, HIGH);               //R - Localization Done !
       delay(1000);
       digitalWrite(LED_R, LOW);               //R - Localization Done !
       fake_detect();
       digitalWrite(LED_Y, HIGH);               //Y - Detection Done !
       delay(1000);
       digitalWrite(LED_Y, LOW);               //R - Localization Done !
       delay(1000);
       block_detection();
       maze_sensors();
       while  (distance_FT > 9.5){
            move_forward_minor(); 
            ultra_FT();
       }    
       if ((distance_LF < 11) && (distance_LB < 11)){
            rotate_cw_90();
            delay(4000);
            unloading_D();
       }
       else if ((distance_RF < 11) && (distance_RB < 11)){
            rotate_cw_180();
            delay(4000);
            unloading_D();
       }
       digitalWrite(LED_B, HIGH);                //B -Delivery Done !
       state = 0;
    } else if (state == 'm') {
       digitalWrite(LED_R, HIGH);               //R - Localization Done !
       delay(1000);
       digitalWrite(LED_R, LOW);               //R - Localization Done !
       fake_detect();
       digitalWrite(LED_Y, HIGH);               //Y - Detection Done !
       delay(1000);
       digitalWrite(LED_Y, LOW);               //R - Localization Done !
       delay(1000);
       block_detection();
       maze_sensors();
       while  (distance_FT > 9.5){
            move_forward_minor(); 
            ultra_FT();
       }    
       if ((distance_LF < 11) && (distance_LB < 11)){
            rotate_cw_90();
            delay(4000);
            unloading_D();
       }
       else if ((distance_RF < 11) && (distance_RB < 11)){
            rotate_cw_180();
            delay(4000);
            unloading_D();
       }
       digitalWrite(LED_B, HIGH);                //B -Delivery Done !
       state = 0;
    } else if (state == 'n') {
       digitalWrite(LED_R, HIGH);               //R - Localization Done !
       digitalWrite(LED_Y, HIGH);               //Y - Detection Done !
       open_grip();
       delay(1000);
       drop_grip();
       delay(3000);
       close_grip();
       delay(1000);
       lift_grip();
       delay(1000);
       maze_sensors();
       while  (distance_FT > 9.5){
            move_forward_minor(); 
            ultra_FT();
       }    
       if ((distance_LF < 11) && (distance_LB < 11)){
            rotate_cw_90();
            delay(4000);
            unloading_D();
       }
       else if ((distance_RF < 11) && (distance_RB < 11)){
            rotate_cw_180();
            delay(4000);
            unloading_D();
       }
       digitalWrite(LED_B, HIGH);               //B -Delivery Done !
       state = 0;
    } else {
       state = 0;
    }
}
