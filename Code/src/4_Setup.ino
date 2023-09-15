void setup() {
  pinMode(RM_DIREC, OUTPUT);
  pinMode(RM_BRAKE, OUTPUT);
  pinMode(LM_DIREC, OUTPUT);
  pinMode(LM_BRAKE, OUTPUT);
  pinMode(BM_DIREC, OUTPUT);
  pinMode(BM_BRAKE, OUTPUT);
  pinMode(RM_ENCODER_A, INPUT);
  pinMode(RM_ENCODER_B, INPUT);
  pinMode(LM_ENCODER_A, INPUT);
  pinMode(LM_ENCODER_B, INPUT);
  pinMode(BM_ENCODER_A, INPUT);
  pinMode(BM_ENCODER_B, INPUT);
  pinMode(trigPin_FT, OUTPUT);
  pinMode(trigPin_FB, OUTPUT);
  pinMode(trigPin_LF, OUTPUT);
  pinMode(trigPin_RF, OUTPUT);
  pinMode(trigPin_LB, OUTPUT);
  pinMode(trigPin_RB, OUTPUT);
  pinMode(trigPin_BK, OUTPUT);
  pinMode(echoPin_FT, INPUT);
  pinMode(echoPin_FB, INPUT);
  pinMode(echoPin_LF, INPUT);
  pinMode(echoPin_RF, INPUT);
  pinMode(echoPin_LB, INPUT);
  pinMode(echoPin_RB, INPUT);
  pinMode(echoPin_BK, INPUT);
  pinMode(LED_R, OUTPUT);
  pinMode(LED_Y, OUTPUT);
  pinMode(LED_B, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(RM_ENCODER_A), RMEncoderEvent, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LM_ENCODER_A), LMEncoderEvent, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BM_ENCODER_A), BMEncoderEvent, CHANGE);

  SERVO_ARM.attach(SERVO_A);
  SERVO_GRIP.attach(SERVO_G);
  
  digitalWrite(LED_R, LOW);
  digitalWrite(LED_Y, LOW);
  digitalWrite(LED_B, LOW);

  Serial.begin(9600);

  motor_calc();
  delay(10);
  motor_calc();
  rm_distance = 0;
  lm_distance = 0;
  bm_distance = 0;

  BT_comm();          // matlab via bluetooth communication
  close_grip();
  lift_grip();
}
