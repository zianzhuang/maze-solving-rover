#include <SoftwareSerial.h>
SoftwareSerial BT(16, 17);

int state = 0;

void loop() {
     if (BT.available() > 0 ){
        state = BT.read();
     }
     
     if (state == 0){
        fullstop();
        state = 0;
     }
     else if (state == 'w'){
        move_straight_3in();
        state = 0;
     }
     else if (state == 's'){
        move_back_3in();
        state = 0;
     }
     else if (state == 'd'){
        move_right();
        delay(100);
        fullstop();
        state = 0;
     }
     else if (state == 'a'){
        move_left();
        delay(100);
        fullstop();
        state = 0;
     }
     else if (state == 'q'){
        rotation(true);
        delay(50);
        fullstop();
        state = 0;
     }  
     else if (state == 'e'){
        rotation(false);
        delay(50);
        fullstop();
        state = 0;
     }  
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void block_up(){
     //gripper open()
     //arm down()
     //gripper close()
     //arm up()
}

void block_down(){
    //arm down()
    //gripper open()
    //arm up()
    //gripper close()
}
