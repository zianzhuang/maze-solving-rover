//unloaded == false;
//void unloading_A(){
//     maze_sensors();
//     pocket_counter = 0;
//     while (unloaded == false){
//            if (distance_LF < 11){
//               while (pocket_counter < 1){
//                     maze_LHR();
//               }
//               drop_grip();
//               open_grip();
//               fullstop();
//               unloaded == true;
//            }else if (distance_RF < 11){
//               rotate_cw_180();
//               while (pocket_counter < 1){
//                     maze_LHR();
//               }
//               drop_grip();
//               open_grip();
//               fullstop();
//               unloaded == true;
//            }else{
//               move_forward_minor();
//               maze_sensors();
//            }
//     }
//}
//
//void unloading_B(){
//     maze_sensors();
//     pocket_counter = 0;
//     while (unloaded == false){
//            if (distance_LF < 11){
//               while (pocket_counter < 2){
//                     maze_LHR();
//               }
//               drop_grip();
//               open_grip();
//               fullstop();
//               unloaded == true;
//            }else if (distance_RF < 11){
//               rotate_cw_180();
//               while (pocket_counter < 2){
//                     maze_LHR();
//               }
//               drop_grip();
//               open_grip();
//               fullstop();
//               unloaded == true;
//            }else{
//               move_forward_minor();
//               maze_sensors();
//            }
//     }
//} 
//
//void unloading_C(){
//     maze_sensors();
//     pocket_counter = 0;
//     while (unloaded == false){
//            if (distance_LF < 11){
//               while (pocket_counter < 3){
//                     maze_LHR();
//               }
//               drop_grip();
//               open_grip();
//               fullstop();
//               unloaded == true;
//            }else if (distance_RF < 11){
//               rotate_cw_180();
//               while (pocket_counter < 3){
//                     maze_LHR();
//               }
//               drop_grip();
//               open_grip();
//               fullstop();
//               unloaded == true;
//            }else{
//               move_forward_minor();
//               maze_sensors();
//            }
//     }
//}
