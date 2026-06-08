#include "Motion_Task.h"
#include "remote_control.h"
#include "chassis.h"
#include "cmsis_os.h"

Motion_mode motion_mode;
Control_mode control_mode;
Remote_mode remote_mode;

//void Motion_Task(void const * argument)
//{

//  for(;;)
//  {
//      if(RCctrl.CH6 < 1000)
//      {
//          control_mode = master_control;
//          remote_mode = remote_none;
//      }
//      else
//      {
//          Chassis.Chassis_Calc(&Chassis);
//          control_mode = remote_control;
//          if(RCctrl.CH5 < 800)
//          {
//              remote_mode = chassis_move;
//              motion_mode = motion_none;
//          }
//          else if(RCctrl.CH5 > 800 && RCctrl.CH5 < 1200)
//          {
//              remote_mode = motion_switch;
//              if(RCctrl.CH7 < CH7_MID - 100)
//              {
//                  motion_mode = weapon_collect;
//              }
//              else if((RCctrl.CH7 > CH7_MID - 100) && (RCctrl.CH7 < CH7_MID + 100))
//              {
//                  motion_mode = kfs_collect;
//              }
//              else
//              {
//                  motion_mode = R2_lift;
//              }
//          }
//          else
//          {
//              remote_mode = cowork;
//              motion_mode = motion_none;
//          }
//      }
      
      
      
      
//      
//      
//    osDelay(1);
//  }

//}

