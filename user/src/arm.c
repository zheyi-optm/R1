#include "arm.h"

    Arm_Info_TypeDef arm;
    ArmAnglesTypeDef angles;
    ArmTorqueTypeDef Torque;

/**
 * @brief 计算机械臂的三个角度参数.
 * @param arm:  指向一个包含模式、目标水平距离和目标垂直高度的 Arm_Info_TypeDef 结构体的指针
 * @retval ArmAnglesTypeDef：包含三个角度的结构体
 * @author  wuzhuohan
 */
ArmAnglesTypeDef Arm_Inverse_Solution(Arm_Info_TypeDef *arm)
{
    ArmAnglesTypeDef angles;
    float theta1=0.0f;
    float theta2=0.0f;
    float theta3=0.0f;
    float x=0.0f;
    float y=0.0f;
    float L_0=0.0f;
    float s=arm->s;
    float h=arm->h;
    switch(arm->mode)
    {
        /*0--从梅林取kfs(低-30cm) 1--捡kfs  2--夹weapon  3--将KFS悬空回收（给定值） 4--将weapon放到车上（给定值）
          5--从梅林取kfs(中-55cm) 6--从梅林取kfs(高75cm) 7--将kfs放在架子第一层*/
        case 0: //控s
        case 5:
        case 6:
        case 7:
        theta1=PI/2.0f-acosf((L_1*L_1+h*h+(s-L_3)*(s-L_3)-L_2*L_2)/
               (2.0f*L_1*sqrtf(h*h+(s-L_3)*(s-L_3))))-atan2f(h,s-L_3);
        theta2=acosf((L_1*L_1+L_2*L_2-h*h-(s-L_3)*(s-L_3))/(2.0f*L_1*L_2)); 
        theta3=PI/2.0f+theta1-theta2;
        break;
        
        case 1://控h
        theta1=PI/2.0f-acosf((L_1*L_1+s*s+(h+L_3)*(h+L_3)-L_2*L_2)/
                   (2.0f*L_1*sqrtf(s*s+(h+L_3)*(h+L_3))))-atan2f(h+L_3,s);
        theta2=acosf((L_1*L_1+L_2*L_2-s*s-(h+L_3)*(h+L_3))/(2.0f*L_1*L_2)); 
        theta3=theta2-theta1;    
        break;
        
        case 2: //控s
        x=s-L_4*cosf(a_weapon);        
        y=h+L_4*sinf(a_weapon);
        L_0=sqrt(x*x+y*y);
        theta1=PI/2.0f-acosf((L_0*L_0+L_1*L_1-L_2*L_2)/(2.0f*L_1*L_0))-atan2f(y,x);
        theta2=acosf((L_2*L_2+L_1*L_1-L_0*L_0)/(2.0f*L_1*L_2));
        theta3=-(theta2-theta1+a_weapon);
        break;
        
        
    }
    angles.theta_1 = theta1;
    angles.theta_2 = theta2;
    angles.theta_3 = theta3;
    
    return angles;
}

/**
 * @brief 根据电机反馈计算机械臂的三个补偿力矩（闭环）.
 * @param arm:  指向一个包含模式、目标水平距离和目标垂直高度的 Arm_Info_TypeDef 结构体的指针
 * @param angle:  指向一个包含电机三个旋转角的 ArmAnglesTypeDef 结构体的指针
 * @retval ArmTorqueTypeDef：包含三个力矩的结构体
 * @author  wuzhuohan
 */
 ArmTorqueTypeDef Torque_Comp_global(DM_MotorModule *arm_1,DM_MotorModule *arm_2,DM_MotorModule *arm_3)
{
    ArmTorqueTypeDef Torque;
    float theta1=arm_1->position;
    float theta2=-arm_2->position;
    float theta3=arm_3->position/25;
    float x_1=0.0f;
    float x_2=0.0f; 
    float x_3=0.0f; 
    float x_4=0.0f;
    float x_5=0.0f;
    
    x_1=L_1*sinf(theta1)+L_c2*sinf(theta2-theta1);
    x_2=L_1*sinf(theta1)+L_2*sinf(theta2-theta1)+L_c3*cosf(theta3+theta2-theta1); 
    x_3=L_1*sinf(theta1)+L_2*sinf(theta2-theta1)+L_c4*sinf(theta3+theta2-theta1);
    x_4=x_2-L_1*sinf(theta1);
    x_5=x_3-L_1*sinf(theta1);
    
    Torque.Torque_1=m_1*gravity*sinf(theta1)*L_c1+m_2*gravity*x_1+m_3*gravity*x_2+m_4*gravity*x_3;
    Torque.Torque_2=m_2*gravity*L_c2*sin(theta2-theta1)+m_4*gravity*x_4+m_3*gravity*x_5;
    Torque.Torque_3=m_4*gravity*L_c4*sinf(theta3+theta2-theta1)+m_3*gravity*L_c3*cosf(theta3+theta2-theta1); 

    return Torque;
}
/*机械臂任务函数*/
HAL_StatusTypeDef Arm_task(DM_MotorModule *arm_1,DM_MotorModule *arm_2,DM_MotorModule *arm_3)
{
            float kfs_1[5]={0,0,0,0,0};//位置,速度,kp,kd,t
            float kfs_2[5]={0,0,0,0,0};
            float kfs_3[5]={0,0,0,0,0};

//          //模式0参数最低kfs抓取（已调）
          if(RCctrl.chassis == 1 && RCctrl.zone == 1 && RCctrl.key ==0)  //
          {
              arm.mode=0;
              arm.s=data_convert(RCctrl.accel, ACCEL_LOW, ACCEL_HIGH, 0.32,0.6);
              arm.h=0.20;
              angles=Arm_Inverse_Solution(&arm);             
              Torque=Torque_Comp_global(arm_1,arm_2,arm_3);
                           
              arm_1->set_mit_data(arm_1,angles.theta_1+0.43f,0.0f,70.0f,5.0f,-0.25*Torque.Torque_1);
              arm_2->set_mit_data(arm_2,-angles.theta_2,0.0f,50.0f,4.5f,-0.6*Torque.Torque_2);
              arm_3->set_mit_data(arm_3,25*(PI/2.0f+arm_1->position-0.43+arm_2->position-PI*0.12f),0.0f,0.2f,0.15f,0.8*Torque.Torque_3);
          }
         //模式1参数 捡kfs（已调）
          else if(RCctrl.chassis == 1 && RCctrl.zone == 2 && RCctrl.bottomPos ==1)   //
          {    
              arm.mode=1;
              arm.s=0.5;
              arm.h=data_convert(RCctrl.accel, ACCEL_LOW, ACCEL_HIGH,-0.05,0.25);
              angles=Arm_Inverse_Solution(&arm);             
              Torque=Torque_Comp_global(arm_1,arm_2,arm_3);
              
              arm_2->set_mit_data(arm_2,-angles.theta_2,0.0f,30.0f,2.5f,-0.8*Torque.Torque_2);
             // osDelay(200);
              arm_3->set_mit_data(arm_3,-25*(angles.theta_3+0.11*PI),0.0f,0.15f,0.1f,0.0f);
              arm_1->set_mit_data(arm_1,angles.theta_1+0.45f,0.0f,40.0f,4.0f,-1.2f*Torque.Torque_1);
          }
//         //模式3参数回收kfs
            else if(RCctrl.chassis == 1 && RCctrl.zone == 1 && RCctrl.key ==3)  //
           {
               Torque=Torque_Comp_global(arm_1,arm_2,arm_3);
              // t_position, *sent_p, c_position, *sent_sp, u_speed, *s_kp, h_kp, *s_kd, h_kd, hold_kd, *s_t, h_t, hold_t ignore_diff
             if(arm_2->position>-1.6f)  
             {                    
                in_place(-1.6f,&kfs_2[0],arm_2->position,&kfs_2[1],0.6f,&kfs_2[2],
                       30.0f,&kfs_2[3],5.0f,4.0f,&kfs_2[4],-0.5f*Torque.Torque_2,-2.0f*Torque.Torque_2,0.0f);
                arm_2->set_mit_data(arm_2,kfs_2[0],kfs_2[1],kfs_2[2],kfs_2[3],kfs_2[4]);
             }
             if(arm_2->position>-2.92f&&arm_2->position<-1.6f)  
             {   
                in_place(0.2f,&kfs_1[0],arm_1->position,&kfs_1[1],2.0f,&kfs_1[2],
                       55.0f,&kfs_1[3],5.0f,5.0f,&kfs_1[4],-0.5f*Torque.Torque_1,-0.5f*Torque.Torque_1,0.3f);
                arm_1->set_mit_data(arm_1,kfs_1[0],kfs_1[1],kfs_1[2],kfs_1[3],kfs_1[4]);                 
                in_place(-2.92f,&kfs_2[0],arm_2->position,&kfs_2[1],0.1f,&kfs_2[2],
                       30.0f,&kfs_2[3],5.0f,4.0f,&kfs_2[4],-0.2f*Torque.Torque_2,-1.8f*Torque.Torque_2,0.0f);
                arm_2->set_mit_data(arm_2,kfs_2[0],kfs_2[1],kfs_2[2],kfs_2[3],kfs_2[4]);
             }
             if(arm_2->position<-2.92f)
             {
                arm_3->set_mit_data(arm_3,-52.0f,0.0f,0.13f,0.6f,1.0*Torque.Torque_3);
                arm_1->set_mit_data(arm_1,0.6f,0.0f,60.0f,5.0f,-0.8f*Torque.Torque_1);
                in_place_1(-4.3f,&kfs_2[0],arm_2->position,&kfs_2[1],-0.0f,&kfs_2[2],
                       65.0f,&kfs_2[3],3.0f,5.0f,&kfs_2[4],-0.12f*Torque.Torque_2,-1.0f*Torque.Torque_2,0.28f);
                arm_2->set_mit_data(arm_2,kfs_2[0],kfs_2[1],kfs_2[2],kfs_2[3],kfs_2[4]);
             }
             if(arm_2->position<-2.8f&&arm_2->position>-2.9f)
             {
                arm_3->set_mit_data(arm_3,-52.0f,0.0f,0.13f,0.6f,1.0*Torque.Torque_3);
                arm_1->set_mit_data(arm_1,0.4f,-0.3f,55.0f,5.0f,-0.8f*Torque.Torque_1);  
             }
           }

					  //模式8参数放回（已调）
           else if(RCctrl.chassis == 1 && RCctrl.zone == 1 && RCctrl.key ==4)  
           {
             Torque=Torque_Comp_global(arm_1,arm_2,arm_3);
             if(arm_2->position<-2.55f)
             {
                arm_3->set_mit_data(arm_3,-32.0f,0.0f,0.13f,0.6f,0.1*Torque.Torque_3);
                arm_1->set_mit_data(arm_1,0.6f,0.0f,55.0f,5.0f,-0.8f*Torque.Torque_1);
                in_place(-2.55f,&kfs_2[0],arm_2->position,&kfs_2[1],3.5f,&kfs_2[2],
                       50.0f,&kfs_2[3],5.0f,5.0f,&kfs_2[4],-1.65f*Torque.Torque_2,-1.0f*Torque.Torque_2,0.0f);
                arm_2->set_mit_data(arm_2,kfs_2[0],kfs_2[1],kfs_2[2],kfs_2[3],kfs_2[4]);
             }
             if(arm_2->position>-2.55f)  
             {   
                in_place(-1.0f,&kfs_2[0],arm_2->position,&kfs_2[1],1.5f,&kfs_2[2],
                       60.0f,&kfs_2[3],5.0f,5.0f,&kfs_2[4],-0.75f*Torque.Torque_2,-3.0f*Torque.Torque_2,0.3f);
                arm_2->set_mit_data(arm_2,kfs_2[0],kfs_2[1],kfs_2[2],kfs_2[3],kfs_2[4]); 
             }
             if(arm_2->position<-1.65f&&arm_2->position>-1.85f)
             {
                arm_3->set_mit_data(arm_3,8.25f,0.0f,0.3f,0.5f,2.0f*Torque.Torque_3);
                arm_1->set_mit_data(arm_1,0.4f,0.3f,55.0f,5.0f,-0.8f*Torque.Torque_1);                 
             }
             if(arm_2->position>-1.1f&&arm_2->position<-0.9)
             {
                arm_1->set_mit_data(arm_1,0.18f,0.0f,55.0f,5.0f,-1.2f*Torque.Torque_1); 
             }
             
           }        
					 

         //模式2参数夹取武器（已调）
         else if(RCctrl.chassis == 1 && RCctrl.zone == 0 && RCctrl.takePos ==0)  //
         {
              arm.mode=2;
              arm.s=data_convert(RCctrl.accel, ACCEL_LOW, ACCEL_HIGH, 0.39,0.65);
              arm.h=0.11f; 
              angles=Arm_Inverse_Solution(&arm);             
              Torque=Torque_Comp_global(arm_1,arm_2,arm_3);
             
              arm_1->set_mit_data(arm_1,angles.theta_1+0.45f,0.0f,20.0f,1.5f,-0.25*Torque.Torque_1);
              arm_2->set_mit_data(arm_2,-angles.theta_2,0.0f,30.0f,2.0f,-Torque.Torque_2);
              arm_3->set_mit_data(arm_3,-25*(-arm_2->position-arm_1->position+0.43+a_weapon+0.2*PI),0.0f,3.0f,1.5f,Torque.Torque_3);
         }
         //模式4参数回收武器（已调）
         else if(RCctrl.chassis == 1 && RCctrl.zone == 0 && RCctrl.takePos ==1)  //
         {
              Torque=Torque_Comp_global(arm_1,arm_2,arm_3);
              arm_1->set_mit_data(arm_1,0.4f,0.0f,20.0f,2.0f,-0.35*Torque.Torque_1);
              arm_2->set_mit_data(arm_2,-0.39f,0.0f,30.0f,3.0f,-0.3f*Torque.Torque_2);
              //arm_3->set_mit_data(arm_3,20.8f,0.0f,1.0f,1.5f,0.0f*Torque.Torque_3); 
					    in_place(20.8,&kfs_3[0],arm_3->position,&kfs_3[1],0.1f,&kfs_3[2],1.0f,&kfs_3[3],0.8f,0.4,&kfs_3[4],0.0f,0.0f,0.30f*PI);
					    arm_3->set_mit_data(arm_3,kfs_3[0],kfs_3[1],kfs_3[2],kfs_3[3],kfs_3[4]);
					 
         }

//          //模式5参数中高度梅林kfs抓取（已调）
          else if(RCctrl.chassis == 1 && RCctrl.zone == 1 && RCctrl.key ==1)  //
          {
              arm.mode=5;
              arm.s=data_convert(RCctrl.accel, ACCEL_LOW, ACCEL_HIGH, 0.32,0.6);
              arm.h=0.435;
              angles=Arm_Inverse_Solution(&arm);             
              Torque=Torque_Comp_global(arm_1,arm_2,arm_3);;
              
              arm_1->set_mit_data(arm_1,angles.theta_1+0.45f,0.0f,25.0f,2.0f,-0.25*Torque.Torque_1);
              arm_2->set_mit_data(arm_2,-angles.theta_2,0.0f,40.0f,4.0f,-0.5*Torque.Torque_2);
              arm_3->set_mit_data(arm_3,25*(PI/2.0f+arm_1->position-0.43+arm_2->position-0.1*PI),0.0f,0.15f,0.1f,0.8*Torque.Torque_3);
          }
          
//          //模式6参数高梅林kfs抓取(已调）
          else if(RCctrl.chassis == 1 && RCctrl.zone == 1 && RCctrl.key ==2)  //
          {
               arm.mode=6;
              arm.s=data_convert(RCctrl.accel, ACCEL_LOW, ACCEL_HIGH, 0.32,0.6);
              arm.h=0.65;
              angles=Arm_Inverse_Solution(&arm);             
              Torque=Torque_Comp_global(arm_1,arm_2,arm_3);
              
              arm_1->set_mit_data(arm_1,angles.theta_1+0.45f,0.0f,25.0f,2.0f,-0.25*Torque.Torque_1);
              arm_2->set_mit_data(arm_2,-angles.theta_2,0.0f,40.0f,4.0f,-0.5*Torque.Torque_2);
              arm_3->set_mit_data(arm_3,25*(PI/2.0f+arm_1->position-0.43+arm_2->position-0.09*PI),0.0f,0.15f,0.1f,0.8*Torque.Torque_3);
          }

		    //模式7参数放KFS（已调）
         else if(RCctrl.chassis == 1 && RCctrl.zone == 2 && RCctrl.bottomPos ==0)   //
          {
              arm.mode=7;
              arm.s=data_convert(RCctrl.accel, ACCEL_LOW, ACCEL_HIGH, 0.32,0.75);
              arm.h=0.51;
              angles=Arm_Inverse_Solution(&arm);             
              Torque=Torque_Comp_global(arm_1,arm_2,arm_3);
              
              arm_1->set_mit_data(arm_1,angles.theta_1+0.45f,0.0f,65.0f,5.0f,-1.5*Torque.Torque_1);
              arm_2->set_mit_data(arm_2,-angles.theta_2+(arm_1->position-0.11)*arm_1->position,0.0f,65.0f,5.0f,-0.25*Torque.Torque_2);
              arm_3->set_mit_data(arm_3,25*(PI/2.0f+1.5*arm_1->position-0.43+arm_2->position-0.03*PI+(0.11-arm_1->position)*arm_1->position),0.0f,0.3f,0.15f,1.5*Torque.Torque_3);
          }        
					
					
	        //模式9参数   放武器到地上   //只能从放杆状态切过来，否则无法触发第一层判断。可确认必要性
         if(RCctrl.chassis == 1 && RCctrl.zone == 2 && RCctrl.bottomPos ==2)  //
         {
					    arm.mode=9;
              Torque=Torque_Comp_global(arm_1,arm_2,arm_3);
              if(arm_2->position<-0.27f&&arm_2->position>-0.74f)
              {
                arm_2->set_mit_data(arm_2,-0.76f,0.0f,50.0f,5.0f,-1.0f*Torque.Torque_2);
              }
              if(arm_2->position<-0.74f&&arm_2->position>-0.78f)
              {
                  in_place(-60.5f,&kfs_3[0],arm_3->position,&kfs_3[1],-0.3f,&kfs_3[2],1.0f,&kfs_3[3],1.0f,0.4,&kfs_3[4],0.4f*Torque.Torque_3,0.15f*Torque.Torque_3,0.3f*25);
                  arm_3->set_mit_data(arm_3,kfs_3[0],kfs_3[1],kfs_3[2],kfs_3[3],kfs_3[4]); 
                  if(arm_3->position<-41.0f&&arm_3->position>-43.0f)
                  {
                      arm_1->set_mit_data(arm_1,0.62f,0.0f,65.0f,5.0f,-0.35*Torque.Torque_1);
                  }
              }
         }
				 
				 //模式10参数 越区指令
          if(RCctrl.chassis == 1 && RCctrl.zone == 1 && RCctrl.key ==5)
          {
              arm.mode=10;
              Torque=Torque_Comp_global(arm_1,arm_2,arm_3);
               
              arm_1->set_mit_data(arm_1,0.56f,0.0f,65.0f,5.0f,-1.0f*Torque.Torque_1);              
              in_place(-2.9f,&kfs_2[0],arm_2->position,&kfs_2[1],5.5f,&kfs_2[2],
                      50.0f,&kfs_2[3],5.0f,5.0f,&kfs_2[4],-1.65f*Torque.Torque_2,0.1f*Torque.Torque_2,0.3f);
              arm_2->set_mit_data(arm_2,kfs_2[0],kfs_2[1],kfs_2[2],kfs_2[3],kfs_2[4]);
              if(arm_2->position<-2.8f&&arm_2->position>-3.1f)  //区域待改
              {
                arm_3->set_mit_data(arm_3,-35.0f,0.0f,0.3f,0.6f,5.0f*Torque.Torque_3);  
              }
          }
					
         //纯力矩补偿
//					else
//				{
//					Torque=Torque_Comp_global(arm_1,arm_2,arm_3);
//          arm_1->set_mit_data(arm_1,0.0f,0.0f,0.0f,0.0f,-0.8*Torque.Torque_1);
//          arm_2->set_mit_data(arm_2,0.0f,0.0f,0.0f,0.0f,-0.75*Torque.Torque_2);
//          arm_3->set_mit_data(arm_3,0.0f,0.0f,0.0f,0.0f,0.7*Torque.Torque_3);
//        }

            
          return HAL_OK;
}


