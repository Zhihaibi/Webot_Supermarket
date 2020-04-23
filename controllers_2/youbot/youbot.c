#include <webots/keyboard.h>
#include <webots/robot.h>
#include <webots/camera.h>
#include <webots/range_finder.h>
#include <webots/camera_recognition_object.h>
#include <webots/gps.h>
#include <webots/compass.h>
#include <webots/distance_sensor.h>
#include <arm.h>
#include <base.h>
#include <gripper.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TIME_STEP 32
int count = 0;
int BUSY = 0;//judge if the robot gripping object now

static void step() {
  if (wb_robot_step(TIME_STEP) == -1) {
    wb_robot_cleanup();
    exit(EXIT_SUCCESS);
  }
}

static void passive_wait(double sec) {
  double start_time = wb_robot_get_time();
  do {
    step();
  } while (start_time + sec > wb_robot_get_time());
}

static void display_helper_message() {
  printf("SB+Control commands:\n");
  printf(" Arrows:       Move the robot\n");
  printf(" Page Up/Down: Rotate the robot\n");
  printf(" +/-:          (Un)grip\n");
  printf(" Shift + arrows:   Handle the arm\n");
  printf(" Space: Reset\n");
}


double get_bearing_in_degrees(double *north) {
  double rad = atan2(north[0], north[1]);
  double bearing = (rad - 1.5708) / M_PI * 180.0;
  if (bearing < 0.0)
    bearing = bearing+360;
  return bearing;
}

int IS_turn_point(double x, double y) {
  double dis0 = (x-1.0)*(x-1.0)+(y-0.306)*(y-0.306);
  double dis1 = (x-1.04)*(x-1.04)+(y-2.75)*(y-2.75);
  double dis2 = (x-3.5)*(x-3.5)+(y-2.691)*(y-2.691);
  double dis3 = (x-3.43)*(x-3.43)+(y-0.25)*(y-0.25);
  double dis = 0.005;
 // printf("dis: %.3f %.3f %.3f %.3f\n", dis0, dis1, dis2,dis3);
  if(dis0<dis || dis1<dis || dis2<dis || dis3<dis)
    return 1;
  else
    return 0;
}

//标准巡逻
static void Patrol() { 
  printf("Patrol\n");
  base_forwards();
  passive_wait(11.85);
  base_turn_right();
  passive_wait(4.032);
}

//标准抓手姿态
static void Grip_ready() {
  base_reset();
  arm_reset();
  passive_wait(1.0);
  arm_init();
  printf("Grip_ready\n");
  passive_wait(1.0);
  arm_set_orientation(ARM_RIGHT);
  gripper_release();
  passive_wait(2.0);
  arm_set_height(ARM_HANOI_PREPARE);
  passive_wait(6.0);
  gripper_grip();
  passive_wait(2.0);
  arm_set_height(ARM_FRONT_CARDBOARD_BOX);
  passive_wait(2.0);
  arm_reset();
  passive_wait(1.0);
}

//最高二层放物体姿态
static void Top_SecondFloor_ready() {
  passive_wait(0.7);
  base_reset();
  printf("SecondFloor_ready\n");
  passive_wait(2.0);
  arm_set_orientation(ARM_LEFT);
  passive_wait(1.0);
  arm_set_height(ARM_BACK_PLATE_LOW);
  passive_wait(3.0);
  base_strafe_right();
  passive_wait(2.0);
  gripper_release();
  passive_wait(1.0);
  base_strafe_left();
  passive_wait(2.0);
  passive_wait(1.0);
  arm_set_height(ARM_FRONT_CARDBOARD_BOX);
}

//次高二层放物体姿态
static void Low_SecondFloor_ready() {
  passive_wait(0.7);
  base_reset();
  printf("SecondFloor_ready\n");
  passive_wait(2.0);
  arm_set_orientation(ARM_LEFT);
  passive_wait(1.0);
  arm_set_height(ARM_BACK_PLATE_HIGH);
  passive_wait(1.0);
  base_strafe_right();
  passive_wait(2.0);
  gripper_release();
  passive_wait(1.0);
  base_strafe_left();
  passive_wait(2.0);
  passive_wait(1.0);
  arm_set_height(ARM_FRONT_CARDBOARD_BOX);
}

//一层放物体姿态
static void FirstFloor_ready() {
  passive_wait(0.7);
  base_reset();
  printf("FirstFloor_ready\n");
  passive_wait(1.0);
  arm_set_orientation(ARM_LEFT);
  passive_wait(1.0);
  arm_set_height(ARM_HANOI_PREPARE);
  passive_wait(5.0);
  base_strafe_right();
  passive_wait(1.3);
  gripper_release();
  passive_wait(1.0);
  base_strafe_left();
  passive_wait(1.3);
  passive_wait(1.0);
  arm_set_height(ARM_FRONT_CARDBOARD_BOX);
}

static WbDeviceTag camera,camera_top,camera_low;
static WbDeviceTag range_finder;


int main(int argc, char **argv) {
  wb_robot_init();
  base_init();
  arm_init();
  gripper_init();
  passive_wait(2.0);
  int TURN_COUNT = 0,TURN_FLAG = 0;
  int pc = 0;
  int i, j,OBJECT_AREA,ID,IMAGE_X,NEAREST_INDEX = 0;
  float NEAREST;
  double ANGEL,ANGEL_new = 0;
  char MODEL;
  double angle[4] = {270,0,90,180};
  int angle_index = 0;
  int id = 0;
  int cabinet_num=0;
  int STOP = 0;//取放货物的个数
  int id1_flag=0,id2_flag=0,id3_flag=0,id4_flag=0,id5_flag=0,id6_flag=0,id7_flag=0,id8_flag=0,id9_flag=0;
  int grip_num1=0,grip_num2=0,grip_num3=0,grip_num4=0,grip_num5=0,grip_num6=0,grip_num7=0,grip_num8=0,grip_num9=0;
  const char* my_models[9] = {"biscuit box","cereal box1","honey jar","water bottle1",\
  "beer bottle","cereal box_r1","jam jar1","can1","can_g"};
  wb_keyboard_enable(TIME_STEP);
  
  //摄像头
  camera = wb_robot_get_device("camera");
  wb_camera_enable(camera,TIME_STEP);
  wb_camera_recognition_enable(camera,TIME_STEP);

  camera_top = wb_robot_get_device("camera_top");
  wb_camera_enable(camera_top,TIME_STEP);
  wb_camera_recognition_enable(camera_top,TIME_STEP);
  camera_low = wb_robot_get_device("camera_low");
  wb_camera_enable(camera_low,TIME_STEP);
  wb_camera_recognition_enable(camera_low,TIME_STEP);

  //距离传感器
  range_finder = wb_robot_get_device("range-finder");
  wb_range_finder_enable(range_finder,TIME_STEP);
  
  //GPS
  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, TIME_STEP);
  
  //Compass
  WbDeviceTag compass = wb_robot_get_device("compass");
  wb_compass_enable(compass, TIME_STEP);
  
  //Distance sensor
  WbDeviceTag distance_sensor = wb_robot_get_device("distance sensor");
  wb_distance_sensor_enable(distance_sensor, TIME_STEP);
  //main loop
  while (true) {
    step();
    NEAREST_INDEX = 0;
    NEAREST = 10;
    TURN_COUNT = TURN_COUNT+1;
    const double *gps_values = wb_gps_get_values(gps);
    double *north = wb_compass_get_values(compass);
    double distance = wb_distance_sensor_get_value(distance_sensor);
    double max_distance = wb_distance_sensor_get_max_value(distance_sensor);
    ANGEL = get_bearing_in_degrees(north);
    ANGEL_new = ANGEL;
    TURN_FLAG = IS_turn_point(gps_values[0], gps_values[2]);
    
    printf("the distance from the obstacle: %f \n",distance/max_distance);
    if(distance/max_distance < 0.5)
    {
       base_reset();
       passive_wait(5);
    }
    
    if(BUSY == 0&&STOP == 4)
     {
        
         if (fabs(gps_values[0]-3.35)<0.05&&fabs(gps_values[2]-0.3)<0.05)
            {
              printf("%s \n","Go back to the starting point!");
              //arm_reset();
              //base_reset();
              //passive_wait(10);
              base_reset();
              arm_reset();
              passive_wait(3);
              base_strafe_right();
              passive_wait(1.5);
              base_backwards();
              passive_wait(2.5);
              base_reset();
              while(1)
                 step();
            }
     }
    
    if(TURN_FLAG==1 && TURN_COUNT >200)
    {
       base_reset();
       passive_wait(0.5);
       base_turn_right();
       while(abs(ANGEL_new -angle[angle_index])>0.5 )
       {
         step();
         double *north1 = wb_compass_get_values(compass);
         ANGEL_new = get_bearing_in_degrees(north1);
         //printf("Angel %.3f\n", ANGEL_new);
       }
       //passive_wait(3.841);
       //STOP = STOP + 1;
       base_reset();
       
       angle_index = angle_index + 1;
       if(angle_index==4)
         angle_index = 0;
       TURN_COUNT = 0;
       TURN_FLAG = 0;
    }
    
   // printf("Using the GPS device: %.3f %.3f %.3f\n", gps_values[0], gps_values[2], ANGEL);
    
    int number_of_objects = wb_camera_recognition_get_number_of_objects(camera);
    int width = wb_range_finder_get_width(range_finder);
    const float *image = wb_range_finder_get_range_image(range_finder);
    const WbCameraRecognitionObject *objects = wb_camera_recognition_get_objects(camera);
    int number_of_objects_top = wb_camera_recognition_get_number_of_objects(camera_top);
    int number_of_objects_low = wb_camera_recognition_get_number_of_objects(camera_low);
    const WbCameraRecognitionObject *objects_top = wb_camera_recognition_get_objects(camera_top);
    const WbCameraRecognitionObject *objects_low = wb_camera_recognition_get_objects(camera_low);
    //printf("number_of_objects_low: %d \n",number_of_objects_low);

    //选取距离最近的物体信息输出
    for (i = 0; i < number_of_objects; ++i) {  
        float depth = wb_range_finder_image_get_depth(image, width,objects[i].position_on_image[0],objects[i].position_on_image[1]);
        if(NEAREST > depth){
            NEAREST = depth;
            NEAREST_INDEX = i;
        }
    }
    OBJECT_AREA = objects[NEAREST_INDEX].size_on_image[0] * objects[NEAREST_INDEX].size_on_image[1];
    printf("model: %s id: %d   x: %d   y: %d  distance: %f\n",objects[NEAREST_INDEX].model,objects[NEAREST_INDEX].id,objects[NEAREST_INDEX].position_on_image[0],objects[NEAREST_INDEX].position_on_image[1],NEAREST);
    printf("Area of the object in image %d\n", OBJECT_AREA);
    
    //根据id号、物体在图像X轴相对位置、物体在图像中的面积抓取物体
    ID = objects[NEAREST_INDEX].id;
    IMAGE_X = objects[NEAREST_INDEX].position_on_image[0];
    const char* Model = objects[NEAREST_INDEX].model;
    //判断分别为                          biscuit box                                                     cereal box                                       honey jar                         water bottle                                  beer   bottle                                                                                                                           
    if( BUSY == 0 && IMAGE_X <= 67 && IMAGE_X >= 63 && NEAREST < 0.32&&\
    ( (strcmp(Model,my_models[0])==0 && OBJECT_AREA<4500 && OBJECT_AREA>2900&&grip_num1==0) ||\
      (strcmp(Model,my_models[1])==0 && OBJECT_AREA<5000 && OBJECT_AREA>3500&&grip_num2==0) ||\
      (strcmp(Model,my_models[2])==0 && OBJECT_AREA>1500&&grip_num3==0)  ||\
      (strcmp(Model,my_models[3])==0 && OBJECT_AREA>3800&&grip_num4==0)  ||\
      (strcmp(Model,my_models[4])==0 && OBJECT_AREA>2200&&grip_num5==0)  ||\
      (strcmp(Model,my_models[5])==0 && OBJECT_AREA<5000 && OBJECT_AREA>3500&&grip_num6==0) ||\
      (strcmp(Model,my_models[6])==0 && OBJECT_AREA>1500&&grip_num7==0)  ||\
      (strcmp(Model,my_models[7])==0  && OBJECT_AREA>1700&&grip_num8==0)  ||\
      (strcmp(Model,my_models[8])==0  && OBJECT_AREA>1700&&grip_num9==0)))
    {
      for (int i =0;i<9;i++)
      {
          if (strcmp(Model,my_models[i])==0)
              id = i+1;
      }  

      printf("%d \n",id);
      Grip_ready();
      BUSY = 1; //目标机器人已经抓有物体，等待放置
      STOP = STOP + 1;
      number_of_objects_top = wb_camera_recognition_get_number_of_objects(camera_top);
      number_of_objects_low = wb_camera_recognition_get_number_of_objects(camera_low);
      const WbCameraRecognitionObject *objects_top = wb_camera_recognition_get_objects(camera_top);
      const WbCameraRecognitionObject *objects_low = wb_camera_recognition_get_objects(camera_low);
      for (int j=0;j<number_of_objects_top;j++)
      {
          if (strcmp(objects_top[j].model,"cabinet_a")==0)
             cabinet_num = 1;
          else if (strcmp(objects_top[j].model,"cabinet_b")==0)
             cabinet_num=2;
          else if (strcmp(objects_top[j].model,"cabinet_c")==0)
             cabinet_num=3;
          else if (strcmp(objects_top[j].model,"cabinet_d")==0)
             cabinet_num=4;
      }
      if ((id == 5 || id ==6) && cabinet_num == 2)
         {//printf("%s \n","11111111111111111111111");
         while ((gps_values[2]-0.620)>0.005)//0.586
             {
                step();
                base_backwards();
             }
         number_of_objects_top = wb_camera_recognition_get_number_of_objects(camera_top);
         number_of_objects_low = wb_camera_recognition_get_number_of_objects(camera_low);
         const WbCameraRecognitionObject *objects_top = wb_camera_recognition_get_objects(camera_top);
         const WbCameraRecognitionObject *objects_low = wb_camera_recognition_get_objects(camera_low);
         printf("number_of_objects_top: %d \n",number_of_objects_top);
         }
      else if ((id == 3||id == 4||id == 7) && cabinet_num == 4)
              {
               while ((2.451-gps_values[2])>0.005)
               {
                  step();
                  base_backwards();
               }
               number_of_objects_top = wb_camera_recognition_get_number_of_objects(camera_top);
               number_of_objects_low = wb_camera_recognition_get_number_of_objects(camera_low);
               const WbCameraRecognitionObject *objects_top = wb_camera_recognition_get_objects(camera_top);
               const WbCameraRecognitionObject *objects_low = wb_camera_recognition_get_objects(camera_low);
               printf("number_of_objects_top: %d \n",number_of_objects_top);
              }
      else if ((id == 1||id == 8) && cabinet_num == 1)
              {
               while ((3.14-gps_values[0])>0.005)
               {
                  step();
                  base_backwards();
               }
               number_of_objects_top = wb_camera_recognition_get_number_of_objects(camera_top);
               number_of_objects_low = wb_camera_recognition_get_number_of_objects(camera_low);
               const WbCameraRecognitionObject *objects_top = wb_camera_recognition_get_objects(camera_top);
               const WbCameraRecognitionObject *objects_low = wb_camera_recognition_get_objects(camera_low);
               printf("number_of_objects_top: %d \n",number_of_objects_top);
              }
      else if ((id == 2||id == 9) && cabinet_num == 3)
              {
               while ((gps_values[0]-1.33)>0.005)
               {
                  step();
                  base_backwards();
               }
               number_of_objects_top = wb_camera_recognition_get_number_of_objects(camera_top);
               number_of_objects_low = wb_camera_recognition_get_number_of_objects(camera_low);
               const WbCameraRecognitionObject *objects_top = wb_camera_recognition_get_objects(camera_top);
               const WbCameraRecognitionObject *objects_low = wb_camera_recognition_get_objects(camera_low);
               printf("number_of_objects_top: %d \n",number_of_objects_top);
              }
    }
    else{
      if (id1_flag==0&&id2_flag==0&&id3_flag==0&&id4_flag==0&&id5_flag==0\
          &&id6_flag==0&&id7_flag==0&&id8_flag==0&&id9_flag==0)
          base_forwards();
    }
    printf("id: %d \n",id); 
    //放物品,A货架id=762,B货架id=963,C货架id=561,D货架id=1164
    if(BUSY == 1){
      //STOP = 0;
      number_of_objects_top = wb_camera_recognition_get_number_of_objects(camera_top);
      number_of_objects_low = wb_camera_recognition_get_number_of_objects(camera_low);
      objects_top = wb_camera_recognition_get_objects(camera_top);
      objects_low = wb_camera_recognition_get_objects(camera_low);
      
      if(id == 1){
        int flag_1=0;
        for (int i=0;i<number_of_objects_top;++i)
        {
            if (strcmp(objects_top[i].model,"cabinet_a") == 0)
               flag_1 ++;
        }
        if (flag_1 != 0)
        {
          /*printf("number_of_objects_top: %d \n",number_of_objects_top);
          if (number_of_objects_top != 1)
          {
              //step();
              base_reset();
              //passive_wait(0);
              base_forwards();
              passive_wait(0);
              base_backwards();
              passive_wait(0.1);
              
              //number_of_objects_top = wb_camera_recognition_get_number_of_objects(camera_top);
          }
          if (number_of_objects_top == 1)
            {
               Low_SecondFloor_ready();
               BUSY = 0;
            }*/
          if (number_of_objects_top == 1)
          {
            if (id1_flag==1)
             {

               //passive_wait(1.2);
               //base_backwards();
               //base_forwards();
               //passive_wait(1.6);
             }
            Low_SecondFloor_ready();
            BUSY = 0;
            id1_flag=0;
            grip_num1=1;
          }
          else{
             id1_flag=1;
             //passive_wait(1.2);
             base_forwards();
             passive_wait(0.01);
             base_reset();
             //passive_wait(0.6);

          }
        }
      }
      else if (id == 2){
        int flag_2=0;
        for (int i=0;i<number_of_objects_top;++i)
        {
            if (strcmp(objects_top[i].model,"cabinet_c") == 0)
               flag_2 ++;
        }
        if (flag_2 != 0)
        {
          printf("number_of_objects_top: %d \n",number_of_objects_top);
          if ((number_of_objects_top == 1||number_of_objects_top == 0)\
              &&number_of_objects_low != 0)
            {
               if (id2_flag==1)
               { 
                 //base_backwards();
                 //passive_wait(0);
               }
              Low_SecondFloor_ready();
              BUSY = 0;
              id2_flag=0;
              grip_num2=1;
            }
            else{
               id2_flag=1;
               base_forwards();
               passive_wait(0.1);
               base_reset();
            }
        }
      }
      else if (id == 3){
        int flag_3=0;
        for (int i=0;i<number_of_objects_top;++i)
        {
            if (strcmp(objects_top[i].model,"cabinet_d") == 0)
               flag_3 ++;
        }
        if (flag_3 != 0)
        {
          printf("number_of_objects_top: %d \n",number_of_objects_top);
          if (number_of_objects_top == 1)
            {
               if (id3_flag==1)
               {
                  //base_backwards();
                  //passive_wait(0);
               }
               Top_SecondFloor_ready();
               BUSY = 0;
               id3_flag=0;
               grip_num3=1;
            }
            else{
               id3_flag=1;
               base_forwards();
               passive_wait(0.97);
               base_reset();
            }
        }
        }
      else if (id == 4){
        int flag_4=0;
        int i;
        for (i=0;i<number_of_objects_low;++i)
        {
            if (strcmp(objects_low[i].model,"cabinet_d") == 0)
            {
               flag_4 ++;
               //break;
            }
        }
        if (flag_4 != 0)
        {
            //FirstFloor_ready();
            printf("number_of_objects_low: %d \n",number_of_objects_low);
            if ((number_of_objects_low == 1||number_of_objects_low == 0)&&
            number_of_objects_top != 0)
            {
               if (id4_flag==1)
               {
                  //base_backwards();
                  //passive_wait(0);
               }
               FirstFloor_ready();
               BUSY = 0;
               id4_flag=0;
               grip_num4=1;
            }
            else{
               id4_flag=1;
               base_forwards();
               passive_wait(1);
               base_reset();
            }
        }
      }
      else if (id == 5){
        int flag_5=0;
        for (int i=0;i<number_of_objects_low;++i)
        {
            if (strcmp(objects_low[i].model,"cabinet_b") == 0)
               flag_5 ++;
        }
        if (flag_5 != 0)
        {
          printf("number_of_objects_low: %d \n",number_of_objects_low);
          if (number_of_objects_low == 1)
            {
               if (id5_flag == 1)
               {
                   //base_backwards();
                   //passive_wait(0);  
               }

               FirstFloor_ready();
               BUSY = 0;
               id5_flag=0;
               grip_num5=1;
            }
            else{
               id5_flag = 1;
               base_forwards();
               passive_wait(2);
               base_reset();
            }
        }
      }
     else if (id == 6){
        int flag_6=0;
        for (int i=0;i<number_of_objects_top;++i)
        {
            if (strcmp(objects_top[i].model,"cabinet_b") == 0)
               flag_6 ++;
        }
        if (flag_6 != 0)
        {
          printf("number_of_objects_top: %d \n",number_of_objects_top);
          if (number_of_objects_top == 1)
            {
               if (id6_flag==1)
               {
                  //base_backwards();
                  //passive_wait(0);
               }
               Top_SecondFloor_ready();
               BUSY = 0;
               id6_flag=0;
               grip_num6=1;
            }
            else{
               id6_flag=1;
               base_forwards();
               passive_wait(0.001);
               base_reset();
            }
        }
      }
     else if (id == 7){
        int flag_7=0;
        for (int i=0;i<number_of_objects_top;++i)
        {
            if (strcmp(objects_top[i].model,"cabinet_d") == 0)
               flag_7 ++;
        }
        printf("flag_7:%d \n",flag_7);
        if (flag_7 != 0)
        {
          printf("number_of_objects_top: %d \n",number_of_objects_top);
          if (number_of_objects_top == 1)
            {
               if (id7_flag==1)
               {
                  //base_backwards();
                  //passive_wait(0);
               }
               Top_SecondFloor_ready();
               BUSY = 0;
               id7_flag=0;
               grip_num7=1;
            }
            else{
               id7_flag=1;
               base_forwards();
               passive_wait(0.97);
               base_reset();
            }
        }
      }
     else if (id == 8){
        int flag_8=0;
        for (int i=0;i<number_of_objects_low;++i)
        {
            if (strcmp(objects_low[i].model,"cabinet_a") == 0)
               flag_8 ++;
        }
        if (flag_8 != 0)
        {
          printf("number_of_objects_low: %d \n",number_of_objects_low);
          if (number_of_objects_low == 1)
            {
               /*if (id8_flag==1)
               {
                  base_backwards();
                  passive_wait(0);
               }*/
               FirstFloor_ready();
               BUSY = 0;
               id8_flag=0;
               grip_num8=1;
            }
            else{
               id8_flag=1;
               base_forwards();
               passive_wait(2);//2
               base_reset();
            }
        }
      }
     else if (id == 9){
        int flag_9=0;
        for (int i=0;i<number_of_objects_low;++i)
        {
            if (strcmp(objects_low[i].model,"cabinet_c") == 0)
               flag_9 ++;
        }
        if (flag_9 != 0)
        {
          printf("number_of_objects_low: %d \n",number_of_objects_low);
          if (number_of_objects_low == 1)
            {
               if (id9_flag==1)
               {
                  //base_backwards();
                  //passive_wait(0);
               }
               FirstFloor_ready();
               BUSY = 0;
               id9_flag=0;
               grip_num9=1;
            }
            else{
               id9_flag=1;
               base_forwards();
               passive_wait(2);
               base_reset();
            }
        }
      }
    }
}
  wb_robot_cleanup();

  return 0;
}
