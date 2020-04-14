#include <webots/keyboard.h>
#include <webots/robot.h>
#include <webots/camera.h>
#include <webots/range_finder.h>
#include <webots/camera_recognition_object.h>
#include <webots/gps.h>
#include <webots/compass.h>
#include <arm.h>
#include <base.h>
#include <gripper.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#define TIME_STEP 32

const char* my_models[8] = {"biscuit box","cereal boxr","cereal boxb","canr","cang",\
"jar","beer bottle","water bottle"};

static WbDeviceTag camera,camera_top,camera_low,camera_low_front,camera_top_front;
static WbDeviceTag range_finder;
int count = 0,BLANK = 0,BUSY = 0;
char Model_target[20];

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

void Mystrcpy1(char *des ,char *src)
{
    int i,j;
    for(j = 0;j<20;j++)
    {
      des[j] = '\0';
    }
    
    for(i=0;src[i]!='\0';i++)
    {
       des[i]=src[i];
    }
    des[i]='\0';
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

//标准抓手姿态
static void Grip_ready() {
  base_reset();
  arm_reset();
  passive_wait(2.0);
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
  passive_wait(1.0);
  arm_reset();
  passive_wait(1.0);
}

//最高二层放物体姿态
static void Top_SecondFloor_ready() {
  passive_wait(0.5);
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
  passive_wait(0.8);
  base_reset();
  printf("Low_SecondFloor_ready\n");
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
  passive_wait(0.8);
  base_reset();
  printf("FirstFloor_ready\n");
  passive_wait(1.0);
  arm_set_orientation(ARM_LEFT);
  passive_wait(1.0);
  arm_set_height(ARM_HANOI_PREPARE);
  passive_wait(5.0);
  base_strafe_right();
  passive_wait(1.5);
  gripper_release();
  passive_wait(1.0);
  base_strafe_left();
  passive_wait(2.5);
  arm_set_height(ARM_FRONT_CARDBOARD_BOX);
}

void Detect()
{
    int number_of_objects_top = wb_camera_recognition_get_number_of_objects(camera_top);
    int number_of_objects_topf = wb_camera_recognition_get_number_of_objects(camera_top_front);
    int number_of_objects_low = wb_camera_recognition_get_number_of_objects(camera_low);
    int number_of_objects_lowf = wb_camera_recognition_get_number_of_objects(camera_low_front);
    if(number_of_objects_topf==0 && number_of_objects_lowf==0)
        BLANK = 1;
    
   // printf("BLANK: %d",BLANK);
}



int main(int argc, char **argv) {
  wb_robot_init();
  base_init();
  arm_init();
  gripper_init();
  passive_wait(2.0);
  int TURN_COUNT = 0,TURN_FLAG = 0;
  int STOP = 0;
  int i, j,OBJECT_AREA,ID,IMAGE_X,NEAREST_INDEX = 0;
  float NEAREST;
  double ANGEL,ANGEL_new = 0;

  double angle[4] = {270,0,90,180};
  int angle_index = 0;
  int OTF = 0, OT = 0, OLF = 0, OL= 0;
  char Model[20];
  int box = 0,boxr = 0,boxb = 0,canr = 0, cang = 0, jar = 0, water = 0, beer = 0;
  //摄像头
  camera = wb_robot_get_device("camera");
  wb_camera_enable(camera,TIME_STEP);
  wb_camera_recognition_enable(camera,TIME_STEP);
  
  camera_top_front = wb_robot_get_device("camera_top_front");
  wb_camera_enable(camera_top_front,TIME_STEP);
  wb_camera_recognition_enable(camera_top_front,TIME_STEP);
  
  camera_top = wb_robot_get_device("camera_top");
  wb_camera_enable(camera_top,TIME_STEP);
  wb_camera_recognition_enable(camera_top,TIME_STEP);
  
  camera_low = wb_robot_get_device("camera_low");
  wb_camera_enable(camera_low,TIME_STEP);
  wb_camera_recognition_enable(camera_low,TIME_STEP);
  
  camera_low_front = wb_robot_get_device("camera_low_front");
  wb_camera_enable(camera_low_front,TIME_STEP);
  wb_camera_recognition_enable(camera_low_front,TIME_STEP);

  //距离传感器
  range_finder = wb_robot_get_device("range-finder");
  wb_range_finder_enable(range_finder,TIME_STEP);
  
  //GPS
  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, TIME_STEP);
  
  //Compass
  WbDeviceTag compass = wb_robot_get_device("compass");
  wb_compass_enable(compass, TIME_STEP);
  
  //main loop
  while (true) {
    step();
    NEAREST_INDEX = 0;
    NEAREST = 10;
    BLANK = 0;
    TURN_COUNT = TURN_COUNT + 1;
    const double *gps_values = wb_gps_get_values(gps);
    double *north = wb_compass_get_values(compass);
    ANGEL = get_bearing_in_degrees(north);
    ANGEL_new = ANGEL;
    TURN_FLAG = IS_turn_point(gps_values[0], gps_values[2]);
    OTF = OT = OLF = OL = 0;
    
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
       STOP = STOP + 1;
       //passive_wait(3.841);
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
    
   
    //选取距离最近的物体信息输出
    for (i = 0; i < number_of_objects; ++i) {  
        float depth = wb_range_finder_image_get_depth(image, width,objects[i].position_on_image[0],objects[i].position_on_image[1]);
        if(NEAREST > depth && strcmp(objects[i].model,"cabinet")!=0 ){
            NEAREST = depth;
            NEAREST_INDEX = i;
        }
    }
    OBJECT_AREA = objects[NEAREST_INDEX].size_on_image[0] * objects[NEAREST_INDEX].size_on_image[1];
    //printf("model: %s id: %d   x: %d   y: %d  distance: %f\n",objects[NEAREST_INDEX].model,objects[NEAREST_INDEX].id,objects[NEAREST_INDEX].position_on_image[0],objects[NEAREST_INDEX].position_on_image[1],NEAREST);
    //printf("Area of the object in image %d\n", OBJECT_AREA);
    
    //根据id号、物体在图像X轴相对位置、物体在图像中的面积抓取物体
    ID = objects[NEAREST_INDEX].id;
    Mystrcpy1(Model ,objects[NEAREST_INDEX].model);
    IMAGE_X = objects[NEAREST_INDEX].position_on_image[0];
                       
    if( BUSY == 0 && IMAGE_X <= 67 && IMAGE_X >= 63 && NEAREST<0.32 &&\
    ( (strcmp(Model,my_models[0])==0 && OBJECT_AREA<4500 && OBJECT_AREA>2900 && box ==0) ||\
      (strcmp(Model,my_models[1])==0 && OBJECT_AREA<5000 && OBJECT_AREA>3500 && boxr == 0) || \
      (strcmp(Model,my_models[2])==0 && OBJECT_AREA<5000 && OBJECT_AREA>3500 && boxb == 0) || \
      (strcmp(Model,my_models[3])==0  && OBJECT_AREA>1700 && canr == 0)  || \
      (strcmp(Model,my_models[4])==0  && OBJECT_AREA>1700 && cang == 0)  || \
      (strcmp(Model,my_models[5])==0 && OBJECT_AREA>1500 && jar == 0)  ||\
      (strcmp(Model,my_models[6])==0 && OBJECT_AREA>2200 && beer == 0)  ||\
      (strcmp(Model,my_models[7])==0 && OBJECT_AREA>3800 && water == 0))  )
    {  
      Mystrcpy1(Model_target,objects[NEAREST_INDEX].model);
      if(strcmp(Model,my_models[0])==0) box = 1;
      if(strcmp(Model,my_models[1])==0) boxr = 1;
      if(strcmp(Model,my_models[2])==0) boxb = 1;
      if(strcmp(Model,my_models[3])==0) canr = 1;
      if(strcmp(Model,my_models[4])==0) cang = 1;
      if(strcmp(Model,my_models[5])==0) jar = 1;
      if(strcmp(Model,my_models[6])==0) beer = 1;
      if(strcmp(Model,my_models[7])==0) water = 1;
      
      printf("target %s\n",Model_target);
      Grip_ready();
      BUSY = 1; //目标机器人已经抓有物体，等待放置
      base_forwards();
      passive_wait(0.8);
      STOP = 0;
    }
    else if(angle_index == 0 && STOP >=4 )
     {
         base_reset();
         arm_reset();
         passive_wait(3);
         base_strafe_right();
         passive_wait(2);
         base_backwards();
         passive_wait(2.5);
         base_reset();
         while(1)
           step();
     }
     
    else{
      base_forwards();
    }
    
    //放物品,A货架id=762,B货架id=963,C货架id=561,D货架id=1164
    int number_of_objects_top = wb_camera_recognition_get_number_of_objects(camera_top);
    int number_of_objects_topf = wb_camera_recognition_get_number_of_objects(camera_top_front);
    int number_of_objects_low = wb_camera_recognition_get_number_of_objects(camera_low);
    int number_of_objects_lowf = wb_camera_recognition_get_number_of_objects(camera_low_front);

    const WbCameraRecognitionObject *objects_top = wb_camera_recognition_get_objects(camera_top);
    const WbCameraRecognitionObject *objects_low = wb_camera_recognition_get_objects(camera_low);
    const WbCameraRecognitionObject *objects_top_front = wb_camera_recognition_get_objects(camera_top_front);
    const WbCameraRecognitionObject *objects_low_front = wb_camera_recognition_get_objects(camera_low_front);

    
    if(BUSY == 1){
      STOP = 0;
      //printf("tf_model1: %s \n",Model_target);
      for (i = 0; i < number_of_objects_topf; ++i) {
        //printf("tf_model: %s \n",objects_top_front[i].model);
        if(strcmp(objects_top_front[i].model, Model_target) == 0)
          OTF = 1;
      }
      for (i = 0; i < number_of_objects_top; ++i) {
        //printf("t_model: %s \n",objects_top[i].model);
        if(strcmp(objects_top[i].model, Model_target) == 0)
          OT = 1;
      }
      for (i = 0; i < number_of_objects_lowf; ++i) {
        //printf("Lf_model: %s \n",objects_low_front[i].model);
        if(strcmp(objects_low_front[i].model, Model_target) == 0)
          OLF = 1;
      }
      for (i = 0; i < number_of_objects_low; ++i) {
        //printf("L_model: %s \n",objects_low[i].model);
        if(strcmp(objects_low[i].model, Model_target) == 0)
          OL = 1;
      }
      
      //printf("OTF: %d OT: %d OLF: %d OL: %d\n",OTF,OT,OLF,OL);
      if(OTF == 1 && OT == 0)
      {
        OTF = 0, OT = 0, OLF = 0, OL= 0;
        printf("detect: %s\n", "top_back is blank");
        base_reset();
        passive_wait(0.5);
        base_forwards();
        if(strcmp("cereal boxb", Model_target) == 0)
        {
          Detect();
          if(BLANK ==0){
            Low_SecondFloor_ready();
            BUSY = 0;
          }
        }
        else if(strcmp("biscuit box", Model_target) == 0 )
        {
           passive_wait(0.2);
           Detect();
           if(BLANK == 0){
             Low_SecondFloor_ready();
             BUSY = 0;
           }
        }
        
        else if(strcmp("jar", Model_target) == 0 )
        {
           passive_wait(0.1);
           Detect();
           if(BLANK == 0){
             Top_SecondFloor_ready();
             BUSY = 0;
           }
        }
        else
        {
           passive_wait(0.35);
           Detect();
           if(BLANK == 0){
             Top_SecondFloor_ready(); 
             BUSY = 0;
           }
        }
      }
      
      if(OTF == 0 && OT == 1)
      {
        OTF = 0, OT = 0, OLF = 0, OL= 0;
        printf("detect: %s\n", "top_front is blank");
        base_reset();
        passive_wait(0.5);
        base_forwards();
        passive_wait(0.8);
        if(strcmp("biscuit box", Model_target) == 0 ||strcmp("cereal boxb", Model_target) == 0)
        {
           Detect();
           if(BLANK == 0){
             Low_SecondFloor_ready();
             BUSY = 0;
           }
        }
        else if(strcmp("jar", Model_target) == 0 )
        {
            passive_wait(0.3);
            Detect();
            if(BLANK == 0){
              Top_SecondFloor_ready();
              BUSY = 0;
            }
        }
        else if(strcmp("cereal boxr", Model_target) == 0)
        {
            passive_wait(0.15);
            Detect();
            if(BLANK == 0){
              Top_SecondFloor_ready();
              BUSY = 0;
            }
        }
        else{
           Detect();
           if(BLANK == 0){
             Top_SecondFloor_ready();
             BUSY = 0;
           }
        }
      }
      
      if(OLF == 1 && OL == 0)
      {
        base_reset();
        OTF = 0, OT = 0, OLF = 0, OL= 0;
        printf("detect: %s\n", "low_back is blank");
        base_reset();
        passive_wait(0.5);
        base_forwards();
        passive_wait(0.1);
        Detect();
        if(BLANK == 0){
          FirstFloor_ready();
          BUSY = 0;
        }        
      }

      if(OLF == 0 && OL == 1)
      {
        base_reset();
        OTF = 0, OT = 0, OLF = 0, OL= 0;
        printf("detect: %s\n", "low_front is blank");
        base_reset();
        passive_wait(0.5);
        base_forwards();
        if(strcmp("water bottle", Model_target) == 0)
        {
            passive_wait(0.8);
        }
        else
          passive_wait(1.1);
        Detect();
        if(BLANK == 0){
          FirstFloor_ready(); 
          BUSY = 0;
        } 
      }
    }
  }
  
  wb_robot_cleanup();
  return 0;
}
