/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "IMU_DCM.h"

const float PI = 3.14159265358979323846f;

// DCM variables
float Accel_Vector[3]= {0, 0, 0}; // Store the acceleration in a vector
float Gyro_Vector[3]= {0, 0, 0}; // Store the gyros turn rate in a vector
float Omega_Vector[3]= {0, 0, 0}; // Corrected Gyro_Vector data
float Omega_P[3]= {0, 0, 0}; // Omega Proportional correction
float Omega_I[3]= {0, 0, 0}; // Omega Integrator
float Omega[3]= {0, 0, 0};
float errorRollPitch[3] = {0, 0, 0};
float errorYaw[3] = {0, 0, 0};
float DCM_Matrix[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
float Update_Matrix[3][3] = {{0, 1, 2}, {3, 4, 5}, {6, 7, 8}};
float Temporary_Matrix[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

float MAG_Heading;

float IMU_Yaw_Offset = 0.0f;
float IMU_Pitch_Offset = 0.0f;
float IMU_Roll_Offset = 0.0f;

// Euler angles
float IMU_Yaw, IMU_Yaw_Final;
float IMU_Pitch, IMU_Pitch_Final;
float IMU_Roll, IMU_Roll_Final;

// DCM timing in the main loop
float G_Dt; // Integration time for DCM algorithm


// Read every sensor and record a time stamp
// Init DCM with unfiltered orientation
// TODO re-init global vars?
void IMU_reset_sensor_fusion(void) {
  float temp1[3];
  float temp2[3];
  float xAxis[] = {1.0f, 0.0f, 0.0f};

  IMU_ReadSensors();

  // GET PITCH
  // Using y-z-plane-component/x-component of gravity vector
  IMU_Pitch = -atan2(IMU_accel[0], sqrt(IMU_accel[1] * IMU_accel[1] + IMU_accel[2] * IMU_accel[2]));

  // GET ROLL
  // Compensate pitch of gravity vector
  Vector_Cross_Product(temp1, IMU_accel, xAxis);
  Vector_Cross_Product(temp2, xAxis, temp1);
  // Normally using x-z-plane-component/y-component of compensated gravity vector
  // roll = atan2(temp2[1], sqrt(temp2[0] * temp2[0] + temp2[2] * temp2[2]));
  // Since we compensated for pitch, x-z-plane-component equals z-component:
  IMU_Roll = atan2(temp2[1], temp2[2]);

  // GET YAW
  IMU_Compass_Heading();
  IMU_Yaw = MAG_Heading;

  // Init rotation matrix
  init_rotation_matrix(DCM_Matrix, IMU_Yaw, IMU_Pitch, IMU_Roll);
}

// DCM algorithm
/**************************************************/
void IMU_Normalize(void)
{
  float error=0;
  float temporary[3][3];
  float renorm=0;

  error= -Vector_Dot_Product(DCM_Matrix[0],DCM_Matrix[1])*.5; //eq.19

  Vector_Scale(temporary[0], DCM_Matrix[1], error); //eq.19
  Vector_Scale(temporary[1], DCM_Matrix[0], error); //eq.19

  Vector_Add(temporary[0], temporary[0], DCM_Matrix[0]);//eq.19
  Vector_Add(temporary[1], temporary[1], DCM_Matrix[1]);//eq.19

  Vector_Cross_Product(temporary[2],temporary[0],temporary[1]); // c= a x b //eq.20

  renorm= .5 *(3 - Vector_Dot_Product(temporary[0],temporary[0])); //eq.21
  Vector_Scale(DCM_Matrix[0], temporary[0], renorm);

  renorm= .5 *(3 - Vector_Dot_Product(temporary[1],temporary[1])); //eq.21
  Vector_Scale(DCM_Matrix[1], temporary[1], renorm);

  renorm= .5 *(3 - Vector_Dot_Product(temporary[2],temporary[2])); //eq.21
  Vector_Scale(DCM_Matrix[2], temporary[2], renorm);
}

float mag_heading_x;
float mag_heading_y;
float errorCourse;
//Compensation the Roll, Pitch and Yaw drift.
static float Scaled_Omega_P[3];
static float Scaled_Omega_I[3];
float Accel_magnitude;
float Accel_weight;
/**************************************************/
void IMU_Drift_correction(void)
{
  //*****Roll and Pitch***************

  // Calculate the magnitude of the accelerometer vector
  Accel_magnitude = sqrt(Accel_Vector[0]*Accel_Vector[0] + Accel_Vector[1]*Accel_Vector[1] + Accel_Vector[2]*Accel_Vector[2]);
  Accel_magnitude = Accel_magnitude / GRAVITY; // Scale to gravity.
  // Dynamic weighting of accelerometer info (reliability filter)
  // Weight for accelerometer info (<0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0)
  Accel_weight = constrain(1 - 2*abs(1 - Accel_magnitude),0,1);  //

  Vector_Cross_Product(errorRollPitch,Accel_Vector,DCM_Matrix[2]); //adjust the ground of reference
  Vector_Scale(Omega_P,errorRollPitch,Kp_ROLLPITCH*Accel_weight);

  Vector_Scale(Scaled_Omega_I,errorRollPitch,Ki_ROLLPITCH*Accel_weight);
  Vector_Add(Omega_I,Omega_I,Scaled_Omega_I);

  //*****YAW***************
  // We make the gyro YAW drift correction based on compass magnetic heading

  mag_heading_x = cos(MAG_Heading);
  mag_heading_y = sin(MAG_Heading);
  errorCourse=(DCM_Matrix[0][0]*mag_heading_y) - (DCM_Matrix[1][0]*mag_heading_x);  //Calculating YAW error
  Vector_Scale(errorYaw,DCM_Matrix[2],errorCourse); //Applys the yaw correction to the XYZ rotation of the aircraft, depeding the position.

  Vector_Scale(Scaled_Omega_P,errorYaw,Kp_YAW);//.01proportional of YAW.
  Vector_Add(Omega_P,Omega_P,Scaled_Omega_P);//Adding  Proportional.

  Vector_Scale(Scaled_Omega_I,errorYaw,Ki_YAW);//.00001Integrator
  Vector_Add(Omega_I,Omega_I,Scaled_Omega_I);//adding integrator to the Omega_I
}

void IMU_Matrix_update(void)
{
  Gyro_Vector[0]=GYRO_SCALED_RAD(IMU_gyro[0]); //gyro x roll
  Gyro_Vector[1]=GYRO_SCALED_RAD(IMU_gyro[1]); //gyro y pitch
  Gyro_Vector[2]=GYRO_SCALED_RAD(IMU_gyro[2]); //gyro z yaw

  Accel_Vector[0]=IMU_accel[0];
  Accel_Vector[1]=IMU_accel[1];
  Accel_Vector[2]=IMU_accel[2];

  Vector_Add(Omega, Gyro_Vector, Omega_I);  //adding proportional term
  Vector_Add(Omega_Vector, Omega, Omega_P); //adding Integrator term

  Update_Matrix[0][0]=0;
  Update_Matrix[0][1]=-G_Dt*Omega_Vector[2];//-z
  Update_Matrix[0][2]=G_Dt*Omega_Vector[1];//y
  Update_Matrix[1][0]=G_Dt*Omega_Vector[2];//z
  Update_Matrix[1][1]=0;
  Update_Matrix[1][2]=-G_Dt*Omega_Vector[0];//-x
  Update_Matrix[2][0]=-G_Dt*Omega_Vector[1];//-y
  Update_Matrix[2][1]=G_Dt*Omega_Vector[0];//x
  Update_Matrix[2][2]=0;

  Matrix_Multiply(DCM_Matrix,Update_Matrix,Temporary_Matrix); //a*b=c

  for(int x=0; x<3; x++) //Matrix Addition (update)
  {
    for(int y=0; y<3; y++)
    {
      DCM_Matrix[x][y]+=Temporary_Matrix[x][y];
    }
  }
}

void IMU_Euler_angles(void)
{
  IMU_Pitch = -asin(DCM_Matrix[2][0]);
  IMU_Roll = atan2(DCM_Matrix[2][1],DCM_Matrix[2][2]);
  IMU_Yaw = atan2(DCM_Matrix[1][0],DCM_Matrix[0][0]);

  IMU_Pitch_Final = IMU_Pitch - IMU_Pitch_Offset;
  IMU_Roll_Final = IMU_Roll - IMU_Roll_Offset;
  IMU_Yaw_Final = IMU_Yaw - IMU_Yaw_Offset;

  if (IMU_Roll < (PI/2)) IMU_Roll_Final += PI;
  else if (IMU_Roll > (PI/2)) IMU_Roll_Final -= PI;
}

float mag_x;
float mag_y;
float cos_roll;
float sin_roll;
float cos_pitch;
float sin_pitch;
void IMU_Compass_Heading(void)
{
	  cos_roll = cos(IMU_Roll);
	  sin_roll = sin(IMU_Roll);
	  cos_pitch = cos(IMU_Pitch);
	  sin_pitch = sin(IMU_Pitch);

	  // Tilt compensated magnetic field X
	  mag_x = IMU_magnetom[0]*cos_pitch + IMU_magnetom[1]*sin_roll*sin_pitch + IMU_magnetom[2]*cos_roll*sin_pitch;
	  // Tilt compensated magnetic field Y
	  mag_y = IMU_magnetom[1]*cos_roll - IMU_magnetom[2]*sin_roll;
	  // Magnetic Heading
	  MAG_Heading = atan2(-mag_y, mag_x);
}
