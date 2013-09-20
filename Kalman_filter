// ---   kalman module  ----------------------------------------------------------------------------


float kalmanCalculateX(float newAngle,float newRate,int looptime) 
{
  static float angle = 0;
  static float bias = 0;
  static float P_00 = 0, P_01 = 0, P_10 = 0, P_11 = 0;
  static float dt, y, S;
  static float K_0, K_1;
  dt = float(looptime)/1000;
  angle += dt * (newRate - bias);
  P_00 +=  - dt * (P_10 + P_01) + Q_angle * dt;
  P_01 +=  - dt * P_11;
  P_10 +=  - dt * P_11;
  P_11 +=  + Q_gyro * dt;

  y = newAngle - angle;
  S = P_00 + R_angle;
  K_0 = P_00 / S;
  K_1 = P_10 / S;

  angle +=  K_0 * y;
  bias  +=  K_1 * y;
  P_00 -= K_0 * P_00;
  P_01 -= K_0 * P_01;
  P_10 -= K_1 * P_00;
  P_11 -= K_1 * P_01;
  return angle;
}

float kalmanCalculateY(float newAngle,float newRate,int looptime) 
{
  static float angle = 0;
  static float bias = 0;
  static float P_00 = 0, P_01 = 0, P_10 = 0, P_11 = 0;
  static float dt, y, S;
  static float K_0, K_1;
  dt = float(looptime)/1000;
  angle += dt * (newRate - bias);
  P_00 +=  - dt * (P_10 + P_01) + Q_angle * dt;
  P_01 +=  - dt * P_11;
  P_10 +=  - dt * P_11;
  P_11 +=  + Q_gyro * dt;

  y = newAngle - angle;
  S = P_00 + R_angle;
  K_0 = P_00 / S;
  K_1 = P_10 / S;

  angle +=  K_0 * y;
  bias  +=  K_1 * y;
  P_00 -= K_0 * P_00;
  P_01 -= K_0 * P_01;
  P_10 -= K_1 * P_00;
  P_11 -= K_1 * P_01;
  return angle;
}

float kalmanCalculateZ(float newAngle,float newRate,int looptime) 
{
  static float angle = 0;
  static float bias = 0;
  static float P_00 = 0, P_01 = 0, P_10 = 0, P_11 = 0;
  static float dt, y, S;
  static float K_0, K_1;
  dt = float(looptime)/1000;
  angle += dt * (newRate - bias);
  P_00 +=  - dt * (P_10 + P_01) + Q_angle * dt;
  P_01 +=  - dt * P_11;
  P_10 +=  - dt * P_11;
  P_11 +=  + Q_gyro * dt;

  y = newAngle - angle;
  S = P_00 + R_angle;
  K_0 = P_00 / S;
  K_1 = P_10 / S;

  angle +=  K_0 * y;
  bias  +=  K_1 * y;
  P_00 -= K_0 * P_00;
  P_01 -= K_0 * P_01;
  P_10 -= K_1 * P_00;
  P_11 -= K_1 * P_01;
  return angle;
}
