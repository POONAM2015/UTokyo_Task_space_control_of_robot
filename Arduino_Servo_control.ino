#include <Servo.h>
#include <math.h>

Servo myservos[6];  // create servo object to keep track of 6 servo states
long int pot_values[6] {};
long int servo_angles[6] {};

// robot arm measurements are in mm (in actual)
constexpr long int BASE_HEIGHT = 75;
constexpr long int P = 85;
constexpr long int Q = 160;


constexpr long int servo_ports[] = {3, 5, 6, 9, 10, 11};
constexpr long int potentiometer_ports[] = {0, 1, 2, 3, 4, 5};

// control space of robot (measurements are in mm)
constexpr long int XMIN = 20;
constexpr long int XMAX = 275;
constexpr long int YMIN = -160;
constexpr long int YMAX = 160;
constexpr long int ZMIN = 0;
constexpr long int ZMAX = 200;

long int x{Q}, y{}, z{P}, t1{}, t2{}, t3{};
bool error2{};
bool error3{};

// Convert an angle from radian to degree
long int rad2deg(const double rad)
{
  return 180 * rad / M_PI;
}

// Convert an angle from degree to radian
double deg2rad(const long int deg)
{
  return M_PI * deg / 180;
}

//calculates servo angle 1 (outputs in a range of -90 to +90 degrees)
long int xy_to_t1(const long int x, const long int y)
{
  return rad2deg(atan2(y, x));
}

//calculates servo angle 2 (outputs in a range of -90 to +90 degrees)
double xyz_to_t2(const long int x, const long int y, const long int z)
{
  const double r = sqrt(x * x + y * y);
  const double R = sqrt(r * r + z * z);
  const long int phi = rad2deg(atan2(z, r));

  const double cos_theta_q = ((double)P * P + (double)R * R - (double)Q * Q) / (2.0 * P * R);
  if (cos_theta_q > 1 || cos_theta_q < -1)
  {
    error2 = true;
    return 0;
  }
  else
  {
    long int theta_q = rad2deg(acos(cos_theta_q));
    error2 = false;
    return 90 - (theta_q + phi);
  }
}

double xyz_to_t3(const long int x, const long int y, const long int z)
{
  const double r = sqrt(x * x + y * y);
  const double R = sqrt(r * r + z * z);

  const double cos_theta = ((double)P * P + (double)Q * Q - R * R) / (2.0 * P * Q);
  if (cos_theta > 1 || cos_theta < -1)
  {
    error3 = true;
    return 0;
  }
  else
  {
    error3 = false;
    return rad2deg(acos(cos_theta)) - 90;
  }
}

long int limit_value(const long int value, const long int min_value = -90, const long int max_value = 90)
{
  if (value < min_value)
    return min_value;
  else if (value > max_value)
    return max_value;
  return value;
}

void setup() {
  for (long int i = 0; i < 6; ++i)
  {
    myservos[i].attach(servo_ports[i]);
  }
  Serial.begin(9600);

  servo_angles[0] = 90;
  servo_angles[1] = 90;
  servo_angles[2] = 90;
  servo_angles[3] = 90;
  servo_angles[4] = 90;
  servo_angles[5] = 180;

  for (long int i = 0; i < 6; ++i)
  {
    myservos[i].write(servo_angles[i]);
    delay(200);
  }
}

void loop() {

  for (long int i = 0; i < 6; ++i)
  {
    pot_values[i] = analogRead(potentiometer_ports[i]);
  }

  x = map(pot_values[0], 0, 1023, XMIN, XMAX);
  y = map(pot_values[1], 0, 1023, -YMIN, -YMAX);
  z = map(pot_values[2], 0, 1023, ZMAX, ZMIN);

  long int r = sqrt(x * x + y * y);
  long int phi = rad2deg(atan2(z, r));


  double t2 = xyz_to_t2(x, y, z);
  double t3 = xyz_to_t3(x, y, z);

  if (!error2 && !error3)
  {
    servo_angles[1] = map(t2, -90, 90, 0, 180);
    servo_angles[2] = map(t3, -90, 90, 0, 180);
  }

  t1 = xy_to_t1(x, y);
  servo_angles[0] = map(t1, -90, 90, 0, 180);
  servo_angles[3] = map(pot_values[3], 0, 1023, 0, 180);

  long int end_effector_target_angle = map(pot_values[4], 0, 1023, -90, 90);
  servo_angles[4] = limit_value(end_effector_target_angle + t2 - t3);
  servo_angles[4] = map(servo_angles[4], -90, 90, 0, 180);
  servo_angles[5] = map(pot_values[5], 0, 1023, 110, 150);

  for (long int i = 0; i < 6; ++i)
  {
    myservos[i].write(servo_angles[i]);
  }

  delay(5);
}
