#include <Arduino.h>
#include "Kinematics.hpp"
#include "Config.hpp"

void Kinematics::handle_kinematics(int state, datatypes::Vector2D _dir, float _turn, float _height, float period)
{
  for (int l = 0; l < 4; l++)
  {
    float base = c_base(90) + base_offset[l] + _height; //> stores the base of each leg

    datatypes::Vector2D dir = {
      precision + _dir.x + (_turn)*l_inv[l][1],
      precision + _dir.y + (_turn)*l_inv[l][0]
    };
    count_c(l, dir, period); //> calls the clock function

    datatypes::Vector2D rDir = c_direction_ratio(dir);
    datatypes::Vector vector = {0, 0, 0}; //> default leg coordinates

    //: these functions run for each leg and return a 3 dimensional vector that stores the desired leg position in cartesian coordinates
    if (state == 1 && (abs(dir.x) > precision || abs(dir.y) > precision))
      vector = trot_gait_func({rDir.x * c[l], l_inv[l][1] * rDir.y * c[l]},
                              dir, boolean(l % 2) ^ boolean(c_inv[l] % 2));
    else if (state == 2)
      vector = yaw_axis(l, _turn);
    else if (state == 3)
      vector = pitch_roll_axis(l, base, {0, _dir.x, _dir.y});
    else if (state == 4)
    {
      vector = yaw_axis(l, stored_0x);
      if (abs(stored_0x + _turn / 4.f) < 32.f)
        stored_0x = inter(stored_0x, stored_0x + _turn / 4.f, 0.5f);
    }

    //: this datatype stores three values which correspond to the three joint angles of each leg,
    /// the 3 dimensional vector is converted through the k_model function into these three angles.
    datatypes::Rotator cRot = k_model(vrt_offset, hrz_offset, base,
                                      0, 0, vector);
    hardware.set_leg(l, cRot); //> this function sets the three servos of each leg to the calculated value
  }
}

void Kinematics::count_c(int inst, datatypes::Vector2D dir, float period)
{
  float w0 = step_extent.x * mm / (2 / max(abs(dir.x), abs(dir.y)));
  float a0 = (w0 * 2) * (c_iter[inst] / round(frequency / period)) - w0;

  c[inst] = a0;
  c_iter[inst] += 1.f;

  if (c_iter[inst] > round(frequency / period))
  {
    c[inst] = -w0;
    c_iter[inst] = 1.f;

    c_inv[inst] = !c_inv[inst];
  }
}

/*
  ::: [KINEMATICS] FUNCTIONS :::
*/

/*
     ::: GAIT FUNCTIONS :::
*/

//: trot function
datatypes::Vector Kinematics::trot_gait_func(datatypes::Vector2D c0, datatypes::Vector2D dir, bool inv)
{
  float w0 = step_extent.x * mm / 2 * dir.x;
  float l0 = step_extent.y * mm * 4 * dir.y;
  float h0 = step_extent.z * mm;

  if (inv == false)
    c0 = { -c0.x, -c0.y};

  float h1 = sqrt(abs((1 - sq(c0.x / w0) - sq(c0.y / l0)) * sq(h0)));
  return {c0.x / mm, c0.y / mm, h1 / mm * int(inv)};
}

/*
  ::: TRIGONOMETRIC FUNCTIONS :::
*/

//: base calculation function
float Kinematics::c_base(float angle1)
{
  return sin(radians(angle1 / 2)) * bone_length * 2;
}

//: pitch-roll axis function
datatypes::Vector Kinematics::pitch_roll_axis(int leg, float base, datatypes::Rotator sRot)
{
  float w0 = body_transform.scl.x / 2 * l_inv[leg][0] + p_joint_origin[leg].x - vrt_offset;
  float l0 = body_transform.scl.z / 2 + hrz_offset;

  float C0 = radians(sRot.pitch);
  float C1 = radians(sRot.roll) * l_inv[leg][1];

  float a0 = sin(C0) * w0;
  float a1 = sin(C1) * l0;

  float d0 = (1 - cos(C0)) * -w0;
  float d1 = (1 - cos(C1)) * l0;

  float var0 = sqrt(sq(base + a0) + sq(d0));
  C0 += asin(d0 / var0);

  float b0 = cos(C0) * var0;
  float c0 = sin(C0) * var0;

  float var1 = sqrt(sq(b0 - a1) + sq(d1));
  C1 += asin(d1 / var1);

  float b1 = cos(C1) * var1;
  float c1 = sin(C1) * var1;

  return {c0, c1, base - b1};
}

//: yaw axis function
datatypes::Vector Kinematics::yaw_axis(int leg, float yaw)
{
  float x = body_transform.scl.x / 2 - abs(p_joint_origin[leg].x) - vrt_offset * l_inv[leg][0];
  float y = body_transform.scl.z / 2 + hrz_offset;
  float radius = sqrt(sq(x) + sq(y));
  float angle = asin(y / radius) - radians(yaw) * l_inv[leg][0] * l_inv[leg][1];

  float rX = (x - cos(angle) * radius) * l_inv[leg][0];
  float rY = sin(angle) * radius - y;
  return {rX, rY, 0};
}

//: direction ratio calculation function
datatypes::Vector2D Kinematics::c_direction_ratio(datatypes::Vector2D dir)
{
  float dirX = dir.x / max(abs(dir.x), abs(dir.y));
  float dirY = dir.y / max(abs(dir.x), abs(dir.y));
  return {dirX, dirY};
}

//: inverse kinematic algorithm
datatypes::Rotator Kinematics::k_model(float x0, float y0, float z0,
                                       float pitch_offset, float roll_offset, datatypes::Vector vec)
{
  float x = x0 + vec.x,
        y = y0 + vec.y,
        z = z0 - vec.z;

  float b0 = sqrt(sq(x) + sq(y));
  float h0 = sqrt(sq(b0) + sq(z));

  float a0 = degrees(atan(x / z));
  float a1 = degrees(atan(y / z));

  return c_triangle(a0 + pitch_offset, a1 + roll_offset, h0);
}

//: final triangle calculation function
datatypes::Rotator Kinematics::c_triangle(float a0, float a1, float b0)
{
  float angle1 = a1;
  float angle3 = degrees(asin((b0 / 2.0) / bone_length)) * 2;
  float angle2 = angle3 / 2 + a0;

  return {angle1, angle2, angle3};
}
