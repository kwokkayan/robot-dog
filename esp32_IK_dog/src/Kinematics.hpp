#ifndef __KINEMATICS__
#define __KINEMATICS__
#include "datatypes.h"
#include "Hardware.hpp"

class Kinematics {

  public:
    Kinematics(Hardware& h):
      base_offset{0, -1, 0, -2}, l_inv{
      { +1.f, -1.f}, // ## {dir, dir}
      { -1.f, -1.f}, // ## {dir, dir}
      { -1.f, +1.f}, // ## {dir, dir}
      { +1.f, +1.f}  // ## {dir, dir}
    }, hardware(h) {

    } //Kinematics
    //
    void handle_kinematics(int state, datatypes::Vector2D _dir, float _turn, float _height, float period);
    void count_c(int inst, datatypes::Vector2D dir, float period);
    datatypes::Vector trot_gait_func(datatypes::Vector2D c0, datatypes::Vector2D dir, bool inv);
    float c_base(float angle1);
    datatypes::Vector pitch_roll_axis(int leg, float base, datatypes::Rotator sRot);
    datatypes::Vector yaw_axis(int leg, float yaw);
    datatypes::Vector2D c_direction_ratio(datatypes::Vector2D dir);
    datatypes::Rotator k_model(float x0, float y0, float z0, float pitch_offset, float roll_offset, datatypes::Vector vec);
    datatypes::Rotator c_triangle(float a0, float a1, float b0);

  public:
    float base_offset[4];
    /// Kinematics Parameters
    float vrt_offset = -16.50;// ## mm
    float hrz_offset = -6.00;
    //: this is an interpolation function used to smooth
    static float inter(float in, float en, float pl) {
      if (in < en - pl) {
        return ((in * 1000.f) + (pl * 1000.f)) / 1000.0;
      } else if (in > en + pl) {
        return ((in * 1000.f) - (pl * 1000.f)) / 1000.0;
      } else
        return en;
    }
    static int sign(float num) {
      return int(num >= 0) - int(num < 0);
    }

  private:
    const float l_inv[4][2];
    float c[4];
    float c_iter[4];
    bool c_inv[4];
    float stored_0x = 0.f;
    Hardware& hardware;

    /*
      ==============================
      IMPORTANT PARAMETERS
      ==============================
    */
    const float precision = 0.001; // ## mm
    //> stores the frequency of the loop function
    const float frequency = 700.0; // ## Hz

    /// Kinematics Parameters
    const float bone_length = 105; // ## mm
    //: stores the location, rotation and scale of the main [body]
    const datatypes::Transform body_transform = {
      {0, 0, 0},     // ## {mm, mm, mm}
      {0, 0, 0},     // ## {deg, deg, deg}
      {300, 40, 180} // ## {mm, mm, mm}
    };
    //: stores the parent joint location relative to the [body]
    const datatypes::Vector p_joint_origin[4] = {
      { -50, 0, 0}, // ## {mm, mm, mm}
      { +50, 0, 0}, // ## {mm, mm, mm}
      { +50, 0, 0}, // ## {mm, mm, mm}
      { -50, 0, 0} // ## {mm, mm, mm}
    };
    //: high level parameters for the step function
    const datatypes::Vector step_extent = {40, 40, 26}; // ## {mm, mm}
};


#endif
