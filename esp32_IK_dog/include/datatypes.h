#ifndef __DATA_TYPES__
#define __DATA_TYPES__

#define mm 0.1 /// millimeter

class datatypes
{
public:
  /* ::: STEP, DATA TYPE ::: */
  struct Step
  {
    float base;
    float angle;
  };

  /* ::: VECTOR, DATA TYPE ::: */
  struct Vector
  {
    double x;
    double y;
    double z;
  };

  /* ::: 2D VECTOR, DATA TYPE ::: */
  struct Vector2D
  {
    float x;
    float y;
  };

  /* ::: ROTATOR, DATA TYPE ::: */
  struct Rotator
  {
    float yaw;
    float pitch;
    float roll;
  };

  /* ::: TRANFORM, DATA TYPE ::: */
  struct Transform
  {
    Vector pos;
    Rotator rot;
    Vector scl;
  };
};
#endif
