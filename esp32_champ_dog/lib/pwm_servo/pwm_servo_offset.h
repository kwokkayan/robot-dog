#ifndef PWM_SERVO_OFFSET_H
#define PWM_SERVO_OFFSET_H
// servo offset config
// #define MG996
#define MG958

#ifdef MG996
// 90 degrees
#define LOWER_OFFSET 1.5707999999999998f
#define SHOULDER_MARGIN 0.05f

#define LF_HIP_JOINT_OFFSET 0.039616040000000075f - SHOULDER_MARGIN
#define LF_UPPER_LEG_JOINT_OFFSET 0.08566096000000001f
#define LF_LOWER_LEG_JOINT_OFFSET -0.05707607f + LOWER_OFFSET

#define LH_HIP_JOINT_OFFSET -0.039616039999999964f + SHOULDER_MARGIN
#define LH_UPPER_LEG_JOINT_OFFSET 0.05717711999999997f
#define LH_LOWER_LEG_JOINT_OFFSET -0.02858856f + LOWER_OFFSET

#define RF_HIP_JOINT_OFFSET -0.04764396000000004f - SHOULDER_MARGIN
#define RF_UPPER_LEG_JOINT_OFFSET 0.047542880000000176f
#define RF_LOWER_LEG_JOINT_OFFSET -0.07142271f + LOWER_OFFSET

#define RH_HIP_JOINT_OFFSET -0.09511340000000001f + SHOULDER_MARGIN
#define RH_UPPER_LEG_JOINT_OFFSET -0.15226288f
#define RH_LOWER_LEG_JOINT_OFFSET 0.02858489f + LOWER_OFFSET
#endif

#ifdef MG958
// 90 degrees
#define LOWER_OFFSET 1.5707999999999998f
#define SHOULDER_MARGIN 0.05f

// #define LF_HIP_JOINT_OFFSET 0.024607319999999988f
// #define LF_UPPER_LEG_JOINT_OFFSET -0.12105632f
// #define LF_LOWER_LEG_JOINT_OFFSET LOWER_OFFSET + 0.0832524f
// // 0.0, -0.07079071999999997, -1.5374990399999997
// #define RF_HIP_JOINT_OFFSET -0.023734719999999987f
// #define RF_UPPER_LEG_JOINT_OFFSET -0.07079071999999997f
// #define RF_LOWER_LEG_JOINT_OFFSET LOWER_OFFSET + 0.03330096f

// #define LH_HIP_JOINT_OFFSET 0.12495632000000001f
// #define LH_UPPER_LEG_JOINT_OFFSET -0.12377903999999995f
// #define LH_LOWER_LEG_JOINT_OFFSET LOWER_OFFSET + 0.03811808f

// #define RH_HIP_JOINT_OFFSET -0.12356015999999992f + 0.06f
// #define RH_UPPER_LEG_JOINT_OFFSET -0.17530128f
// #define RH_LOWER_LEG_JOINT_OFFSET LOWER_OFFSET + 0.03330096f

/**
 * joint_names: [lf_hip_joint, lf_upper_leg_joint, lf_lower_leg_joint, rf_hip_joint, rf_upper_leg_joint,
  rf_lower_leg_joint, lh_hip_joint, lh_upper_leg_joint, lh_lower_leg_joint, rh_hip_joint,
  rh_upper_leg_joint, rh_lower_leg_joint]
points: 
  - 
    positions: [-0.06352528000000002, 0.019059039999999916, -1.43749144, -0.08726, 0.0, -1.5859843999999998, 
    0.19040131999999987, -0.1618971199999999, -1.50891048, -0.05549735999999994, -0.15226288, -1.6326895199999998]

*/

#define LF_HIP_JOINT_OFFSET -0.06352528000000002f
#define LF_UPPER_LEG_JOINT_OFFSET 0.00f
#define LF_LOWER_LEG_JOINT_OFFSET LOWER_OFFSET + (LOWER_OFFSET - 1.43749144f)

#define RF_HIP_JOINT_OFFSET -0.08726f
#define RF_UPPER_LEG_JOINT_OFFSET 0.00f
#define RF_LOWER_LEG_JOINT_OFFSET LOWER_OFFSET + (LOWER_OFFSET - 1.5859843999999998f)

#define LH_HIP_JOINT_OFFSET 0.19040131999999987f
#define LH_UPPER_LEG_JOINT_OFFSET -0.1618971199999999f
#define LH_LOWER_LEG_JOINT_OFFSET LOWER_OFFSET + (LOWER_OFFSET - 1.50891048f)

#define RH_HIP_JOINT_OFFSET -0.05549735999999994f
#define RH_UPPER_LEG_JOINT_OFFSET -0.15226288f
#define RH_LOWER_LEG_JOINT_OFFSET LOWER_OFFSET + (LOWER_OFFSET - 1.6326895199999998f)

#endif

#endif