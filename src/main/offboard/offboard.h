


typedef struct offboard
{
    // 单位是度（°）
    float yaw_angle;
    float pitch_angle;
    float roll_angle;

    // 单位是度每秒*（°/s）
    float yaw_angle_rate;
    float pitch_angle_rate;
    float roll_angle_rate;



}offboard_t;

offboard_t offboard;
