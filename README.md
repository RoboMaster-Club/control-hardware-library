## About The Library

This is a shareable hardware library for robomaster embedded system. It contains board support packages, device-drivers, and algorithms for control systems.
## How To
Add the submodule to your project
```
git submodule add <https://github.com/jia-xie/control-hardware-library.git> FOLDER-NAME
```
Initialize the submodule
```
git submodule update --init
```
The submodule will exist as a folder in the project. In Git environment, it is a link pointing to a specific commit version. To manipulate the library itself, you can open the library directory and treat it as a regular Git repository.
```
cd FOLDER-NAME
git status
```
You should be able to view git information about this library.

## Motor Initialization

an example usage
```C
Motor_Config_t yaw_motor_config = {
        // Comm Config
        .can_bus = 1, // set can bus currently using
        .speed_controller_id = 3,
        .offset = 3690,
        
        // Motor Reversal Config (if motor is installed in 
        // opposite direction, change to MOTOR_REVERSAL_REVERSED)
        .motor_reversal = MOTOR_REVERSAL_NORMAL,
        
        //external sensor config
        .use_external_feedback = 1,
        .external_feedback_dir = 1, // 1 if the feedback matches with task space direction, 0 otherwise
        .external_angle_feedback_ptr = &g_imu.rad.yaw, // assign the pointer to the external angle feedback
        .external_velocity_feedback_ptr = &(g_imu.bmi088_raw.gyro[2]), // assign the poitner to the external velocity feedback
        
        // Controller Config
        .control_mode = POSITION_CONTROL, // Control Mode, see control mode for detail
        .angle_pid =
            {
                .kp = 20000.0f,
                .kd = 1000000.0f,
                .output_limit = GM6020_MAX_CURRENT,
            },
        .velocity_pid =
            {
                .kp = 500.0f,
                .output_limit = GM6020_MAX_CURRENT,
            },
    };
```

## Modifications
- Change ```samepleFreq``` in [MahonyAHRS.c](Algo/Src/MahonyAHRS.c?plain=1#L23), this will affect the fusion result
- Initialize a task for imu in FreeRTOS environment
