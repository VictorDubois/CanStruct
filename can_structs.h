#pragma once

namespace CAN
{
    enum can_ids
    {
        // priority, in case we need to shutdown
        DIGITAL_OUTPUTS = 1, // DigitalOutputs

        // 10/20Hz
        SERVO_1 = 10, // ServoMessage
        SERVO_2,      // ServoMessage
        SERVO_3,      // ServoMessage
        SERVO_4,      // ServoMessage
        SERVO_5,      // ServoMessage
        SERVO_6,      // ServoMessage
        SERVO_7,      // ServoMessage

        AX12_W1 = 20, // AX12Write
        AX12_W2,      // AX12Write
        AX12_W3,      // AX12Write
        AX12_W4,      // AX12Write
        AX12_W5,      // AX12Write
        AX12_W6,      // AX12Write

        AX12_R1 = 30, // AX12Read
        AX12_R2,      // AX12Read
        AX12_R3,      // AX12Read
        AX12_R4,      // AX12Read
        AX12_R5,      // AX12Read
        AX12_R6,      // AX12Read

        STEPPER_CMD,  // Stepper
        STEPPER_INFO, // StepperInfo

        // 10/20Hz
        CMD_VEL,       // CmdVel
        CMD_VEL_FLOAT, // CmdVelFloat

        // High frequency, ~100Hz
        ODOMETRY_LIGHT,       // OdometryLight
        ODOMETRY_XY,          // OdometryXY
        ODOMETRY_XY_FLOAT,    // OdometryXYFloat
        ODOMETRY_THETA,       // OdometryThetaAndCurrent
        ODOMETRY_THETA_FLOAT, // OdometryThetaFloat
        ODOMETRY_SPEED,       // SpeedOdometry
        ODOMETRY_SPEED_FLOAT, // SpeedOdometryFloat

        // Low frequency
        MOTOR_BOARD_CURRENT_OUPUT, // MotorBoardCurrentOutput
        MOTOR_BOARD_CMD_INPUT,     // MotorBoardCmdInput
        MOTOR_BOARD_CURRENT_INPUT, // MotorBoardCurrentInput
        MOTOR_BOARD_ENABLE,        // MotorEnable

        ANALOG_SENSORS = 60, // AnalogSensors
        DIGITAL_INPUTS,      // DigitalInputs

        SCORE = 100 // Score
    };

    enum stepper_mode
    {
        DISABLE = 0,
        POSITION = 10,
        SPEED = 20,
        HOMING = 30
    };

#pragma pack(push, 1) // Ensure tight packing of the struct
    struct ServoMessage
    {
        uint8_t angle_s1;
        uint8_t speed_s1; // 0 means disable the servo
        uint8_t angle_s2;
        uint8_t speed_s2; // 0 means disable the servo
        uint8_t angle_s3;
        uint8_t speed_s3; // 0 means disable the servo
        uint8_t angle_s4;
        uint8_t speed_s4; // 0 means disable the servo
    };

    struct Score
    {
        uint8_t score;               // 1 byte
        uint8_t remaining_time_s;    // 1 bytes
        int8_t isBlue_1_otherwise_0; // 1 bytes
        int8_t _unused;              // 1 bytes
        int32_t _unused2;            // 4 bytes
    };

    struct AX12Write
    {
        uint8_t mode;             // 1 byte
        int16_t position;         // 2 bytes
        uint8_t max_accel;        // 1 byte
        uint8_t max_speed;        // 1 byte
        uint8_t torque_enable;    // 1 byte
        uint8_t temperatureLimit; // 1 byte
        uint8_t currentLimit;     // 1 byte
    };

    struct AX12Read
    {
        uint8_t hardwareErrorStatus; // 1 byte
        int16_t current_position;    // 2 bytes
        uint8_t presentTemperature;  // 1 byte
        uint16_t presentCurrent;     // 2 byte
        uint8_t moving;              // 1 byte
        uint8_t mode;                // 1 byte
    };

    struct DigitalInputs
    {
        // array of inputs
        int8_t digital_inputs; // 1 byte
        int8_t _unused;        // 1 bytes
        int16_t _unused2;      // 2 bytes
        int32_t _unused3;      // 4 bytes
    };

    struct DigitalOutputs
    {
        // array of ouputs
        int16_t enable_outputs; // 2 bytes
        int8_t enable_power;    // 1 byte
        int8_t _unused;         // 1 bytes
        int32_t _unused2;       // 4 bytes
    };

    struct AnalogSensors
    {
        int16_t battery_power_mV; // 2 bytes
        int16_t battery_elec_mV;  // 2 bytes
        int16_t vacuum_1;         // 2 bytes
        int16_t vacuum_2;         // 2 bytes
    };

    struct CmdVelFloat
    {
        float linear_x_m;    // 4 bytes
        float angular_z_rad; // 4 bytes
    };

    struct CmdVel
    {
        int32_t linear_x_um_s;    // 4 bytes
        int32_t angular_z_urad_s; // 4 bytes
    };

    struct MotorBoardCurrentInput
    {
        int16_t max_current_left_mA;  // 2 bytes
        int16_t max_current_right_mA; // 2 bytes
        int16_t max_current_mA;       // 2 bytes
        int16_t _unused;              // 2 bytes
    };

    struct MotorBoardCmdInput
    {
        int8_t enable_motors;       // 1 bytes
        int8_t override_PWM;        // 1 bytes
        int16_t PWM_override_left;  // 2 bytes - valeur brute à envoyer au PWM
        int16_t PWM_override_right; // 2 bytes - valeur brute à envoyer au PWM
        int8_t reset_encoders;      // 1 bytes
        int8_t _unused;             // 1 bytes
    };

    struct MotorBoardCurrentOutput
    {
        int16_t current_left_mA;      // 2 bytes
        int16_t current_right_mA;     // 2 bytes
        int16_t remaining_timeout_ms; // 2 bytes
        int16_t motor_battery_mV;     // 2 bytes
    };

    struct OdometryLight
    {
        int8_t poseX_complete;     // 1 bytes
        int16_t poseX_mm;          // 2 bytes
        int8_t poseY_complete;     // 1 bytes
        int16_t poseY_mm;          // 2 bytes
        int16_t angleRz_centi_deg; // 2 bytes
    };

    struct OdometryXY
    {
        int32_t poseX_mm; // 4 bytes
        int32_t poseY_mm; // 4 bytes
    };

    struct OdometryThetaAndCurrent
    {
        int32_t angleRz_centi_deg; // 4 bytes
        int16_t currentLeft;       // 2 bytes
        int16_t currentRight;      // 2 bytes
    };

    struct OdometryXYFloat
    {
        float poseX_m; // 4 bytes
        float poseY_m; // 4 bytes
    };

    struct OdometryThetaFloat
    {
        float angleRz;   // 4 bytes
        int32_t _unused; // 4 bytes
    };

    struct SpeedOdometry
    {
        int32_t speedVx_um_s;   // 4 bytes
        int32_t speedWz_mrad_s; // 4 bytes
    };

    struct SpeedOdometryFloat
    {
        float speedVx_m_s;   // 4 bytes
        float speedWz_rad_s; // 4 bytes
    };

    struct Stepper
    {
        uint16_t speed;   // mm/s 0 => disable
        uint16_t accel;   // mm/s²
        int16_t position; // mm
        uint8_t current;  // *50mA
        uint8_t mode;     // 0 => disable, 10 => position, 20 => speed, 30 => homing
    };

    struct StepperInfo
    {
        uint16_t distance_to_go; // mm
        uint8_t homing_sequences_done;
        uint8_t homing_switches_on;
        int32_t _unused; // 4 bytes
    };

    struct MotorEnable
    {
        uint8_t motor_enable;
        int8_t _unused;   // 1 bytes
        int16_t _unused2; // 2 bytes
        int32_t _unused3; // 4 bytes
    };

#pragma pack(pop)

    /*
    class CANMessage {
      public : uint32_t id = 0 ;  // Frame identifier
      public : bool ext = false ; // false -> standard frame, true -> extended frame
      public : bool rtr = false ; // false -> data frame, true -> remote frame
      public : uint8_t idx = 0 ;  // This field is used by the driver
      public : uint8_t len = 0 ;  // Length of data (0 ... 8)
      public : union {
            uint64_t data64        ; // Caution: subject to endianness
            int64_t  data_s64      ; // Caution: subject to endianness
            uint32_t data32    [2] ; // Caution: subject to endianness
            int32_t  data_s32  [2] ; // Caution: subject to endianness
            float    dataFloat [2] ; // Caution: subject to endianness
            uint16_t data16    [4] ; // Caution: subject to endianness
            int16_t  data_s16  [4] ; // Caution: subject to endianness
            int8_t   data_s8   [8] ;
            uint8_t  data      [8] = {0, 0, 0, 0, 0, 0, 0, 0} ;
      } ;
    } ;
    */
}; // namespace CAN
