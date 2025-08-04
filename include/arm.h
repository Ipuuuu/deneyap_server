#ifndef ARM_H
#define ARM_H

#include <Arduino.h>
// #include <Servo.h>
#include <ESP32Servo.h>

class RoboticArm {
public:
    // Servo objects
    Servo baseServo;
    Servo shoulderServo;
    Servo elbowServo;
    Servo wristServo;
    Servo gripperServo;
    Servo dumperServo;
    
    // Pin assignments
    static const uint8_t BASE_PIN = D0; // PURPLE
    static const uint8_t SHOULDER_PIN = D12; //GREEN
    static const uint8_t ELBOW_PIN = D13; //YELLOW
    static const uint8_t WRIST_PIN = D14; //ORANGE
    static const uint8_t GRIPPER_PIN = D1; // BLUE
    static const uint8_t DUMPER_PIN = D8; // RED
    
    // Calibration angles
    static const uint8_t HOME_BASE = 90;
    static const uint8_t HOME_SHOULDER = 90;
    static const uint8_t HOME_ELBOW = 90;
    static const uint8_t HOME_WRIST = 90;
    static const uint8_t HOME_GRIPPER = 10;
    static const uint8_t HOME_DUMPER = 0;
    
    static const uint8_t PICKUP_SHOULDER = 45;
    static const uint8_t PICKUP_ELBOW = 135;
    static const uint8_t PICKUP_WRIST = 45;
    
    static const uint8_t RELEASE_BASE = 180;
    static const uint8_t RELEASE_SHOULDER = 120;
    static const uint8_t RELEASE_ELBOW = 90;
    static const uint8_t RELEASE_WRIST = 90;
    
    static const uint8_t GRIPPER_OPEN = 10;
    static const uint8_t GRIPPER_CLOSED = 80;
    
    static const uint8_t SCAN_LEFT = 45;
    static const uint8_t SCAN_RIGHT = 135;
    static const uint8_t DUMPER_OPEN = 90;
    static const uint8_t DUMPER_CLOSED = 0;
    
    
    // Timing
    static const uint16_t SERVO_DELAY = 500;
    
    // State tracking
    bool isGripperOpen;
    bool isInitialized;
    
public:
    // Constructor
    RoboticArm();
    
    // Initialization
    void begin();
    void testAllServos();
    
    // Web server command functions
    String pickUp();      // Returns JSON status
    String release();     // Returns JSON status
    String scanLeft();    // Returns JSON status
    String scanRight();   // Returns JSON status
    String open();        // Returns JSON status
    String close();       // Returns JSON status
    String dumperOpen();    // Returns JSON status
    String dumperClose();   // Returns JSON status
    
    // Status functions
    bool getGripperState(); // Returns true if open, false if closed
    bool isReady();         // Returns true if initialized
    String getStatus();     // Returns JSON status of arm
};

#endif // ARM_H