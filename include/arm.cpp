#include "arm.h"

RoboticArm::RoboticArm() {
    isGripperOpen = true;
    isInitialized = false;
}

void RoboticArm::begin() {
    Serial.println("Initializing 5-Servo Robotic Arm...");
    
    // Attach all servos
    baseServo.attach(BASE_PIN);
    shoulderServo.attach(SHOULDER_PIN);
    elbowServo.attach(ELBOW_PIN);
    wristServo.attach(WRIST_PIN);
    gripperServo.attach(GRIPPER_PIN);
    
    delay(100); // Allow servos to attach
    
    // Move to home position
    Serial.println("Moving to home position...");
    baseServo.write(HOME_BASE);
    delay(SERVO_DELAY);
    shoulderServo.write(HOME_SHOULDER);
    delay(SERVO_DELAY);
    elbowServo.write(HOME_ELBOW);
    delay(SERVO_DELAY);
    wristServo.write(HOME_WRIST);
    delay(SERVO_DELAY);
    gripperServo.write(HOME_GRIPPER);
    delay(SERVO_DELAY);
    
    // Run startup test
    Serial.println("Running startup servo test...");
    testAllServos();
    
    isInitialized = true;
    Serial.println("Robotic Arm initialized and ready!");
}

void RoboticArm::testAllServos() {
    Serial.println("Testing all servos...");
    
    // Test base servo
    Serial.println("  Testing base servo");
    baseServo.write(45);
    delay(800);
    baseServo.write(135);
    delay(800);
    baseServo.write(HOME_BASE);
    delay(500);
    
    // Test shoulder servo
    Serial.println("  Testing shoulder servo");
    shoulderServo.write(60);
    delay(800);
    shoulderServo.write(120);
    delay(800);
    shoulderServo.write(HOME_SHOULDER);
    delay(500);
    
    // Test elbow servo
    Serial.println("  Testing elbow servo");
    elbowServo.write(60);
    delay(800);
    elbowServo.write(120);
    delay(800);
    elbowServo.write(HOME_ELBOW);
    delay(500);
    
    // Test wrist servo
    Serial.println("  Testing wrist servo");
    wristServo.write(60);
    delay(800);
    wristServo.write(120);
    delay(800);
    wristServo.write(HOME_WRIST);
    delay(500);
    
    // Test gripper
    Serial.println("  Testing gripper");
    gripperServo.write(GRIPPER_CLOSED);
    isGripperOpen = false;
    delay(1000);
    gripperServo.write(GRIPPER_OPEN);
    isGripperOpen = true;
    delay(1000);
    
    Serial.println("Servo test complete!");
}

// WEB SERVER COMMAND FUNCTIONS 
String RoboticArm::pickUp() {
    if (!isInitialized) {
        return "{\"error\":\"Arm not initialized\"}";
    }
    
    Serial.println("Executing ball pickup sequence");
    
    // Open gripper first
    Serial.println("  Opening gripper");
    gripperServo.write(GRIPPER_OPEN);
    isGripperOpen = true;
    delay(SERVO_DELAY);
    
    // Move to pickup position
    Serial.println("  Moving to pickup position");
    shoulderServo.write(PICKUP_SHOULDER);
    delay(SERVO_DELAY);
    elbowServo.write(PICKUP_ELBOW);
    delay(SERVO_DELAY);
    wristServo.write(PICKUP_WRIST);
    delay(SERVO_DELAY);
    
    // Close gripper to grab ball
    Serial.println("  Closing gripper to grab ball");
    gripperServo.write(GRIPPER_CLOSED);
    isGripperOpen = false;
    delay(SERVO_DELAY);
    
    // Lift arm up
    Serial.println("  Lifting arm");
    shoulderServo.write(HOME_SHOULDER);
    delay(SERVO_DELAY);
    elbowServo.write(HOME_ELBOW);
    delay(SERVO_DELAY);
    
    Serial.println("Ball pickup sequence complete");
    return "{\"status\":\"Ball pickup completed\"}";
}

String RoboticArm::release() {
    if (!isInitialized) {
        return "{\"error\":\"Arm not initialized\"}";
    }
    
    Serial.println("Executing ball release sequence");
    
    // Move to release position
    Serial.println("  Moving to release position");
    baseServo.write(RELEASE_BASE);
    delay(SERVO_DELAY);
    shoulderServo.write(RELEASE_SHOULDER);
    delay(SERVO_DELAY);
    elbowServo.write(RELEASE_ELBOW);
    delay(SERVO_DELAY);
    wristServo.write(RELEASE_WRIST);
    delay(SERVO_DELAY);
    
    // Open gripper to release ball
    Serial.println("  Opening gripper to release ball");
    gripperServo.write(GRIPPER_OPEN);
    isGripperOpen = true;
    delay(SERVO_DELAY);
    
    Serial.println("  Ball released into dump truck");
    
    // Return to home
    Serial.println("  Returning to home position");
    baseServo.write(HOME_BASE);
    delay(SERVO_DELAY);
    shoulderServo.write(HOME_SHOULDER);
    delay(SERVO_DELAY);
    elbowServo.write(HOME_ELBOW);
    delay(SERVO_DELAY);
    wristServo.write(HOME_WRIST);
    delay(SERVO_DELAY);
    
    Serial.println("Ball release sequence complete");
    return "{\"status\":\"Ball released and arm returned home\"}";
}

String RoboticArm::scanLeft() {
    if (!isInitialized) {
        return "{\"error\":\"Arm not initialized\"}";
    }
    
    Serial.println("Scanning left for balls");
    baseServo.write(SCAN_LEFT);
    delay(SERVO_DELAY);
    
    return "{\"status\":\"Scanning left completed\"}";
}

String RoboticArm::scanRight() {
    if (!isInitialized) {
        return "{\"error\":\"Arm not initialized\"}";
    }
    
    Serial.println("Scanning right for balls");
    baseServo.write(SCAN_RIGHT);
    delay(SERVO_DELAY);
    
    return "{\"status\":\"Scanning right completed\"}";
}

String RoboticArm::open() {
    if (!isInitialized) {
        return "{\"error\":\"Arm not initialized\"}";
    }
    
    Serial.println("Opening gripper");
    gripperServo.write(GRIPPER_OPEN);
    isGripperOpen = true;
    
    return "{\"status\":\"Gripper opened\"}";
}

String RoboticArm::close() {
    if (!isInitialized) {
        return "{\"error\":\"Arm not initialized\"}";
    }
    
    Serial.println("Closing gripper");
    gripperServo.write(GRIPPER_CLOSED);
    isGripperOpen = false;
    
    return "{\"status\":\"Gripper closed\"}";
}

//  STATUS FUNCTIONS 
bool RoboticArm::getGripperState() {
    return isGripperOpen;
}

bool RoboticArm::isReady() {
    return isInitialized;
}

String RoboticArm::getStatus() {
    String status = "{";
    status += "\"initialized\":" + String(isInitialized ? "true" : "false") + ",";
    status += "\"gripper_open\":" + String(isGripperOpen ? "true" : "false") + ",";
    status += "\"ready\":" + String(isReady() ? "true" : "false");
    status += "}";
    return status;
}