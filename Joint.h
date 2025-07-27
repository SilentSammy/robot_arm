#ifndef JOINT_H
#define JOINT_H

#include <Servo.h>

class Joint {
private:
  Servo servo;
  int pin;
  float currentAngle;
  unsigned long lastUpdateTime;
  float target;
  float speed;
  bool isAttached;  // Track servo attachment state
public:
  float lbound;  // Lower bound (minimum angle)
  float ubound;  // Upper bound (maximum angle)
  
  Joint(int servoPin) {
    pin = servoPin;
    currentAngle = 90.0;
    target = currentAngle;
    speed = 1.0;  // Default 1 degree/second
    lastUpdateTime = 0;
    lbound = 0.0;    // Default lower bound
    ubound = 180.0;  // Default upper bound
    isAttached = false;  // Start detached
  }
  
  void start() {
    servo.attach(pin, 600, 2400);
    lastUpdateTime = millis();
    isAttached = true;
  }
  
  void stop() {
    servo.detach();
    isAttached = false;
  }
  
  bool isStarted() {
    return isAttached;
  }
  
  void write(float angle) {
    // Only write if servo is attached
    if (!isAttached) return;
    
    // Clamp angle to bounds before writing
    angle = max(lbound, min(angle, ubound));
    servo.write(angle);
    currentAngle = angle;  // Remember the angle we just wrote
  }
  
  // Helper method: Move to target at specified speed
  void moveTo(float targetAngle, float moveSpeed) {
    target = targetAngle;
    speed = moveSpeed;
  }
  
  // Helper method: Move by increment at specified speed
  void moveBy(float angleIncrement, float moveSpeed) {
    target = currentAngle + angleIncrement;
    speed = moveSpeed;
  }

  // Helper method: Snap instantly to angle (like standard servo.write)
  void snapTo(float angle) {
    speed = 32767.0;  // Max int value for instant movement
    target = angle;
  }
  
  // Helper method: Snap instantly by increment
  void snapBy(float angleIncrement) {
    speed = 32767.0;  // Max int value for instant movement
    target = currentAngle + angleIncrement;
  }
  
  // Helper method: Set angular velocity (continuous rotation style)
  void setAngularVel(float avel) {
    speed = abs(avel);  // Use magnitude for speed
    target = (avel >= 0) ? 180.0 : 0.0;  // Direction determines target
  }

  void update() {
    // Only update if servo is attached
    if (!isAttached) return;
    
    unsigned long currentTime = millis();
    float dt = (currentTime - lastUpdateTime) / 1000.0;  // Convert to seconds
    
    float dtheta = speed * dt;  // Calculate angle change based on speed
    float nextAngle = currentAngle;
    
    // Move toward target
    if (currentAngle < target) {
      nextAngle = min(currentAngle + dtheta, target);  // Move up, don't overshoot
    } else if (currentAngle > target) {
      nextAngle = max(currentAngle - dtheta, target);  // Move down, don't overshoot
    }
    
    write(nextAngle);
    lastUpdateTime = currentTime;
  }
};

#endif
