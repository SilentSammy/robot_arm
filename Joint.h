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
  unsigned long goalTime;  // Time when target was reached (-1 if not at target)
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
    goalTime = -1;  // Not at target initially - force this even though currentAngle == target
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
    goalTime = -1;  // Reset goal tracking when new target is set
  }
  
  // Helper method: Move by increment at specified speed
  void moveBy(float angleIncrement, float moveSpeed) {
    target = currentAngle + angleIncrement;
    speed = moveSpeed;
    goalTime = -1;  // Reset goal tracking when new target is set
  }

  // Helper method: Snap instantly to angle (like standard servo.write)
  void snapTo(float angle) {
    speed = 32767.0;  // Max int value for instant movement
    target = angle;
    goalTime = -1;  // Reset goal tracking when new target is set
  }
  
  // Helper method: Snap instantly by increment
  void snapBy(float angleIncrement) {
    speed = 32767.0;  // Max int value for instant movement
    target = currentAngle + angleIncrement;
    goalTime = -1;  // Reset goal tracking when new target is set
  }
  
  // Helper method: Set angular velocity (continuous rotation style)
  void setAngularVel(float avel) {
    speed = abs(avel);  // Use magnitude for speed
    target = (avel >= 0) ? 180.0 : 0.0;  // Direction determines target
    goalTime = -1;  // Reset goal tracking when new target is set
  }

  long getTimeAtGoal() {
    if (goalTime == -1) {
      return -1;  // Not at goal
    } else {
      return millis() - goalTime;  // Time elapsed since reaching goal
    }
  }

  void updateGoal() {
    float tolerance = 0;  // Close enough threshold
    
    // Check if we're at goal: either reached target position OR velocity is zero (stopped)
    bool atGoal = (abs(currentAngle - target) <= tolerance) || (speed == 0);
    
    if (!atGoal) {
      // Not at target and not stopped
      goalTime = -1;
    } else {
      // At target or stopped - set goalTime only on first arrival
      if (goalTime == -1) {
        goalTime = millis();
      }
    }
  }

  void update() {
    // Auto-start/stop logic - check this BEFORE the early return
    long timeAtGoal = getTimeAtGoal();
    if (timeAtGoal > 2000) {  // 2 seconds at goal
      if (isAttached) {
        stop();
      }
    } else if (timeAtGoal == -1) {
      // Not at goal - make sure servo is started
      if (!isAttached) {
        start();
      }
    }
    
    // Only update movement if servo is attached
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
    
    // Update goal tracking
    updateGoal();
  }
};

#endif
