void handleMoveTo(std::vector<String>& args) {
  int jointIndex = args[0].toInt();
  int angle = args[1].toInt();
  float speed = args[2].toFloat();
  
  if (jointIndex >= 0 && jointIndex < NUM_JOINTS) {
    joints[jointIndex].moveTo(angle, speed);
  }
}

void handleSnapTo(std::vector<String>& args) {
  int jointIndex = args[0].toInt();
  int angle = args[1].toInt();
  
  if (jointIndex >= 0 && jointIndex < NUM_JOINTS) {
    joints[jointIndex].snapTo(angle);
  }
}

void handleVelocity(std::vector<String>& args) {
  int jointIndex = args[0].toInt();
  float velocity = args[1].toFloat();
  
  if (jointIndex >= 0 && jointIndex < NUM_JOINTS) {
    joints[jointIndex].setAngularVel(velocity);
  }
}

void handleMoveBy(std::vector<String>& args) {
  int jointIndex = args[0].toInt();
  float increment = args[1].toFloat();
  float speed = args[2].toFloat();
  
  if (jointIndex >= 0 && jointIndex < NUM_JOINTS) {
    joints[jointIndex].moveBy(increment, speed);
  }
}

void handleSnapBy(std::vector<String>& args) {
  int jointIndex = args[0].toInt();
  float increment = args[1].toFloat();
  
  if (jointIndex >= 0 && jointIndex < NUM_JOINTS) {
    joints[jointIndex].snapBy(increment);
  }
}

void handleStop(std::vector<String>& args) {
  int jointIndex = args[0].toInt();
  
  if (jointIndex >= 0 && jointIndex < NUM_JOINTS) {
    joints[jointIndex].stop();
  }
}

void handleStart(std::vector<String>& args) {
  int jointIndex = args[0].toInt();
  
  if (jointIndex >= 0 && jointIndex < NUM_JOINTS) {
    joints[jointIndex].start();
  }
}
