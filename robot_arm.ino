/*
  Simple Servo Test with Joint Class
  
  Reads integers from serial and writes them directly to servo.
  Uses a simple Joint class that wraps servo.write()
*/

#include <Servo.h>
#include "Joint.h"
#include <map>
#include <vector>

// Command structure for parsing
struct ParsedCommand {
  String function;
  std::vector<String> args;
  bool valid;
};

// Joint array for multi-joint control
Joint joints[] = {
  Joint(16), // Joint 0 (shoulder)
  Joint(17), // Joint 1 (elbow)
};
const int NUM_JOINTS = sizeof(joints) / sizeof(joints[0]);

// Function pointer type for command handlers
typedef void (*CommandHandler)(std::vector<String>&);

// Command dispatch table
std::map<String, CommandHandler> commandMap;

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  // Populate command dispatch table
  commandMap["MOV"] = handleMoveTo;    // MOV:joint,angle,speed
  commandMap["SET"] = handleSnapTo;    // SET:joint,angle
  commandMap["VEL"] = handleVelocity;  // VEL:joint,velocity
  commandMap["ADD"] = handleMoveBy;    // ADD:joint,increment,speed
  commandMap["JMP"] = handleSnapBy;    // JMP:joint,increment
  commandMap["STP"] = handleStop;      // STP:joint
  commandMap["START"] = handleStart;   // START:joint


  // Initialize joints
  joints[0].lbound = 15;        // Elbow bounds
  joints[0].start();
  
  joints[1].lbound = 60;        // Shoulder bounds
  joints[1].ubound = 120;
  joints[1].start();
  
  Serial.println("Ready");
}

// Helper function to parse command string
ParsedCommand parseCommand(String input) {
  ParsedCommand result;
  result.valid = false;
  
  input.trim();
  if (input.length() == 0) return result;
  
  int colonIndex = input.indexOf(':');
  if (colonIndex <= 0) return result;
  
  result.function = input.substring(0, colonIndex);
  result.function.toUpperCase();
  
  String argString = input.substring(colonIndex + 1);
  if (argString.length() == 0) return result;
  
  // Split arguments by comma
  int lastIndex = 0;
  int commaIndex = argString.indexOf(',');
  
  while (commaIndex >= 0) {
    String arg = argString.substring(lastIndex, commaIndex);
    arg.trim();
    if (arg.length() > 0) {
      result.args.push_back(arg);
    }
    lastIndex = commaIndex + 1;
    commaIndex = argString.indexOf(',', lastIndex);
  }
  
  String lastArg = argString.substring(lastIndex);
  lastArg.trim();
  if (lastArg.length() > 0) {
    result.args.push_back(lastArg);
  }
  
  result.valid = true;
  return result;
}

// Helper function to execute parsed command
bool executeCommand(ParsedCommand cmd) {
  if (!cmd.valid) return false;
  
  auto it = commandMap.find(cmd.function);
  if (it != commandMap.end()) {
    it->second(cmd.args);
    return true;
  }
  return false;  // Command not found
}

void loop() {
  // Update all joints
  for (int i = 0; i < NUM_JOINTS; i++) {
    joints[i].update();
  }
  
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    
    ParsedCommand cmd = parseCommand(input);
    if (!executeCommand(cmd)) {
      Serial.println("ERR");
    }
  }
}
