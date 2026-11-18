#include <Arduino.h>

// Direction enumeration
enum Direction {
  STOPPED,
  CLOCKWISE,         // Pulling the belt out
  COUNTER_CLOCKWISE  // Belt retracting
};

// Passenger state enumeration
enum BeltState {
  UNKNOWN_STATE,
  NO_PASSENGER,
  PASSENGER_NO_BELT,
  PASSENGER_BYPASS_BELT,
  PASSENGER_WITH_BELT
};

// ==========================================
// Class: RotaryEncoder
// Handles pulse counting and direction logic
// ==========================================
class RotaryEncoder {
private:
  uint8_t pinCLK;
  uint8_t pinDT;
  volatile long position;
  volatile Direction currentDir;
  volatile uint8_t lastStateCLK;
  volatile unsigned long lastInterruptTime;

public:
  RotaryEncoder(uint8_t clk, uint8_t dt)
    : pinCLK(clk), pinDT(dt), position(0), currentDir(STOPPED), lastInterruptTime(0) {}

  void begin() {
    pinMode(pinCLK, INPUT_PULLUP);
    pinMode(pinDT, INPUT_PULLUP);
    lastStateCLK = digitalRead(pinCLK);
  }

  // Called inside the Interrupt Service Routine (ISR)
  void update() {
    unsigned long interruptTime = millis();
    
    // 2ms software debounce to filter mechanical noise
    if (interruptTime - lastInterruptTime > 2) {
      uint8_t currentStateCLK = digitalRead(pinCLK);

      if (currentStateCLK != lastStateCLK && currentStateCLK == LOW) {
        if (digitalRead(pinDT) != currentStateCLK) {
          position++;
          currentDir = CLOCKWISE;
        } else {
          // Prevent position from dropping below 0 when fully retracted
          if (position > 0) {
            position--;
          }
          currentDir = COUNTER_CLOCKWISE;
        }
      }
      lastStateCLK = currentStateCLK;
      lastInterruptTime = interruptTime;
    }
  }

  long getPosition() {
    noInterrupts();
    long pos = position;
    interrupts();
    return pos;
  }

  Direction getDirection() {
    noInterrupts();
    Direction dir = currentDir;
    interrupts();
    return dir;
  }
};

// ==========================================
// Class: SmartSeatbelt
// Checks FSR first, then evaluates belt usage
// ==========================================
class SmartSeatbelt {
private:
  RotaryEncoder& encoder;
  const uint8_t pinFSR;
  const int fsrThreshold;
  const long idleTolerance;
  const long humanThreshold;
  BeltState currentState;
  long lastPrintedPosition;

  void printStatus(int fsrVal, long pos, Direction dir) {
    Serial.print(F("[FSR: "));
    Serial.print(fsrVal);
    Serial.print(F(" | Pos: "));
    Serial.print(pos);
    Serial.print(F(" | Dir: "));
    
    if (dir == CLOCKWISE) {
      Serial.print(F("CW (Pulling)"));
    } else if (dir == COUNTER_CLOCKWISE) {
      Serial.print(F("CCW (Retracting)"));
    } else {
      Serial.print(F("NONE"));
    }
    Serial.print(F("] -> "));

    switch (currentState) {
      case NO_PASSENGER:
        Serial.println(F("No passenger detected"));
        break;
      case PASSENGER_NO_BELT:
        Serial.println(F("Passenger detected with no belt on"));
        break;
      case PASSENGER_BYPASS_BELT:
        Serial.println(F("Passenger detected with bypass belt (Behind back)"));
        break;
      case PASSENGER_WITH_BELT:
        Serial.println(F("Passenger with belt on"));
        break;
      default:
        break;
    }
  }

public:
  SmartSeatbelt(RotaryEncoder& enc, uint8_t fsrPin, int fsrLim, long idleLim, long humanLim)
    : encoder(enc), pinFSR(fsrPin), fsrThreshold(fsrLim), 
      idleTolerance(idleLim), humanThreshold(humanLim), 
      currentState(UNKNOWN_STATE), lastPrintedPosition(-1) {}

  void evaluate() {
    int fsrValue = analogRead(pinFSR);
    long pos = encoder.getPosition();
    Direction dir = encoder.getDirection();
    BeltState newState;

    // Step 1: Detect if a passenger is sitting on the seat via FSR
    if (fsrValue < fsrThreshold) {
      newState = NO_PASSENGER;
    } 
    // Step 2: Passenger is sitting; evaluate how the seatbelt is used
    else {
      if (pos <= idleTolerance) {
        newState = PASSENGER_NO_BELT;      // Belt hasn't been pulled at all
      } else if (pos < humanThreshold) {
        newState = PASSENGER_BYPASS_BELT;  // Pulled slightly (routed behind seat/back)
      } else {
        newState = PASSENGER_WITH_BELT;    // Pulled enough to wrap around a human torso
      }
    }

    // Trigger serial output only when the state changes or belt moves while seated
    if (newState != currentState || (newState != NO_PASSENGER && pos != lastPrintedPosition)) {
      currentState = newState;
      lastPrintedPosition = pos;
      printStatus(fsrValue, pos, dir);
    }
  }

  BeltState getState() {
    return currentState;
  }
};

// ==========================================
// Global Instances & Pin Setup
// ==========================================
const uint8_t PIN_CLK    = 2;  // Nano Interrupt Pin D2
const uint8_t PIN_DT     = 3;  // Nano Pin D3
const uint8_t PIN_BUZZER = 4;  // Buzzer Pin D4
const uint8_t PIN_FSR    = A0; // FSR Analog Pin A0

// Calibration Constants
const int  FSR_THRESHOLD   = 200; // Analog threshold (0-1023) to confirm a seated person
const long IDLE_TOLERANCE  = 0;   // 0 pulses = fully retracted
const long HUMAN_THRESHOLD = 30;  // Minimum pulses required to wrap around a human

RotaryEncoder beltEncoder(PIN_CLK, PIN_DT);
SmartSeatbelt seatbeltSystem(beltEncoder, PIN_FSR, FSR_THRESHOLD, IDLE_TOLERANCE, HUMAN_THRESHOLD);

// Global ISR wrapper for the C++ class method
void handleEncoderISR() {
  beltEncoder.update();
}

// ==========================================
// Non-Blocking Buzzer Function (2 beeps/sec)
// ==========================================
void handleBuzzer(bool shouldBeep) {
  static unsigned long cycleStartTime = 0;

  if (!shouldBeep) {
    digitalWrite(PIN_BUZZER, LOW);
    cycleStartTime = millis(); // Reset timer so it starts cleanly next time
    return;
  }

  unsigned long currentMillis = millis();
  unsigned long elapsed = currentMillis - cycleStartTime;

  // Reset the 1000ms (1 second) cycle
  if (elapsed >= 1000) {
    cycleStartTime = currentMillis;
    elapsed = 0;
  }

  // Double-beep pattern within a 1000ms window:
  // Beep 1: 0ms - 100ms   (ON)
  // Pause : 100ms - 200ms (OFF)
  // Beep 2: 200ms - 300ms (ON)
  // Rest  : 300ms - 1000ms (OFF)
  if ((elapsed < 100) || (elapsed >= 200 && elapsed < 300)) {
    digitalWrite(PIN_BUZZER, HIGH);
  } else {
    digitalWrite(PIN_BUZZER, LOW);
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_FSR, INPUT);
  digitalWrite(PIN_BUZZER, LOW);

  beltEncoder.begin();

  // Attach hardware interrupt to CLK pin
  attachInterrupt(digitalPinToInterrupt(PIN_CLK), handleEncoderISR, CHANGE);

  Serial.println(F("--- Smart Seatbelt Detection Initialized ---"));
}

void loop() {
  seatbeltSystem.evaluate();

  BeltState state = seatbeltSystem.getState();

  // Beep if passenger has no belt OR is bypassing the belt; silence otherwise
  if (state == PASSENGER_NO_BELT || state == PASSENGER_BYPASS_BELT) {
    handleBuzzer(true);
  } else {
    handleBuzzer(false);
  }
}