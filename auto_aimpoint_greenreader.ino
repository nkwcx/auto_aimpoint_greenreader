#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Servo.h>

// --- Display ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// --- MPU direct I2C ---
uint8_t mpuAddr = 0x68;

// --- Pins ---
#define BUTTON_PIN   2
#define POT_DIST_PIN A0
#define POT_STIMP_PIN A1
#define SERVO_PIN    9

// --- Button timing ---
#define LONG_PRESS_1      2000
#define LONG_PRESS_2      4000
#define DOUBLE_PRESS_TIME 400

unsigned long buttonPressedAt = 0;
unsigned long lastPressAt = 0;
int pressCount = 0;
bool buttonDown = false;
bool longPress1Fired = false;
bool longPress2Fired = false;
bool mpuOnline = false;

// --- Readings ---
float slopeReadings[10];
float uphillReadings[10];
int readingCount = 0;
float avgSlope = 0;
float avgUphill = 0;
float breakInches = 0;
float aimAngle = 0;
float playsLikeDistance = 0;
float zeroRoll = 0;
float zeroPitch = 0;
float zeroAx = 0;
float zeroAy = 0;
float zeroAz = 0;

const float CUP_DIAMETER_IN = 4.25;

const int SERVO_CENTER = 99;  // Mechanical trim center
const float SERVO_SCALE = 2.0;      // Amplify tiny aim angles into visible servo movement
const int SERVO_MIN_STEP = 2;       // Ensure small but real move when break exists
const float BREAK_DEADBAND_IN = 0.05;

// AimPoint-like side-hill baseline chart for Stimp 8 (no uphill/downhill adjustment yet).
// Distances: 5, 10, 15, 20 ft
// Slopes:    1%, 2% (90-side values from the chart)
const float CHART_DIST_FT[4] = {5.0, 10.0, 15.0, 20.0};
const float CHART_BREAK_1PCT[4] = {0.0, 2.0, 5.0, 7.0};
const float CHART_BREAK_2PCT[4] = {1.0, 6.0, 12.0, 17.0};

// --- Green speed (from stimp pot) ---
float currentStimp = 8.0;

// --- Servo ---
Servo aimServo;

// --- States ---
enum State { READY, TAKING_READINGS, CALCULATED, RESETTING };
State currentState = READY;

// ------------------------------------------------

float lerp(float a, float b, float t) {
  return a + (b - a) * t;
}

float interpolateByDistance(const float table[4], float distFt) {
  if (distFt <= CHART_DIST_FT[0]) {
    // Fade in from 0 at 0 ft to chart value at 5 ft.
    return table[0] * (distFt / CHART_DIST_FT[0]);
  }

  for (int i = 0; i < 3; i++) {
    float d0 = CHART_DIST_FT[i];
    float d1 = CHART_DIST_FT[i + 1];
    if (distFt <= d1) {
      float t = (distFt - d0) / (d1 - d0);
      return lerp(table[i], table[i + 1], t);
    }
  }

  // Slight linear extrapolation above 20 ft using last segment slope.
  float tailSlope = (table[3] - table[2]) / (CHART_DIST_FT[3] - CHART_DIST_FT[2]);
  return table[3] + tailSlope * (distFt - CHART_DIST_FT[3]);
}

float getBreakInchesFromChart(float slopePctAbs, float distFt, float stimp) {
  float onePct = interpolateByDistance(CHART_BREAK_1PCT, distFt);
  float twoPct = interpolateByDistance(CHART_BREAK_2PCT, distFt);

  // Interpolate/extrapolate by side slope using 1% and 2% chart anchors.
  float breakAtStimp8;
  if (slopePctAbs <= 1.0) {
    breakAtStimp8 = onePct * slopePctAbs;
  } else {
    float slopeStep = twoPct - onePct;
    breakAtStimp8 = onePct + (slopePctAbs - 1.0) * slopeStep;
  }

  // Scale from Stimp 8 baseline.
  float stimpFactor = stimp / 8.0;
  return breakAtStimp8 * stimpFactor;
}

float getPlaysLikeDistance(float distFt, float uphillPct) {
  // Simple pace model: uphill plays longer, downhill shorter.
  // Example: 10 ft at +2.0% uphill -> ~11.0 ft plays-like.
  float factor = 1.0 + (uphillPct * 0.05);
  factor = constrain(factor, 0.70, 1.50);
  return distFt * factor;
}

float getHillBreakFactor(float uphillPct) {
  // Break model: uphill reduces break, downhill increases break.
  float factor = 1.0 - (uphillPct * 0.10);
  return constrain(factor, 0.60, 1.50);
}

String getBreakDisplay(float breakInches, float slope) {
  // Cup-relative display: break measured from cup edge
  float sixthCup = CUP_DIAMETER_IN / 6.0;   // ~0.71 inches
  float halfCup = CUP_DIAMETER_IN / 2.0;    // 2.125 inches
  
  float absBreak = fabs(breakInches);
  
  if (absBreak < sixthCup) {
    return "inside c";
  } else if (absBreak < halfCup) {
    return "inside";
  } else {
    // Outside cup - measure from edge (subtract 0.5 cups), display in cups
    float breakCups = absBreak / CUP_DIAMETER_IN;
    float edgeMeasurement = breakCups - 0.5;
    return String(edgeMeasurement, 1) + " cups";
  }
}

// ------------------------------------------------

void initMPU() {
  Wire.beginTransmission(mpuAddr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  delay(100);
}

bool detectMPUAddress() {
  uint8_t candidates[2] = {0x68, 0x69};
  for (int i = 0; i < 2; i++) {
    uint8_t addr = candidates[i];
    Wire.beginTransmission(addr);
    Wire.write(0x75); // WHO_AM_I
    if (Wire.endTransmission(false) != 0) {
      continue;
    }

    int bytes = Wire.requestFrom((int)addr, 1, true);
    if (bytes != 1 || !Wire.available()) {
      continue;
    }

    uint8_t who = Wire.read();
    if (who == 0x68 || who == 0x69) {
      mpuAddr = addr;
      return true;
    }
  }
  return false;
}

void scanI2CDevices() {
  Serial.println("I2C scan start...");
  int found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    if (err == 0) {
      Serial.print("I2C device at 0x");
      if (addr < 16) Serial.print("0");
      Serial.println(addr, HEX);
      found++;
    }
  }
  if (found == 0) {
    Serial.println("No I2C devices found");
  }
  Serial.println("I2C scan end.");
}

bool readMPU(int16_t &ax, int16_t &ay, int16_t &az) {
  for (int attempt = 0; attempt < 3; attempt++) {
    Wire.beginTransmission(mpuAddr);
    Wire.write(0x3B);
    if (Wire.endTransmission(false) != 0) {
      delay(2);
      continue;
    }

    int bytes = Wire.requestFrom((int)mpuAddr, 6, true);
    if (bytes != 6 || Wire.available() < 6) {
      delay(2);
      continue;
    }

    ax = Wire.read() << 8 | Wire.read();
    ay = Wire.read() << 8 | Wire.read();
    az = Wire.read() << 8 | Wire.read();
    return true;
  }
  return false;
}

bool readMPUAveraged(float &axAvg, float &ayAvg, float &azAvg, int samples = 20) {
  long axSum = 0;
  long aySum = 0;
  long azSum = 0;
  int valid = 0;

  for (int i = 0; i < samples; i++) {
    int16_t ax, ay, az;
    if (!readMPU(ax, ay, az)) {
      delay(5);
      continue;
    }
    axSum += ax;
    aySum += ay;
    azSum += az;
    valid++;
    delay(5);
  }

  if (valid == 0) {
    axAvg = 0;
    ayAvg = 0;
    azAvg = 0;
    return false;
  }

  axAvg = (float)axSum / valid;
  ayAvg = (float)aySum / valid;
  azAvg = (float)azSum / valid;
  return true;
}

void computeAnglesFromAccel(float ax, float ay, float az, float &rollDeg, float &pitchDeg) {
  // Roll: break axis, Pitch: uphill axis
  rollDeg = atan2(ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
  pitchDeg = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / PI;
}

void calibrateMPUZero() {
  float ax, ay, az;
  if (!readMPUAveraged(ax, ay, az, 30)) {
    Serial.println("MPU calibration failed: no valid samples");
    zeroAx = 0;
    zeroAy = 0;
    zeroAz = 16384;
    zeroRoll = 0;
    zeroPitch = 0;
    return;
  }
  zeroAx = ax;
  zeroAy = ay;
  zeroAz = az;
  computeAnglesFromAccel(ax, ay, az, zeroRoll, zeroPitch);

  Serial.println("MPU zero calibrated (place device at reference level)");
  Serial.print("Zero ax:"); Serial.print(zeroAx, 0);
  Serial.print(" ay:"); Serial.print(zeroAy, 0);
  Serial.print(" az:"); Serial.println(zeroAz, 0);
  Serial.print("Zero roll: "); Serial.print(zeroRoll, 2); Serial.println(" deg");
  Serial.print("Zero pitch:"); Serial.print(zeroPitch, 2); Serial.println(" deg");
}

// ------------------------------------------------

void setup() {
  Serial.begin(9600);
  Wire.begin();
  Wire.setClock(100000);
  delay(100);

  scanI2CDevices();

  if (!detectMPUAddress()) {
    Serial.println("MPU not detected at 0x68 or 0x69");
  } else {
    Serial.print("MPU detected at 0x"); Serial.println(mpuAddr, HEX);
  }

  initMPU();
  Serial.println("MPU initialized");
  if (detectMPUAddress()) {
    mpuOnline = true;
    calibrateMPUZero();
  } else {
    mpuOnline = false;
  }

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED: FAILED");
    while (true);
  }
  display.cp437(true);  // Enable extended glyphs (includes degree symbol)
  Serial.println("OLED: OK");

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Read stimp pot at startup.
  currentStimp = getStimp();

  // Set target before attach to avoid a momentary jump to default 90.
  aimServo.write(SERVO_CENTER);
  aimServo.attach(SERVO_PIN);
  delay(20);
  aimServo.write(SERVO_CENTER);

  Serial.println("--- READY ---");
  updateDisplay(mpuOnline ? "READY" : "NO MPU", 0, currentStimp, 0, 0);
}

// ------------------------------------------------

void loop() {
  handleButton();

  // Update distance live
  static float lastDistance = -1;
  static float lastStimp = -1;
  float distance = getDistance();
  currentStimp = getStimp();
  playsLikeDistance = getPlaysLikeDistance(distance, avgUphill);

  if (distance != lastDistance) {
    lastDistance = distance;
    Serial.print("Distance: "); Serial.print(distance, 1); Serial.println("ft");
  }

  if (fabs(currentStimp - lastStimp) >= 0.1) {
    lastStimp = currentStimp;
    Serial.print("Stimp: "); Serial.println(currentStimp, 1);
  }

  if (currentState == READY || currentState == TAKING_READINGS) {
    updateDisplay(
      currentState == READY ? "READY" : "READING " + String(readingCount),
      distance, currentStimp, avgSlope, breakInches
    );
  }
}

// ------------------------------------------------

void handleButton() {
  int state = digitalRead(BUTTON_PIN);

  if (state == LOW && !buttonDown) {
    buttonDown = true;
    buttonPressedAt = millis();
    longPress1Fired = false;
    longPress2Fired = false;
    Serial.println("Button: pressed");
  }

  if (state == LOW && buttonDown) {
    unsigned long held = millis() - buttonPressedAt;
    if (held >= LONG_PRESS_2 && !longPress2Fired) {
      longPress2Fired = true;
      onLongPress2();
    } else if (held >= LONG_PRESS_1 && !longPress1Fired) {
      longPress1Fired = true;
      onLongPress1();
    }
  }

  if (state == HIGH && buttonDown) {
    buttonDown = false;
    unsigned long held = millis() - buttonPressedAt;
    Serial.print("Button: released after "); Serial.print(held); Serial.println("ms");
    if (!longPress1Fired && !longPress2Fired) {
      pressCount++;
      lastPressAt = millis();
      Serial.print("Press count: "); Serial.println(pressCount);
    }
  }

  if (pressCount > 0 && millis() - lastPressAt > DOUBLE_PRESS_TIME) {
    if (pressCount == 1) {
      Serial.println("Button: SINGLE PRESS");
      onSinglePress();
    } else if (pressCount >= 2) {
      Serial.println("Button: DOUBLE PRESS");
      onDoublePress();
    }
    pressCount = 0;
  }
}

// ------------------------------------------------

void onSinglePress() {
  if (!mpuOnline) {
    if (detectMPUAddress()) {
      mpuOnline = true;
      initMPU();
      calibrateMPUZero();
    } else {
      Serial.println("Reading blocked: MPU offline");
      updateDisplay("NO MPU", getDistance(), currentStimp, avgSlope, breakInches);
      return;
    }
  }

  float ax, ay, az;
  if (!readMPUAveraged(ax, ay, az, 20)) {
    Serial.println("Reading failed: no valid MPU data");
    updateDisplay("MPU READ FAIL", getDistance(), currentStimp, avgSlope, breakInches);
    return;
  }

  // Baseline-compensated axis angles (more robust with arbitrary board mounting)
  float rawBreakDeg = atan2(ax, az) * 180.0 / PI;
  float rawUphillDeg = atan2(ay, az) * 180.0 / PI;
  float zeroBreakDeg = atan2(zeroAx, zeroAz) * 180.0 / PI;
  float zeroUphillDeg = atan2(zeroAy, zeroAz) * 180.0 / PI;

  float breakDeg = rawBreakDeg - zeroBreakDeg;
  float uphillDeg = rawUphillDeg - zeroUphillDeg;

  float slopeX = tan(breakDeg * PI / 180.0) * 100.0;
  float slopeY = tan(uphillDeg * PI / 180.0) * 100.0;

  Serial.println("--- Reading ---");
  Serial.print("Raw ax:"); Serial.print(ax, 0);
  Serial.print(" ay:"); Serial.print(ay, 0);
  Serial.print(" az:"); Serial.println(az, 0);
  Serial.print("Break deg (raw/adj): "); Serial.print(rawBreakDeg, 2); Serial.print(" / "); Serial.println(breakDeg, 2);
  Serial.print("Uphilldeg(raw/adj): "); Serial.print(rawUphillDeg, 2); Serial.print(" / "); Serial.println(uphillDeg, 2);
  Serial.print("Break slope: "); Serial.print(slopeX, 2); Serial.println("%");
  Serial.print("Uphill slope: "); Serial.print(slopeY, 2); Serial.println("%");

  if (readingCount < 10) {
    slopeReadings[readingCount]   = slopeX;
    uphillReadings[readingCount]  = slopeY;
    readingCount++;
  }

  // Running averages
  float sumSlope = 0, sumUphill = 0;
  for (int i = 0; i < readingCount; i++) {
    sumSlope   += slopeReadings[i];
    sumUphill  += uphillReadings[i];
  }
  avgSlope   = sumSlope / readingCount;
  avgUphill  = sumUphill / readingCount;

  Serial.print("Avg break slope: ");  Serial.print(avgSlope, 2);  Serial.println("%");
  Serial.print("Avg uphill slope: "); Serial.print(avgUphill, 2); Serial.println("%");
  Serial.print("Reading count: ");    Serial.println(readingCount);

  currentState = TAKING_READINGS;
  updateDisplay("READING " + String(readingCount), getDistance(), currentStimp, avgSlope, breakInches);
}

void onDoublePress() {
  float distance = getDistance();
  playsLikeDistance = getPlaysLikeDistance(distance, avgUphill);
  float hillBreakFactor = getHillBreakFactor(avgUphill);

  float baseBreakInches = getBreakInchesFromChart(fabs(avgSlope), distance, currentStimp);
  breakInches = baseBreakInches * hillBreakFactor;
  aimAngle    = atan(breakInches / (distance * 12.0)) * 180.0 / PI;
  float breakCups = breakInches / CUP_DIAMETER_IN;

  // Uphill reduces break, downhill increases it
  String uphillStr = avgUphill > 0.5 ? "UPHILL" : avgUphill < -0.5 ? "DOWNHILL" : "FLAT";
  String direction = avgSlope > 0 ? "AIM R" : "AIM L";

  // Drive servo with scaled+rounded offset so small valid aims still move hardware.
  int servoOffset = (int)round(aimAngle * SERVO_SCALE);
  if (breakInches > BREAK_DEADBAND_IN && servoOffset < SERVO_MIN_STEP) {
    servoOffset = SERVO_MIN_STEP;
  }
  if (breakInches <= BREAK_DEADBAND_IN) {
    servoOffset = 0;
  }

  int servoPos = avgSlope > 0 ? SERVO_CENTER - servoOffset : SERVO_CENTER + servoOffset;
  servoPos = constrain(servoPos, 0, 180);
  aimServo.write(servoPos);

  Serial.println("=== Break Calculation ===");
  Serial.print("Distance:    "); Serial.print(distance, 1);   Serial.println("ft");
  Serial.print("Plays like:  "); Serial.print(playsLikeDistance, 1); Serial.println("ft");
  Serial.print("Stimp:       "); Serial.println(currentStimp, 1);
  Serial.print("Model:       "); Serial.println("AimPoint side-chart + hill factor");
  Serial.print("Avg slope:   "); Serial.print(avgSlope, 2);   Serial.println("%");
  Serial.print("Avg uphill:  "); Serial.print(avgUphill, 2);  Serial.println("%");
  Serial.print("Hill factor: "); Serial.println(hillBreakFactor, 3);
  Serial.print("Base break:  "); Serial.print(baseBreakInches, 2); Serial.println(" inches");
  Serial.print("Uphill:      "); Serial.println(uphillStr);
  Serial.print("Break:       "); Serial.print(breakInches, 2); Serial.println(" inches");
  Serial.print("Break cups:  "); Serial.print(breakCups, 2);  Serial.println(" cups");
  Serial.print("Aim angle:   "); Serial.print(aimAngle, 2);   Serial.println("deg");
  Serial.print("Servo off:   "); Serial.println(servoOffset);
  Serial.print("Direction:   "); Serial.println(direction);
  Serial.print("Servo pos:   "); Serial.println(servoPos);
  Serial.println("=========================");

  currentState = CALCULATED;
  String aimStatus = direction + " | " + getBreakDisplay(breakInches, avgSlope);
  aimStatus += " " + String(aimAngle, 1) + (char)248; // Degree symbol in CP437
  updateDisplay(aimStatus, distance, currentStimp, avgSlope, breakInches);
}

void onLongPress1() {
  Serial.println("Button: LONG PRESS 1 - hold to reset");
  updateDisplay("HOLD TO RESET", getDistance(), currentStimp, avgSlope, breakInches);
}

void onLongPress2() {
  Serial.println("Button: LONG PRESS 2 - full reset");
  readingCount = 0;
  avgSlope     = 0;
  avgUphill    = 0;
  breakInches  = 0;
  aimAngle     = 0;
  aimServo.write(SERVO_CENTER);  // re-center servo
  currentState = READY;

  Serial.println("--- READY ---");
  updateDisplay("READY", getDistance(), currentStimp, 0, 0);
}

// ------------------------------------------------

float getDistance() {
  int raw = analogRead(POT_DIST_PIN);
  return map(raw, 0, 1023, 3, 29);
}

float getStimp() {
  int raw = analogRead(POT_STIMP_PIN);
  // Map to a practical stimp range: 6.0 to 14.0.
  return map(raw, 0, 1023, 60, 160) / 10.0;
}

// ------------------------------------------------

void updateDisplay(String status, float dist, float stimp, float slope, float brk) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // Bi-color OLED layout:
  // - Top yellow region: show only AIM direction/status
  // - Lower blue region: all measurement data
  bool isAimStatus = status.startsWith("AIM ");
  if (isAimStatus) {
    display.setCursor(0, 0);
    display.print(status);
  }

  // Row 1 - distance and stimp
  display.setCursor(0, 18);
  display.print("Dist:"); display.print(dist, 0); display.print("ft");
  display.setCursor(70, 18);
  display.print("PL:"); display.print(playsLikeDistance, 0); display.print("ft");

  // Row 2 - slope and break
  display.setCursor(0, 30);
  display.print("Slp:"); display.print(slope, 1); display.print("%");
  display.setCursor(70, 30);
  display.print("Brk:"); display.print(brk, 1); display.print("in");

  // Row 3 - uphill
  display.setCursor(0, 42);
  display.print("Hill:"); 
  display.print(avgUphill, 1); display.print("%");
  display.setCursor(70, 42);
  display.print("Stmp:"); display.print(stimp, 0);

  // Non-aim statuses stay in blue region.
  if (!isAimStatus) {
    display.setCursor(0, 54);
    display.print(status);
  }

  display.display();
}

