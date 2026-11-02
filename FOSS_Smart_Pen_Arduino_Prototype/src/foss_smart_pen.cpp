/*
  FOSS SMART PEN - firmware v0.1   *** EXPERIMENTAL, IMU-ONLY PROTOTYPE ***

  Board   : Arduino Nano, ATmega328P (Old Bootloader), 16 MHz
  Sensor  : GY-521 (MPU-6050) on I2C via Wire.h
  Output  : ASCII lines over USB serial, 500 points/second (see PROTOCOL)

  WHAT IT DOES
    1. Configures the MPU-6050 from the MpuConfig table below (ranges, filter, rate...)
    2. Button on D2: the pen is held still in its starting pose and the button is pressed
       The sketch averages ~0.5 s of data, then records:
         - the sensor's starting 3D angle (roll/pitch from gravity, yaw := 0)
         - gyro bias, measured gravity
         - the writing plane: horizontal, PLANE_OFFSET_M below the sensor
         - the sensor position origin (0,0,0)
    3. After that, every 2 ms it updates orientation + position and computes the
       sensor height h above the plane.
    4. h > PEN_UP_HEIGHT_M  -> pen up, nothing is sent except a "U" event
       h <= PEN_UP_HEIGHT_M -> a ray is shot from the sensor along RAY_AXIS_SENSOR and intersected with the plane; 
                              the hit point is sent


  HONEST LIMITS (read these!)
    - Position comes from DOUBLE INTEGRATION of acceleration. It drifts. A tilt
      error of just 0.5 degree already looks like 0.09 m/s^2 of fake acceleration,
      i.e. ~1 cm of fake position after ~0.5 s of motion. The 1 cm pen-up threshold
      is about the size of that drift, so expect random pen-up/pen-down flicker
      and wandering XY after a second or two. Zero-velocity updates (ZUPT) help only
      when the pen is actually still. This is a first experiment, not a finished tracker.
    - Yaw has no absolute reference (no magnetometer). It drifts with gyro bias.
    - The ray-plane math is only meaningful if the ray origin (sensor position) is
      known; here that origin is the drifting integrated position.
    - Set INTEGRATE_POSITION = false to freeze the sensor position at the origin.
      Then only orientation moves the ray: a drift-free way to test the orientation
      and ray-plane code.
    - Verification so far: built on a PC with stub Arduino/Wire headers and driven by a simulated
      MPU-6050 (rotation, slide, lift, noise, sensor unplug, bad calibration). It has NOT been compiled
      for the AVR or run on real hardware, and the 2 ms tick budget on the ATmega328P is unmeasured:
      watch overruns and loopmax_us in the S line, and lower OUTPUT_RATE_HZ if overruns keep rising.


  CONVENTIONS
    - Sensor frame = the axes printed on the GY-521. Flat, face up: accel Z = +1 g
    - World frame at calibration: Z up (from gravity), X = sensor heading at the
      moment of calibration (yaw := 0), Y completes a right-handed system
    - Rotation body->world:  R = Rz(yaw) * Ry(pitch) * Rx(roll)
      roll  = atan2(ay, az),   pitch = atan2(-ax, sqrt(ay^2 + az^2))
    - Orientation is stored as a quaternion (w,x,y,z) that represents that same R.
    - Accelerometer reads SPECIFIC FORCE: at rest in world frame it is (0,0,+g)


  PROTOCOL (one ASCII line per message, '\n' terminated, fields comma separated)
    D,stroke,t_us,x,y    pen-down point. x,y in 0.01 mm relative to where the ray hit the plane
                         at calibration (that point = page origin). +X = sensor heading at calibration.
    U,stroke,t_us,overruns,txdrops    pen-up event (end of stroke)
    C,START              calibration started - hold still
    C,OK,roll_cdeg,pitch_cdeg,g_mg,gbx,gby,gbz   calibration done (gyro bias in 0.01 dps)
    E,CODE               error (MPU_NOT_FOUND, MPU_CONFIG, I2C_LOST, CAL_MOVED,
                         CAL_GRAVITY, CAL_RAY, NUMERIC)
    W,CODE,value         warning
    I,text               info
    S,state,h_0p1mm,overruns,txdrops,i2cfails,loopmax_us      status (only while pen up)
    R,ax_mg,ay_mg,az_mg,gx_0p1dps,gy_0p1dps,gz_0p1dps         raw readings (only while pen up)
    Q,w,x,y,z            orientation quaternion x1000 (only while pen up)
  Only lines starting with 'D' are drawing coordinates. A new stroke id means a new stroke

  WIRING (Nano <-> GY-521)      Button: D2 -> pushbutton -> GND (INPUT_PULLUP)
    5V  -> VCC   GND -> GND   A4 -> SDA   A5 -> SCL   GND -> AD0 (address 0x68)

  ARDUINO IDE: Tools > Board > Arduino Nano, Processor > ATmega328P (Old Bootloader)
  Put this file in a folder named foss_smart_pen
  Serial monitor / PC: 500000 baud
*/

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// ============================================================================
// 1. USER CONFIGURATION
// ============================================================================

// ---- 1.1 Serial link --------------------------------------------------------
// A drawing line is 22-35 bytes. 500 lines/s = 11-17.5 kB/s; 115200 baud only carries 11.5 kB/s, so it is marginal at best
// 500000 and 250000 are exact on a 16 MHz ATmega328P
// If your PC sees garbage, try 250000 (still enough) and keep OUTPUT_RATE_HZ
const uint32_t SERIAL_BAUD = 500000UL;

// ---- 1.2 Pins ---------------------------------------------------------------
const uint8_t  PIN_CAL_BUTTON      = 2;            // to GND, INPUT_PULLUP
const uint8_t  PIN_STATUS_LED      = LED_BUILTIN;  // D13
const uint16_t BUTTON_DEBOUNCE_MS  = 30;

// ---- 1.3 Timing -------------------------------------------------------------
const uint16_t OUTPUT_RATE_HZ        = 500;
const uint32_t SAMPLE_PERIOD_US      = 1000000UL / OUTPUT_RATE_HZ;
const uint8_t  I2C_FAIL_LIMIT        = 10;    // consecutive failed reads before "I2C_LOST"
const uint16_t IMU_RETRY_INTERVAL_MS = 500;   // reconnect attempts while the IMU is missing
const uint16_t DIAG_INTERVAL_MS      = 100;   // one status/raw/quaternion line per interval

// ---- 1.4 MPU-6050 settings ---------------------------------------------------
// Range codes are the 2-bit values written to FS_SEL / AFS_SEL.
const uint8_t ACCEL_RANGE_2G  = 0;   // 16384 LSB/g
const uint8_t ACCEL_RANGE_4G  = 1;   //  8192 LSB/g
const uint8_t ACCEL_RANGE_8G  = 2;   //  4096 LSB/g
const uint8_t ACCEL_RANGE_16G = 3;   //  2048 LSB/g
const uint8_t GYRO_RANGE_250DPS  = 0;  // 131.0 LSB/(deg/s)
const uint8_t GYRO_RANGE_500DPS  = 1;  //  65.5
const uint8_t GYRO_RANGE_1000DPS = 2;  //  32.8
const uint8_t GYRO_RANGE_2000DPS = 3;  //  16.4

struct MpuConfig {
  uint8_t  i2cAddress;         // 0x68 (AD0 = GND) or 0x69 (AD0 = VCC)
  uint32_t i2cClockHz;         // 400000 needed for 500 Hz (100000 is too slow)
  uint8_t  accelRange;         // ACCEL_RANGE_*
  uint8_t  gyroRange;          // GYRO_RANGE_*
  uint8_t  dlpfConfig;         // CONFIG.DLPF_CFG 0..6. 1 = ~184/188 Hz bandwidth, ~2 ms delay
                               //   0: 260/256 Hz (gyro rate 8 kHz), 2: 94/98 Hz, 3: 44/42 Hz ...
                               //   Lower bandwidth = less noise but MORE latency.
  uint8_t  sampleRateDivider;  // SMPLRT_DIV. Internal rate = 1 kHz/(1+div) when DLPF is on.
                               //   0 -> sensor refreshes at 1 kHz, we read every 2 ms
                               //   so the data is at most ~1 ms old.
  uint8_t  clockSource;        // PWR_MGMT_1.CLKSEL. 1 = PLL with X gyro (recommended), 0 = internal 8 MHz
  uint8_t  accelHighPass;      // ACCEL_CONFIG.ACCEL_HPF 0..7. MUST stay 0: the high-pass removes gravity.
};

MpuConfig mpuConfig = {
  0x68,                 // i2cAddress
  400000UL,             // i2cClockHz
  ACCEL_RANGE_2G,       // accelRange
  GYRO_RANGE_500DPS,    // gyroRange
  1,                    // dlpfConfig
  0,                    // sampleRateDivider
  1,                    // clockSource
  0                     // accelHighPass
};

// Set true to switch off the Arduino's internal 20-50k pull-ups on SDA/SCL after
// Wire.begin(). This keeps the bus at the GY-521's 3.3 V level, but ONLY do it if your
// GY-521 has its own pull-ups (most do, typically 4.7k). Default false = standard Wire.
const bool DISABLE_I2C_INTERNAL_PULLUPS = false;



// ---- 1.5 Orientation filter ---------------------------------------------------
// Complementary filter, 3D form:  each step the estimate is rotated by the gyro, then
// pulled toward the accelerometer's gravity direction by a fraction (1 - alpha) of the
// remaining error. Two alphas, because the right trust in the accelerometer depends on motion:
//   FILTER_ALPHA        : pen STILL. Time constant = dt*alpha/(1-alpha) = ~1 s at 0.998.
//                         The first experiment used 0.98 (~0.1 s). It is NOT a proven optimum, and
//                         in simulation it hurt: at the first milliseconds of a movement (before the
//                         "still" detector releases) it absorbed ~0.13 deg of fake tilt, which cost
//                         ~7% of a 10 cm slide. 0.998 cut that to ~1%.
//   FILTER_ALPHA_MOVING : pen MOVING. Hand acceleration looks exactly like tilt to an accelerometer
//                         and barely changes |a|, so it cannot be gated out with a |a| test. With
//                         alpha = 0.98 the filter explained real motion away as tilt and the gravity
//                         removal used for position went wrong (wrong sign, 58% size in simulation).
//                         1.0 = ignore the accelerometer while moving, trust the gyro. Use e.g.
//                         0.9999 to pull weakly toward gravity during very long movements.
const float FILTER_ALPHA        = 0.998f;
const float FILTER_ALPHA_MOVING = 1.0f;
// Accelerometer is only trusted as a tilt reference if |a| is within this of measured g.
const float ACCEL_TRUST_BAND_G  = 0.15f;


// ---- 1.6 Geometry ---------------------------------------------------------------
// Ray direction in the SENSOR frame (any non-zero vector; normalised at start-up).
// Default -Z = "straight down when the board lies flat, face up" (handy for bench tests).
// For a pen, point it along the board axis that runs toward the pen tip, e.g. {1,0,0}.
const float RAY_AXIS_SENSOR[3] = { 0.0f, 0.0f, -1.0f };

// Writing plane distance below the sensor at calibration. YOU MUST MEASURE THIS.
// (The earlier experiment used 0.135 m for a sensor 14 cm from the tip; the requirement
//  for this version says 0.5 cm.)
const float PLANE_OFFSET_M             = 0.005f;
const float PEN_UP_HEIGHT_M            = 0.010f;  // h above this = pen up
const float PEN_HEIGHT_HYSTERESIS_M    = 0.001f;  // pen goes down again below (threshold - this); 0 = off
const float MIN_RAY_DOWNWARD           = 0.15f;   // ray must point at least this far "into" the plane
const float MAX_RAY_LENGTH_M           = 0.30f;   // reject hits further than this along the ray


// ---- 1.7 Position integration -----------------------------------------------------
const bool    INTEGRATE_POSITION     = true;     // false = sensor stays at the origin (orientation-only test)
const float   ZUPT_MAX_GYRO_DPS      = 2.0f;     // "still" if rotation rate below this (after bias removal)
const float   ZUPT_MAX_LINEAR_ACCEL_G = 0.03f;   // ... and gravity-removed acceleration below this (g)
const uint8_t ZUPT_MIN_SAMPLES       = 20;       // ... for this many consecutive samples (40 ms)


// ---- 1.8 Calibration -----------------------------------------------------------------
const uint16_t CAL_SETTLE_MS         = 300;      // wait after the press (button bump) before sampling
const uint16_t CAL_SAMPLE_COUNT      = 250;      // 0.5 s at 500 Hz
const float    CAL_MAX_GYRO_STD_DPS  = 2.0f;     // reject calibration if the pen moved more than this
const float    CAL_GRAVITY_MIN_G     = 0.85f;
const float    CAL_GRAVITY_MAX_G     = 1.15f;


// ---- 1.9 Output scaling ---------------------------------------------------------------
const float   OUTPUT_UNITS_PER_M = 100000.0f;    // 1 count = 0.01 mm
const int32_t OUTPUT_LIMIT       = 32767;        // saturate at +-327.67 mm


// ---- Constants --------------------------------------------------------------------------
const float G_MPS2    = 9.80665f;
const float DEG2RAD_F = 0.017453292f;
const float RAD2DEG_F = 57.29578f;




// ============================================================================
// 2. TYPES AND GLOBAL STATE
// ============================================================================

const uint8_t STATE_NO_IMU       = 0;
const uint8_t STATE_WAIT_CAL     = 1;
const uint8_t STATE_CAL_SETTLE   = 2;
const uint8_t STATE_CAL_SAMPLING = 3;
const uint8_t STATE_TRACKING     = 4;

struct ImuSample {
  float    accelG[3];     // g, sensor frame (specific force)
  float    gyroDps[3];    // deg/s, sensor frame (bias NOT removed)
  uint32_t timestampUs;   // micros() just before the I2C read
};

struct Plane {
  float point[3];         // a point on the plane (world frame, metres)
  float normal[3];        // unit normal (world frame)
};

// Small fixed buffer for building one protocol line without String/malloc.
const uint32_t POW10_TABLE[10] = {
  1000000000UL, 100000000UL, 10000000UL, 1000000UL, 100000UL, 
  10000UL, 1000UL, 100UL, 10UL, 1UL
};

struct LineBuf {
  char    data[64];       // 64 = Nano TX buffer size; longer lines could block
  uint8_t len;

  void reset() { len = 0; }
  
  void putChar(char c) { 
    if (len < sizeof(data)) { 
      data[len++] = c;
    }
  }
  void comma()   { putChar(','); }
  void endLine() { putChar('\n'); }

  // Text stored in flash, e.g. lb.putText(F("D,"))
  void putText(const __FlashStringHelper* text) {

    const char* p = reinterpret_cast<const char*>(text);

    for (char c = (char)pgm_read_byte(p);  c != 0;  c = (char)pgm_read_byte(++p)) { 
      putChar(c);
    }
  }

  // Digits by repeated subtraction: much faster on an AVR than 32-bit division.
  void putUnsigned(uint32_t v) {
    bool started = false;

    for (uint8_t i = 0; i < 10; i++) {
      uint8_t digit = 0;

      while (v >= POW10_TABLE[i]) { 
        v -= POW10_TABLE[i]; 
        digit++; 
      }

      if (digit != 0 || started || i == 9) { 
        putChar((char)('0' + digit)); 
        started = true; 
      }
    }
  }

  void putSigned(int32_t v) {
    if (v < 0) { 
      putChar('-'); 
      putUnsigned((uint32_t)(-(v + 1)) + 1UL); 
    }
    else       { 
      putUnsigned((uint32_t)v); 
    }
  }
};

// --- MPU ---
float accelPerLsb = 1.0f / 16384.0f;   // g per LSB      (set in mpuBegin)
float gyroPerLsb  = 1.0f / 65.5f;      // deg/s per LSB  (set in mpuBegin)

// --- system ---
uint8_t   systemState = STATE_NO_IMU;
ImuSample lastSample;
uint32_t  lastSampleUs  = 0;
uint8_t   i2cFailStreak = 0;
uint16_t  i2cFailTotal  = 0;
uint32_t  lastRetryMs   = 0;

// --- calibration accumulators ---
uint32_t calStateStartMs = 0;
uint16_t calSamplesTaken = 0;
float    calSumAccel[3], calSumGyro[3], calSumGyroSq;

// --- calibration results ---
float gyroBiasDps[3]     = { 0, 0, 0 };
float gravityRefG        = 1.0f;
float calibrationQuat[4] = { 1, 0, 0, 0 };                // sensor 3D angle at calibration (kept for later use)
float rayAxisUnit[3]     = { 0, 0, -1 };
Plane writingPlane       = { { 0, 0, 0 }, { 0, 0, 1 } };
float originX_m          = 0, originY_m = 0;              // intersection point at calibration -> page (0,0)

// --- tracking state ---
float   quat[4]         = { 1, 0, 0, 0 };  // w,x,y,z  body -> world
float   posM[3]         = { 0, 0, 0 };
float   velMps[3]       = { 0, 0, 0 };
uint8_t stationaryCount = 0;               // consecutive "pen is still" samples
float   linAccel[3]     = { 0, 0, 0 };     // world-frame acceleration without gravity, m/s^2
float   prevLinAccel[3] = { 0, 0, 0 };     // previous tick (trapezoid rule)
float   prevGyroRad[3]  = { 0, 0, 0 };     // previous tick (trapezoid rule)
bool    penDown = false;
uint16_t strokeId = 0;

// --- diagnostics ---
uint16_t overrunCount = 0;
uint16_t txDropCount  = 0;
uint16_t loopMaxUs    = 0;
uint32_t lastDiagMs   = 0;
uint8_t  diagPhase    = 0;
float    lastHeightM  = 0;




// ============================================================================
// 3. SERIAL OUTPUT
// ============================================================================

// Never block the real-time loop for drawing/diagnostic lines: 
// - if the Nano's 64-byte TX buffer cannot take the whole line, drop it and count it
// - Events (mustSend) may block for a moment, they are rare
bool sendLine(const LineBuf& lb, bool mustSend) {
  if (!mustSend && (uint8_t)Serial.availableForWrite() < lb.len) {
    txDropCount++;
    return false;
  }
  Serial.write((const uint8_t*)lb.data, lb.len);
  return true;
}

void sendError(const __FlashStringHelper* code) {
  LineBuf lb;

  lb.reset();
  lb.putText(F("E,")); lb.putText(code); 
  lb.endLine();

  sendLine(lb, true);
}

void sendInfo(const __FlashStringHelper* text) {
  LineBuf lb; 
  
  lb.reset();
  lb.putText(F("I,")); lb.putText(text); 
  lb.endLine();

  sendLine(lb, true);
}

int32_t toOutputUnits(float metres) {
  float v = metres * OUTPUT_UNITS_PER_M;

  v += (v >= 0.0f) ? 0.5f : -0.5f;

  if (v >  (float)OUTPUT_LIMIT) return  OUTPUT_LIMIT;
  if (v < -(float)OUTPUT_LIMIT) return -OUTPUT_LIMIT;

  return (int32_t)v;
}




// ============================================================================
// 4. MPU-6050 DRIVER (register map: InvenSense MPU-6000/MPU-6050 register map)
// ============================================================================

const uint8_t MPU_REG_SMPLRT_DIV    = 0x19,
              MPU_REG_CONFIG        = 0x1A,
              MPU_REG_GYRO_CONFIG   = 0x1B,
              MPU_REG_ACCEL_CONFIG  = 0x1C,
              MPU_REG_ACCEL_XOUT_H  = 0x3B,   // 14 bytes: accel xyz, temp, gyro xyz (big endian)
              MPU_REG_PWR_MGMT_1    = 0x6B,
              MPU_REG_WHO_AM_I      = 0x75,
              MPU_WHO_AM_I_VALUE    = 0x68;

const uint8_t MPU_OK         = 0, 
              MPU_NOT_FOUND  = 1, 
              MPU_BAD_CONFIG = 2;

const float ACCEL_LSB_PER_G[4]  = { 
  16384.0f, 
  8192.0f, 
  4096.0f, 
  2048.0f 
};
const float GYRO_LSB_PER_DPS[4] = { 
  131.0f, 
  65.5f, 
  32.8f, 
  16.4f 
};

bool mpuWriteReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(mpuConfig.i2cAddress);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool mpuReadRegs(uint8_t firstReg, uint8_t* out, uint8_t count) {
  Wire.beginTransmission(mpuConfig.i2cAddress);
  Wire.write(firstReg);

  if (Wire.endTransmission(false) != 0) { // repeated start, no STOP
    return false; 
  }  

  uint8_t got = Wire.requestFrom((uint8_t)mpuConfig.i2cAddress, (uint8_t)count, (uint8_t)true);

  if (got != count) {
    while (Wire.available()) { // flush a partial transfer
      Wire.read(); 
    }

#if defined(WIRE_HAS_TIMEOUT)
    if (Wire.getWireTimeoutFlag()) Wire.clearWireTimeoutFlag();
#endif

    return false;
  }

  for (uint8_t i = 0; i < count; i++) { 
    out[i] = Wire.read();
  }

  return true;
}

// Full (re)initialisation from mpuConfig. Uses delay(): only called at start-up / reconnect.
uint8_t mpuBegin() {
  Wire.setClock(mpuConfig.i2cClockHz);

  uint8_t who = 0;

  if (!mpuReadRegs(MPU_REG_WHO_AM_I, &who, 1)) {
    return MPU_NOT_FOUND; 
  }
  if (who != MPU_WHO_AM_I_VALUE) {                  // clones may report another ID; warn, keep going
    LineBuf lb; 
    
    lb.reset();
    lb.putText(F("W,WHO_AM_I,")); 
    lb.putUnsigned(who); 
    lb.endLine();

    sendLine(lb, true);
  }

  if (!mpuWriteReg(MPU_REG_PWR_MGMT_1, 0x80)) { // DEVICE_RESET
    return MPU_NOT_FOUND;
  }

  delay(100);

  if (!mpuWriteReg(MPU_REG_PWR_MGMT_1, mpuConfig.clockSource & 0x07)) { // wake + clock
    return MPU_NOT_FOUND;
  }

  delay(10);

  uint8_t gyroCfg  = (uint8_t)((mpuConfig.gyroRange   & 0x03) << 3);
  uint8_t accelCfg = (uint8_t)(((mpuConfig.accelRange & 0x03) << 3)   |   (mpuConfig.accelHighPass & 0x07));

  bool ok = mpuWriteReg( MPU_REG_SMPLRT_DIV,   mpuConfig.sampleRateDivider )
         && mpuWriteReg( MPU_REG_CONFIG,       mpuConfig.dlpfConfig & 0x07 )
         && mpuWriteReg( MPU_REG_GYRO_CONFIG,  gyroCfg                     )
         && mpuWriteReg( MPU_REG_ACCEL_CONFIG, accelCfg                    )
  ;
  
  if (!ok) {
    return MPU_NOT_FOUND;
  }

  // Read back so a silent write failure cannot go unnoticed.
  uint8_t rbGyro = 0xFF, rbAccel = 0xFF;
  if (!mpuReadRegs(MPU_REG_GYRO_CONFIG, &rbGyro, 1)) {
    return MPU_NOT_FOUND;
  }
  if (!mpuReadRegs(MPU_REG_ACCEL_CONFIG, &rbAccel, 1)) return MPU_NOT_FOUND;
  if ((rbGyro & 0x18) != gyroCfg || (rbAccel & 0x1F) != accelCfg) return MPU_BAD_CONFIG;

  accelPerLsb = 1.0f / ACCEL_LSB_PER_G[mpuConfig.accelRange & 0x03];
  gyroPerLsb  = 1.0f / GYRO_LSB_PER_DPS[mpuConfig.gyroRange & 0x03];
  delay(50);                                         // let the gyro settle after wake-up
  return MPU_OK;
}

// One burst read of accel + gyro. Returns false on any I2C problem (never returns stale data).
bool mpuReadSample(ImuSample& s) {
  uint8_t raw[14];
  s.timestampUs = micros();
  if (!mpuReadRegs(MPU_REG_ACCEL_XOUT_H, raw, 14)) return false;
  int16_t ax = (int16_t)(((uint16_t)raw[0]  << 8) | raw[1]);
  int16_t ay = (int16_t)(((uint16_t)raw[2]  << 8) | raw[3]);
  int16_t az = (int16_t)(((uint16_t)raw[4]  << 8) | raw[5]);
  // raw[6], raw[7] = temperature (unused)
  int16_t gx = (int16_t)(((uint16_t)raw[8]  << 8) | raw[9]);
  int16_t gy = (int16_t)(((uint16_t)raw[10] << 8) | raw[11]);
  int16_t gz = (int16_t)(((uint16_t)raw[12] << 8) | raw[13]);
  s.accelG[0] = ax * accelPerLsb;  s.accelG[1] = ay * accelPerLsb;  s.accelG[2] = az * accelPerLsb;
  s.gyroDps[0] = gx * gyroPerLsb;  s.gyroDps[1] = gy * gyroPerLsb;  s.gyroDps[2] = gz * gyroPerLsb;
  return true;
}

// ============================================================================
// 5. ORIENTATION (quaternion, complementary filter)
// ============================================================================

float dot3(const float a[3], const float b[3]) { return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]; }

// Rotation matrix (row-major) of body->world for the current quaternion.
// Equals Rz(yaw)*Ry(pitch)*Rx(roll).
void quaternionToMatrix(float R[9]) {
  float w = quat[0], x = quat[1], y = quat[2], z = quat[3];
  R[0] = 1.0f - 2.0f*(y*y + z*z);  R[1] = 2.0f*(x*y - w*z);        R[2] = 2.0f*(x*z + w*y);
  R[3] = 2.0f*(x*y + w*z);         R[4] = 1.0f - 2.0f*(x*x + z*z); R[5] = 2.0f*(y*z - w*x);
  R[6] = 2.0f*(x*z - w*y);         R[7] = 2.0f*(y*z + w*x);        R[8] = 1.0f - 2.0f*(x*x + y*y);
}

// Quaternion for yaw = 0 and the given roll/pitch (radians): q = qy(pitch) * qx(roll).
void quaternionFromRollPitch(float roll, float pitch, float q[4]) {
  float cr = cosf(0.5f * roll),  sr = sinf(0.5f * roll);
  float cp = cosf(0.5f * pitch), sp = sinf(0.5f * pitch);
  q[0] = cr * cp;  q[1] = sr * cp;  q[2] = cr * sp;  q[3] = -sr * sp;
}

// One filter step.
//   gyroRad : bias-corrected angular rate, rad/s, sensor frame
//   accelG  : accelerometer, g
// Complementary filter idea (same as  angle = a*(angle + gyro*dt) + (1-a)*acc_angle):
//   1) rotate by gyro*dt                       -> "trust the gyro short term"
//   2) add (1-a) * error toward gravity        -> "trust gravity long term"
// error e = a_measured_up x a_predicted_up (both unit vectors, sensor frame). Its length is
// sin(angle between them), so for small errors it is the angle itself.
// Predicted "up" in the sensor frame is the third ROW of R.
// Gravity only fixes roll and pitch; yaw (rotation about vertical) is never corrected.
void updateOrientation(const float accelG[3], const float gyroRad[3], float dt) {
  // Rotation angle vector (rad) over this step. Trapezoid rule: average the previous and the
  // current rate. Using only the current rate lags by half a sample, which showed up as ~0.06 deg
  // of tilt error (= fake acceleration) in simulation during a 55 deg/s rotation.
  float step[3] = { 0.5f * (gyroRad[0] + prevGyroRad[0]) * dt,
                    0.5f * (gyroRad[1] + prevGyroRad[1]) * dt,
                    0.5f * (gyroRad[2] + prevGyroRad[2]) * dt };
  prevGyroRad[0] = gyroRad[0]; prevGyroRad[1] = gyroRad[1]; prevGyroRad[2] = gyroRad[2];

  // Accelerometer tilt correction. Skipped entirely (no sqrt, no division) when the gain is zero,
  // i.e. while moving with FILTER_ALPHA_MOVING = 1.0 - this saves a lot of time on an 8-bit AVR.
  float alpha = (stationaryCount >= ZUPT_MIN_SAMPLES) ? FILTER_ALPHA : FILTER_ALPHA_MOVING;
  float k = 1.0f - alpha;
  if (k > 0.0f) {
    // Trust the accelerometer only if |a| is within the band around measured gravity
    // (compared as squares to avoid a sqrt).
    float n2a = dot3(accelG, accelG);
    float lo = gravityRefG - ACCEL_TRUST_BAND_G, hi = gravityRefG + ACCEL_TRUST_BAND_G;
    if (n2a > lo * lo && n2a < hi * hi) {
      float inv = 1.0f / sqrtf(n2a);
      float ax = accelG[0] * inv, ay = accelG[1] * inv, az = accelG[2] * inv;
      float w = quat[0], x = quat[1], y = quat[2], z = quat[3];
      float vx = 2.0f * (x*z - w*y);               // predicted up, sensor frame
      float vy = 2.0f * (y*z + w*x);
      float vz = 1.0f - 2.0f * (x*x + y*y);
      step[0] += k * (ay*vz - az*vy);
      step[1] += k * (az*vx - ax*vz);
      step[2] += k * (ax*vy - ay*vx);
    }
  }

  // q <- q + 0.5 * q (x) (0, step)   (first-order integration, fine for a 2 ms step)
  float hx = 0.5f * step[0], hy = 0.5f * step[1], hz = 0.5f * step[2];
  float w = quat[0], x = quat[1], y = quat[2], z = quat[3];
  float nw = w - (x*hx + y*hy + z*hz);
  float nx = x + (w*hx + y*hz - z*hy);
  float ny = y + (w*hy + z*hx - x*hz);
  float nz = z + (w*hz + x*hy - y*hx);

  // Renormalise with one Newton step for 1/sqrt(n2): valid because n2 is within ~1e-3 of 1.
  float n2 = nw*nw + nx*nx + ny*ny + nz*nz;
  float s = 1.5f - 0.5f * n2;
  quat[0] = nw * s;  quat[1] = nx * s;  quat[2] = ny * s;  quat[3] = nz * s;
}

// ============================================================================
// 6. POSITION (double integration + zero-velocity update)  -- EXPERIMENTAL
// ============================================================================

void resetMotionState() {
  for (uint8_t i = 0; i < 3; i++) { 
    posM[i] = 0; velMps[i] = 0; 
    prevLinAccel[i] = 0; 
    prevGyroRad[i] = 0; 
  }
  stationaryCount = 0;
}

// Gravity-removed acceleration in the world frame + "is the pen still?" detector.
// Runs every tick (also when INTEGRATE_POSITION is false) because the orientation filter uses it.
// "Still" = low rotation rate AND low gravity-removed acceleration. (Testing |a| ~ g would be
// wrong: sideways acceleration changes |a| only at second order, so motion would look still.)
void updateStillness(const float R[9], const float accelG[3], const float gyroRad[3]) {
  // a_lin = R * a_specific_force - (0,0,g)
  linAccel[0] = (R[0]*accelG[0] + R[1]*accelG[1] + R[2]*accelG[2]              ) * G_MPS2;
  linAccel[1] = (R[3]*accelG[0] + R[4]*accelG[1] + R[5]*accelG[2]              ) * G_MPS2;
  linAccel[2] = (R[6]*accelG[0] + R[7]*accelG[1] + R[8]*accelG[2] - gravityRefG) * G_MPS2;

  float wMax = ZUPT_MAX_GYRO_DPS * DEG2RAD_F;
  float aMax = ZUPT_MAX_LINEAR_ACCEL_G * G_MPS2;
  float w2   = dot3(gyroRad, gyroRad);
  float lin2 = dot3(linAccel, linAccel);

  bool still = (w2 < wMax * wMax) && (lin2 < aMax * aMax);

  if (still) { 
    if (stationaryCount < 255) {
      stationaryCount++; 
    } 
    else { 
      stationaryCount = 0; 
    }
  }
}


// Double integration with zero-velocity update (ZUPT): once the pen has been still for
// ZUPT_MIN_SAMPLES, velocity is zero by definition and position is held.
void updatePosition(float dt) {
  if (stationaryCount >= ZUPT_MIN_SAMPLES) {

    velMps[0] = velMps[1] = velMps[2] = 0.0f;

    for (uint8_t i = 0; i < 3; i++) {
      prevLinAccel[i] = linAccel[i]; 
    }

    return;
  }

  for (uint8_t i = 0; i < 3; i++) {

    float vNew      = velMps[i] + 0.5f * (prevLinAccel[i] + linAccel[i]) * dt;   // trapezoid rule

    posM[i]        += 0.5f * (velMps[i] + vNew) * dt;
    velMps[i]       = vNew;
    prevLinAccel[i] = linAccel[i];
  }
}




// ============================================================================
// 7. RAY - PLANE
// ============================================================================

// Intersect the LINE origin + t*dir with the plane. Only accepts rays that point into the
// plane (dir . normal < -MIN_RAY_DOWNWARD), so near-parallel rays never produce huge jumps.
// t may be slightly negative if drift put the sensor below the plane; the line is still used.
bool intersectRayWithPlane(const float origin[3], const float dir[3], float hit[3]) {
  float denom = dot3(dir, writingPlane.normal);
  if (denom > -MIN_RAY_DOWNWARD) {
    return false;
  }
  float toPlane[3] = { writingPlane.point[0] - origin[0],
                       writingPlane.point[1] - origin[1],
                       writingPlane.point[2] - origin[2] };

  float t = dot3(toPlane, writingPlane.normal) / denom;

  if (fabsf(t) > MAX_RAY_LENGTH_M) {
    return false;
  }

  hit[0] = origin[0] + t * dir[0];
  hit[1] = origin[1] + t * dir[1];
  hit[2] = origin[2] + t * dir[2];

  return true;
}




// ============================================================================
// 8. CALIBRATION (non-blocking: runs inside the normal 500 Hz tick)
// ============================================================================

void startCalibration(uint32_t nowMs) {

  if (systemState == STATE_NO_IMU) {
    return;
  }

  if (penDown) {                      // close any open stroke first
    penDown = false;
    
    LineBuf lb; 

    lb.reset();
    lb.putText(F("U,")); lb.putUnsigned(strokeId); lb.comma();
    lb.putUnsigned(micros()); lb.comma();
    lb.putUnsigned(overrunCount); lb.comma(); lb.putUnsigned(txDropCount); lb.endLine();

    sendLine(lb, true);
  }

  for (uint8_t i = 0; i < 3; i++) { 
    calSumAccel[i] = 0; 
    calSumGyro[i] = 0; 
  }

  calSumGyroSq    = 0;
  calSamplesTaken = 0;
  calStateStartMs = nowMs;
  systemState     = STATE_CAL_SETTLE;

  LineBuf lb; 
  
  lb.reset(); lb.putText(F("C,START")); lb.endLine(); 
  sendLine(lb, true);
}


// Called after CAL_SAMPLE_COUNT samples. Returns to WAIT_CAL (with an error line) if anything is off.
void finishCalibration() {

  float n = (float)calSamplesTaken;
  float meanA[3], meanW[3];


  for (uint8_t i = 0; i < 3; i++) { 
    meanA[i] = calSumAccel[i] / n; 
    meanW[i] = calSumGyro[i]  / n; 
  }


  float gyroVar = calSumGyroSq / n - dot3(meanW, meanW);

  if (gyroVar < 0.0f) {
    gyroVar = 0.0f;
  }

  if (sqrtf(gyroVar) > CAL_MAX_GYRO_STD_DPS) { 
    sendError(F("CAL_MOVED")); 
    systemState = STATE_WAIT_CAL; 
    return; 
  }


  float g = sqrtf(dot3(meanA, meanA));

  if (g < CAL_GRAVITY_MIN_G || g > CAL_GRAVITY_MAX_G) { 
    sendError(F("CAL_GRAVITY")); 
    systemState = STATE_WAIT_CAL; 
    return; 
  }


  // Initial roll/pitch from gravity, yaw := 0 (see header for the convention).
  float roll  = atan2f(meanA[1], meanA[2]);
  float pitch = atan2f(-meanA[0], sqrtf(meanA[1]*meanA[1] + meanA[2]*meanA[2]));
  float q0[4];

  quaternionFromRollPitch(roll, pitch, q0);


  // Check the ray axis really points at the paper in this starting pose.
  float saveQ[4] = { quat[0], quat[1], quat[2], quat[3] };

  for (uint8_t i = 0; i < 4; i++) {
    quat[i] = q0[i];
  }

  float R[9];
  quaternionToMatrix(R);


  float d0[3] = { R[0]*rayAxisUnit[0] + R[1]*rayAxisUnit[1] + R[2]*rayAxisUnit[2],
                  R[3]*rayAxisUnit[0] + R[4]*rayAxisUnit[1] + R[5]*rayAxisUnit[2],
                  R[6]*rayAxisUnit[0] + R[7]*rayAxisUnit[1] + R[8]*rayAxisUnit[2] };
  float planePoint[3] = { 0.0f, 0.0f, -PLANE_OFFSET_M };
  float planeNormal[3] = { 0.0f, 0.0f, 1.0f };

  for (uint8_t i = 0; i < 3; i++) { 
    writingPlane.point[i] = planePoint[i]; 
    writingPlane.normal[i] = planeNormal[i]; 
  }


  float zero[3] = { 0, 0, 0 };
  float hit0[3];

  if (!intersectRayWithPlane(zero, d0, hit0)) {        // wrong axis/sign, or plane too far

    for (uint8_t i = 0; i < 4; i++) {
      quat[i] = saveQ[i];
    }

    sendError(F("CAL_RAY"));
    systemState = STATE_WAIT_CAL;
    return;
  }


  // Commit.
  gravityRefG = g;
  for (uint8_t i = 0; i < 3; i++) {
    gyroBiasDps[i] = meanW[i];
  }
  for (uint8_t i = 0; i < 4; i++) {
    calibrationQuat[i] = q0[i];
  }
  originX_m = hit0[0];
  originY_m = hit0[1];
  resetMotionState();
  penDown = false;
  strokeId = 0;
  lastSampleUs = lastSample.timestampUs;
  systemState = STATE_TRACKING;


  LineBuf lb; 

  lb.reset();
  lb.putText(F("C,OK,"));
  lb.putSigned((int32_t)(   roll  * RAD2DEG_F * 100.0f )); lb.comma();
  lb.putSigned((int32_t)(   pitch * RAD2DEG_F * 100.0f )); lb.comma();
  lb.putSigned((int32_t)(                   g * 1000.0f)); lb.comma();
  lb.putSigned((int32_t)(            meanW[0] * 100.0f )); lb.comma();
  lb.putSigned((int32_t)(            meanW[1] * 100.0f )); lb.comma();
  lb.putSigned((int32_t)(            meanW[2] * 100.0f )); 
  lb.endLine();

  sendLine(lb, true);
}




// ============================================================================
// 9. TRACKING TICK
// ============================================================================

void sendPenUp(uint32_t tUs) {
  LineBuf lb; lb.reset();
  lb.putText(F("U,")); lb.putUnsigned(strokeId); lb.comma(); lb.putUnsigned(tUs); lb.comma();
  lb.putUnsigned(overrunCount); lb.comma(); lb.putUnsigned(txDropCount); lb.endLine();
  sendLine(lb, true);
}


void enterErrorState(const __FlashStringHelper* code) {
  if (penDown) { penDown = false; sendPenUp(micros()); }
  sendError(code);
  systemState = STATE_WAIT_CAL;
}


void trackingTick(const ImuSample& s, float dt) {

  // 1. Bias-corrected gyro in rad/s.
  float gyroRad[3];
  for (uint8_t i = 0; i < 3; i++) gyroRad[i] = (s.gyroDps[i] - gyroBiasDps[i]) * DEG2RAD_F;


  // 2. Orientation, then the rotation matrix used by everything below.
  updateOrientation(s.accelG, gyroRad, dt);
  float R[9];
  quaternionToMatrix(R);


  // 3. Stillness detector (also steers the orientation filter), then sensor position (experimental).
  updateStillness(R, s.accelG, gyroRad);
  if (INTEGRATE_POSITION) updatePosition(dt);


  // 4. Sanity: catch NaN/inf/runaway values instead of sending garbage.
  float qn2 = quat[0]*quat[0] + quat[1]*quat[1] + quat[2]*quat[2] + quat[3]*quat[3];
  if (!(qn2 > 0.9f && qn2 < 1.1f) ||
      !(fabsf(posM[0]) < 100.0f && fabsf(posM[1]) < 100.0f && fabsf(posM[2]) < 100.0f)) {
    enterErrorState(F("NUMERIC"));
    return;
  }


  // 5. Height above the plane (signed distance along the plane normal) and pen state.
  float rel[3] = { posM[0] - writingPlane.point[0], posM[1] - writingPlane.point[1], posM[2] - writingPlane.point[2] };
  float h = dot3(rel, writingPlane.normal);
  lastHeightM = h;

  bool wasDown = penDown;
  if (penDown) { if (h > PEN_UP_HEIGHT_M) penDown = false; }
  else         { if (h < PEN_UP_HEIGHT_M - PEN_HEIGHT_HYSTERESIS_M) penDown = true; }

  if (!wasDown && penDown) {
    strokeId++;
  }
  if (wasDown && !penDown) { 
    sendPenUp(s.timestampUs); 
    return; 
  }
  if (!penDown) { // h too large: do nothing
    return;
  }


  // 6. Pen is down: shoot the ray along the chosen sensor axis and intersect with the plane.
  float dir[3] = { R[0]*rayAxisUnit[0] + R[1]*rayAxisUnit[1] + R[2]*rayAxisUnit[2],
                   R[3]*rayAxisUnit[0] + R[4]*rayAxisUnit[1] + R[5]*rayAxisUnit[2],
                   R[6]*rayAxisUnit[0] + R[7]*rayAxisUnit[1] + R[8]*rayAxisUnit[2] };
  float hit[3];
  if (!intersectRayWithPlane(posM, dir, hit)) { // no valid hit: send nothing, never a fake point
    return;
  }   


  LineBuf lb; 

  lb.reset();
  lb.putText(F("D,")); lb.putUnsigned(strokeId);   lb.comma();
  lb.putUnsigned(s.timestampUs);                   lb.comma();
  lb.putSigned(toOutputUnits(hit[0] - originX_m)); lb.comma();
  lb.putSigned(toOutputUnits(hit[1] - originY_m)); 
  lb.endLine();

  sendLine(lb, false);
}




// ============================================================================
// 10. DIAGNOSTICS, LED, BUTTON, SCHEDULER
// ============================================================================


// Sent only while no stroke is active, so they never compete with drawing data for the
// 64-byte TX buffer. One line per DIAG_INTERVAL_MS, rotating S -> R -> Q.
void maybeSendDiagnostics(uint32_t nowMs) {

  if (penDown) {
   return;
  }
  if ((uint32_t)(nowMs - lastDiagMs) < DIAG_INTERVAL_MS) {
    return;
  }
  lastDiagMs = nowMs;

  LineBuf lb; 

  lb.reset();
  uint8_t phase = diagPhase;
  diagPhase = (diagPhase + 1) % 3;
  if (phase == 0) {
    lb.putText(F("S,")); 
    lb.putUnsigned(systemState);                                                         lb.comma();
    lb.putSigned(systemState == STATE_TRACKING ? (int32_t)(lastHeightM * 10000.0f) : 0); lb.comma();
    lb.putUnsigned(overrunCount);                                                        lb.comma(); 
    lb.putUnsigned(txDropCount);                                                         lb.comma();
    lb.putUnsigned(i2cFailTotal);                                                        lb.comma(); 
    lb.putUnsigned(loopMaxUs); 
    lb.endLine();
    loopMaxUs = 0;
  } 
  else if (phase == 1) {
    if (systemState == STATE_NO_IMU) {
      return;
    }
    lb.putText(F("R,"));
    for (uint8_t i = 0; i < 3; i++) { 
      lb.putSigned((int32_t)(lastSample.accelG[i] * 1000.0f)); lb.comma(); 
    }
    for (uint8_t i = 0; i < 3; i++) { 
      lb.putSigned((int32_t)(lastSample.gyroDps[i] * 10.0f)); if (i < 2) lb.comma(); 
    }

    lb.endLine();
  } 
  else {
    if (systemState != STATE_TRACKING) {
      return;
    }

    lb.putText(F("Q,"));

    for (uint8_t i = 0; i < 4; i++) { 

      lb.putSigned((int32_t)(quat[i] * 1000.0f)); 

      if (i < 3) {
        lb.comma();
      } 
    }

    lb.endLine();
  }

  sendLine(lb, false);
}


void updateStatusLed(uint32_t nowMs) {
  bool on = false;
  switch (systemState) {
    case STATE_NO_IMU:       on = ((nowMs / 100) % 2) == 0; break;   // fast blink
    case STATE_WAIT_CAL:     on = (nowMs % 1000) < 100;     break;   // short flash each second
    case STATE_CAL_SETTLE:
    case STATE_CAL_SAMPLING: on = true;                     break;   // solid while calibrating
    default:                 on = penDown;                  break;   // tracking: lit while pen is down
  }
  digitalWrite(PIN_STATUS_LED, on ? HIGH : LOW);
}


// Debounced press event: true exactly once per press, never while the button is held.
bool buttonPressEvent(uint32_t nowMs) {

  static bool lastRaw = HIGH, 
              stable  = HIGH;

  static uint32_t lastChangeMs = 0;

  bool raw = digitalRead(PIN_CAL_BUTTON);

  if (raw != lastRaw) { 
    lastRaw = raw; 
    lastChangeMs = nowMs; 
  }

  if ((uint32_t)(nowMs - lastChangeMs) >= BUTTON_DEBOUNCE_MS && raw != stable) {
    stable = raw;
    if (stable == LOW) return true;
  }

  return false;
}


// Fixed-rate schedule: ticks stay on a 2 ms grid; if we fall a whole period behind we skip
// ahead rather than firing a burst of catch-up ticks.
bool sampleDue(uint32_t nowUs) {

  static uint32_t nextTickUs = 0;
  
  static bool started = false;

  if (!started) { 
    nextTickUs = nowUs; 
    started = true; 
  }

  if ((int32_t)(nowUs - nextTickUs) < 0) { 
    return false;
  }

  nextTickUs += SAMPLE_PERIOD_US;

  if ((int32_t)(nowUs - nextTickUs) >= 0) {
    nextTickUs = nowUs + SAMPLE_PERIOD_US;
  }

  return true;
}


void handleReadFailure(uint32_t nowMs) {

  i2cFailTotal++;

  if (i2cFailStreak < 255) {
    i2cFailStreak++;
  }

  if (i2cFailStreak >= I2C_FAIL_LIMIT) {

    if (penDown) { 
      penDown = false; 
      sendPenUp(micros()); 
    }

    sendError(F("I2C_LOST"));
    systemState = STATE_NO_IMU;
    lastRetryMs = nowMs;
  }
}


void runTick(uint32_t nowMs) {

  if (systemState == STATE_NO_IMU) {

    if ((uint32_t)(nowMs - lastRetryMs) >= IMU_RETRY_INTERVAL_MS) {

      lastRetryMs = nowMs;
      uint8_t status = mpuBegin();

      if (status == MPU_OK) {
        i2cFailStreak = 0;
        resetMotionState();
        systemState = STATE_WAIT_CAL;
        sendInfo(F("IMU_OK press button to calibrate"));
      } 
      else {
        sendError(status == MPU_BAD_CONFIG ? F("MPU_CONFIG") : F("MPU_NOT_FOUND"));
      }
    }

    return;
  }

  ImuSample s;
  if (!mpuReadSample(s)) { 
    handleReadFailure(nowMs); 
    return; 
  }
  i2cFailStreak = 0;
  lastSample = s;


  float dt = (float)(uint32_t)(s.timestampUs - lastSampleUs) * 1.0e-6f;
  lastSampleUs = s.timestampUs;
  if (dt < 0.0002f) dt = 0.0002f;                    // guard against a bogus first/duplicate interval
  if (dt > 0.02f)   dt = 0.02f;


  switch (systemState) {

    case STATE_CAL_SETTLE:

      if ((uint32_t)(nowMs - calStateStartMs) >= CAL_SETTLE_MS)  systemState = STATE_CAL_SAMPLING;

      break;


    case STATE_CAL_SAMPLING:

      for (uint8_t i = 0; i < 3; i++) { 
        calSumAccel[i] += s.accelG[i]; 
        calSumGyro[i] += s.gyroDps[i]; 
      }

      calSumGyroSq += dot3(s.gyroDps, s.gyroDps);

      if (++calSamplesTaken >= CAL_SAMPLE_COUNT)  finishCalibration();

      break;

    case STATE_TRACKING:
      trackingTick(s, dt);
      break;

    default: // WAIT_CAL: just keep reading for the R lines
      break;   
  }
}




// ============================================================================
// 11. ARDUINO ENTRY POINTS
// ============================================================================


void setup() {
  pinMode(PIN_CAL_BUTTON, INPUT_PULLUP);
  pinMode(PIN_STATUS_LED, OUTPUT);

  Serial.begin(SERIAL_BAUD);

  Wire.begin();

#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(3000, true);                   // never hang forever on a stuck bus
#endif
  Wire.setClock(mpuConfig.i2cClockHz);
  if (DISABLE_I2C_INTERNAL_PULLUPS) { 
    digitalWrite(SDA, LOW); 
    digitalWrite(SCL, LOW); 
  }

  float n = sqrtf(dot3(RAY_AXIS_SENSOR, RAY_AXIS_SENSOR));
  if (n < 1e-6f) { 
    rayAxisUnit[0] = 0; 
    rayAxisUnit[1] = 0; 
    rayAxisUnit[2] = -1; 
  }
  else for (uint8_t i = 0; i < 3; i++) {
    rayAxisUnit[i] = RAY_AXIS_SENSOR[i] / n;
  }

  sendInfo(F("FOSS_PEN v0.1 experimental"));

  uint8_t status = mpuBegin();

  if (status == MPU_OK) {
    systemState = STATE_WAIT_CAL;
    lastSampleUs = micros();
    sendInfo(F("IMU_OK press button to calibrate"));
  } 
  else {
    systemState = STATE_NO_IMU;
    lastRetryMs = millis();
    sendError(status == MPU_BAD_CONFIG ? F("MPU_CONFIG") : F("MPU_NOT_FOUND"));
  }
}

void loop() {
  uint32_t nowMs = millis();

  if (buttonPressEvent(nowMs)) {
    startCalibration(nowMs);
  }

  updateStatusLed(nowMs);

  maybeSendDiagnostics(nowMs);


  uint32_t nowUs = micros();

  if (!sampleDue(nowUs)) {
    return;
  }

  runTick(nowMs);


  uint32_t busyUs = micros() - nowUs;                // time spent inside this tick

  if (busyUs > 65535UL) {
    busyUs = 65535UL;
  }

  if ((uint16_t)busyUs > loopMaxUs) {
    loopMaxUs = (uint16_t)busyUs;
  }

  if (busyUs > SAMPLE_PERIOD_US) {                  // tick took longer than the 2 ms period
    overrunCount++;
  }
         
}
