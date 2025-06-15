/*
 * Jump table for V5 SDK functions dumped from libv5rt.a using ghidra.
 * Last dumped: March 17, 2024 by Richard
 *
 * Includes header definitions from libv5rt headers.
 *
 * In order to call functions like vexDeviceMotorTypeGet(), you MUST first use claim_port. This is to ensure the mutex for VDML
 * in PROS is properly locked. You must then return_port() afterwards in order to allow release the mutex. Failure to do so will
 * result in PROS not working.
 */

#pragma once
#include <stdarg.h>
#include <stdint.h>

inline uint32_t fnVexSystemVersion() {
	return *(uint32_t*) (0x037FD000);
}
inline uint32_t fnVexosVersion() {
	return *(uint32_t*) (0x037fd008);
}
inline uint32_t fnStdlibVersion() {
	return *(uint32_t*) (0x037fd004);
}
inline uint32_t fnCpu0Version() {
	return *(uint32_t*) (0x037fd00c);
}

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
	kDeviceTypeNoSensor = 0,
	kDeviceTypeMotorSensor = 2,
	kDeviceTypeLedSensor = 3,
	kDeviceTypeAbsEncSensor = 4,
	kDeviceTypeCrMotorSensor = 5,
	kDeviceTypeImuSensor = 6,
	kDeviceTypeRangeSensor = 7,// obsolete
	kDeviceTypeDistanceSensor = 7,
	kDeviceTypeRadioSensor = 8,
	kDeviceTypeTetherSensor = 9,
	kDeviceTypeBrainSensor = 10,
	kDeviceTypeVisionSensor = 11,
	kDeviceTypeAdiSensor = 12,
	kDeviceTypeRes1Sensor = 13,
	kDeviceTypeRes2Sensor = 14,
	kDeviceTypeRes3Sensor = 15,
	kDeviceTypeOpticalSensor = 16,
	kDeviceTypeMagnetSensor = 17,
	kDeviceTypeGpsSensor = 20,
	kDeviceTypeAicameraSensor = 26,
	kDeviceTypeLightTowerSensor = 27,
	kDeviceTypeArmDevice = 28,
	kDeviceTypeAiVisionSensor = 29,
	kDeviceTypePneumaticSensor = 30,
	kDeviceTypeBumperSensor = 0x40,
	kDeviceTypeGyroSensor = 0x46,
	kDeviceTypeSonarSensor = 0x47,
	kDeviceTypeGenericSensor = 128,
	kDeviceTypeGenericSerial = 129,
	kDeviceTypeUndefinedSensor = 255
} V5_DeviceType;

typedef struct _V5_Device* V5_DeviceT;

/*----------------------------------------------------------------------------*/
/** @brief      V5 Controller definitions                                     */
/*----------------------------------------------------------------------------*/
//
// March 2018, obsolete types now removed
// additional enum for final V5 controller labels added
//
typedef enum _V5_ControllerIndex {
	AnaLeftX = 0,
	AnaLeftY,
	AnaRightX,
	AnaRightY,
	AnaSpare1,
	AnaSpare2,

	Button5U,
	Button5D,
	Button6U,
	Button6D,
	Button7U,
	Button7D,
	Button7L,
	Button7R,
	Button8U,
	Button8D,
	Button8L,
	Button8R,

	ButtonSEL,

	BatteryLevel,

	ButtonAll,
	Flags,
	BatteryCapacity,

	// Final V5 names
	Axis1 = AnaRightX,
	Axis2 = AnaRightY,
	Axis3 = AnaLeftY,
	Axis4 = AnaLeftX,

	ButtonL1 = Button5U,
	ButtonL2 = Button5D,
	ButtonR1 = Button6U,
	ButtonR2 = Button6D,

	ButtonUp = Button7U,
	ButtonDown = Button7D,
	ButtonLeft = Button7L,
	ButtonRight = Button7R,

	ButtonX = Button8U,
	ButtonB = Button8D,
	ButtonY = Button8L,
	ButtonA = Button8R
} V5_ControllerIndex;

typedef enum _V5_ControllerStatus { kV5ControllerOffline = 0, kV5ControllerTethered, kV5ControllerVexnet } V5_ControllerStatus;

typedef enum _V5_ControllerId { kControllerMaster = 0, kControllerPartner } V5_ControllerId;

/*----------------------------------------------------------------------------*/
/** @brief      V5 LED device definitions                                     */
/*----------------------------------------------------------------------------*/
// The LED is an obsolete sensor but we will leave as it's available on the Dev
// systems
typedef enum _V5_DeviceLedColor {
	kLedColorBlack = 0,
	kLedColorRed = 0xFF0000,
	kLedColorGreen = 0x00FF00,
	kLedColorBlue = 0x0000FF,
	kLedColorYellow = 0xFFFF00,
	kLedColorCyan = 0x00FFFF,
	kLedColorMagenta = 0xFF00FF,
	kLedColorWhite = 0xFFFFFF
} V5_DeviceLedColor;

/*----------------------------------------------------------------------------*/
/** @brief      V5 ADI device definitions                                     */
/*----------------------------------------------------------------------------*/
typedef enum _V5_AdiPortConfiguration {
	kAdiPortTypeAnalogIn = 0,
	kAdiPortTypeAnalogOut,
	kAdiPortTypeDigitalIn,
	kAdiPortTypeDigitalOut,

	kAdiPortTypeSmartButton,
	kAdiPortTypeSmartPot,

	kAdiPortTypeLegacyButton,
	kAdiPortTypeLegacyPotentiometer,
	kAdiPortTypeLegacyLineSensor,
	kAdiPortTypeLegacyLightSensor,
	kAdiPortTypeLegacyGyro,
	kAdiPortTypeLegacyAccelerometer,

	kAdiPortTypeLegacyServo,
	kAdiPortTypeLegacyPwm,

	kAdiPortTypeQuadEncoder,
	kAdiPortTypeSonar,

	kAdiPortTypeLegacyPwmSlew,

	kAdiPortTypeUndefined = 255
} V5_AdiPortConfiguration;

// ADI sensor has 8 ports
#define V5_ADI_PORT_NUM 8

/*----------------------------------------------------------------------------*/
/** @brief      V5 Bumper switch device definitions                           */
/*----------------------------------------------------------------------------*/
typedef enum _V5_DeviceBumperState {
	kBumperReleased = 0,/// Switch pressed
	kBumperPressed = 1  /// Switch released
} V5_DeviceBumperState;

/*----------------------------------------------------------------------------*/
/** @brief      V5 Motor definitions                                          */
/*----------------------------------------------------------------------------*/
// Most of this is still TBD and is currently based on the IQ implementation
typedef enum {
	kMotorControlModeOFF = 0,     /// Motor is off and in coast mode
	kMotorControlModeBRAKE = 1,   /// Motor is off and in brake mode
	kMotorControlModeHOLD = 2,    /// Motor is holding at current position
	kMotorControlModeSERVO = 3,   /// Motor is in "Servo" mode. Move to position and
	                              /// hold at that position
	kMotorControlModePROFILE = 4, /// Motor moves to set position and stops.
	kMotorControlModeVELOCITY = 5,/// Motor is unlimited movement at set 'velocity'
	kMotorControlModeUNDEFINED = 6///
} V5MotorControlMode;

typedef enum _V5_MotorBrakeMode {
	kV5MotorBrakeModeCoast = 0,/// Motor will coast when stopped
	kV5MotorBrakeModeBrake = 1,/// Motor will brake when stopped
	kV5MotorBrakeModeHold = 2  /// Motor will hold position when stopped
} V5MotorBrakeMode;

typedef enum {
	kMotorEncoderDegrees = 0,  /// degrees Encoder Count Mode
	kMotorEncoderRotations = 1,/// rotations Encoder Count Mode
	kMotorEncoderCounts = 2    /// Raw Encoder Count Mode
} V5MotorEncoderUnits;

typedef enum _V5MotorGearset {
	kMotorGearSet_36 = 0,/// 36:1 gear set, torque
	kMotorGearSet_18 = 1,/// 18:1 gear set, speed
	kMotorGearSet_06 = 2 /// 6:1 gear set, high speed
} V5MotorGearset;

// This is for 36:1 gear set
#define V5_MOTOR_COUNTS_PER_ROT 1800.0

//
// preliminary, used for position and velocity
//
typedef struct __attribute__((__packed__)) _V5_DeviceMotorPid {
	uint8_t kf;
	uint8_t kp;
	uint8_t ki;
	uint8_t kd;
	uint8_t filter;
	uint8_t pad1;
	uint16_t limit;
	uint8_t threshold;
	uint8_t loopspeed;
	uint8_t pad2[2];
} V5_DeviceMotorPid;

/*----------------------------------------------------------------------------*/
/** @brief      V5 Vision sensor definitions                                  */
/*----------------------------------------------------------------------------*/

// subject to change
typedef enum { kVisionModeNormal = 0, kVisionModeMixed = 1, kVisionModeLineDetect = 2, kVisionTypeTest = 3 } V5VisionMode;

typedef enum { kVisionTypeNormal = 0, kVisionTypeColorCode = 1, kVisionTypeLineDetect = 2 } V5VisionBlockType;

// White balance
typedef enum { kVisionWBNormal = 0, kVisionWBStart = 1, kVisionWBManual = 2 } V5VisionWBMode;

// LED control, from V5 or by the vision sensor
typedef enum { kVisionLedModeAuto = 0, kVisionLedModeManual = 1 } V5VisionLedMode;

// Wifi mode
typedef enum { kVisionWifiModeOff = 0, kVisionWifiModeOn = 1 } V5VisionWifiMode;

// signature should be 40 bytes and is the same data
// as sent to the vision sensor

// if bit0 is now set you know signature is read back
#define VISION_SIG_FLAG_STATUS (1 << 0)

typedef struct __attribute__((__packed__)) _V5_DeviceVisionSignature {
	uint8_t id;
	uint8_t flags;
	uint8_t pad[2];
	float range;
	int32_t uMin;
	int32_t uMax;
	int32_t uMean;
	int32_t vMin;
	int32_t vMax;
	int32_t vMean;
	uint32_t mRgb;
	uint32_t mType;
} V5_DeviceVisionSignature;

typedef struct __attribute__((__packed__)) _V5_DeviceVisionObject {
	uint16_t signature;    /// block signature
	V5VisionBlockType type;/// block type
	uint16_t xoffset;      /// left side of block
	uint16_t yoffset;      /// top of block
	uint16_t width;        /// width of block
	uint16_t height;       /// height of block
	uint16_t angle;        /// angle of CC block in 0.1 deg units
} V5_DeviceVisionObject;

// Color structure, used for LED and white balance
typedef struct __attribute__((__packed__)) _V5_DeviceVisionRgb {
	uint8_t red;
	uint8_t green;
	uint8_t blue;
	uint8_t brightness;/// not used yet
} V5_DeviceVisionRgb;

/*----------------------------------------------------------------------------*/
/** @brief      V5 IMU sensor definitions                                     */
/*----------------------------------------------------------------------------*/

// Quaternion data from IMU
typedef struct __attribute__((__packed__)) _V5_DeviceImuQuaternion {
	double a;
	double b;
	double c;
	double d;
} V5_DeviceImuQuaternion;

// Attitude data from IMU
typedef struct __attribute__((__packed__)) _V5_DeviceImuAttitude {
	double pitch;// x
	double roll; // y
	double yaw;  // z
} V5_DeviceImuAttitude;

// Raw data from IMU
typedef struct __attribute__((__packed__)) _V5_DeviceImuRaw {
	double x;
	double y;
	double z;
	double w;
} V5_DeviceImuRaw;

// native is same direction as 3wire gyro
// clockwise is positive
typedef enum _V5ImuHeadingnMode { kImuHeadingNative = 0x00, kImuHeadingIQ = 0x01 } _V5ImuHeadingnMode;

// Orientation mode
typedef enum _V5ImuOrientationMode {
	kImuOrientationZUp = 0x00,
	kImuOrientationZDown = 0x10,
	kImuOrientationXUp = 0x20,
	kImuOrientationXDown = 0x30,
	kImuOrientationYUp = 0x40,
	kImuOrientationYDown = 0x50,
	kImuOrientationAuto = 0x80
} V5ImuOrientationMode;

// Quaternion mode
typedef enum _V5ImuQuaternionMode { kImuQuaternionProcessed = 0x000, kImuQuaternionRaw = 0x100 } V5ImuQuaternionMode;

/*----------------------------------------------------------------------------*/
/** @brief      V5 Color sensor definitions                                   */
/*----------------------------------------------------------------------------*/
// for raw crgb data
typedef struct _V5_DeviceOpticalRaw {
	uint16_t clear;
	uint16_t red;
	uint16_t green;
	uint16_t blue;
} V5_DeviceOpticalRaw;

typedef struct _V5_DeviceOpticalRgb {
	double red;
	double green;
	double blue;
	double brightness;
} V5_DeviceOpticalRgb;

// gesture data
typedef struct _V5_DeviceOpticalGesture {
	uint8_t udata;
	uint8_t ddata;
	uint8_t ldata;
	uint8_t rdata;
	uint8_t type;
	uint8_t pad;
	uint16_t count;
	uint32_t time;
} V5_DeviceOpticalGesture;

/*----------------------------------------------------------------------------*/
/** @brief      V5 magnet definitions                                         */
/*----------------------------------------------------------------------------*/
typedef enum _V5_DeviceMagnetDuration {
	kMagnetDurationShort,
	kMagnetDurationMedium,
	kMagnetDurationLong,
	kMagnetDurationExtraLong,
} V5_DeviceMagnetDuration;

/*----------------------------------------------------------------------------*/
/** @brief      V5 gps definitions                                            */
/*----------------------------------------------------------------------------*/
// Quaternion data from GPS
typedef struct __attribute__((__packed__)) _V5_DeviceGpsQuaternion {
	double a;
	double b;
	double c;
	double d;
} V5_DeviceGpsQuaternion;

// Attitude data from GPS, collect all useful info into this structure now
typedef struct __attribute__((__packed__)) _V5_DeviceGpsAttitude {
	double pitch;// x
	double roll; // y
	double yaw;  // z

	// spacial position on the field
	double position_x;
	double position_y;
	double position_z;

	// alternative roll, pitch and yaw
	double az;
	double el;
	double rot;
} V5_DeviceGpsAttitude;

// Raw data from GPS
typedef struct __attribute__((__packed__)) _V5_DeviceGpsRaw {
	double x;
	double y;
	double z;
	double w;
} V5_DeviceGpsRaw;

/*----------------------------------------------------------------------------*/
/** @brief      V5 AI Camera definitions                                      */
/*----------------------------------------------------------------------------*/

// detected object 1.1.3 and later
typedef struct __attribute__((__packed__)) _V5_DeviceAicamObject {
	uint8_t group_id;
	uint8_t object_id;
	uint8_t classID;
	uint8_t probability;
	int16_t depth;
	int16_t mapX;
	int16_t mapY;
	int16_t mapZ;
	int16_t screenWidth;
	int16_t screenHeight;
	int16_t screenX;
	int16_t screenY;
} V5_DeviceAicamObject;

/*----------------------------------------------------------------------------*/
/** @brief      V5 CTE Arm definitions                                        */
/*----------------------------------------------------------------------------*/

typedef struct _V5_DeviceArmTipPosition {
	int32_t tip_x;
	int32_t tip_y;
	int32_t tip_z;
	int32_t tip_roll;
	int32_t tip_pitch;
	int32_t tip_yaw;
	int8_t pose;
	uint16_t velocity;
} V5_DeviceArmTipPosition;

/*----------------------------------------------------------------------------*/
/** @brief      V5 AI Vision definitions                                      */
/*----------------------------------------------------------------------------*/

// detected object
typedef struct __attribute((packed)) _V5_DeviceAiVisionObject {
	uint8_t id;  /// object color/id
	uint8_t type;/// object type
	union {
		struct {
			uint16_t xoffset;/// left side of object
			uint16_t yoffset;/// top of object
			uint16_t width;  /// width of object
			uint16_t height; /// height of object
			uint16_t angle;  /// angle of CC object in 0.1 deg units
		} color;

		struct {
			int16_t x0;/// apriltag coords
			int16_t y0;///
			int16_t x1;///
			int16_t y1;///
			int16_t x2;///
			int16_t y2;///
			int16_t x3;///
			int16_t y3;///
		} tag;

		struct {
			uint16_t xoffset;/// left side of object
			uint16_t yoffset;/// top of object
			uint16_t width;  /// width of object
			uint16_t height; /// height of object
			uint16_t score;  /// confidence score
		} model;
	} object;
} V5_DeviceAiVisionObject;

typedef struct __attribute__((__packed__)) _V5_DeviceAiVisionColor {
	uint8_t id;
	uint8_t red;
	uint8_t grn;
	uint8_t blu;
	float hangle;
	float hdsat;
	uint32_t reserved;
} V5_DeviceAiVisionColor;

typedef struct __attribute__((__packed__)) _V5_DeviceAiVisionCode {
	uint8_t id;
	uint8_t len;
	int16_t c1;
	int16_t c2;
	int16_t c3;
	int16_t c4;
	int16_t c5;
	int16_t c6;
	int16_t c7;
} V5_DeviceAiVisionCode;

/*----------------------------------------------------------------------------*/
/** @brief      V5 low level pneumatic control - dev only                     */
/*----------------------------------------------------------------------------*/

typedef struct _V5_DevicePneumaticCtrl {
	uint16_t flags;
	uint8_t m1_pwm;
	uint8_t m2_pwm;
	uint8_t m3_pwm;
	uint8_t m4_pwm;
	uint8_t m1_time;
	uint8_t m2_time;
	uint8_t m3_time;
	uint8_t m4_time;
	uint8_t comp_pwm;
} V5_DevicePneumaticCtrl;

/*----------------------------------------------------------------------------*/
/** @brief      V5 SD File system definitions                                 */
/*----------------------------------------------------------------------------*/

// Opaque pointer (FIL *) to file structure
typedef void FIL;

// seek flags
// changed, Mar 6 2018 to be more consistent with stdio.h
#define FS_SEEK_SET 0
#define FS_SEEK_CUR 1
#define FS_SEEK_END 2

// new 1.0.13, file status
#define FS_FILE_EXIST 1
#define FS_FILE_DIR 2

// File function return code (FRESULT)
typedef enum {
	FR_OK = 0U,            /// (0) Succeeded
	FR_DISK_ERR,           /// (1) A hard error occurred in the low level disk I/O layer
	FR_INT_ERR,            /// (2) Assertion failed
	FR_NOT_READY,          /// (3) The physical drive cannot work
	FR_NO_FILE,            /// (4) Could not find the file
	FR_NO_PATH,            /// (5) Could not find the path
	FR_INVALID_NAME,       /// (6) The path name format is invalid
	FR_DENIED,             /// (7) Access denied due to prohibited access or directory full
	FR_EXIST,              /// (8) Access denied due to prohibited access
	FR_INVALID_OBJECT,     /// (9) The file/directory object is invalid
	FR_WRITE_PROTECTED,    /// (10) The physical drive is write protected
	FR_INVALID_DRIVE,      /// (11) The logical drive number is invalid
	FR_NOT_ENABLED,        /// (12) The volume has no work area
	FR_NO_FILESYSTEM,      /// (13) There is no valid FAT volume
	FR_MKFS_ABORTED,       /// (14) The f_mkfs() aborted due to any parameter error
	FR_TIMEOUT,            /// (15) Could not get a grant to access the volume within defined
	                       /// period
	FR_LOCKED,             /// (16) The operation is rejected according to the file sharing
	                       /// policy
	FR_NOT_ENOUGH_CORE,    /// (17) LFN working buffer could not be allocated
	FR_TOO_MANY_OPEN_FILES,/// (18) Number of open files > _FS_SHARE
	FR_INVALID_PARAMETER   /// (19) Given parameter is invalid
} FRESULT;

/*----------------------------------------------------------------------------*/
/** @brief      V5 touch events                                               */
/*----------------------------------------------------------------------------*/
typedef enum _touchEvent { kTouchEventRelease, kTouchEventPress, kTouchEventPressAuto } V5_TouchEvent;

typedef struct _V5_TouchStatus {
	V5_TouchEvent lastEvent;
	int16_t lastXpos;
	int16_t lastYpos;
	int32_t pressCount;
	int32_t releaseCount;
} V5_TouchStatus;

/*----------------------------------------------------------------------------*/
/** @brief      V5 competition status bits                                    */
/*----------------------------------------------------------------------------*/

#define V5_COMP_BIT_EBL 1  // if set then robot disabled
#define V5_COMP_BIT_MODE 2 // if set then robot in autonomous
#define V5_COMP_BIT_COMP 4 // if set then either comp switch or field control connected
#define V5_COMP_BIT_GAME 8 // if set then field control connected
#define V5_COMP_BIT_OPT1 16// if set then option1 bit is set (vexos 1.1.3 and later)
#define V5_COMP_BIT_OPT2 32// if set then option2 bit is set (vexos 1.1.3 and later)

/*----------------------------------------------------------------------------*/
/** @brief   structure holding image info - used by bmp/png read code         */
/*----------------------------------------------------------------------------*/
// data must point to suitable buffer now
typedef struct __attribute__((__packed__)) _v5_image {
	uint16_t width;
	uint16_t height;
	uint32_t* data;
	uint32_t* p;
} v5_image;

#define SYSTEM_DISPLAY_WIDTH 480
#define SYSTEM_DISPLAY_HEIGHT 272

// PROS STUFF
/**
 * Claims the mutex for the given port.
 *
 * Reserves the mutex for this port. Any other tasks trying to access this port
 * will block until the mutex is returned. If a higher-priortiy task attempts
 * to claim this port, the task which has the port claimed will temporarily be
 * raised to an equal priority until the mutex is given, reducing the impact of
 * the delay. See FreeRTOS documentation for more details.
 *
 * This MUST be called before any call to the v5 api to maintain thread saftey.
 *
 * This function uses the following values of errno when an error state is
 * reached:
 * ENXIO - The given value is not within the range of V5 ports (0-20).
 *
 * \param port
 *        The V5 port number to claim from 0-20
 *
 *
eturn 1 if the mutex was successfully taken, 0 if not, -1 if port is
 * invalid.
 */
int port_mutex_take(uint8_t port);

/**
 * Returns the mutex for the given port.
 *
 * Frees the mutex for this port, allowing other tasks to continue.
 *
 * WARNING: If a mutex was claimed by a task, this MUST be called immediately
 * after the port is no longer needed by that task in order to prevent delays in
 * other tasks.
 *
 * This function uses the following values of errno when an error state is
 * reached:
 * ENXIO - The given value is not within the range of V5 ports (0-20).
 *
 * \param port
 *        The V5 port number to free from 0-20
 */
int port_mutex_give(uint8_t port);

// Port between 0-20
#define claim_port(port)                                                                                                       \
	V5_DeviceT device = fnVexDeviceGetByIndex(port);                                                                           \
	if (device == nullptr) { return; }                                                                                         \
	if (!port_mutex_take(port)) { return; }

// Port between 0-20
#define return_port(port) port_mutex_give(port)

typedef bool (*t_vexScratchMemoryLock)(void);
inline t_vexScratchMemoryLock fnVexScratchMemoryLock = (t_vexScratchMemoryLock) (*(uint32_t*) 0x37fc998);

typedef void (*t_vexScratchMemoryUnlock)(void);
inline t_vexScratchMemoryUnlock fnVexScratchMemoryUnlock = (t_vexScratchMemoryUnlock) (*(uint32_t*) 0x37fc99c);

typedef uint32_t (*t_vexDevicesGetNumber)(void);
inline t_vexDevicesGetNumber fnVexDevicesGetNumber = (t_vexDevicesGetNumber) (*(uint32_t*) 0x37fc190);

typedef uint32_t (*t_vexDevicesGetNumberByType)(V5_DeviceType type);
inline t_vexDevicesGetNumberByType fnVexDevicesGetNumberByType = (t_vexDevicesGetNumberByType) (*(uint32_t*) 0x37fc194);

typedef V5_DeviceT (*t_vexDevicesGet)(void);
inline t_vexDevicesGet fnVexDevicesGet = (t_vexDevicesGet) (*(uint32_t*) 0x37fc198);

typedef V5_DeviceT (*t_vexDeviceGetByIndex)(uint32_t index);
inline t_vexDeviceGetByIndex fnVexDeviceGetByIndex = (t_vexDeviceGetByIndex) (*(uint32_t*) 0x37fc19c);

typedef int32_t (*t_vexDeviceGetStatus)(V5_DeviceType* buffer);
inline t_vexDeviceGetStatus fnVexDeviceGetStatus = (t_vexDeviceGetStatus) (*(uint32_t*) 0x37fc1a0);

typedef int32_t (*t_vexDeviceGetTimestamp)(V5_DeviceT device);
inline t_vexDeviceGetTimestamp fnVexDeviceGetTimestamp = (t_vexDeviceGetTimestamp) (*(uint32_t*) 0x37fc1b0);

typedef uint32_t (*t_vexDeviceButtonStateGet)(void);
inline t_vexDeviceButtonStateGet fnVexDeviceButtonStateGet = (t_vexDeviceButtonStateGet) (*(uint32_t*) 0x37fc1b4);

typedef int32_t (*t_vexControllerGet)(V5_ControllerId id, V5_ControllerIndex index);
inline t_vexControllerGet fnVexControllerGet = (t_vexControllerGet) (*(uint32_t*) 0x37fc1a4);

typedef V5_ControllerStatus (*t_vexControllerConnectionStatusGet)(V5_ControllerId id);
inline t_vexControllerConnectionStatusGet fnVexControllerConnectionStatusGet =
        (t_vexControllerConnectionStatusGet) (*(uint32_t*) 0x37fc1a8);

typedef bool (*t_vexControllerTextSet)(V5_ControllerId id, uint32_t line, uint32_t col, const char* str);
inline t_vexControllerTextSet fnVexControllerTextSet = (t_vexControllerTextSet) (*(uint32_t*) 0x37fc1ac);

typedef void (*t_vexDeviceLedSet)(V5_DeviceT device, V5_DeviceLedColor value);
inline t_vexDeviceLedSet fnVexDeviceLedSet = (t_vexDeviceLedSet) (*(uint32_t*) 0x37fc1e0);

typedef void (*t_vexDeviceLedRgbSet)(V5_DeviceT device, uint32_t color);
inline t_vexDeviceLedRgbSet fnVexDeviceLedRgbSet = (t_vexDeviceLedRgbSet) (*(uint32_t*) 0x37fc1e4);

typedef V5_DeviceLedColor (*t_vexDeviceLedGet)(V5_DeviceT device);
inline t_vexDeviceLedGet fnVexDeviceLedGet = (t_vexDeviceLedGet) (*(uint32_t*) 0x37fc1e8);

typedef uint32_t (*t_vexDeviceLedRgbGet)(V5_DeviceT device);
inline t_vexDeviceLedRgbGet fnVexDeviceLedRgbGet = (t_vexDeviceLedRgbGet) (*(uint32_t*) 0x37fc1ec);

typedef void (*t_vexDeviceAdiPortConfigSet)(V5_DeviceT device, uint32_t port, V5_AdiPortConfiguration type);
inline t_vexDeviceAdiPortConfigSet fnVexDeviceAdiPortConfigSet = (t_vexDeviceAdiPortConfigSet) (*(uint32_t*) 0x37fc208);

typedef V5_AdiPortConfiguration (*t_vexDeviceAdiPortConfigGet)(V5_DeviceT device, uint32_t port);
inline t_vexDeviceAdiPortConfigGet fnVexDeviceAdiPortConfigGet = (t_vexDeviceAdiPortConfigGet) (*(uint32_t*) 0x37fc20c);

typedef void (*t_vexDeviceAdiValueSet)(V5_DeviceT device, uint32_t port, int32_t value);
inline t_vexDeviceAdiValueSet fnVexDeviceAdiValueSet = (t_vexDeviceAdiValueSet) (*(uint32_t*) 0x37fc210);

typedef int32_t (*t_vexDeviceAdiValueGet)(V5_DeviceT device, uint32_t port);
inline t_vexDeviceAdiValueGet fnVexDeviceAdiValueGet = (t_vexDeviceAdiValueGet) (*(uint32_t*) 0x37fc214);

typedef V5_DeviceBumperState (*t_vexDeviceBumperGet)(V5_DeviceT device);
inline t_vexDeviceBumperGet fnVexDeviceBumperGet = (t_vexDeviceBumperGet) (*(uint32_t*) 0x37fc230);

typedef void (*t_vexDeviceGyroReset)(V5_DeviceT device);
inline t_vexDeviceGyroReset fnVexDeviceGyroReset = (t_vexDeviceGyroReset) (*(uint32_t*) 0x37fc258);

typedef double (*t_vexDeviceGyroHeadingGet)(V5_DeviceT device);
inline t_vexDeviceGyroHeadingGet fnVexDeviceGyroHeadingGet = (t_vexDeviceGyroHeadingGet) (*(uint32_t*) 0x37fc25c);

typedef double (*t_vexDeviceGyroDegreesGet)(V5_DeviceT device);
inline t_vexDeviceGyroDegreesGet fnVexDeviceGyroDegreesGet = (t_vexDeviceGyroDegreesGet) (*(uint32_t*) 0x37fc260);

typedef int32_t (*t_vexDeviceSonarValueGet)(V5_DeviceT device);
inline t_vexDeviceSonarValueGet fnVexDeviceSonarValueGet = (t_vexDeviceSonarValueGet) (*(uint32_t*) 0x37fc280);

typedef int32_t (*t_vexDeviceGenericValueGet)(V5_DeviceT device);
inline t_vexDeviceGenericValueGet fnVexDeviceGenericValueGet = (t_vexDeviceGenericValueGet) (*(uint32_t*) 0x37fc2a8);

typedef void (*t_vexDeviceMotorVelocitySet)(V5_DeviceT device, int32_t velocity);
inline t_vexDeviceMotorVelocitySet fnVexDeviceMotorVelocitySet = (t_vexDeviceMotorVelocitySet) (*(uint32_t*) 0x37fc2d0);

typedef void (*t_vexDeviceMotorVelocityUpdate)(V5_DeviceT device, int32_t velocity);
inline t_vexDeviceMotorVelocityUpdate fnVexDeviceMotorVelocityUpdate =
        (t_vexDeviceMotorVelocityUpdate) (*(uint32_t*) 0x37fc374);

typedef void (*t_vexDeviceMotorVoltageSet)(V5_DeviceT device, int32_t value);
inline t_vexDeviceMotorVoltageSet fnVexDeviceMotorVoltageSet =
    (t_vexDeviceMotorVoltageSet)(*(uint32_t *)0x37fc35c);

typedef int32_t (*t_vexDeviceMotorVelocityGet)(V5_DeviceT device);
inline t_vexDeviceMotorVelocityGet fnVexDeviceMotorVelocityGet =
    (t_vexDeviceMotorVelocityGet)(*(uint32_t *)0x37fc2d4);

typedef double (*t_vexDeviceMotorActualVelocityGet)(V5_DeviceT device);
inline t_vexDeviceMotorActualVelocityGet fnVexDeviceMotorActualVelocityGet =
    (t_vexDeviceMotorActualVelocityGet)(*(uint32_t *)0x37fc2d8);

typedef int32_t (*t_vexDeviceMotorDirectionGet)(V5_DeviceT device);
inline t_vexDeviceMotorDirectionGet fnVexDeviceMotorDirectionGet =
    (t_vexDeviceMotorDirectionGet)(*(uint32_t *)0x37fc2dc);

typedef void (*t_vexDeviceMotorModeSet)(V5_DeviceT device,
                                        V5MotorControlMode mode);
inline t_vexDeviceMotorModeSet fnVexDeviceMotorModeSet = (t_vexDeviceMotorModeSet) (*(uint32_t*) 0x37fc2e0);

typedef V5MotorControlMode (*t_vexDeviceMotorModeGet)(V5_DeviceT device);
inline t_vexDeviceMotorModeGet fnVexDeviceMotorModeGet = (t_vexDeviceMotorModeGet) (*(uint32_t*) 0x37fc2e4);

typedef void (*t_vexDeviceMotorPwmSet)(V5_DeviceT device, int32_t value);
inline t_vexDeviceMotorPwmSet fnVexDeviceMotorPwmSet = (t_vexDeviceMotorPwmSet) (*(uint32_t*) 0x37fc2e8);

typedef int32_t (*t_vexDeviceMotorPwmGet)(V5_DeviceT device);
inline t_vexDeviceMotorPwmGet fnVexDeviceMotorPwmGet = (t_vexDeviceMotorPwmGet) (*(uint32_t*) 0x37fc2ec);

typedef void (*t_vexDeviceMotorCurrentLimitSet)(V5_DeviceT device, int32_t value);
inline t_vexDeviceMotorCurrentLimitSet fnVexDeviceMotorCurrentLimitSet =
        (t_vexDeviceMotorCurrentLimitSet) (*(uint32_t*) 0x37fc2f0);

typedef int32_t (*t_vexDeviceMotorCurrentLimitGet)(V5_DeviceT device);
inline t_vexDeviceMotorCurrentLimitGet fnVexDeviceMotorCurrentLimitGet =
        (t_vexDeviceMotorCurrentLimitGet) (*(uint32_t*) 0x37fc2f4);

typedef void (*t_vexDeviceMotorVoltageLimitSet)(V5_DeviceT device, int32_t value);
inline t_vexDeviceMotorVoltageLimitSet fnVexDeviceMotorVoltageLimitSet =
        (t_vexDeviceMotorVoltageLimitSet) (*(uint32_t*) 0x37fc36c);

typedef int32_t (*t_vexDeviceMotorVoltageLimitGet)(V5_DeviceT device);
inline t_vexDeviceMotorVoltageLimitGet fnVexDeviceMotorVoltageLimitGet =
        (t_vexDeviceMotorVoltageLimitGet) (*(uint32_t*) 0x37fc370);

typedef void (*t_vexDeviceMotorPositionPidSet)(V5_DeviceT device, V5_DeviceMotorPid* pid);
inline t_vexDeviceMotorPositionPidSet fnVexDeviceMotorPositionPidSet =
        (t_vexDeviceMotorPositionPidSet) (*(uint32_t*) 0x37fc378);

typedef void (*t_vexDeviceMotorVelocityPidSet)(V5_DeviceT device, V5_DeviceMotorPid* pid);
inline t_vexDeviceMotorVelocityPidSet fnVexDeviceMotorVelocityPidSet =
        (t_vexDeviceMotorVelocityPidSet) (*(uint32_t*) 0x37fc37c);

typedef int32_t (*t_vexDeviceMotorCurrentGet)(V5_DeviceT device);
inline t_vexDeviceMotorCurrentGet fnVexDeviceMotorCurrentGet = (t_vexDeviceMotorCurrentGet) (*(uint32_t*) 0x37fc2f8);

typedef int32_t (*t_vexDeviceMotorVoltageGet)(V5_DeviceT device);
inline t_vexDeviceMotorVoltageGet fnVexDeviceMotorVoltageGet = (t_vexDeviceMotorVoltageGet) (*(uint32_t*) 0x37fc360);

typedef double (*t_vexDeviceMotorPowerGet)(V5_DeviceT device);
inline t_vexDeviceMotorPowerGet fnVexDeviceMotorPowerGet = (t_vexDeviceMotorPowerGet) (*(uint32_t*) 0x37fc2fc);

typedef double (*t_vexDeviceMotorTorqueGet)(V5_DeviceT device);
inline t_vexDeviceMotorTorqueGet fnVexDeviceMotorTorqueGet = (t_vexDeviceMotorTorqueGet) (*(uint32_t*) 0x37fc300);

typedef double (*t_vexDeviceMotorEfficiencyGet)(V5_DeviceT device);
inline t_vexDeviceMotorEfficiencyGet fnVexDeviceMotorEfficiencyGet = (t_vexDeviceMotorEfficiencyGet) (*(uint32_t*) 0x37fc304);

typedef double (*t_vexDeviceMotorTemperatureGet)(V5_DeviceT device);
inline t_vexDeviceMotorTemperatureGet fnVexDeviceMotorTemperatureGet =
        (t_vexDeviceMotorTemperatureGet) (*(uint32_t*) 0x37fc308);

typedef bool (*t_vexDeviceMotorOverTempFlagGet)(V5_DeviceT device);
inline t_vexDeviceMotorOverTempFlagGet fnVexDeviceMotorOverTempFlagGet =
        (t_vexDeviceMotorOverTempFlagGet) (*(uint32_t*) 0x37fc30c);

typedef bool (*t_vexDeviceMotorCurrentLimitFlagGet)(V5_DeviceT device);
inline t_vexDeviceMotorCurrentLimitFlagGet fnVexDeviceMotorCurrentLimitFlagGet =
        (t_vexDeviceMotorCurrentLimitFlagGet) (*(uint32_t*) 0x37fc310);

typedef uint32_t (*t_vexDeviceMotorFaultsGet)(V5_DeviceT device);
inline t_vexDeviceMotorFaultsGet fnVexDeviceMotorFaultsGet = (t_vexDeviceMotorFaultsGet) (*(uint32_t*) 0x37fc354);

typedef bool (*t_vexDeviceMotorZeroVelocityFlagGet)(V5_DeviceT device);
inline t_vexDeviceMotorZeroVelocityFlagGet fnVexDeviceMotorZeroVelocityFlagGet =
        (t_vexDeviceMotorZeroVelocityFlagGet) (*(uint32_t*) 0x37fc314);

typedef bool (*t_vexDeviceMotorZeroPositionFlagGet)(V5_DeviceT device);
inline t_vexDeviceMotorZeroPositionFlagGet fnVexDeviceMotorZeroPositionFlagGet =
        (t_vexDeviceMotorZeroPositionFlagGet) (*(uint32_t*) 0x37fc318);

typedef uint32_t (*t_vexDeviceMotorFlagsGet)(V5_DeviceT device);
inline t_vexDeviceMotorFlagsGet fnVexDeviceMotorFlagsGet = (t_vexDeviceMotorFlagsGet) (*(uint32_t*) 0x37fc358);

typedef void (*t_vexDeviceMotorReverseFlagSet)(V5_DeviceT device, bool value);
inline t_vexDeviceMotorReverseFlagSet fnVexDeviceMotorReverseFlagSet =
        (t_vexDeviceMotorReverseFlagSet) (*(uint32_t*) 0x37fc31c);

typedef bool (*t_vexDeviceMotorReverseFlagGet)(V5_DeviceT device);
inline t_vexDeviceMotorReverseFlagGet fnVexDeviceMotorReverseFlagGet =
        (t_vexDeviceMotorReverseFlagGet) (*(uint32_t*) 0x37fc320);

typedef void (*t_vexDeviceMotorEncoderUnitsSet)(V5_DeviceT device, V5MotorEncoderUnits units);
inline t_vexDeviceMotorEncoderUnitsSet fnVexDeviceMotorEncoderUnitsSet =
        (t_vexDeviceMotorEncoderUnitsSet) (*(uint32_t*) 0x37fc324);

typedef V5MotorEncoderUnits (*t_vexDeviceMotorEncoderUnitsGet)(V5_DeviceT device);
inline t_vexDeviceMotorEncoderUnitsGet fnVexDeviceMotorEncoderUnitsGet =
        (t_vexDeviceMotorEncoderUnitsGet) (*(uint32_t*) 0x37fc328);

typedef void (*t_vexDeviceMotorBrakeModeSet)(V5_DeviceT device, V5MotorBrakeMode mode);
inline t_vexDeviceMotorBrakeModeSet fnVexDeviceMotorBrakeModeSet = (t_vexDeviceMotorBrakeModeSet) (*(uint32_t*) 0x37fc32c);

typedef V5MotorBrakeMode (*t_vexDeviceMotorBrakeModeGet)(V5_DeviceT device);
inline t_vexDeviceMotorBrakeModeGet fnVexDeviceMotorBrakeModeGet = (t_vexDeviceMotorBrakeModeGet) (*(uint32_t*) 0x37fc330);

typedef void (*t_vexDeviceMotorPositionSet)(V5_DeviceT device, double position);
inline t_vexDeviceMotorPositionSet fnVexDeviceMotorPositionSet = (t_vexDeviceMotorPositionSet) (*(uint32_t*) 0x37fc334);

typedef double (*t_vexDeviceMotorPositionGet)(V5_DeviceT device);
inline t_vexDeviceMotorPositionGet fnVexDeviceMotorPositionGet = (t_vexDeviceMotorPositionGet) (*(uint32_t*) 0x37fc338);

typedef int32_t (*t_vexDeviceMotorPositionRawGet)(V5_DeviceT device, uint32_t* timestamp);
inline t_vexDeviceMotorPositionRawGet fnVexDeviceMotorPositionRawGet =
        (t_vexDeviceMotorPositionRawGet) (*(uint32_t*) 0x37fc33c);

typedef void (*t_vexDeviceMotorPositionReset)(V5_DeviceT device);
inline t_vexDeviceMotorPositionReset fnVexDeviceMotorPositionReset = (t_vexDeviceMotorPositionReset) (*(uint32_t*) 0x37fc340);

typedef double (*t_vexDeviceMotorTargetGet)(V5_DeviceT device);
inline t_vexDeviceMotorTargetGet fnVexDeviceMotorTargetGet = (t_vexDeviceMotorTargetGet) (*(uint32_t*) 0x37fc344);

typedef void (*t_vexDeviceMotorServoTargetSet)(V5_DeviceT device, double position);
inline t_vexDeviceMotorServoTargetSet fnVexDeviceMotorServoTargetSet =
        (t_vexDeviceMotorServoTargetSet) (*(uint32_t*) 0x37fc348);

typedef void (*t_vexDeviceMotorAbsoluteTargetSet)(V5_DeviceT device, double position, int32_t velocity);
inline t_vexDeviceMotorAbsoluteTargetSet fnVexDeviceMotorAbsoluteTargetSet =
        (t_vexDeviceMotorAbsoluteTargetSet) (*(uint32_t*) 0x37fc34c);

typedef void (*t_vexDeviceMotorRelativeTargetSet)(V5_DeviceT device, double position, int32_t velocity);
inline t_vexDeviceMotorRelativeTargetSet fnVexDeviceMotorRelativeTargetSet =
        (t_vexDeviceMotorRelativeTargetSet) (*(uint32_t*) 0x37fc350);

typedef void (*t_vexDeviceMotorGearingSet)(V5_DeviceT device, V5MotorGearset value);
inline t_vexDeviceMotorGearingSet fnVexDeviceMotorGearingSet = (t_vexDeviceMotorGearingSet) (*(uint32_t*) 0x37fc364);

typedef V5MotorGearset (*t_vexDeviceMotorGearingGet)(V5_DeviceT device);
inline t_vexDeviceMotorGearingGet fnVexDeviceMotorGearingGet = (t_vexDeviceMotorGearingGet) (*(uint32_t*) 0x37fc368);

typedef void (*t_vexDeviceMotorExternalProfileSet)(V5_DeviceT device, double position, int32_t velocity);
inline t_vexDeviceMotorExternalProfileSet fnVexDeviceMotorExternalProfileSet =
        (t_vexDeviceMotorExternalProfileSet) (*(uint32_t*) 0x37fc380);

typedef int32_t (*t_vexDeviceMotorTypeGet)(V5_DeviceT device);
inline t_vexDeviceMotorTypeGet fnVexDeviceMotorTypeGet = (t_vexDeviceMotorTypeGet) (*(uint32_t*) 0x37fc384);

typedef void (*t_vexDeviceVisionModeSet)(V5_DeviceT device, V5VisionMode mode);
inline t_vexDeviceVisionModeSet fnVexDeviceVisionModeSet = (t_vexDeviceVisionModeSet) (*(uint32_t*) 0x37fc398);

typedef V5VisionMode (*t_vexDeviceVisionModeGet)(V5_DeviceT device);
inline t_vexDeviceVisionModeGet fnVexDeviceVisionModeGet = (t_vexDeviceVisionModeGet) (*(uint32_t*) 0x37fc39c);

typedef int32_t (*t_vexDeviceVisionObjectCountGet)(V5_DeviceT device);
inline t_vexDeviceVisionObjectCountGet fnVexDeviceVisionObjectCountGet =
        (t_vexDeviceVisionObjectCountGet) (*(uint32_t*) 0x37fc3a0);

typedef int32_t (*t_vexDeviceVisionObjectGet)(V5_DeviceT device, uint32_t indexObj, V5_DeviceVisionObject* pObject);
inline t_vexDeviceVisionObjectGet fnVexDeviceVisionObjectGet = (t_vexDeviceVisionObjectGet) (*(uint32_t*) 0x37fc3a4);

typedef void (*t_vexDeviceVisionSignatureSet)(V5_DeviceT device, V5_DeviceVisionSignature* pSignature);
inline t_vexDeviceVisionSignatureSet fnVexDeviceVisionSignatureSet = (t_vexDeviceVisionSignatureSet) (*(uint32_t*) 0x37fc3a8);

typedef bool (*t_vexDeviceVisionSignatureGet)(V5_DeviceT device, uint32_t id, V5_DeviceVisionSignature* pSignature);
inline t_vexDeviceVisionSignatureGet fnVexDeviceVisionSignatureGet = (t_vexDeviceVisionSignatureGet) (*(uint32_t*) 0x37fc3ac);

typedef void (*t_vexDeviceVisionBrightnessSet)(V5_DeviceT device, uint8_t percent);
inline t_vexDeviceVisionBrightnessSet fnVexDeviceVisionBrightnessSet =
        (t_vexDeviceVisionBrightnessSet) (*(uint32_t*) 0x37fc3b0);

typedef uint8_t (*t_vexDeviceVisionBrightnessGet)(V5_DeviceT device);
inline t_vexDeviceVisionBrightnessGet fnVexDeviceVisionBrightnessGet =
        (t_vexDeviceVisionBrightnessGet) (*(uint32_t*) 0x37fc3b4);

typedef void (*t_vexDeviceVisionWhiteBalanceModeSet)(V5_DeviceT device, V5VisionWBMode mode);
inline t_vexDeviceVisionWhiteBalanceModeSet fnVexDeviceVisionWhiteBalanceModeSet =
        (t_vexDeviceVisionWhiteBalanceModeSet) (*(uint32_t*) 0x37fc3b8);

typedef V5VisionWBMode (*t_vexDeviceVisionWhiteBalanceModeGet)(V5_DeviceT device);
inline t_vexDeviceVisionWhiteBalanceModeGet fnVexDeviceVisionWhiteBalanceModeGet =
        (t_vexDeviceVisionWhiteBalanceModeGet) (*(uint32_t*) 0x37fc3bc);

typedef void (*t_vexDeviceVisionWhiteBalanceSet)(V5_DeviceT device, V5_DeviceVisionRgb color);
inline t_vexDeviceVisionWhiteBalanceSet fnVexDeviceVisionWhiteBalanceSet =
        (t_vexDeviceVisionWhiteBalanceSet) (*(uint32_t*) 0x37fc3c0);

typedef V5_DeviceVisionRgb (*t_vexDeviceVisionWhiteBalanceGet)(V5_DeviceT device);
inline t_vexDeviceVisionWhiteBalanceGet fnVexDeviceVisionWhiteBalanceGet =
        (t_vexDeviceVisionWhiteBalanceGet) (*(uint32_t*) 0x37fc3c4);

typedef void (*t_vexDeviceVisionLedModeSet)(V5_DeviceT device, V5VisionLedMode mode);
inline t_vexDeviceVisionLedModeSet fnVexDeviceVisionLedModeSet = (t_vexDeviceVisionLedModeSet) (*(uint32_t*) 0x37fc3c8);

typedef V5VisionLedMode (*t_vexDeviceVisionLedModeGet)(V5_DeviceT device);
inline t_vexDeviceVisionLedModeGet fnVexDeviceVisionLedModeGet = (t_vexDeviceVisionLedModeGet) (*(uint32_t*) 0x37fc3cc);

typedef void (*t_vexDeviceVisionLedBrigntnessSet)(V5_DeviceT device, uint8_t percent);
inline t_vexDeviceVisionLedBrigntnessSet fnVexDeviceVisionLedBrigntnessSet =
        (t_vexDeviceVisionLedBrigntnessSet) (*(uint32_t*) 0x37fc3d0);

typedef uint8_t (*t_vexDeviceVisionLedBrigntnessGet)(V5_DeviceT device);
inline t_vexDeviceVisionLedBrigntnessGet fnVexDeviceVisionLedBrigntnessGet =
        (t_vexDeviceVisionLedBrigntnessGet) (*(uint32_t*) 0x37fc3d4);

typedef void (*t_vexDeviceVisionLedColorSet)(V5_DeviceT device, V5_DeviceVisionRgb color);
inline t_vexDeviceVisionLedColorSet fnVexDeviceVisionLedColorSet = (t_vexDeviceVisionLedColorSet) (*(uint32_t*) 0x37fc3d8);

typedef V5_DeviceVisionRgb (*t_vexDeviceVisionLedColorGet)(V5_DeviceT device);
inline t_vexDeviceVisionLedColorGet fnVexDeviceVisionLedColorGet = (t_vexDeviceVisionLedColorGet) (*(uint32_t*) 0x37fc3dc);

typedef void (*t_vexDeviceVisionWifiModeSet)(V5_DeviceT device, V5VisionWifiMode mode);
inline t_vexDeviceVisionWifiModeSet fnVexDeviceVisionWifiModeSet = (t_vexDeviceVisionWifiModeSet) (*(uint32_t*) 0x37fc3e0);

typedef V5VisionWifiMode (*t_vexDeviceVisionWifiModeGet)(V5_DeviceT device);
inline t_vexDeviceVisionWifiModeGet fnVexDeviceVisionWifiModeGet = (t_vexDeviceVisionWifiModeGet) (*(uint32_t*) 0x37fc3e4);

typedef void (*t_vexDeviceImuReset)(V5_DeviceT device);
inline t_vexDeviceImuReset fnVexDeviceImuReset = (t_vexDeviceImuReset) (*(uint32_t*) 0x37fc410);

typedef double (*t_vexDeviceImuHeadingGet)(V5_DeviceT device);
inline t_vexDeviceImuHeadingGet fnVexDeviceImuHeadingGet = (t_vexDeviceImuHeadingGet) (*(uint32_t*) 0x37fc414);

typedef double (*t_vexDeviceImuDegreesGet)(V5_DeviceT device);
inline t_vexDeviceImuDegreesGet fnVexDeviceImuDegreesGet = (t_vexDeviceImuDegreesGet) (*(uint32_t*) 0x37fc418);

typedef void (*t_vexDeviceImuQuaternionGet)(V5_DeviceT device, V5_DeviceImuQuaternion* data);
inline t_vexDeviceImuQuaternionGet fnVexDeviceImuQuaternionGet = (t_vexDeviceImuQuaternionGet) (*(uint32_t*) 0x37fc41c);

typedef void (*t_vexDeviceImuAttitudeGet)(V5_DeviceT device, V5_DeviceImuAttitude* data);
inline t_vexDeviceImuAttitudeGet fnVexDeviceImuAttitudeGet = (t_vexDeviceImuAttitudeGet) (*(uint32_t*) 0x37fc420);

typedef void (*t_vexDeviceImuRawGyroGet)(V5_DeviceT device, V5_DeviceImuRaw* data);
inline t_vexDeviceImuRawGyroGet fnVexDeviceImuRawGyroGet = (t_vexDeviceImuRawGyroGet) (*(uint32_t*) 0x37fc424);

typedef void (*t_vexDeviceImuRawAccelGet)(V5_DeviceT device, V5_DeviceImuRaw* data);
inline t_vexDeviceImuRawAccelGet fnVexDeviceImuRawAccelGet = (t_vexDeviceImuRawAccelGet) (*(uint32_t*) 0x37fc428);

typedef uint32_t (*t_vexDeviceImuStatusGet)(V5_DeviceT device);
inline t_vexDeviceImuStatusGet fnVexDeviceImuStatusGet = (t_vexDeviceImuStatusGet) (*(uint32_t*) 0x37fc42c);

typedef void (*t_vexDeviceImuModeSet)(V5_DeviceT device, uint32_t mode);
inline t_vexDeviceImuModeSet fnVexDeviceImuModeSet = (t_vexDeviceImuModeSet) (*(uint32_t*) 0x37fc438);

typedef uint32_t (*t_vexDeviceImuModeGet)(V5_DeviceT device);
inline t_vexDeviceImuModeGet fnVexDeviceImuModeGet = (t_vexDeviceImuModeGet) (*(uint32_t*) 0x37fc43c);

typedef void (*t_vexDeviceImuDataRateSet)(V5_DeviceT device, uint32_t rate);
inline t_vexDeviceImuDataRateSet fnVexDeviceImuDataRateSet = (t_vexDeviceImuDataRateSet) (*(uint32_t*) 0x37fc444);

typedef int32_t (*t_vexDeviceRangeValueGet)(V5_DeviceT device);
inline t_vexDeviceRangeValueGet fnVexDeviceRangeValueGet = (t_vexDeviceRangeValueGet) (*(uint32_t*) 0x37fc4d8);

typedef void (*t_vexDeviceAbsEncReset)(V5_DeviceT device);
inline t_vexDeviceAbsEncReset fnVexDeviceAbsEncReset = (t_vexDeviceAbsEncReset) (*(uint32_t*) 0x37fc488);

typedef void (*t_vexDeviceAbsEncPositionSet)(V5_DeviceT device, int32_t position);
inline t_vexDeviceAbsEncPositionSet fnVexDeviceAbsEncPositionSet = (t_vexDeviceAbsEncPositionSet) (*(uint32_t*) 0x37fc48c);

typedef int32_t (*t_vexDeviceAbsEncPositionGet)(V5_DeviceT device);
inline t_vexDeviceAbsEncPositionGet fnVexDeviceAbsEncPositionGet = (t_vexDeviceAbsEncPositionGet) (*(uint32_t*) 0x37fc490);

typedef int32_t (*t_vexDeviceAbsEncVelocityGet)(V5_DeviceT device);
inline t_vexDeviceAbsEncVelocityGet fnVexDeviceAbsEncVelocityGet = (t_vexDeviceAbsEncVelocityGet) (*(uint32_t*) 0x37fc494);

typedef int32_t (*t_vexDeviceAbsEncAngleGet)(V5_DeviceT device);
inline t_vexDeviceAbsEncAngleGet fnVexDeviceAbsEncAngleGet = (t_vexDeviceAbsEncAngleGet) (*(uint32_t*) 0x37fc498);

typedef void (*t_vexDeviceAbsEncReverseFlagSet)(V5_DeviceT device, bool value);
inline t_vexDeviceAbsEncReverseFlagSet fnVexDeviceAbsEncReverseFlagSet =
        (t_vexDeviceAbsEncReverseFlagSet) (*(uint32_t*) 0x37fc49c);

typedef bool (*t_vexDeviceAbsEncReverseFlagGet)(V5_DeviceT device);
inline t_vexDeviceAbsEncReverseFlagGet fnVexDeviceAbsEncReverseFlagGet =
        (t_vexDeviceAbsEncReverseFlagGet) (*(uint32_t*) 0x37fc4a0);

typedef uint32_t (*t_vexDeviceAbsEncStatusGet)(V5_DeviceT device);
inline t_vexDeviceAbsEncStatusGet fnVexDeviceAbsEncStatusGet = (t_vexDeviceAbsEncStatusGet) (*(uint32_t*) 0x37fc4a4);

typedef void (*t_vexDeviceAbsEncDataRateSet)(V5_DeviceT device, uint32_t rate);
inline t_vexDeviceAbsEncDataRateSet fnVexDeviceAbsEncDataRateSet = (t_vexDeviceAbsEncDataRateSet) (*(uint32_t*) 0x37fc4c0);

typedef double (*t_vexDeviceOpticalHueGet)(V5_DeviceT device);
inline t_vexDeviceOpticalHueGet fnVexDeviceOpticalHueGet = (t_vexDeviceOpticalHueGet) (*(uint32_t*) 0x37fc528);

typedef double (*t_vexDeviceOpticalSatGet)(V5_DeviceT device);
inline t_vexDeviceOpticalSatGet fnVexDeviceOpticalSatGet = (t_vexDeviceOpticalSatGet) (*(uint32_t*) 0x37fc52c);

typedef double (*t_vexDeviceOpticalBrightnessGet)(V5_DeviceT device);
inline t_vexDeviceOpticalBrightnessGet fnVexDeviceOpticalBrightnessGet =
        (t_vexDeviceOpticalBrightnessGet) (*(uint32_t*) 0x37fc530);

typedef int32_t (*t_vexDeviceOpticalProximityGet)(V5_DeviceT device);
inline t_vexDeviceOpticalProximityGet fnVexDeviceOpticalProximityGet =
        (t_vexDeviceOpticalProximityGet) (*(uint32_t*) 0x37fc534);

typedef void (*t_vexDeviceOpticalRgbGet)(V5_DeviceT device, V5_DeviceOpticalRgb* data);
inline t_vexDeviceOpticalRgbGet fnVexDeviceOpticalRgbGet = (t_vexDeviceOpticalRgbGet) (*(uint32_t*) 0x37fc538);

typedef void (*t_vexDeviceOpticalLedPwmSet)(V5_DeviceT device, int32_t value);
inline t_vexDeviceOpticalLedPwmSet fnVexDeviceOpticalLedPwmSet = (t_vexDeviceOpticalLedPwmSet) (*(uint32_t*) 0x37fc53c);

typedef int32_t (*t_vexDeviceOpticalLedPwmGet)(V5_DeviceT device);
inline t_vexDeviceOpticalLedPwmGet fnVexDeviceOpticalLedPwmGet = (t_vexDeviceOpticalLedPwmGet) (*(uint32_t*) 0x37fc540);

typedef uint32_t (*t_vexDeviceOpticalStatusGet)(V5_DeviceT device);
inline t_vexDeviceOpticalStatusGet fnVexDeviceOpticalStatusGet = (t_vexDeviceOpticalStatusGet) (*(uint32_t*) 0x37fc544);

typedef void (*t_vexDeviceOpticalRawGet)(V5_DeviceT device, V5_DeviceOpticalRaw* data);
inline t_vexDeviceOpticalRawGet fnVexDeviceOpticalRawGet = (t_vexDeviceOpticalRawGet) (*(uint32_t*) 0x37fc548);

typedef void (*t_vexDeviceOpticalModeSet)(V5_DeviceT device, uint32_t mode);
inline t_vexDeviceOpticalModeSet fnVexDeviceOpticalModeSet = (t_vexDeviceOpticalModeSet) (*(uint32_t*) 0x37fc550);

typedef uint32_t (*t_vexDeviceOpticalModeGet)(V5_DeviceT device);
inline t_vexDeviceOpticalModeGet fnVexDeviceOpticalModeGet = (t_vexDeviceOpticalModeGet) (*(uint32_t*) 0x37fc554);

typedef uint32_t (*t_vexDeviceOpticalGestureGet)(V5_DeviceT, V5_DeviceOpticalGesture* pData);
inline t_vexDeviceOpticalGestureGet fnVexDeviceOpticalGestureGet = (t_vexDeviceOpticalGestureGet) (*(uint32_t*) 0x37fc558);

typedef void (*t_vexDeviceOpticalGestureEnable)(V5_DeviceT);
inline t_vexDeviceOpticalGestureEnable fnVexDeviceOpticalGestureEnable =
        (t_vexDeviceOpticalGestureEnable) (*(uint32_t*) 0x37fc55c);

typedef void (*t_vexDeviceOpticalGestureDisable)(V5_DeviceT);
inline t_vexDeviceOpticalGestureDisable fnVexDeviceOpticalGestureDisable =
        (t_vexDeviceOpticalGestureDisable) (*(uint32_t*) 0x37fc560);

typedef int32_t (*t_vexDeviceOpticalProximityThreshold)(V5_DeviceT device, int32_t value);
inline t_vexDeviceOpticalProximityThreshold fnVexDeviceOpticalProximityThreshold =
        (t_vexDeviceOpticalProximityThreshold) (*(uint32_t*) 0x37fc564);

typedef void (*t_vexDeviceOpticalIntegrationTimeSet)(V5_DeviceT device, double timeMs);
inline t_vexDeviceOpticalIntegrationTimeSet fnVexDeviceOpticalIntegrationTimeSet =
        (t_vexDeviceOpticalIntegrationTimeSet) (*(uint32_t*) 0x37fcb40);

typedef double (*t_vexDeviceOpticalIntegrationTimeGet)(V5_DeviceT device);
inline t_vexDeviceOpticalIntegrationTimeGet fnVexDeviceOpticalIntegrationTimeGet =
        (t_vexDeviceOpticalIntegrationTimeGet) (*(uint32_t*) 0x37fcb44);

typedef void (*t_vexDeviceMagnetPowerSet)(V5_DeviceT device, int32_t value, int32_t time);
inline t_vexDeviceMagnetPowerSet fnVexDeviceMagnetPowerSet = (t_vexDeviceMagnetPowerSet) (*(uint32_t*) 0x37fc578);

typedef int32_t (*t_vexDeviceMagnetPowerGet)(V5_DeviceT device);
inline t_vexDeviceMagnetPowerGet fnVexDeviceMagnetPowerGet = (t_vexDeviceMagnetPowerGet) (*(uint32_t*) 0x37fc57c);

typedef void (*t_vexDeviceMagnetPickup)(V5_DeviceT device, V5_DeviceMagnetDuration duration);
inline t_vexDeviceMagnetPickup fnVexDeviceMagnetPickup = (t_vexDeviceMagnetPickup) (*(uint32_t*) 0x37fc580);

typedef void (*t_vexDeviceMagnetDrop)(V5_DeviceT device, V5_DeviceMagnetDuration duration);
inline t_vexDeviceMagnetDrop fnVexDeviceMagnetDrop = (t_vexDeviceMagnetDrop) (*(uint32_t*) 0x37fc584);

typedef double (*t_vexDeviceMagnetTemperatureGet)(V5_DeviceT device);
inline t_vexDeviceMagnetTemperatureGet fnVexDeviceMagnetTemperatureGet =
        (t_vexDeviceMagnetTemperatureGet) (*(uint32_t*) 0x37fc588);

typedef double (*t_vexDeviceMagnetCurrentGet)(V5_DeviceT device);
inline t_vexDeviceMagnetCurrentGet fnVexDeviceMagnetCurrentGet = (t_vexDeviceMagnetCurrentGet) (*(uint32_t*) 0x37fc58c);

typedef uint32_t (*t_vexDeviceMagnetStatusGet)(V5_DeviceT device);
inline t_vexDeviceMagnetStatusGet fnVexDeviceMagnetStatusGet = (t_vexDeviceMagnetStatusGet) (*(uint32_t*) 0x37fc590);

typedef void (*t_vexDeviceLightTowerRgbSet)(V5_DeviceT device, uint32_t rgb_value, uint32_t xyw_value);
inline t_vexDeviceLightTowerRgbSet fnVexDeviceLightTowerRgbSet = (t_vexDeviceLightTowerRgbSet) (*(uint32_t*) 0x37fc5a0);

typedef void (*t_vexDeviceLightTowerColorSet)(V5_DeviceT device, uint32_t color_id, uint32_t value);
inline t_vexDeviceLightTowerColorSet fnVexDeviceLightTowerColorSet = (t_vexDeviceLightTowerColorSet) (*(uint32_t*) 0x37fc5a4);

typedef uint32_t (*t_vexDeviceLightTowerRgbGet)(V5_DeviceT device);
inline t_vexDeviceLightTowerRgbGet fnVexDeviceLightTowerRgbGet = (t_vexDeviceLightTowerRgbGet) (*(uint32_t*) 0x37fc5a8);

typedef uint32_t (*t_vexDeviceLightTowerXywGet)(V5_DeviceT device);
inline t_vexDeviceLightTowerXywGet fnVexDeviceLightTowerXywGet = (t_vexDeviceLightTowerXywGet) (*(uint32_t*) 0x37fc5ac);

typedef uint32_t (*t_vexDeviceLightTowerStatusGet)(V5_DeviceT device);
inline t_vexDeviceLightTowerStatusGet fnVexDeviceLightTowerStatusGet =
        (t_vexDeviceLightTowerStatusGet) (*(uint32_t*) 0x37fc5b0);

typedef void (*t_vexDeviceLightTowerBlinkSet)(V5_DeviceT device, uint8_t select, uint8_t mask, int32_t onTime, int32_t offTime);
inline t_vexDeviceLightTowerBlinkSet fnVexDeviceLightTowerBlinkSet = (t_vexDeviceLightTowerBlinkSet) (*(uint32_t*) 0x37fc5b8);

typedef uint32_t (*t_vexDeviceDistanceDistanceGet)(V5_DeviceT device);
inline t_vexDeviceDistanceDistanceGet fnVexDeviceDistanceDistanceGet =
        (t_vexDeviceDistanceDistanceGet) (*(uint32_t*) 0x37fc500);

typedef uint32_t (*t_vexDeviceDistanceConfidenceGet)(V5_DeviceT device);
inline t_vexDeviceDistanceConfidenceGet fnVexDeviceDistanceConfidenceGet =
        (t_vexDeviceDistanceConfidenceGet) (*(uint32_t*) 0x37fc504);

typedef int32_t (*t_vexDeviceDistanceObjectSizeGet)(V5_DeviceT device);
inline t_vexDeviceDistanceObjectSizeGet fnVexDeviceDistanceObjectSizeGet =
        (t_vexDeviceDistanceObjectSizeGet) (*(uint32_t*) 0x37fc518);

typedef double (*t_vexDeviceDistanceObjectVelocityGet)(V5_DeviceT device);
inline t_vexDeviceDistanceObjectVelocityGet fnVexDeviceDistanceObjectVelocityGet =
        (t_vexDeviceDistanceObjectVelocityGet) (*(uint32_t*) 0x37fc51c);

typedef uint32_t (*t_vexDeviceDistanceStatusGet)(V5_DeviceT device);
inline t_vexDeviceDistanceStatusGet fnVexDeviceDistanceStatusGet = (t_vexDeviceDistanceStatusGet) (*(uint32_t*) 0x37fc508);

typedef void (*t_vexDeviceGpsReset)(V5_DeviceT device);
inline t_vexDeviceGpsReset fnVexDeviceGpsReset = (t_vexDeviceGpsReset) (*(uint32_t*) 0x37fc5c8);

typedef double (*t_vexDeviceGpsHeadingGet)(V5_DeviceT device);
inline t_vexDeviceGpsHeadingGet fnVexDeviceGpsHeadingGet = (t_vexDeviceGpsHeadingGet) (*(uint32_t*) 0x37fc5cc);

typedef double (*t_vexDeviceGpsDegreesGet)(V5_DeviceT device);
inline t_vexDeviceGpsDegreesGet fnVexDeviceGpsDegreesGet = (t_vexDeviceGpsDegreesGet) (*(uint32_t*) 0x37fc5d0);

typedef void (*t_vexDeviceGpsQuaternionGet)(V5_DeviceT device, V5_DeviceGpsQuaternion* data);
inline t_vexDeviceGpsQuaternionGet fnVexDeviceGpsQuaternionGet = (t_vexDeviceGpsQuaternionGet) (*(uint32_t*) 0x37fc5d4);

typedef void (*t_vexDeviceGpsAttitudeGet)(V5_DeviceT device, V5_DeviceGpsAttitude* data, bool bRaw);
inline t_vexDeviceGpsAttitudeGet fnVexDeviceGpsAttitudeGet = (t_vexDeviceGpsAttitudeGet) (*(uint32_t*) 0x37fc5d8);

typedef void (*t_vexDeviceGpsRawGyroGet)(V5_DeviceT device, V5_DeviceGpsRaw* data);
inline t_vexDeviceGpsRawGyroGet fnVexDeviceGpsRawGyroGet = (t_vexDeviceGpsRawGyroGet) (*(uint32_t*) 0x37fc5dc);

typedef void (*t_vexDeviceGpsRawAccelGet)(V5_DeviceT device, V5_DeviceGpsRaw* data);
inline t_vexDeviceGpsRawAccelGet fnVexDeviceGpsRawAccelGet = (t_vexDeviceGpsRawAccelGet) (*(uint32_t*) 0x37fc5e0);

typedef uint32_t (*t_vexDeviceGpsStatusGet)(V5_DeviceT device);
inline t_vexDeviceGpsStatusGet fnVexDeviceGpsStatusGet = (t_vexDeviceGpsStatusGet) (*(uint32_t*) 0x37fc5e4);

typedef void (*t_vexDeviceGpsModeSet)(V5_DeviceT device, uint32_t mode);
inline t_vexDeviceGpsModeSet fnVexDeviceGpsModeSet = (t_vexDeviceGpsModeSet) (*(uint32_t*) 0x37fc5f0);

typedef uint32_t (*t_vexDeviceGpsModeGet)(V5_DeviceT device);
inline t_vexDeviceGpsModeGet fnVexDeviceGpsModeGet = (t_vexDeviceGpsModeGet) (*(uint32_t*) 0x37fc5f4);

typedef void (*t_vexDeviceGpsDataRateSet)(V5_DeviceT device, uint32_t rate);
inline t_vexDeviceGpsDataRateSet fnVexDeviceGpsDataRateSet = (t_vexDeviceGpsDataRateSet) (*(uint32_t*) 0x37fc5f8);

typedef void (*t_vexDeviceGpsOriginSet)(V5_DeviceT device, double ox, double oy);
inline t_vexDeviceGpsOriginSet fnVexDeviceGpsOriginSet = (t_vexDeviceGpsOriginSet) (*(uint32_t*) 0x37fc5fc);

typedef void (*t_vexDeviceGpsOriginGet)(V5_DeviceT device, double* ox, double* oy);
inline t_vexDeviceGpsOriginGet fnVexDeviceGpsOriginGet = (t_vexDeviceGpsOriginGet) (*(uint32_t*) 0x37fc600);

typedef void (*t_vexDeviceGpsRotationSet)(V5_DeviceT device, double value);
inline t_vexDeviceGpsRotationSet fnVexDeviceGpsRotationSet = (t_vexDeviceGpsRotationSet) (*(uint32_t*) 0x37fc604);

typedef double (*t_vexDeviceGpsRotationGet)(V5_DeviceT device);
inline t_vexDeviceGpsRotationGet fnVexDeviceGpsRotationGet = (t_vexDeviceGpsRotationGet) (*(uint32_t*) 0x37fc608);

typedef void (*t_vexDeviceGpsInitialPositionSet)(V5_DeviceT device, double initial_x, double initial_y,
                                                 double initial_rotation);
inline t_vexDeviceGpsInitialPositionSet fnVexDeviceGpsInitialPositionSet =
        (t_vexDeviceGpsInitialPositionSet) (*(uint32_t*) 0x37fc60c);

typedef double (*t_vexDeviceGpsErrorGet)(V5_DeviceT device);
inline t_vexDeviceGpsErrorGet fnVexDeviceGpsErrorGet = (t_vexDeviceGpsErrorGet) (*(uint32_t*) 0x37fc614);

typedef void (*t_vexDeviceAiVisionModeSet)(V5_DeviceT device, uint32_t mode);
inline t_vexDeviceAiVisionModeSet fnVexDeviceAiVisionModeSet = (t_vexDeviceAiVisionModeSet) (*(uint32_t*) 0x37fcca8);

typedef uint32_t (*t_vexDeviceAiVisionModeGet)(V5_DeviceT device);
inline t_vexDeviceAiVisionModeGet fnVexDeviceAiVisionModeGet = (t_vexDeviceAiVisionModeGet) (*(uint32_t*) 0x37fccac);

typedef int32_t (*t_vexDeviceAiVisionObjectCountGet)(V5_DeviceT device);
inline t_vexDeviceAiVisionObjectCountGet fnVexDeviceAiVisionObjectCountGet =
        (t_vexDeviceAiVisionObjectCountGet) (*(uint32_t*) 0x37fccb0);

typedef int32_t (*t_vexDeviceAiVisionObjectGet)(V5_DeviceT device, uint32_t indexObj, V5_DeviceAiVisionObject* pObject);
inline t_vexDeviceAiVisionObjectGet fnVexDeviceAiVisionObjectGet = (t_vexDeviceAiVisionObjectGet) (*(uint32_t*) 0x37fccb4);

typedef void (*t_vexDeviceAiVisionColorSet)(V5_DeviceT device, V5_DeviceAiVisionColor* pColor);
inline t_vexDeviceAiVisionColorSet fnVexDeviceAiVisionColorSet = (t_vexDeviceAiVisionColorSet) (*(uint32_t*) 0x37fccb8);

typedef bool (*t_vexDeviceAiVisionColorGet)(V5_DeviceT device, uint32_t id, V5_DeviceAiVisionColor* pColor);
inline t_vexDeviceAiVisionColorGet fnVexDeviceAiVisionColorGet = (t_vexDeviceAiVisionColorGet) (*(uint32_t*) 0x37fccbc);

typedef void (*t_vexDeviceAiVisionCodeSet)(V5_DeviceT device, V5_DeviceAiVisionCode* pCode);
inline t_vexDeviceAiVisionCodeSet fnVexDeviceAiVisionCodeSet = (t_vexDeviceAiVisionCodeSet) (*(uint32_t*) 0x37fccc0);

typedef bool (*t_vexDeviceAiVisionCodeGet)(V5_DeviceT device, uint32_t id, V5_DeviceAiVisionCode* pCode);
inline t_vexDeviceAiVisionCodeGet fnVexDeviceAiVisionCodeGet = (t_vexDeviceAiVisionCodeGet) (*(uint32_t*) 0x37fccc4);

typedef uint32_t (*t_vexDeviceAiVisionStatusGet)(V5_DeviceT device);
inline t_vexDeviceAiVisionStatusGet fnVexDeviceAiVisionStatusGet = (t_vexDeviceAiVisionStatusGet) (*(uint32_t*) 0x37fccc8);

typedef double (*t_vexDeviceAiVisionTemperatureGet)(V5_DeviceT device);
inline t_vexDeviceAiVisionTemperatureGet fnVexDeviceAiVisionTemperatureGet =
        (t_vexDeviceAiVisionTemperatureGet) (*(uint32_t*) 0x37fcccc);

typedef int32_t (*t_vexDeviceAiVisionClassNameGet)(V5_DeviceT device, int32_t id, uint8_t* pName);
inline t_vexDeviceAiVisionClassNameGet fnVexDeviceAiVisionClassNameGet =
        (t_vexDeviceAiVisionClassNameGet) (*(uint32_t*) 0x37fccd4);

typedef void (*t_vexDevicePneumaticCompressorSet)(V5_DeviceT device, bool bState);
inline t_vexDevicePneumaticCompressorSet fnVexDevicePneumaticCompressorSet =
        (t_vexDevicePneumaticCompressorSet) (*(uint32_t*) 0x37fcc08);

typedef void (*t_vexDevicePneumaticCylinderSet)(V5_DeviceT device, uint32_t id, bool bState);
inline t_vexDevicePneumaticCylinderSet fnVexDevicePneumaticCylinderSet =
        (t_vexDevicePneumaticCylinderSet) (*(uint32_t*) 0x37fcc0c);

typedef void (*t_vexDevicePneumaticCtrlSet)(V5_DeviceT device, V5_DevicePneumaticCtrl* pCtrl);
inline t_vexDevicePneumaticCtrlSet fnVexDevicePneumaticCtrlSet = (t_vexDevicePneumaticCtrlSet) (*(uint32_t*) 0x37fcc10);

typedef uint32_t (*t_vexDevicePneumaticStatusGet)(V5_DeviceT device);
inline t_vexDevicePneumaticStatusGet fnVexDevicePneumaticStatusGet = (t_vexDevicePneumaticStatusGet) (*(uint32_t*) 0x37fcc14);

typedef void (*t_vexDevicePneumaticPwmSet)(V5_DeviceT device, uint8_t pwm);
inline t_vexDevicePneumaticPwmSet fnVexDevicePneumaticPwmSet = (t_vexDevicePneumaticPwmSet) (*(uint32_t*) 0x37fcc18);

typedef uint32_t (*t_vexDevicePneumaticPwmGet)(V5_DeviceT device);
inline t_vexDevicePneumaticPwmGet fnVexDevicePneumaticPwmGet = (t_vexDevicePneumaticPwmGet) (*(uint32_t*) 0x37fcc1c);

typedef void (*t_vexDevicePneumaticCylinderPwmSet)(V5_DeviceT device, uint32_t id, bool bState, uint8_t pwm);
inline t_vexDevicePneumaticCylinderPwmSet fnVexDevicePneumaticCylinderPwmSet =
        (t_vexDevicePneumaticCylinderPwmSet) (*(uint32_t*) 0x37fcc20);

typedef uint32_t (*t_vexDevicePneumaticActuationStatusGet)(V5_DeviceT device, uint16_t* ac1, uint16_t* ac2, uint16_t* ac3,
                                                           uint16_t* ac4);
inline t_vexDevicePneumaticActuationStatusGet fnVexDevicePneumaticActuationStatusGet =
        (t_vexDevicePneumaticActuationStatusGet) (*(uint32_t*) 0x37fcc28);

typedef void (*t_vexDisplayForegroundColor)(uint32_t col);
inline t_vexDisplayForegroundColor fnVexDisplayForegroundColor = (t_vexDisplayForegroundColor) (*(uint32_t*) 0x37fc640);

typedef void (*t_vexDisplayBackgroundColor)(uint32_t col);
inline t_vexDisplayBackgroundColor fnVexDisplayBackgroundColor = (t_vexDisplayBackgroundColor) (*(uint32_t*) 0x37fc644);

typedef uint32_t (*t_vexDisplayForegroundColorGet)(void);
inline t_vexDisplayForegroundColorGet fnVexDisplayForegroundColorGet =
        (t_vexDisplayForegroundColorGet) (*(uint32_t*) 0x37fc6b8);

typedef uint32_t (*t_vexDisplayBackgroundColorGet)(void);
inline t_vexDisplayBackgroundColorGet fnVexDisplayBackgroundColorGet =
        (t_vexDisplayBackgroundColorGet) (*(uint32_t*) 0x37fc6bc);

typedef void (*t_vexDisplayErase)(void);
inline t_vexDisplayErase fnVexDisplayErase = (t_vexDisplayErase) (*(uint32_t*) 0x37fc648);

typedef void (*t_vexDisplayScroll)(int32_t nStartLine, int32_t nLines);
inline t_vexDisplayScroll fnVexDisplayScroll = (t_vexDisplayScroll) (*(uint32_t*) 0x37fc64c);

typedef void (*t_vexDisplayScrollRect)(int32_t x1, int32_t y1, int32_t x2, int32_t y2, int32_t nLines);
inline t_vexDisplayScrollRect fnVexDisplayScrollRect = (t_vexDisplayScrollRect) (*(uint32_t*) 0x37fc650);

typedef void (*t_vexDisplayCopyRect)(int32_t x1, int32_t y1, int32_t x2, int32_t y2, uint32_t* pSrc, int32_t srcStride);
inline t_vexDisplayCopyRect fnVexDisplayCopyRect = (t_vexDisplayCopyRect) (*(uint32_t*) 0x37fc654);

typedef void (*t_vexDisplayPixelSet)(uint32_t x, uint32_t y);
inline t_vexDisplayPixelSet fnVexDisplayPixelSet = (t_vexDisplayPixelSet) (*(uint32_t*) 0x37fc658);

typedef void (*t_vexDisplayPixelClear)(uint32_t x, uint32_t y);
inline t_vexDisplayPixelClear fnVexDisplayPixelClear = (t_vexDisplayPixelClear) (*(uint32_t*) 0x37fc65c);

typedef void (*t_vexDisplayLineDraw)(int32_t x1, int32_t y1, int32_t x2, int32_t y2);
inline t_vexDisplayLineDraw fnVexDisplayLineDraw = (t_vexDisplayLineDraw) (*(uint32_t*) 0x37fc660);

typedef void (*t_vexDisplayLineClear)(int32_t x1, int32_t y1, int32_t x2, int32_t y2);
inline t_vexDisplayLineClear fnVexDisplayLineClear = (t_vexDisplayLineClear) (*(uint32_t*) 0x37fc664);

typedef void (*t_vexDisplayRectDraw)(int32_t x1, int32_t y1, int32_t x2, int32_t y2);
inline t_vexDisplayRectDraw fnVexDisplayRectDraw = (t_vexDisplayRectDraw) (*(uint32_t*) 0x37fc668);

typedef void (*t_vexDisplayRectClear)(int32_t x1, int32_t y1, int32_t x2, int32_t y2);
inline t_vexDisplayRectClear fnVexDisplayRectClear = (t_vexDisplayRectClear) (*(uint32_t*) 0x37fc66c);

typedef void (*t_vexDisplayRectFill)(int32_t x1, int32_t y1, int32_t x2, int32_t y2);
inline t_vexDisplayRectFill fnVexDisplayRectFill = (t_vexDisplayRectFill) (*(uint32_t*) 0x37fc670);

typedef void (*t_vexDisplayCircleDraw)(int32_t xc, int32_t yc, int32_t radius);
inline t_vexDisplayCircleDraw fnVexDisplayCircleDraw = (t_vexDisplayCircleDraw) (*(uint32_t*) 0x37fc674);

typedef void (*t_vexDisplayCircleClear)(int32_t xc, int32_t yc, int32_t radius);
inline t_vexDisplayCircleClear fnVexDisplayCircleClear = (t_vexDisplayCircleClear) (*(uint32_t*) 0x37fc678);

typedef void (*t_vexDisplayCircleFill)(int32_t xc, int32_t yc, int32_t radius);
inline t_vexDisplayCircleFill fnVexDisplayCircleFill = (t_vexDisplayCircleFill) (*(uint32_t*) 0x37fc67c);

typedef void (*t_vexDisplayPrintf)(int32_t xpos, int32_t ypos, uint32_t bOpaque, const char* format, ...);
inline t_vexDisplayPrintf fnVexDisplayPrintf = (t_vexDisplayPrintf) (*(uint32_t*) 0x37fc680);

typedef void (*t_vexDisplayVPrintf)(int32_t xpos, int32_t ypos, uint32_t bOpaque, const char* format, va_list args);
inline t_vexDisplayVPrintf fnVexDisplayVPrintf = (t_vexDisplayVPrintf) (*(uint32_t*) 0x37fc680);

typedef void (*t_vexDisplayString)(const int32_t nLineNumber, const char* format, ...);
inline t_vexDisplayString fnVexDisplayString = (t_vexDisplayString) (*(uint32_t*) 0x37fc684);

typedef void (*t_vexDisplayVString)(const int32_t nLineNumber, const char* format, va_list args);
inline t_vexDisplayVString fnVexDisplayVString = (t_vexDisplayVString) (*(uint32_t*) 0x37fc684);

typedef void (*t_vexDisplayStringAt)(int32_t xpos, int32_t ypos, const char* format, ...);
inline t_vexDisplayStringAt fnVexDisplayStringAt = (t_vexDisplayStringAt) (*(uint32_t*) 0x37fc688);

typedef void (*t_vexDisplayVStringAt)(int32_t xpos, int32_t ypos, const char* format, va_list args);
inline t_vexDisplayVStringAt fnVexDisplayVStringAt = (t_vexDisplayVStringAt) (*(uint32_t*) 0x37fc688);

typedef void (*t_vexDisplayBigString)(const int32_t nLineNumber, const char* format, ...);
inline t_vexDisplayBigString fnVexDisplayBigString = (t_vexDisplayBigString) (*(uint32_t*) 0x37fc68c);

typedef void (*t_vexDisplayVBigString)(const int32_t nLineNumber, const char* format, va_list args);
inline t_vexDisplayVBigString fnVexDisplayVBigString = (t_vexDisplayVBigString) (*(uint32_t*) 0x37fc68c);

typedef void (*t_vexDisplayBigStringAt)(int32_t xpos, int32_t ypos, const char* format, ...);
inline t_vexDisplayBigStringAt fnVexDisplayBigStringAt = (t_vexDisplayBigStringAt) (*(uint32_t*) 0x37fc690);

typedef void (*t_vexDisplayVBigStringAt)(int32_t xpos, int32_t ypos, const char* format, va_list args);
inline t_vexDisplayVBigStringAt fnVexDisplayVBigStringAt = (t_vexDisplayVBigStringAt) (*(uint32_t*) 0x37fc690);

typedef void (*t_vexDisplaySmallStringAt)(int32_t xpos, int32_t ypos, const char* format, ...);
inline t_vexDisplaySmallStringAt fnVexDisplaySmallStringAt = (t_vexDisplaySmallStringAt) (*(uint32_t*) 0x37fc6b0);

typedef void (*t_vexDisplayVSmallStringAt)(int32_t xpos, int32_t ypos, const char* format, va_list args);
inline t_vexDisplayVSmallStringAt fnVexDisplayVSmallStringAt = (t_vexDisplayVSmallStringAt) (*(uint32_t*) 0x37fc6b0);

typedef void (*t_vexDisplayCenteredString)(const int32_t nLineNumber, const char* format, ...);
inline t_vexDisplayCenteredString fnVexDisplayCenteredString = (t_vexDisplayCenteredString) (*(uint32_t*) 0x37fc694);

typedef void (*t_vexDisplayVCenteredString)(const int32_t nLineNumber, const char* format, va_list args);
inline t_vexDisplayVCenteredString fnVexDisplayVCenteredString = (t_vexDisplayVCenteredString) (*(uint32_t*) 0x37fc694);

typedef void (*t_vexDisplayBigCenteredString)(const int32_t nLineNumber, const char* format, ...);
inline t_vexDisplayBigCenteredString fnVexDisplayBigCenteredString = (t_vexDisplayBigCenteredString) (*(uint32_t*) 0x37fc698);

typedef void (*t_vexDisplayVBigCenteredString)(const int32_t nLineNumber, const char* format, va_list args);
inline t_vexDisplayVBigCenteredString fnVexDisplayVBigCenteredString =
        (t_vexDisplayVBigCenteredString) (*(uint32_t*) 0x37fc698);

typedef void (*t_vexDisplayTextSize)(uint32_t n, uint32_t d);
inline t_vexDisplayTextSize fnVexDisplayTextSize = (t_vexDisplayTextSize) (*(uint32_t*) 0x37fc6a8);

typedef void (*t_vexDisplayFontNamedSet)(const char* pFontName);
inline t_vexDisplayFontNamedSet fnVexDisplayFontNamedSet = (t_vexDisplayFontNamedSet) (*(uint32_t*) 0x37fc6b4);

typedef int32_t (*t_vexDisplayStringWidthGet)(const char* pString);
inline t_vexDisplayStringWidthGet fnVexDisplayStringWidthGet = (t_vexDisplayStringWidthGet) (*(uint32_t*) 0x37fc6c0);

typedef int32_t (*t_vexDisplayStringHeightGet)(const char* pString);
inline t_vexDisplayStringHeightGet fnVexDisplayStringHeightGet = (t_vexDisplayStringHeightGet) (*(uint32_t*) 0x37fc6c4);

typedef FRESULT (*t_vexFileMountSD)(void);
inline t_vexFileMountSD fnVexFileMountSD = (t_vexFileMountSD) (*(uint32_t*) 0x37fc7d0);

typedef FRESULT (*t_vexFileDirectoryGet)(const char* path, char* buffer, uint32_t len);
inline t_vexFileDirectoryGet fnVexFileDirectoryGet = (t_vexFileDirectoryGet) (*(uint32_t*) 0x37fc7d4);

typedef FIL* (*t_vexFileOpen)(const char* filename, const char* mode);
inline t_vexFileOpen fnVexFileOpen = (t_vexFileOpen) (*(uint32_t*) 0x37fc7d8);

typedef FIL* (*t_vexFileOpenWrite)(const char* filename);
inline t_vexFileOpenWrite fnVexFileOpenWrite = (t_vexFileOpenWrite) (*(uint32_t*) 0x37fc7dc);

typedef FIL* (*t_vexFileOpenCreate)(const char* filename);
inline t_vexFileOpenCreate fnVexFileOpenCreate = (t_vexFileOpenCreate) (*(uint32_t*) 0x37fc7e0);

typedef void (*t_vexFileClose)(FIL* fdp);
inline t_vexFileClose fnVexFileClose = (t_vexFileClose) (*(uint32_t*) 0x37fc7e4);

typedef int32_t (*t_vexFileRead)(char* buf, uint32_t size, uint32_t nItems, FIL* fdp);
inline t_vexFileRead fnVexFileRead = (t_vexFileRead) (*(uint32_t*) 0x37fc7f8);

typedef int32_t (*t_vexFileWrite)(char* buf, uint32_t size, uint32_t nItems, FIL* fdp);
inline t_vexFileWrite fnVexFileWrite = (t_vexFileWrite) (*(uint32_t*) 0x37fc7ec);

typedef int32_t (*t_vexFileSize)(FIL* fdp);
inline t_vexFileSize fnVexFileSize = (t_vexFileSize) (*(uint32_t*) 0x37fc7f0);

typedef FRESULT (*t_vexFileSeek)(FIL* fdp, uint32_t offset, int32_t whence);
inline t_vexFileSeek fnVexFileSeek = (t_vexFileSeek) (*(uint32_t*) 0x37fc7f4);

typedef bool (*t_vexFileDriveStatus)(uint32_t drive);
inline t_vexFileDriveStatus fnVexFileDriveStatus = (t_vexFileDriveStatus) (*(uint32_t*) 0x37fc7fc);

typedef int32_t (*t_vexFileTell)(FIL* fdp);
inline t_vexFileTell fnVexFileTell = (t_vexFileTell) (*(uint32_t*) 0x37fc800);

typedef void (*t_vexFileSync)(FIL* fdp);
inline t_vexFileSync fnVexFileSync = (t_vexFileSync) (*(uint32_t*) 0x37fc804);

typedef uint32_t (*t_vexFileStatus)(const char* filename);
inline t_vexFileStatus fnVexFileStatus = (t_vexFileStatus) (*(uint32_t*) 0x37fc808);

typedef int32_t (*t_vexSerialWriteChar)(uint32_t channel, uint8_t c);
inline t_vexSerialWriteChar fnVexSerialWriteChar = (t_vexSerialWriteChar) (*(uint32_t*) 0x37fc898);

typedef int32_t (*t_vexSerialWriteBuffer)(uint32_t channel, uint8_t* data, uint32_t data_len);
inline t_vexSerialWriteBuffer fnVexSerialWriteBuffer = (t_vexSerialWriteBuffer) (*(uint32_t*) 0x37fc89c);

typedef int32_t (*t_vexSerialReadChar)(uint32_t channel);
inline t_vexSerialReadChar fnVexSerialReadChar = (t_vexSerialReadChar) (*(uint32_t*) 0x37fc8a0);

typedef int32_t (*t_vexSerialPeekChar)(uint32_t channel);
inline t_vexSerialPeekChar fnVexSerialPeekChar = (t_vexSerialPeekChar) (*(uint32_t*) 0x37fc8a4);

typedef int32_t (*t_vexSerialWriteFree)(uint32_t channel);
inline t_vexSerialWriteFree fnVexSerialWriteFree = (t_vexSerialWriteFree) (*(uint32_t*) 0x37fc8ac);

typedef void (*t_vexSystemTimerStop)(void);
inline t_vexSystemTimerStop fnVexSystemTimerStop = (t_vexSystemTimerStop) (*(uint32_t*) 0x37fc8c0);

typedef void (*t_vexSystemTimerClearInterrupt)(void);
inline t_vexSystemTimerClearInterrupt fnVexSystemTimerClearInterrupt =
        (t_vexSystemTimerClearInterrupt) (*(uint32_t*) 0x37fc8c4);

typedef int32_t (*t_vexSystemTimerReinitForRtos)(uint32_t priority, void (*handler)(void* data));
inline t_vexSystemTimerReinitForRtos fnVexSystemTimerReinitForRtos = (t_vexSystemTimerReinitForRtos) (*(uint32_t*) 0x37fc8c8);

typedef void (*t_vexSystemApplicationIRQHandler)(uint32_t ulICCIAR);
inline t_vexSystemApplicationIRQHandler fnVexSystemApplicationIRQHandler =
        (t_vexSystemApplicationIRQHandler) (*(uint32_t*) 0x37fc8cc);

typedef int32_t (*t_vexSystemWatchdogReinitRtos)(void);
inline t_vexSystemWatchdogReinitRtos fnVexSystemWatchdogReinitRtos = (t_vexSystemWatchdogReinitRtos) (*(uint32_t*) 0x37fc8d0);

typedef uint32_t (*t_vexSystemWatchdogGet)(void);
inline t_vexSystemWatchdogGet fnVexSystemWatchdogGet = (t_vexSystemWatchdogGet) (*(uint32_t*) 0x37fc8d4);

typedef void (*t_vexSystemBoot)(void);
inline t_vexSystemBoot fnVexSystemBoot = (t_vexSystemBoot) (*(uint32_t*) 0x37fc910);

typedef void (*t_vexSystemUndefinedException)(void);
inline t_vexSystemUndefinedException fnVexSystemUndefinedException = (t_vexSystemUndefinedException) (*(uint32_t*) 0x37fc914);

typedef void (*t_vexSystemFIQInterrupt)(void);
inline t_vexSystemFIQInterrupt fnVexSystemFIQInterrupt = (t_vexSystemFIQInterrupt) (*(uint32_t*) 0x37fc918);

typedef void (*t_vexSystemSWInterrupt)(void);
inline t_vexSystemSWInterrupt fnVexSystemSWInterrupt = (t_vexSystemSWInterrupt) (*(uint32_t*) 0x37fc920);

typedef void (*t_vexSystemDataAbortInterrupt)(void);
inline t_vexSystemDataAbortInterrupt fnVexSystemDataAbortInterrupt = (t_vexSystemDataAbortInterrupt) (*(uint32_t*) 0x37fc924);

typedef void (*t_vexSystemPrefetchAbortInterrupt)(void);
inline t_vexSystemPrefetchAbortInterrupt fnVexSystemPrefetchAbortInterrupt =
        (t_vexSystemPrefetchAbortInterrupt) (*(uint32_t*) 0x37fc928);

typedef void (*t_vexTouchUserCallbackSet)(void (*callback)(V5_TouchEvent, int32_t, int32_t));
inline t_vexTouchUserCallbackSet fnVexTouchUserCallbackSet = (t_vexTouchUserCallbackSet) (*(uint32_t*) 0x37fc960);

typedef bool (*t_vexTouchDataGet)(V5_TouchStatus* status);
inline t_vexTouchDataGet fnVexTouchDataGet = (t_vexTouchDataGet) (*(uint32_t*) 0x37fc964);

typedef uint32_t (*t_vexImageBmpRead)(const uint8_t* ibuf, v5_image* oBuf, uint32_t maxw, uint32_t maxh);
inline t_vexImageBmpRead fnVexImageBmpRead = (t_vexImageBmpRead) (*(uint32_t*) 0x37fc990);

typedef uint32_t (*t_vexImagePngRead)(const uint8_t* ibuf, v5_image* oBuf, uint32_t maxw, uint32_t maxh, uint32_t ibuflen);
inline t_vexImagePngRead fnVexImagePngRead = (t_vexImagePngRead) (*(uint32_t*) 0x37fc994);

typedef void (*t_vexDisplayClipRegionSet)(int32_t x1, int32_t y1, int32_t x2, int32_t y2);
inline t_vexDisplayClipRegionSet fnVexDisplayClipRegionSet = (t_vexDisplayClipRegionSet) (*(uint32_t*) 0x37fc794);

typedef bool (*t_vexDisplayRender)(bool bVsyncWait, bool bRunScheduler);
inline t_vexDisplayRender fnVexDisplayRender = (t_vexDisplayRender) (*(uint32_t*) 0x37fc7a0);

typedef void (*t_vexDisplayDoubleBufferDisable)(void);
inline t_vexDisplayDoubleBufferDisable fnVexDisplayDoubleBufferDisable =
        (t_vexDisplayDoubleBufferDisable) (*(uint32_t*) 0x37fc7a4);

typedef uint32_t (*t_vexCompetitionStatus)(void);
inline t_vexCompetitionStatus fnVexCompetitionStatus = (t_vexCompetitionStatus) (*(uint32_t*) 0x37fc9d8);

typedef void (*t_vexCompetitionControl)(uint32_t data);
inline t_vexCompetitionControl fnVexCompetitionControl = (t_vexCompetitionControl) (*(uint32_t*) 0x37fc9dc);

typedef int32_t (*t_vexBatteryVoltageGet)(void);
inline t_vexBatteryVoltageGet fnVexBatteryVoltageGet = (t_vexBatteryVoltageGet) (*(uint32_t*) 0x37fca00);

typedef int32_t (*t_vexBatteryCurrentGet)(void);
inline t_vexBatteryCurrentGet fnVexBatteryCurrentGet = (t_vexBatteryCurrentGet) (*(uint32_t*) 0x37fca04);

typedef double (*t_vexBatteryTemperatureGet)(void);
inline t_vexBatteryTemperatureGet fnVexBatteryTemperatureGet = (t_vexBatteryTemperatureGet) (*(uint32_t*) 0x37fca08);

typedef double (*t_vexBatteryCapacityGet)(void);
inline t_vexBatteryCapacityGet fnVexBatteryCapacityGet = (t_vexBatteryCapacityGet) (*(uint32_t*) 0x37fca0c);

typedef void (*t_vexDeviceGenericSerialEnable)(V5_DeviceT device, int32_t options);
inline t_vexDeviceGenericSerialEnable fnVexDeviceGenericSerialEnable =
        (t_vexDeviceGenericSerialEnable) (*(uint32_t*) 0x37fca50);

typedef void (*t_vexDeviceGenericSerialBaudrate)(V5_DeviceT device, int32_t baudrate);
inline t_vexDeviceGenericSerialBaudrate fnVexDeviceGenericSerialBaudrate =
        (t_vexDeviceGenericSerialBaudrate) (*(uint32_t*) 0x37fca54);

typedef int32_t (*t_vexDeviceGenericSerialWriteChar)(V5_DeviceT device, uint8_t c);
inline t_vexDeviceGenericSerialWriteChar fnVexDeviceGenericSerialWriteChar =
        (t_vexDeviceGenericSerialWriteChar) (*(uint32_t*) 0x37fca58);

typedef int32_t (*t_vexDeviceGenericSerialWriteFree)(V5_DeviceT device);
inline t_vexDeviceGenericSerialWriteFree fnVexDeviceGenericSerialWriteFree =
        (t_vexDeviceGenericSerialWriteFree) (*(uint32_t*) 0x37fca5c);

typedef int32_t (*t_vexDeviceGenericSerialTransmit)(V5_DeviceT device, uint8_t* buffer, int32_t length);
inline t_vexDeviceGenericSerialTransmit fnVexDeviceGenericSerialTransmit =
        (t_vexDeviceGenericSerialTransmit) (*(uint32_t*) 0x37fca60);

typedef int32_t (*t_vexDeviceGenericSerialReadChar)(V5_DeviceT device);
inline t_vexDeviceGenericSerialReadChar fnVexDeviceGenericSerialReadChar =
        (t_vexDeviceGenericSerialReadChar) (*(uint32_t*) 0x37fca64);

typedef int32_t (*t_vexDeviceGenericSerialPeekChar)(V5_DeviceT device);
inline t_vexDeviceGenericSerialPeekChar fnVexDeviceGenericSerialPeekChar =
        (t_vexDeviceGenericSerialPeekChar) (*(uint32_t*) 0x37fca68);

typedef int32_t (*t_vexDeviceGenericSerialReceiveAvail)(V5_DeviceT device);
inline t_vexDeviceGenericSerialReceiveAvail fnVexDeviceGenericSerialReceiveAvail =
        (t_vexDeviceGenericSerialReceiveAvail) (*(uint32_t*) 0x37fca6c);

typedef int32_t (*t_vexDeviceGenericSerialReceive)(V5_DeviceT device, uint8_t* buffer, int32_t length);
inline t_vexDeviceGenericSerialReceive fnVexDeviceGenericSerialReceive =
        (t_vexDeviceGenericSerialReceive) (*(uint32_t*) 0x37fca70);

typedef void (*t_vexDeviceGenericSerialFlush)(V5_DeviceT device);
inline t_vexDeviceGenericSerialFlush fnVexDeviceGenericSerialFlush = (t_vexDeviceGenericSerialFlush) (*(uint32_t*) 0x37fca74);

typedef void (*t_vexDeviceGenericRadioConnection)(V5_DeviceT device, char* pName, bool bMaster, bool bAllowRadioOverride);
inline t_vexDeviceGenericRadioConnection fnVexDeviceGenericRadioConnection =
        (t_vexDeviceGenericRadioConnection) (*(uint32_t*) 0x37fcaa4);

typedef int32_t (*t_vexDeviceGenericRadioWriteChar)(V5_DeviceT device, uint8_t c);
inline t_vexDeviceGenericRadioWriteChar fnVexDeviceGenericRadioWriteChar =
        (t_vexDeviceGenericRadioWriteChar) (*(uint32_t*) 0x37fcaa8);

typedef int32_t (*t_vexDeviceGenericRadioWriteFree)(V5_DeviceT device);
inline t_vexDeviceGenericRadioWriteFree fnVexDeviceGenericRadioWriteFree =
        (t_vexDeviceGenericRadioWriteFree) (*(uint32_t*) 0x37fcaac);

typedef int32_t (*t_vexDeviceGenericRadioTransmit)(V5_DeviceT device, uint8_t* buffer, int32_t length);
inline t_vexDeviceGenericRadioTransmit fnVexDeviceGenericRadioTransmit =
        (t_vexDeviceGenericRadioTransmit) (*(uint32_t*) 0x37fcab0);

typedef int32_t (*t_vexDeviceGenericRadioReadChar)(V5_DeviceT device);
inline t_vexDeviceGenericRadioReadChar fnVexDeviceGenericRadioReadChar =
        (t_vexDeviceGenericRadioReadChar) (*(uint32_t*) 0x37fcab4);

typedef int32_t (*t_vexDeviceGenericRadioPeekChar)(V5_DeviceT device);
inline t_vexDeviceGenericRadioPeekChar fnVexDeviceGenericRadioPeekChar =
        (t_vexDeviceGenericRadioPeekChar) (*(uint32_t*) 0x37fcab8);

typedef int32_t (*t_vexDeviceGenericRadioReceiveAvail)(V5_DeviceT device);
inline t_vexDeviceGenericRadioReceiveAvail fnVexDeviceGenericRadioReceiveAvail =
        (t_vexDeviceGenericRadioReceiveAvail) (*(uint32_t*) 0x37fcabc);

typedef int32_t (*t_vexDeviceGenericRadioReceive)(V5_DeviceT device, uint8_t* buffer, int32_t length);
inline t_vexDeviceGenericRadioReceive fnVexDeviceGenericRadioReceive =
        (t_vexDeviceGenericRadioReceive) (*(uint32_t*) 0x37fcac0);

typedef void (*t_vexDeviceGenericRadioFlush)(V5_DeviceT device);
inline t_vexDeviceGenericRadioFlush fnVexDeviceGenericRadioFlush = (t_vexDeviceGenericRadioFlush) (*(uint32_t*) 0x37fcac4);

typedef bool (*t_vexDeviceGenericRadioLinkStatus)(V5_DeviceT device);
inline t_vexDeviceGenericRadioLinkStatus fnVexDeviceGenericRadioLinkStatus =
        (t_vexDeviceGenericRadioLinkStatus) (*(uint32_t*) 0x37fcac8);

#ifdef __cplusplus
}
#endif