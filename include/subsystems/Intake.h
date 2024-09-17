#pragma once
#include "Constants.h"
#include "Logger.h"
#include "RobotBase.h"
#include "Subsystem.h"
#include "main.h"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/motors.hpp"

class Intake : public Subsystem {
private:
	pros::MotorGroup motors{ports::intake};
	pros::Distance distance{ports::intakeDistance};
	pros::adi::DigitalOut extender{'A'};

	RobotThread runner();

public:
	struct flags {
		bool isExtended{false};
		bool isMoving{false};
		bool torqueStop{false};
		bool distStop{false};

		// ring status
		bool partiallyIn{false};
		bool fullyIn{false};
		bool storeSecond{false};
	};

	explicit Intake(RobotBase* robot);

	void registerTasks() override;

	void moveVoltage(int mv);
	void moveVel(int vel);
	void brake();

	void setExtender(bool extended);
	void toggleExtender();

	void setTorqueStop(bool val);
	void setDistStop(bool val);
};


//

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

namespace vex {
	/** @brief Use the device class to get information about a vex device plugged into the V5.
	 */
	class device {
	private:
		V5_DeviceT _ptr;

	protected:
		int32_t _index;
		int32_t _threadID;
		int32_t flags();

	public:
		device();
		device(int32_t index);
		virtual ~device();

		V5_DeviceType type();
		int32_t index();
		void init(int32_t index);
		virtual bool installed();
		virtual int32_t value();
		uint32_t timestamp();
	};
};// namespace vex

/*-----------------------------------------------------------------------------*/
/** @brief a safe array template                                               */
/*-----------------------------------------------------------------------------*/

namespace vex {
	template<class T, int len>
	class safearray {
	private:
		int length;

	protected:
		T arr[len];
		T outofbounds;

	public:
		safearray() {
			length = len;
			outofbounds = 0;
		};
		~safearray(){};

		T& operator[](int i);
		int getLength() {
			return length;
		};
	};

	template<class T, int len>
	T& safearray<T, len>::operator[](int i) {
		// bounds check the array index
		if (i < 0 || i > (length - 1)) {
			return (outofbounds);
		} else {
			return (arr[i]);
		}
	}
};// namespace vex

namespace vex {
	/**
	 * @brief Use the color class to create Color objects.
	 */
	class color {
	private:
		uint32_t _argb;
		bool _transparent;

		// special constructor only used to create a transparent color
		color(int value, bool transparent);

	public:
		color();
		/**
		 * @brief Creates a color.
		 * @param value The value of the color.
		 */
		color(int value);
		/**
		 * @brief Creates a color using red, green, and blue values.
		 * @param r the color of red on a scale of 0 to 255.
		 * @param g the color of green on a scale of 0 to 255.
		 * @param b the color of blue on a scale of 0 to 255.
		 */
		color(uint8_t r, uint8_t g, uint8_t b);
		~color();

		// Assignment
		uint32_t rgb(uint32_t value);
		uint32_t rgb(uint8_t r, uint8_t g, uint8_t b);
		void operator=(uint32_t value);

		// Get value
		uint32_t rgb() const;
		operator uint32_t() const;

		/**
		 * @brief Gets the state of the color's transparency.
		 * @return Returns true if the color is transparent.
		 */
		bool isTransparent() const;

		/**
		 * @brief Creates a color using hue, saturation, and brightness values.
		 * @return Returns a reference to a color.
		 * @param hue An integer from 0 to 360 that represents the hue of the color.
		 * @param sat A double from 0.0 to 1.0 that represents the saturation of the color.
		 * @param value A double from 0.0 to 1.0 that represents the brightness of the color.
		 */
		color& hsv(uint32_t hue, double sat, double value);

		/**
		 * @brief Creates a color using a hexadecimal value.
		 * @return Returns a reference to a color.
		 * @param color A hexadecimal or web color value that defines a specific color.
		 */
		color& web(const char* color);

		/**
		 * @brief Gets the hue of the color
		 * @return Returns the hue of the color in the range 0 to 360 degrees
		 */
		double hue(void) const;

		/**
		 * @brief Gets the saturation of the color
		 * @return Returns the saturation of the color in the range 0 to 1.0
		 */
		double saturation(void) const;

		/**
		 * @brief Gets the value (brightness) of the color
		 * @return Returns the value (brightness) of the color in the range 0 to 1.0
		 */
		double brightness(void) const;

		// predefined colors

		/**
		 *@brief Represents the color black.
		 */
		static const color black;
		/**
		 *@brief Represents the color white.
		 */
		static const color white;
		/**
		 *@brief Represents the color red.
		 */
		static const color red;
		/**
		 *@brief Represents the color green.
		 */
		static const color green;
		/**
		 *@brief Represents the color blue.
		 */
		static const color blue;
		/**
		 *@brief Represents the color yellow.
		 */
		static const color yellow;
		/**
		 *@brief Represents the color orange.
		 */
		static const color orange;
		/**
		 *@brief Represents the color purple.
		 */
		static const color purple;
		/**
		 *@brief Represents the color cyan.
		 */
		static const color cyan;
		/**
		 *@brief Represents the color transparent.
		 */
		static const color transparent;
	};
}// namespace vex


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


namespace vex {
	/**
	 * @brief Use this class when programming the AI Vision camera.
	 */
	class aivision : public device {
	private:
#define AIVISION_MAX_OBJECTS 24
#define AIVISION_DEFAULT_SNAPSHOT_OBJECTS 8

		enum class objectType {
			unknownObject = 0,
			colorObject = (1 << 0),
			codeObject = (1 << 1),
			modelObject = (1 << 2),
			tagObject = (1 << 3),
			allObject = (0x3F)
		};

		class object {
			friend class aivision;
			friend class safearray<object, AIVISION_MAX_OBJECTS>;

		public:
			class tagcoords {
			public:
				int16_t x[4];
				int16_t y[4];
			};

		private:
#define AIVISION_MAX_CLASS_NAME 20

			int32_t _id;
			objectType _type;
			int16_t _originX;
			int16_t _originY;
			int16_t _centerX;
			int16_t _centerY;
			int16_t _width;
			int16_t _height;
			float _angle;
			bool _exists;

			tagcoords _tag;

			char _className[AIVISION_MAX_CLASS_NAME];

			vex::color _color;

			/**
			 * @brief Copies all properties of the passed in object into this object.
			 * @param obj The object whose properties are to be copied.
			 */
			void set(V5_DeviceAiVisionObject obj, char* cname = NULL);

			/**
			 * @brief Sets all properties for this object to default value exceot id;
			 */
			object& operator=(int32_t id);

		public:
			/**
			 * @brief Creates a new AI Vision camera object with all properties set to default values.
			 */
			object();
			~object();

			/**
			 * @brief Sets all properties for this object to default values.
			 */
			void clear();

			/**
			 * @brief Copies an object.
			 */
			object& operator=(const object& obj);

			/**
			 * @brief The unique ID of the object.
			 */
			const int32_t& id;
			/**
			 * @brief The type of the object.
			 */
			const objectType& type;
			/**
			 * @brief The top left x position of the object.
			 */
			const int16_t& originX;
			/**
			 * @brief The top left y position of the object.
			 */
			const int16_t& originY;
			/**
			 * @brief The center x position of the object.
			 */
			const int16_t& centerX;
			/**
			 * @brief The center y position of the object.
			 */
			const int16_t& centerY;
			/**
			 * @brief The width of the object.
			 */
			const int16_t& width;
			/**
			 * @brief The height of the object.
			 */
			const int16_t& height;
			/**
			 * @brief The angle of the object.
			 */
			const float& angle;
			/**
			 * @brief If the AI Vision camera detects the object or not.
			 */
			const bool& exists;

			/**
			 * @brief The raw coordinates of an apriltag.
			 */
			const tagcoords& tag;

			/**
			 * @brief Read only pointer to object class name, only valid for model objects.
			 */
			const char* const className;

			/**
			 * @brief The color for this object, only valid for color objects.
			 */
			const vex::color& color;
		};

		class objdesc {
		protected:
			uint8_t _id;

		public:
			objdesc();
			objdesc(uint8_t id);

			// read only references to internal variables
			const uint8_t& id = _id;
		};

		/**
		 * @brief Use this class when programming the AI 2D camera.
		 */
		class colordesc : public objdesc {
		private:
			uint8_t _red;
			uint8_t _green;
			uint8_t _blue;
			float _hangle;
			float _hdsat;

			void clear();

		public:
			colordesc();
			~colordesc();

			/**
			 * @brief Creates a new AI Vision camera color description object.
			 * @param id The color description id.
			 */
			colordesc(int32_t id, uint8_t red, uint8_t green, uint8_t blue, float hangle, float hdsat);

			// read only references to internal variables
			const uint8_t& red = _red;
			const uint8_t& green = _green;
			const uint8_t& blue = _blue;
			const float& hangle = _hangle;
			const float& hdsat = _hdsat;
		};

		/**
		 * @brief Use this class when programming the AI Vision camera.
		 */
		class codedesc : public objdesc {
			friend class vex::aivision;

		private:
			V5_DeviceAiVisionCode _code;

		public:
			codedesc(int32_t id, int32_t c1, int32_t c2, int32_t c3 = 0, int32_t c4 = 0, int32_t c5 = 0);

			/**
			 * @brief Creates a new AI Vision camera code description object.
			 * @param c1 The first color description which is part of the color code.
			 * @param c2 The second color description which is part of the color code.
			 */
			codedesc(int32_t id, colordesc& c1, colordesc& c2);
			codedesc(int32_t id, colordesc& c1, colordesc& c2, colordesc& c3);
			codedesc(int32_t id, colordesc& c1, colordesc& c2, colordesc& c3, colordesc& c4);
			codedesc(int32_t id, colordesc& c1, colordesc& c2, colordesc& c3, colordesc& c4, colordesc& c5);
		};

		class tagdesc : public objdesc {
		public:
			tagdesc(int32_t id);
		};

		class aiobjdesc : public objdesc {
		public:
			aiobjdesc(int32_t id);
		};

		static const tagdesc ALL_TAGS;
		static const colordesc ALL_COLORS;
		static const codedesc ALL_CODES;
		static const aiobjdesc ALL_AIOBJS;
		static const objdesc ALL_OBJECTS;

		static const uint32_t FLG_COLORMERGE;
		static const uint32_t FLG_OVLENABLE;


		int32_t objectCount;
		object largestObject;
		safearray<object, AIVISION_MAX_OBJECTS> objects;
		bool _color_enabled;
		bool _tags_enabled;
		bool _aiobj_enabled;
		bool _merge_enabled;
		bool _ovl_enabled;

		V5_DeviceAiVisionObject _objects[AIVISION_MAX_OBJECTS];
	};
};// namespace vex