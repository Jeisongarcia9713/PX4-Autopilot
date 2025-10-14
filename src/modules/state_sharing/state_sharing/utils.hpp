#pragma once

#include <px4_platform_common/module_params.h>

#include <drivers/drv_hrt.h>
#include "args_parser.hpp"
#include <matrix/math.hpp>
#include <geo/geo.h>

constexpr int UNDEFINED_ENUM = 0;

template<typename CustomEnumType, size_t MaxValue>
class CustomEnum
{
	using EnumType = CustomEnumType;

public :
	CustomEnum()
		: value_(static_cast<EnumType>(UNDEFINED_ENUM))
	{
	}
	bool operator==(const CustomEnum &other) const { return value_ == other.value_; }
	bool operator!=(const CustomEnum &other) const { return !(*this == other); }
	bool operator==(const EnumType &value) const { return value_ == value; }
	bool operator!=(const EnumType &value) const { return value_ != value; }

	CustomEnum &operator=(EnumType value)
	{
		value_ = value;
		return *this;
	}

	CustomEnum &operator=(int raw)
	{
		value_ = static_cast<EnumType>(raw);
		return *this;
	}

	/**
	 * @brief Check if the enum value is undefined.
	 *
	 * @return true if value is undefined, false otherwise
	 */
	bool isUndefined() const
	{
		return value_ == static_cast<EnumType>(UNDEFINED_ENUM);
	}

	/**
	 * @brief Check if the enum value is valid.
	 *
	 * @return true if value is valid, false otherwise
	 */
	bool isValid() const
	{
		if (isUndefined()) {
			return false;
		}

		return value_ > 0 && value_ <= MaxValue ? true : false;
	}

	operator EnumType() const
	{
		return value_;
	}

	const EnumType &get() const
	{
		return value_;
	}

	static constexpr size_t getMaxValue()
	{
		return MaxValue;
	}

private :
	EnumType value_;
};

/**
 * @brief Set a float parameter value from a ArgsParser object.
 *
 * @param[in] args ArgsParser object
 * @param[in] name_args Argument name in command line
 * @param[in] name Parameter name
 * @param[in] default_value Default value if not found
 */
void setParameter(const ArgParser &args, const char *name_args,
		  const char *name, float default_value = 0.0);

/**
 * @brief Set a int parameter value from a ArgsParser object.
 *
 * @param[in] args ArgsParser object
 * @param[in] name_args Argument name in command line
 * @param[in] name Parameter name
 * @param[in] default_value Default value if not found
 */
void setParameter(const ArgParser &args, const char *name_args,
		  const char *name, int default_value = 0);

/**
 * @brief Get current real time in nanoseconds.
 *
 * @return Current time in nanoseconds
 */
uint64_t getRealTimeNs();

/**
 * @brief Convert body frame coordinates to NED frame.
 *
 * @param[in] body Body frame coordinates
 * @param[in] yaw Yaw angle [rad]
 * @return NED frame coordinates x, y [m]
 */
matrix::Vector2f bodyToNed(matrix::Vector2f  body, float yaw);

/**
 * @brief Convert NED frame coordinates to latitude/longitude.
 *
 * @param[in] ned NED frame coordinates
 * @param[in] lat_ref Reference latitude [deg]
 * @param[in] lon_ref Reference longitude [deg]
 * @return Latitude/longitude coordinates [deg]
 */
matrix::Vector2d nedToLla(matrix::Vector2f ned, float lat_ref, float lon_ref);
