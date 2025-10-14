#pragma once

#include <px4_platform_common/log.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/airspeed_wind.h>
#include <uORB/topics/vehicle_command.h>

#include "utils.hpp"

enum PredictionMethodEnum {
	UNDEFINED_PREDICTION = UNDEFINED_ENUM,
	LINEAR_PREDICTION = 1,

};

enum PredictionSpeedMethodEnum {
	UNDEFINED_SPEED = UNDEFINED_ENUM,
	CRUISE_SPEED = 1,
	GROUND_SPEED,
	AIR_SPEED
};

class  PredictionMethod : public CustomEnum<PredictionMethodEnum, LINEAR_PREDICTION>
{
public:
	using CustomEnum<PredictionMethodEnum, LINEAR_PREDICTION>::operator=;
};

class  SpeedMethod : public CustomEnum<PredictionSpeedMethodEnum, AIR_SPEED>
{
public:
	using CustomEnum<PredictionSpeedMethodEnum, AIR_SPEED>::operator=;
};

struct PredictionParams {
	PredictionParams() {};
	PredictionParams(int ident,
			 int speed_method_param,
			 int num_timeslots,
			 float sharing_period_param,
			 float cruise_speed_param)
		: prediction_speed_method(speed_method_param),
		  sharing_period(sharing_period_param),
		  timeslot_delay(ident * sharing_period / (num_timeslots + 1)),
		  cruise_speed(cruise_speed_param) {};

	int prediction_speed_method {UNDEFINED_ENUM};
	float sharing_period {NAN};
	float timeslot_delay {NAN};
	float cruise_speed {NAN};
};

class PredictionBase
{
public:
	virtual ~PredictionBase() {};

	/**
	 * @brief Update prediction parameters.
	 *
	 * @param[in] params New prediction parameters to use
	 */
	virtual void updateParameters(const PredictionParams &params) = 0;

	/**
	 * @brief Update prediction data from vehicle state.
	 *
	 * Differs from each specific prediction class
	 */
	virtual void updateData() = 0;

	/**
	 * @brief Get 2D position prediction in latitude/longitude.
	 *
	 * @param[in,out] lat Current latitude [deg], updated with prediction
	 * @param[in,out] lon Current longitude [deg], updated with prediction
	 */
	virtual void getPrediction2D(float &lat, float &lon) const = 0;

	/**
	 * @brief Check if specific prediction is enabled and valid.
	 *
	 * @return true if prediction is enabled and valid, false otherwise
	 */
	virtual bool enabled() const = 0;
protected:
	PredictionParams _parameters;
};

class LinearPrediction : public PredictionBase
{
public:
	void updateParameters(const PredictionParams &params) override;
	void updateData() override;

	/**
	 * @brief Calculate the ground speed based on airspeed and wind.
	 *
	 * @return Calculated ground speed [m/s]
	 */
	float calculate_ground_speed() const;

	/**
	 * @brief Get linear prediction distance based on speed.
	 *
	 * @param[in] speed Current speed [m/s]
	 * @return Predicted distance [m]
	 */
	float getLinearPrediction(float speed) const;

	/**
	 * @brief Get the current speed based on the selected speed method.
	 *
	 * @return Current speed [m/s] or NAN if invalid
	 */
	float getSpeed() const;
	void getPrediction2D(float &lat, float &lon) const override;
	bool enabled() const override;
private:
	SpeedMethod _speed_method;
	float _ground_speed[2];
	float _airspeed_sp;
	float _yaw;
	float _windspeed_north;
	float _windspeed_east;
	uORB::Subscription  _vehicle_odometry_sub{ORB_ID(vehicle_odometry)};
	uORB::Subscription  _airspeed_wind_sub{ORB_ID(airspeed_wind)};
	uORB::Subscription  _vehicle_command_sub{ORB_ID(vehicle_command)};
};

class Predictions : public ModuleParams
{
public:
	Predictions(ModuleParams *parent);
	~Predictions();
	void init();
	void updateParameters();
	void updateData();
	void getPrediction2D(float &lat, float &lon) const;

	/**
	 * @brief Get the current prediction method.
	 *
	 * @return Current prediction method
	 */
	PredictionMethodEnum getCurrentPredictionMethod() const;
	bool enabled() const;
private:
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::PREDICT_METHOD>) _param_prediction_method,
		(ParamInt<px4::params::PRED_SPD_METHOD>) _param_speed_method,
		(ParamFloat<px4::params::SHARING_PERIOD>) _param_sharing_period,
		(ParamInt<px4::params::NUM_TIMESLOTS>) _param_num_timeslots,
		(ParamFloat<px4::params::FW_AIRSPD_TRIM>) _param_cruise_speed,
		(ParamInt<px4::params::IDENT>) _param_ident
	)

	PredictionMethod _current_prediction_method;
	PredictionBase  *_prediction_method{nullptr};
};
