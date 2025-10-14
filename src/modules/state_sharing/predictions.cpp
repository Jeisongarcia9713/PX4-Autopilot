#include "state_sharing/predictions.hpp"
#include <px4_platform_common/events.h>
#include <matrix/math.hpp>
#include <cmath>

void LinearPrediction::updateParameters(const PredictionParams &params)
{
	_speed_method = params.prediction_speed_method;
	_parameters.sharing_period = params.sharing_period;
	_parameters.timeslot_delay = params.timeslot_delay;
	_parameters.cruise_speed = params.cruise_speed;
}

void LinearPrediction::updateData()
{
	if (_vehicle_odometry_sub.updated()) {
		vehicle_odometry_s vehicle_odometry{};

		if (_vehicle_odometry_sub.copy(&vehicle_odometry)) {
			memcpy(&_ground_speed, &vehicle_odometry.velocity, sizeof(_ground_speed));
			_yaw = atan2(_ground_speed[1], _ground_speed[0]);
		}
	}

	if (_airspeed_wind_sub.updated()) {
		airspeed_wind_s airspeed_wind{};

		if (_airspeed_wind_sub.copy(&airspeed_wind)) {
			_windspeed_north = airspeed_wind.windspeed_north;
			_windspeed_east = airspeed_wind.windspeed_east;
		}
	}

	if (_vehicle_command_sub.updated()) {
		vehicle_command_s vehicle_command{};

		if (_vehicle_command_sub.copy(&vehicle_command)) {
			if (vehicle_command.command == vehicle_command_s::VEHICLE_CMD_DO_CHANGE_SPEED) {
				_airspeed_sp = vehicle_command.param2;
			}
		}
	}
}

float LinearPrediction::calculate_ground_speed() const
{
	matrix::Vector2f air_velocity({_airspeed_sp * std::cos(_yaw),
				       _airspeed_sp * std::sin(_yaw)});
	matrix::Vector2f wind_velocity({_windspeed_north, _windspeed_east});
	return static_cast<matrix::Vector2f>(air_velocity + wind_velocity).norm();
}

float LinearPrediction::getLinearPrediction(float speed) const
{
	auto time_interval = _parameters.sharing_period - _parameters.timeslot_delay;
	auto prediction = speed * time_interval;
	return prediction;
}

float LinearPrediction::getSpeed() const
{
	float speed = NAN;

	switch (_speed_method) {
	case CRUISE_SPEED:
		speed = _parameters.cruise_speed;
		break;

	case GROUND_SPEED:
		speed = matrix::Vector2f(_ground_speed).norm();
		break;

	case AIR_SPEED:
		speed = calculate_ground_speed();
		break;

	default: break;
	}

	return speed;
}

void LinearPrediction::getPrediction2D(float &lat, float &lon) const
{
	float speed = getSpeed();
	float body[2] = {getLinearPrediction(speed), 0.0};
	auto ned = bodyToNed(matrix::Vector2f(body), _yaw);
	auto lla = nedToLla(ned, lat, lon);

	PX4_DEBUG("speed with method %d for prediction %f, angle %f, ned (%f, %f), timeslot %f",
		  _speed_method.get(), (double)speed, (double)_yaw, (double)ned(0), (double)ned(1),
		  (double) _parameters.timeslot_delay);
	PX4_DEBUG("lat %f, lon %f, lat_pred %f, long_pred %f",
		  (double)lat, (double)lon, (double)lla(0), (double)lla(1));
	PX4_DEBUG("Distance in NED frame: %f", (double)ned.norm());
	PX4_DEBUG("Horizontal distance between LLA points %f \n ", (double)get_distance_to_next_waypoint(
			  lat, lon, lla(0), lla(1)));
	lat = (float)lla(0);
	lon = (float)lla(1);
}

bool LinearPrediction::enabled() const
{
	auto speed = getSpeed();

	if (!_speed_method.isValid() || (double)std::abs(speed) < 1e-2 || std::isnan(speed)) {
		return false;
	}

	return true;

}

Predictions::Predictions(ModuleParams *parent)
	: ModuleParams(parent)
{
}

Predictions::~Predictions()
{
	if (_prediction_method) {
		delete _prediction_method;
	}

}

void Predictions::init()
{
	updateParameters();
}

void Predictions::updateParameters()
{
	if (_current_prediction_method != _param_prediction_method.get()) {
		_current_prediction_method = _param_prediction_method.get();

		if (_prediction_method) {
			delete _prediction_method;
			_prediction_method = nullptr;
		}

		switch (_current_prediction_method) {
		case UNDEFINED_PREDICTION:
			break;

		case LINEAR_PREDICTION:
			_prediction_method = new LinearPrediction();
			break;

		default:
			events::send<int16_t>(events::ID("state_sharing_update_parameteres"),
					      events::Log::Error,
					      "[STATE_SHARING]: There isn't behavior defined for prediction method: {1}",
					      _param_prediction_method.get());
			PX4_ERR("There isn't behavior defined prediction method: %d",
				_current_prediction_method.get());
			break;
		}
	}

	if (!_current_prediction_method.isValid() || !_prediction_method) {
		return;
	}

	PredictionParams parameters(
		_param_ident.get(), _param_speed_method.get(), _param_num_timeslots.get(),
		_param_sharing_period.get(), _param_cruise_speed.get()
	);
	_prediction_method->updateParameters(parameters);
}

void Predictions::updateData()
{
	if (!_current_prediction_method.isValid() || !_prediction_method) {
		return;
	}

	_prediction_method->updateData();
}

void Predictions::getPrediction2D(float &lat, float &lon) const
{
	if (!enabled()) {
		return;
	}

	_prediction_method->getPrediction2D(lat, lon);
}

PredictionMethodEnum Predictions::getCurrentPredictionMethod() const
{
	return _current_prediction_method.get();
}

bool Predictions::enabled() const
{
	if (!_current_prediction_method.isValid() || !_prediction_method) {
		return false;
	}

	return _prediction_method->enabled();
}
