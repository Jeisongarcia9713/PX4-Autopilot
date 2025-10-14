#include "state_sharing/utils.hpp"

void setParameter(const ArgParser &args, const char *name_args,
		  const char *name, const float default_value)
{
	if (args.hasArgument(name_args)) {
		param_t param = param_find_no_notification(name);
		auto value = args.getFloat(name_args, default_value);

		if (param != PARAM_INVALID) {
			PX4_DEBUG("setting argument %s with value %f", name, (double) value);
			param_set(param, &value);
		}
	}

}

void setParameter(const ArgParser &args, const char *name_args,
		  const char *name, const int default_value)
{
	if (args.hasArgument(name_args)) {
		param_t param = param_find_no_notification(name);
		auto value = args.getInt(name_args, default_value);

		if (param != PARAM_INVALID) {
			PX4_DEBUG("setting argument %s with value %d", name, value);
			param_set(param, &value);
		}
	}

}

uint64_t getRealTimeNs()
{
	timespec tv = {};
	px4_clock_gettime(CLOCK_REALTIME, &tv);
	return tv.tv_sec * 1e9 + tv.tv_nsec;
}

matrix::Vector2f bodyToNed(matrix::Vector2f body, float yaw)
{
	matrix::Dcm2f R_inv(yaw);
	return R_inv * body;
}

matrix::Vector2d nedToLla(matrix::Vector2f ned, float lat_ref, float lon_ref)
{
	MapProjection map_projection(lat_ref, lon_ref);
	matrix::Vector2d res;
	map_projection.reproject(ned(0), ned(1), res(0), res(1));
	return res;
}
