#pragma once

#include <px4_platform_common/log.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#define MAX_ARGS 10
#define MAX_KEY_LEN 32
#define MAX_VALUE_LEN 32

typedef struct {
	char key[MAX_KEY_LEN];
	char value[MAX_VALUE_LEN];
} ArgPair;

class ArgParser
{
private:
	ArgPair _args[MAX_ARGS];
	size_t _arg_count;

	/**
	 * @brief Trim whitespace from a string.
	 *
	 * @param[in,out] str String to trim
	 */
	static void trim(char *str);

	/**
	 * @brief Convert a string to lowercase.
	 *
	 * @param[in,out] str String to convert
	 */
	static void toLowerCase(char *str);

public:
	/**
	 * @brief Construct from a comma-separated key=value string.
	 *
	 * @param[in] data Input string to parse
	 */
	ArgParser(const char *data);

	/**
	 * @brief Check if argument exists.
	 *
	 * @param[in] arg Argument name to check
	 * @return true if argument exists, false otherwise
	 */
	bool hasArgument(const char *arg) const;

	/**
	 * @brief Get argument value as string.
	 *
	 * @param[in] arg Argument name to get
	 * @param[in] defaultValue Default value if argument not found
	 * @return Argument value as string
	 */
	const char *getArgument(const char *arg, const char *defaultValue = "") const;

	/**
	 * @brief Get argument value as int.
	 *
	 * @param[in] arg Argument name to get
	 * @param[in] defaultValue Default value if argument not found
	 * @return Argument value as integer
	 */
	int getInt(const char *arg, int defaultValue = 0) const;

	/**
	 * @brief Get argument value as float.
	 *
	 * @param[in] arg Argument name to get
	 * @param[in] defaultValue Default value if argument not found
	 * @return Argument value as float
	 */
	float getFloat(const char *arg, float defaultValue = 0.0f) const;

	/**
	 * @brief Get argument value as bool.
	 *
	 * @param[in] arg Argument name to get
	 * @param[in] defaultValue Default value if argument not found
	 * @return Argument value as boolean
	 */
	bool getBool(const char *arg, bool defaultValue = false) const;

	/**
	 * @brief Print all parsed arguments (for debug).
	 */
	void printArguments() const;
};
