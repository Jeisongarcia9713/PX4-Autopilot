/****************************************************************************
 *
 *   Copyright (c) 2013-2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file state_sharing_params.c
 *
 * Parameters defined for the state sharing
 *
 *
 * @author Jeison Garcia <jeisongarcia9713@gmail.com>
 */


/**
 * Period defined for the state sharing in seconds
 *
 * @group state_sharing
 */
PARAM_DEFINE_FLOAT(SHARING_PERIOD, 1.0);

/**
 * Delay to begin the sharing after the start signal is received
 *
 * @group state_sharing
 */
PARAM_DEFINE_FLOAT(DELAY_START, 0.0);

/**
 * State sharing telemetry data flags bitmask
 *
 * Bitmask defining which telemetry fields are broadcast:
 * 0: Global Position (lat, lon, alt)
 * 1: NED ground velocity (vx, vy, vz)
 * 2: Quaternion attitude (q)
 * 3: NED ground acceleration (ax, ay, az)
 * 4: Auxiliary field 1 (aux[0])
 * 5: Auxiliary field 2 (aux[1])
 * 6: Auxiliary field 3 (aux[2])
 * 7: Auxiliary field 4 (aux[3])
 *
 * @bit 0 Global Position
 * @bit 1 Velocity
 * @bit 2 Attitude
 * @bit 3 Acceleration
 * @bit 4 Aux 1
 * @bit 5 Aux 2
 * @bit 6 Aux 3
 * @bit 7 Aux 4
 * @min 1
 * @max 255
 * @group state_sharing
 */
PARAM_DEFINE_INT32(SHARING_FLAGS, 5);
