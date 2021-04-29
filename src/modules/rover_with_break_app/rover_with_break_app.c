/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
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
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
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
 * @file rover_with_break_apps.c
 * Minimal application example for PX4 autopilot
 *
 * @author czar balangue <czar.balangue@yahoo.com>
 */

#define RC_INTPUT_CHANNEL_6 5
#define RC_INTPUT_CHANNEL_7 6
#define RC_INTPUT_CHANNEL_8 7



#include <px4_platform_common/log.h>
#include <uORB/topics/input_rc.h>
#include <px4_platform/micro_hal.h>

#include </home/czar/Firmware/boards/px4/fmu-v5/src/board_config.h>
#include <uORB/uORB.h>




__EXPORT int rover_with_break_app_main(int argc, char *argv[]);




void toggle_relay(uint16_t rc_input_value, bool *toggle_state, bool *is_signal_high)
{
	int pwm_threshold = 1700;
	if(rc_input_value > pwm_threshold && *is_signal_high == false)
	{
		*toggle_state = !*toggle_state;
		*is_signal_high = true;
	}
	else
	{
		*is_signal_high = false;
	}
}

bool evaluate_momentary_switch(uint16_t rc_input_value)
{
	int pwm_threshold = 1700;
	if(rc_input_value > pwm_threshold)
	{
		return true;
	}
	else
	{
		return false;
	}
}




int rover_with_break_app_main(int argc, char *argv[])
{
	PX4_INFO("Starting rover with breaks apps");

	int _rc_sub = orb_subscribe(ORB_ID(input_rc));

	/* read low-level values from FMU or IO RC inputs (PPM, Spektrum, S.Bus) */
	struct input_rc_s rc_input;
	//orb_copy(ORB_ID(input_rc), _rc_sub, &rc_input);
	//px4_usleep(100000);
	/* limit the update rate to 5 Hz */
	orb_set_interval(_rc_sub, 200);

	bool toggle_state_relay_6 = false;
	bool is_signal_high_6 = false;

	bool toggle_state_relay_7 = false;
	bool is_signal_high_7 = false;

	CONFIGURE_RELAY_ONE();
	CONFIGURE_RELAY_TWO();
	CONFIGURE_RELAY_MOMENTARY_SWITCH();

	//bool rc_updated;

	while (true)
	{
		orb_copy(ORB_ID(input_rc), _rc_sub, &rc_input);

		//orb_check(_rc_sub, &rc_updated);

		//if(rc_updated)
		//{
			toggle_relay(rc_input.values[RC_INTPUT_CHANNEL_6],&toggle_state_relay_6,&is_signal_high_6);
			toggle_relay(rc_input.values[RC_INTPUT_CHANNEL_7],&toggle_state_relay_7,&is_signal_high_7);
			bool momentary_switch_value = evaluate_momentary_switch(rc_input.values[RC_INTPUT_CHANNEL_8]);

			RELAY_ONE(toggle_state_relay_6);
			RELAY_ONE(toggle_state_relay_7);
			MOMENTARY_SWITCH(momentary_switch_value);
		//}
	}

	return OK;
}
