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

#define MOMENTARY_SWITCH_RESET_COUNTER 2
#define TOGGLE_RELAY_RESET_COUNTER 2


#include <px4_platform_common/log.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/relay_control.h>
#include <uORB/topics/rc_channels.h>
#include <px4_platform/micro_hal.h>

#include </home/czar/Firmware/boards/px4/fmu-v5/src/board_config.h>
#include <uORB/uORB.h>


__EXPORT int rover_with_break_app_main(int argc, char *argv[]);

void toggle_relay(float rc_channel_value, bool *toggle_state, bool *is_signal_high)
{
	float threshold = 0.5;
	if(rc_channel_value > threshold)
	{
		if(false == *is_signal_high)
		{
			*toggle_state = !*toggle_state;
			*is_signal_high = true;
		}
	}
	else
	{
		*is_signal_high = false;
	}
}

void handle_relay_cmd(struct relay_control_s relay,
		      bool *toggle_state_six,
		      bool *toggle_state_seven,
		      bool *momentary_switch_value,
		      int *toggle_state_relay_6_counter,
		      int *toggle_state_relay_7_counter,
		      int *momentary_switch_counter)
{
	if(RELAY_CONTROL_RELAY_FMU_MAIN_SIX == relay.relay_id)
	{
		if(true == relay.relay_value)
		{
			if(*toggle_state_relay_6_counter < 1)
			{
				*toggle_state_six = !*toggle_state_six;
			}

			*toggle_state_relay_6_counter = TOGGLE_RELAY_RESET_COUNTER;
		}
	}
	else if(RELAY_CONTROL_RELAY_FMU_MAIN_SEVEN == relay.relay_id)
	{
		if(true == relay.relay_value)
		{
			if(*toggle_state_relay_7_counter < 1)
			{
				*toggle_state_seven = !*toggle_state_seven;
			}

			*toggle_state_relay_7_counter = TOGGLE_RELAY_RESET_COUNTER;
		}
	}
	else if(RELAY_CONTROL_RELAY_FMU_MAIN_EIGHT == relay.relay_id)
	{
		if(true == relay.relay_value)
		{
			*momentary_switch_counter = MOMENTARY_SWITCH_RESET_COUNTER;
		}
	}
	else
	{
		PX4_INFO("rover_with_break_app : unhandled : relay id:%d , relay value: %d\n",relay.relay_id,relay.relay_value);
	}

	*toggle_state_relay_6_counter = *toggle_state_relay_6_counter - 1;
	*toggle_state_relay_7_counter = *toggle_state_relay_7_counter - 1;


	if(*toggle_state_relay_6_counter < 1)
	{
		*toggle_state_relay_6_counter = 0;
	}

	if(*toggle_state_relay_7_counter < 1)
	{
		*toggle_state_relay_7_counter = 0;
	}

	if(*momentary_switch_counter > 0)
	{
		*momentary_switch_counter = *momentary_switch_counter - 1;
		*momentary_switch_value = true;
	}
	else
	{
		*momentary_switch_counter = 0;
		*momentary_switch_value = false;
	}

}

bool evaluate_momentary_switch(float rc_channel_value)
{
	float threshold = 0.5;
	if(rc_channel_value > threshold)
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

	int _rc_sub = orb_subscribe(ORB_ID(rc_channels));
	int _relay_control_sub = orb_subscribe(ORB_ID(relay_control));


	/* limit the update rate to 5 Hz */
	orb_set_interval(_rc_sub, 200);
	orb_set_interval(_relay_control_sub, 200);


	/* read low-level values from FMU or IO RC inputs (PPM, Spektrum, S.Bus) */
	struct rc_channels_s rc_channels_value;
	struct relay_control_s relay;

	bool toggle_state_relay_6 = false;
	bool is_signal_high_6 = false;
	int toggle_state_relay_6_counter = TOGGLE_RELAY_RESET_COUNTER;


	bool toggle_state_relay_7 = false;
	bool is_signal_high_7 = false;
	int toggle_state_relay_7_counter = TOGGLE_RELAY_RESET_COUNTER;


	bool momentary_switch_value = false;
	int momentary_switch_counter = MOMENTARY_SWITCH_RESET_COUNTER;

	px4_arch_configgpio(MAIN_OUT_6);
	px4_arch_configgpio(MAIN_OUT_7);
	px4_arch_configgpio(MAIN_OUT_8);

	while (true)
	{
		orb_copy(ORB_ID(rc_channels), _rc_sub, &rc_channels_value);
		orb_copy(ORB_ID(relay_control), _relay_control_sub, &relay);

		px4_usleep(100000);

		toggle_relay(rc_channels_value.channels[RC_INTPUT_CHANNEL_6],&toggle_state_relay_6,&is_signal_high_6);
		toggle_relay(rc_channels_value.channels[RC_INTPUT_CHANNEL_7],&toggle_state_relay_7,&is_signal_high_7);
		momentary_switch_value = evaluate_momentary_switch(rc_channels_value.channels[RC_INTPUT_CHANNEL_8]);

		handle_relay_cmd(relay,&toggle_state_relay_6,&toggle_state_relay_7,&momentary_switch_value,
				       &toggle_state_relay_6_counter,&toggle_state_relay_7_counter,&momentary_switch_counter);

		px4_arch_gpiowrite(MAIN_OUT_6, toggle_state_relay_6);
		px4_arch_gpiowrite(MAIN_OUT_7, toggle_state_relay_7);
		px4_arch_gpiowrite(MAIN_OUT_8, momentary_switch_value);
	}

	return OK;
}
