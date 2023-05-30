/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#ifndef DEBUG_FLOAT_ARRAY_HPP
#define DEBUG_FLOAT_ARRAY_HPP

#include <uORB/topics/debug_array.h>

template <int N>
class MavlinkStreamDebugFloatArray : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamDebugFloatArray<N>(mavlink); }

	static constexpr const char *get_name_static()
	{
		switch (N)
		{
		case 1:
			return "SIMULINK_INBOUND";
		case 2:
			return "SIMULINK_OUTBOUND";
		default:
			return "DEBUG_FLOAT_ARRAY";
		}
	}
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _debug_array_sub->advertised() ? MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamDebugFloatArray(Mavlink *mavlink) : MavlinkStream(mavlink)
	{
		// XXX this can be removed once the multiplatform system remaps topics
		switch (N) {
		case 1:
			_debug_array_sub = new uORB::Subscription{ORB_ID(simulink_inbound)};
			break;

		case 2:
			_debug_array_sub = new uORB::Subscription{ORB_ID(simulink_outbound)};
			break;

		default:
			_debug_array_sub = new uORB::Subscription{ORB_ID(debug_array)};
			break;
		}
	}

	~MavlinkStreamDebugFloatArray() override
	{
		delete _debug_array_sub;
	}

	uORB::Subscription *_debug_array_sub{nullptr};

	bool send() override
	{
		debug_array_s debug;

		if (_debug_array_sub && _debug_array_sub->update(&debug)) {
			mavlink_debug_float_array_t msg{};

			msg.time_usec = debug.timestamp;

			switch (N) {
			case 1:
			{
				char message_name[10] = "inbound";
				memcpy(msg.name, message_name, sizeof(message_name));
				msg.array_id = debug_array_s::SIMULINK_INBOUND_ID;
				break;
			}
			case 2:
			{
				char message_name[10] = "outbound";
				memcpy(msg.name, message_name, sizeof(message_name));
				msg.array_id = debug_array_s::SIMULINK_OUTBOUND_ID;
			}
				break;

			default:
				msg.array_id = debug.id;
				memcpy(msg.name, debug.name, sizeof(msg.name));
				break;
			}
			msg.name[sizeof(msg.name) - 1] = '\0'; // enforce null termination

			for (size_t i = 0; i < debug_array_s::ARRAY_SIZE; i++) {
				msg.data[i] = debug.data[i];
			}

			mavlink_msg_debug_float_array_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif // DEBUG_FLOAT_ARRAY_HPP
