/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be used to
 *    endorse or promote products derived from this software without specific
 *    prior written permission.
 *
 ****************************************************************************/

#pragma once

#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>

#include <gz/msgs/double.pb.h>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

namespace custom
{

class HnuterServoDynamics :
	public gz::sim::System,
	public gz::sim::ISystemConfigure,
	public gz::sim::ISystemPreUpdate
{
public:
	void Configure(const gz::sim::Entity &entity,
		       const std::shared_ptr<const sdf::Element> &sdf,
		       gz::sim::EntityComponentManager &ecm,
		       gz::sim::EventManager &event_manager) final;

	void PreUpdate(const gz::sim::UpdateInfo &info,
		       gz::sim::EntityComponentManager &ecm) final;

private:
	struct PendingCommand {
		double ready_time_s;
		double command_rad;
	};

	void commandCallback(const gz::msgs::Double &message);
	static std::string modelTopic(const std::string &model_name, const std::string &relative_topic);

	gz::transport::Node _node;
	gz::transport::Node::Publisher _output_publisher;
	std::mutex _command_mutex;
	std::deque<PendingCommand> _pending_commands;

	std::string _channel_name;
	double _latest_command_rad{0.0};
	uint64_t _latest_generation{0};
	uint64_t _processed_generation{0};
	double _delayed_command_rad{0.0};
	double _output_angle_rad{0.0};
	double _last_sim_time_s{-1.0};

	double _gain_positive{1.0};
	double _gain_negative{1.0};
	double _tau_positive_s{0.0};
	double _tau_negative_s{0.0};
	double _delay_positive_s{0.0};
	double _delay_negative_s{0.0};
	double _rate_positive_rad_s{1000.0};
	double _rate_negative_rad_s{1000.0};
	double _angle_min_rad{-3.141592653589793};
	double _angle_max_rad{3.141592653589793};
	bool _configured{false};
};

} // namespace custom
