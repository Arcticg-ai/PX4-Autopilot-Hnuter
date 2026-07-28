/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include "HnuterServoDynamics.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/transport/TopicUtils.hh>

using custom::HnuterServoDynamics;

GZ_ADD_PLUGIN(
	HnuterServoDynamics,
	gz::sim::System,
	gz::sim::ISystemConfigure,
	gz::sim::ISystemPreUpdate
)

GZ_ADD_PLUGIN_ALIAS(HnuterServoDynamics, "custom::HnuterServoDynamics")

std::string HnuterServoDynamics::modelTopic(const std::string &model_name, const std::string &relative_topic)
{
	if (!relative_topic.empty() && relative_topic.front() == '/') {
		return gz::transport::TopicUtils::AsValidTopic(relative_topic);
	}

	return gz::transport::TopicUtils::AsValidTopic("/model/" + model_name + "/" + relative_topic);
}

void HnuterServoDynamics::Configure(const gz::sim::Entity &entity,
		const std::shared_ptr<const sdf::Element> &sdf,
		gz::sim::EntityComponentManager &ecm,
		gz::sim::EventManager &event_manager)
{
	(void)event_manager;
	const gz::sim::Model model(entity);

	if (!model.Valid(ecm)) {
		throw std::runtime_error("HnuterServoDynamics must be attached to a model entity");
	}

	const std::string input_topic_relative = sdf->Get<std::string>("input_topic");
	const std::string output_topic_relative = sdf->Get<std::string>("output_topic");
	_channel_name = sdf->Get<std::string>("channel_name", input_topic_relative).first;
	_gain_positive = sdf->Get<double>("gain_positive", 1.0).first;
	_gain_negative = sdf->Get<double>("gain_negative", _gain_positive).first;
	_tau_positive_s = std::max(sdf->Get<double>("tau_positive", 0.0).first, 0.0);
	_tau_negative_s = std::max(sdf->Get<double>("tau_negative", _tau_positive_s).first, 0.0);
	_delay_positive_s = std::max(sdf->Get<double>("delay_positive", 0.0).first, 0.0);
	_delay_negative_s = std::max(sdf->Get<double>("delay_negative", _delay_positive_s).first, 0.0);
	_rate_positive_rad_s = std::max(sdf->Get<double>("rate_positive", 1000.0).first, 0.0);
	_rate_negative_rad_s = std::max(sdf->Get<double>("rate_negative", _rate_positive_rad_s).first, 0.0);
	_angle_min_rad = sdf->Get<double>("angle_min", -3.141592653589793).first;
	_angle_max_rad = sdf->Get<double>("angle_max", 3.141592653589793).first;

	if (_angle_min_rad >= _angle_max_rad) {
		throw std::runtime_error("HnuterServoDynamics angle_min must be smaller than angle_max");
	}

	const std::string model_name = model.Name(ecm);
	const std::string input_topic = modelTopic(model_name, input_topic_relative);
	const std::string output_topic = modelTopic(model_name, output_topic_relative);

	if (input_topic.empty() || output_topic.empty()) {
		throw std::runtime_error("HnuterServoDynamics received an invalid transport topic");
	}

	_output_publisher = _node.Advertise<gz::msgs::Double>(output_topic);

	if (!_output_publisher.Valid()
	    || !_node.Subscribe(input_topic, &HnuterServoDynamics::commandCallback, this)) {
		throw std::runtime_error("HnuterServoDynamics failed to create transport endpoints");
	}

	_configured = true;
	gzmsg << "HnuterServoDynamics[" << _channel_name << "] " << input_topic << " -> " << output_topic
	      << ", K(+/-)=" << _gain_positive << "/" << _gain_negative
	      << ", tau(+/-)=" << _tau_positive_s << "/" << _tau_negative_s
	      << ", delay(+/-)=" << _delay_positive_s << "/" << _delay_negative_s << std::endl;
}

void HnuterServoDynamics::commandCallback(const gz::msgs::Double &message)
{
	if (!std::isfinite(message.data())) {
		return;
	}

	std::lock_guard<std::mutex> lock(_command_mutex);
	_latest_command_rad = message.data();
	++_latest_generation;
}

void HnuterServoDynamics::PreUpdate(const gz::sim::UpdateInfo &info,
		gz::sim::EntityComponentManager &ecm)
{
	(void)ecm;

	if (!_configured || info.paused) {
		return;
	}

	const double now_s = std::chrono::duration<double>(info.simTime).count();
	const double dt_s = std::chrono::duration<double>(info.dt).count();

	if (!std::isfinite(now_s) || !std::isfinite(dt_s)) {
		return;
	}

	if (_last_sim_time_s >= 0.0 && now_s < _last_sim_time_s) {
		std::lock_guard<std::mutex> lock(_command_mutex);
		_pending_commands.clear();
		_processed_generation = _latest_generation;
		_delayed_command_rad = 0.0;
		_output_angle_rad = 0.0;
	}

	_last_sim_time_s = now_s;

	{
		std::lock_guard<std::mutex> lock(_command_mutex);

		if (_latest_generation != _processed_generation) {
			const double delay_s = _latest_command_rad >= 0.0 ? _delay_positive_s : _delay_negative_s;
			_pending_commands.push_back({now_s + delay_s, _latest_command_rad});
			_processed_generation = _latest_generation;
		}
	}

	while (!_pending_commands.empty() && _pending_commands.front().ready_time_s <= now_s) {
		_delayed_command_rad = _pending_commands.front().command_rad;
		_pending_commands.pop_front();
	}

	if (dt_s <= 0.0) {
		return;
	}

	const bool positive_command = _delayed_command_rad >= 0.0;
	const double gain = positive_command ? _gain_positive : _gain_negative;
	const double tau_s = positive_command ? _tau_positive_s : _tau_negative_s;
	const double target_angle_rad = gain * _delayed_command_rad;
	const double filter_alpha = tau_s > 1e-6 ? 1.0 - std::exp(-dt_s / tau_s) : 1.0;
	const double requested_delta = filter_alpha * (target_angle_rad - _output_angle_rad);
	const double rate_rad_s = requested_delta >= 0.0 ? _rate_positive_rad_s : _rate_negative_rad_s;
	const double max_delta = rate_rad_s * dt_s;
	const double applied_delta = std::clamp(requested_delta, -max_delta, max_delta);
	_output_angle_rad = std::clamp(_output_angle_rad + applied_delta, _angle_min_rad, _angle_max_rad);

	gz::msgs::Double output_message;
	output_message.set_data(_output_angle_rad);
	_output_publisher.Publish(output_message);
}
