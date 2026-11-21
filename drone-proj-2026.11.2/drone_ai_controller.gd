extends AIController3D

# Observation layout (never change after training starts):
# [can_see, dir_x, dir_y, dir_z, distance, vel_x, vel_y, vel_z, ...ray distances]


func get_obs():
	var obs = [0.0, 0.0, 0.0, 0.0, 1.0]  # nothing seen
	var t = _player.seen_target
	if t:
		var v = _player.to_local(t.global_position)
		var d = clamp(v.length(), 0.0, _player.view_range)
		v = v.normalized()
		obs = [1.0, v.x, v.y, v.z, d / _player.view_range]

	var vel = _player.global_basis.inverse() * _player.velocity / _player.max_speed
	obs.append_array([vel.x, vel.y, vel.z])

	obs.append_array($RaycastSensor3D.get_observation())
	return {"obs": obs}


func get_action_space():
	return {
		"move": {"size": 3, "action_type": "continuous"},
		"yaw": {"size": 1, "action_type": "continuous"}
	}


func set_action(action):
	_player.move_input = Vector3(action["move"][0], action["move"][1], action["move"][2]).limit_length(1.0)
	_player.yaw_input = clamp(action["yaw"][0], -1.0, 1.0)


func get_reward():
	return reward
