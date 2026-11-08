extends AIController3D


func get_obs():
	if _player.cur_goal == null:
		_player.pick_goal()

	var obs = [0.0, 0.0, 0.0, 1.0]  # placeholder if no victim
	if _player.cur_goal:
		var v = to_local(_player.cur_goal.global_position)
		var d = clamp(v.length(), 0.0, 50.0)
		v = v.normalized()
		obs = [v.x, v.y, v.z, d / 50.0]

	obs.append_array($RaycastSensor3D.get_observation())
	return {"obs": obs}


func get_action_space():
	return {"move": {"size": 3, "action_type": "continuous"}}


func set_action(action):
	_player.move_input = Vector3(action["move"][0], action["move"][1], action["move"][2])


func get_reward():
	return reward
