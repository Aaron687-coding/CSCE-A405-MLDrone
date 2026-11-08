extends CharacterBody3D

@export var speed = 8.0
@onready var ai_controller = $AIController3D

var move_input = Vector3.ZERO
var cur_goal = null
var best_goal_distance = 10000.0
var spawn_pos


func _ready():
	ai_controller.init(self)
	spawn_pos = global_position


func game_over():
	global_position = spawn_pos + Vector3(randf_range(-2, 2), 0, randf_range(-2, 2))
	velocity = Vector3.ZERO
	pick_goal()
	ai_controller.reset()


func nearest_victim():
	var best = null
	for v in get_tree().get_nodes_in_group("victims"):
		if best == null or global_position.distance_to(v.global_position) < global_position.distance_to(best.global_position):
			best = v
	return best


func pick_goal():
	cur_goal = nearest_victim()
	if cur_goal:
		best_goal_distance = global_position.distance_to(cur_goal.global_position)


func _physics_process(delta):
	if ai_controller.needs_reset:
		game_over()
		return
	if cur_goal == null:
		pick_goal()

	set_input()
	velocity = move_input.limit_length(1.0) * speed
	move_and_slide()
	update_reward()

	if cur_goal and global_position.distance_to(cur_goal.global_position) < 1.5:
		goal_reached()
	if get_slide_collision_count() > 0:
		crashed()


func set_input():
	if ai_controller.heuristic == "model":
		return
	move_input = Vector3(
		float(Input.is_key_pressed(KEY_D)) - float(Input.is_key_pressed(KEY_A)),
		float(Input.is_key_pressed(KEY_SPACE)) - float(Input.is_key_pressed(KEY_SHIFT)),
		float(Input.is_key_pressed(KEY_S)) - float(Input.is_key_pressed(KEY_W))
	)


func update_reward():
	ai_controller.reward -= 0.01
	ai_controller.reward += shaping_reward()


func shaping_reward():
	if cur_goal == null:
		return 0.0
	var d = global_position.distance_to(cur_goal.global_position)
	var r = 0.0
	if d < best_goal_distance:
		r = best_goal_distance - d
		best_goal_distance = d
	return r


func goal_reached():
	ai_controller.reward += 100.0
	cur_goal.respawn()
	pick_goal()


func crashed():
	ai_controller.done = true
	ai_controller.reward -= 10.0
	game_over()
