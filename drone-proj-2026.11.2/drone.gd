extends CharacterBody3D

# --- Flight ---
@export_group("Flight")
@export var max_speed = 8.0
@export var accel = 3.0        # lower = more drift
@export var yaw_speed = 2.0

# --- Vision ---
@export_group("Vision")
@export var view_range = 30.0
@export var view_angle = 45.0  # half the camera's field of view, in degrees
@export var always_see_victim = true  # Session 1 (learn to fly): true. Session 2 and final test: false.

# --- Rescue ---
@export_group("Rescue")
@export var rescue_distance = 2.0

@onready var ai_controller = $AIController3D
@onready var env = get_parent()
@onready var teacher = $OracleTeacher # Trains the model through behavior cloning

var move_input = Vector3.ZERO  # x = strafe, y = up/down, z = forward/back
var yaw_input = 0.0
var seen_target = null
var best_distance = INF
var visited = {}
var spawn_transform


func _ready():
	ai_controller.init(self)
	spawn_transform = global_transform

	# Start with a target
	env.spawn_target()


func targets():
	# Only live Target bodies, not their collision children or ones being removed
	return get_tree().get_nodes_in_group("Target").filter(
		func(t): return t is StaticBody3D and not t.is_queued_for_deletion()
	)


func game_over():
	global_transform = spawn_transform
	velocity = Vector3.ZERO
	seen_target = null
	best_distance = INF
	visited.clear()
	for t in targets():
		t.queue_free()  # a fresh victim spawns next frame
	ai_controller.reset()


func _physics_process(delta):
	if ai_controller.needs_reset:
		game_over()
		return

	if targets().is_empty():
		env.spawn_target()  # uses the environment's own spawner

	set_input()
	fly(delta)
	look_for_target()
	update_reward(delta)

	if seen_target and global_position.distance_to(seen_target.global_position) < rescue_distance:
		rescued()
	elif get_slide_collision_count() > 0 and velocity.length() > 4.0:
		crashed()


func fly(delta):
	rotate_y(-yaw_input * yaw_speed * delta)
	var target_velocity = global_basis * Vector3(move_input.x, move_input.y, -move_input.z) * max_speed
	velocity = velocity.lerp(target_velocity, accel * delta)
	move_and_slide()

	# Visual tilt only, like a real quadcopter
	$DroneMesh.rotation.x = lerp($DroneMesh.rotation.x, -move_input.z * 0.3, 5 * delta)
	$DroneMesh.rotation.z = lerp($DroneMesh.rotation.z, -move_input.x * 0.3, 5 * delta)


func can_see(target):
	var to = target.global_position - global_position
	if to.length() > view_range:
		return false
	var look = (-global_basis.z + Vector3.DOWN * 0.7).normalized()  # camera points forward and down
	if look.angle_to(to) > deg_to_rad(view_angle):
		return false
	# Line of sight: terrain (1) and obstacles (2) block the view
	var query = PhysicsRayQueryParameters3D.create(global_position, target.global_position, 3)
	return get_world_3d().direct_space_state.intersect_ray(query).is_empty()


func look_for_target():
	seen_target = null
	var closest = INF
	for t in targets():
		var d = global_position.distance_to(t.global_position)
		if d < closest and (always_see_victim or can_see(t)):
			seen_target = t
			closest = d


func update_reward(delta: float):
	ai_controller.reward -= 0.6 * delta  # hurry up (-0.01 penalty * 60 frames = -0.6)

	# Small reward for exploring new 5 m cells, so it learns to search
	var cell = Vector3i((global_position / 5.0).floor())
	if not visited.has(cell):
		visited[cell] = true
		ai_controller.reward += 1.0 # Beats cost of exploration

	# Reward getting closer to a victim it can see
	if seen_target:
		var d = global_position.distance_to(seen_target.global_position)
		if best_distance == INF:
			best_distance = d
		elif d < best_distance:
			if best_distance < INF: # Don't add infinity
				ai_controller.reward += best_distance - d
			best_distance = d
		else:
			best_distance = INF # Don't reaward the jump


func rescued():
	ai_controller.reward += 100.0
	seen_target.queue_free()  # a fresh victim spawns next frame
	seen_target = null
	best_distance = INF


func crashed():
	ai_controller.done = true
	ai_controller.reward -= 10.0
	game_over()


func set_input():
	if ai_controller.heuristic == "model":
		return
	# Human mode: W/S forward-back, A/D strafe, Space/Shift up-down, Q/E turn
	move_input = Vector3(
		float(Input.is_key_pressed(KEY_D)) - float(Input.is_key_pressed(KEY_A)),
		float(Input.is_key_pressed(KEY_SPACE)) - float(Input.is_key_pressed(KEY_SHIFT)),
		float(Input.is_key_pressed(KEY_W)) - float(Input.is_key_pressed(KEY_S))
	)
	yaw_input = float(Input.is_key_pressed(KEY_E)) - float(Input.is_key_pressed(KEY_Q))
