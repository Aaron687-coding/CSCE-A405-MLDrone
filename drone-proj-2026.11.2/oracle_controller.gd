extends RigidBody3D
## oracle_drone.gd -- privileged ("cheating") baseline drone. Self-driving.
##
## Knows the target's world position; discovers obstacles with raycasts only.
## Purpose: prove the action space / motor mapping / control rate / episode reset
## work, and give an UPPER BOUND to compare a PPO agent against.
##
## It is NOT a fair opponent: it can see the target through walls.
## For a same-plant or fair baseline use oracle_teacher.gd instead.
##
## Node setup:
##   - this node is a RigidBody3D with a CollisionShape3D child (its own body)
##   - put it on its own collision layer, and keep that layer OUT of drone_ref's mask
##   - obstacle_mask must include terrain + obstacles (3 in the drone.gd project)
##   - can_sleep is disabled in _ready so probes are never skipped while resting

signal reached_target
signal escapes_local_minimum

@export_group("References")
## Optional: your real drone. Used only for the spawn pose and RID exclusion.
@export var drone_ref: CharacterBody3D
## Optional: leave empty and the nearest live node in the "Target" group is used.
@export var target: StaticBody3D

@export_group("Probing")
@export_flags_3d_physics var obstacle_mask: int = 3   # terrain (1) + obstacles (2)
@export var drone_radius: float = 0.75				  # >= real collision radius
@export var probe_length: float = 6.0
@export var ground_probe: float = 8.0
@export var replan_every: int = 2

@export_group("Flight")
@export var cruise_speed: float = 40.0                # *5 because the sim runs really slowly
@export var arrive_distance: float = 2.0              # match drone.rescue_distance
@export var hover_clearance: float = 1.5
@export var max_turn_rate: float = 3.0
@export var kp: float = 6.0
@export var max_force: float = 40.0
@export var compensate_gravity: bool = true

@export_group("Avoidance")
@export var fan_deg: PackedFloat32Array = PackedFloat32Array([20.0, 40.0, 60.0, 80.0, 100.0, 130.0])
@export var w_clearance: float = 4.0
@export var w_deviation: float = 1.0
@export var w_side: float = 0.35
@export var side_commit_time: float = 1.0
@export var los_confirm_time: float = 0.35
@export var stuck_timeout: float = 3.0
@export var progress_epsilon: float = 0.5

@export_group("Baseline metric")
@export var reward_scale: float = 10.0
@export var arrival_bonus: float = 5.0
@export var rng_seed: int = 12345

@export_group("Active")
@export var active: bool = true
## Teleport to drone_ref's position on _ready (keeps the two clips aligned).
@export var spawn_on_drone: bool = true

@onready var env = get_parent()

enum QMode { SEEK, AVOID }

var _mode: QMode = QMode.SEEK
var _desired := Vector3.FORWARD
var _wanted := Vector3.FORWARD
var _side: float = 1.0
var _side_timer: float = 0.0
var _los_timer: float = 0.0
var _stuck_timer: float = 0.0
var _frame: int = 0
var _dist: float = 0.0
var _best_dist: float = INF
var _prev_dist: float = 0.0
var _total_reward: float = 0.0
var _arrived: bool = false
var _rng := RandomNumberGenerator.new()
var _exclude: Array[RID] = []
var _tgt: StaticBody3D = null


func _ready() -> void:
	_rng.seed = rng_seed
	can_sleep = false                       # a resting body would skip probes

	if drone_ref != null and spawn_on_drone:
		# Safe here: the physics server has not started stepping yet.
		global_position = drone_ref.global_position

	if not active:
		set_physics_process(false)
		return

	_tgt = _resolve_target()
	# Always seed, whether or not a target exists yet: otherwise frame 1 books
	# a huge bogus reward of (0 - dist).
	_prev_dist = _dist_to(_tgt)
	_best_dist = _prev_dist


func _physics_process(delta: float) -> void:
	_frame += 1

	var tgt: StaticBody3D = _resolve_target()
	_tgt = tgt
	if tgt == null:
		_drive(Vector3.ZERO)               # idle, no target to chase
		return

	# self + victim + the reference drone: a co-located body would otherwise
	# hit every probe and every direction would read "blocked".
	_exclude = [get_rid(), tgt.get_rid()]
	if drone_ref != null:
		_exclude.append(drone_ref.get_rid())

	var origin: Vector3 = global_position
	var to_target: Vector3 = tgt.global_position - origin
	_dist = to_target.length()

	_total_reward += (_prev_dist - _dist) * reward_scale
	_prev_dist = _dist
	_side_timer = maxf(0.0, _side_timer - delta)

	if _dist <= arrive_distance:
		if not _arrived:
			# _arrived = true
			_total_reward += arrival_bonus
			reached_target.emit()

			# Queue free the target and spawn a new one
			tgt.queue_free()
			env.spawn_target()
			_tgt = _resolve_target() # Select new target
		_drive(Vector3.ZERO)
		return

	_update_stuck(delta, origin)

	var goal: Vector3 = to_target.normalized()
	if _line_of_sight(origin, goal):
		_los_timer += delta
		if _los_timer >= los_confirm_time:
			_mode = QMode.SEEK
	else:
		_los_timer = 0.0
		_mode = QMode.AVOID

	if _frame % maxi(1, replan_every) == 0:
		if _mode == QMode.SEEK:
			_wanted = goal
		else:
			_wanted = _choose_avoid_dir(origin, goal)

	_vertical(origin)
	_steer(delta)
	_drive(_desired * _speed_scale())


# --- control surface ---------------------------------------------------------

func set_active(on: bool) -> void:
	active = on
	set_physics_process(on)
	if on:
		sleeping = false
	else:
		_drive(Vector3.ZERO)


# --- public API, for baseline logging ---------------------------------------

func total_reward() -> float:
	return _total_reward


func distance_to_target() -> float:
	return _dist


func has_arrived() -> bool:
	return _arrived


func reset_baseline() -> void:
	_total_reward = 0.0
	_stuck_timer = 0.0
	_los_timer = 0.0
	_side_timer = 0.0
	_best_dist = INF
	_arrived = false
	_mode = QMode.SEEK
	_desired = Vector3.FORWARD
	_wanted = Vector3.FORWARD
	_tgt = _resolve_target()
	_prev_dist = _dist_to(_tgt)
	_best_dist = _prev_dist


# --- target resolution -------------------------------------------------------

func _resolve_target() -> StaticBody3D:
	if target != null and is_instance_valid(target) and not target.is_queued_for_deletion():
		return target
	# Fallback: nearest live target, so a freed victim cannot crash the run.
	var best: StaticBody3D = null
	var best_d := INF
	for t in get_tree().get_nodes_in_group("Target"):
		if not (t is StaticBody3D) or t.is_queued_for_deletion():
			continue
		var d: float = global_position.distance_squared_to(t.global_position)
		if d < best_d:
			best_d = d
			best = t
	return best


func _dist_to(tgt: StaticBody3D) -> float:
	if tgt == null:
		return 0.0
	return global_position.distance_to(tgt.global_position)


# --- control ----------------------------------------------------------------

func _speed_scale() -> float:
	# floor 0.0: a 0.25 floor leaves 2 m/s at the goal and the drone orbits it
	return clampf(_dist / (arrive_distance * 4.0), 0.0, 1.0)


func _steer(delta: float) -> void:
	if _desired.length_squared() < 0.0001:
		_desired = Vector3.FORWARD
	var ang: float = _desired.angle_to(_wanted)
	var step: float = max_turn_rate * delta
	if ang > step and ang > 0.0001:
		_desired = _desired.slerp(_wanted, step / ang).normalized()
	else:
		_desired = _wanted


func _drive(vel_target: Vector3) -> void:
	var force: Vector3 = (vel_target - linear_velocity) * kp * mass
	if compensate_gravity:
		var gdir: Vector3 = ProjectSettings.get_setting("physics/3d/default_gravity_vector", Vector3.DOWN)
		var gmag: float = float(ProjectSettings.get_setting("physics/3d/default_gravity", 9.8))
		force -= gdir.normalized() * gmag * mass
	if force.length() > max_force:
		force = force.normalized() * max_force
	apply_central_force(force)


## Up-bias when the floor is close. Applied to BOTH _wanted and _desired so the
## turn-rate limiter does not immediately undo it.
func _vertical(origin: Vector3) -> void:
	var q := PhysicsRayQueryParameters3D.create(
		origin, origin + Vector3.DOWN * ground_probe, obstacle_mask, _exclude)
	var hit: Dictionary = get_world_3d().direct_space_state.intersect_ray(q)
	var free_down: float = ground_probe
	if not hit.is_empty():
		free_down = origin.distance_to(hit["position"])
	if free_down < hover_clearance:
		var push: float = (1.0 - free_down / maxf(hover_clearance, 0.01)) * 2.0
		_wanted = (_wanted + Vector3.UP * push).normalized()
		_desired = (_desired + Vector3.UP * push).normalized()
	
	# Render intersect_ray with extension
	if hit:
		DebugDraw3D.draw_line(q.from, hit.position, Color.GREEN)
		DebugDraw3D.draw_sphere(hit.position, 0.1, Color.YELLOW)
	else:
		DebugDraw3D.draw_line(q.from, q.to, Color.RED)


# --- decision ---------------------------------------------------------------

func _choose_avoid_dir(origin: Vector3, goal: Vector3) -> Vector3:
	if _side_timer <= 0.0:
		_side = _pick_side(origin, goal)
		_side_timer = side_commit_time

	var ax := _axes_for(goal)
	var look: float = clampf(_dist, 1.0, probe_length)
	var signs: PackedFloat32Array = PackedFloat32Array([1.0, -1.0])

	var best: Vector3 = goal
	var best_cost: float = INF
	for deg in fan_deg:
		var a: float = deg_to_rad(deg)
		for s in signs:
			var yaw_c: Vector3 = goal.rotated(ax[1], a * s)
			var c: float = _score_dir(origin, yaw_c, goal, look, s)
			if c < best_cost:
				best_cost = c
				best = yaw_c
			var pitch_c: Vector3 = goal.rotated(ax[0], a * 0.5 * s)
			c = _score_dir(origin, pitch_c, goal, look, s)
			if c < best_cost:
				best_cost = c
				best = pitch_c
	return best.normalized()


func _score_dir(origin: Vector3, cand: Vector3, goal: Vector3, look: float, sign: float) -> float:
	var deviation: float = rad_to_deg(cand.angle_to(goal)) / 90.0
	var clearance: float = _corridor_free(origin, cand, look)
	var cost: float = deviation * w_deviation + (1.0 - clearance) * w_clearance
	if sign != _side:
		cost += w_side
	return cost


func _pick_side(origin: Vector3, goal: Vector3) -> float:
	var ax := _axes_for(goal)
	var look: float = clampf(_dist, 1.0, probe_length)
	var left: float = _corridor_free(origin, goal.rotated(ax[1], PI * 0.5), look)
	var right: float = _corridor_free(origin, goal.rotated(ax[1], -PI * 0.5), look)
	return 1.0 if left >= right else -1.0


func _update_stuck(delta: float, origin: Vector3) -> void:
	if _dist < _best_dist - progress_epsilon:
		_best_dist = _dist
		_stuck_timer = 0.0
		return
	_stuck_timer += delta
	if _stuck_timer < stuck_timeout:
		return
	_stuck_timer = 0.0
	_best_dist = _dist
	_side = -_side
	_side_timer = side_commit_time * 2.0
	var goal: Vector3 = (_tgt.global_position - origin).normalized()
	var ax := _axes_for(goal)
	var a: float = _rng.randf_range(PI * 0.5, PI)
	_wanted = goal.rotated(ax[1], a * _side).normalized()
	_desired = _wanted
	escapes_local_minimum.emit()


# --- geometry ---------------------------------------------------------------

## Orthonormal frame perpendicular to dir: [right, up].
func _axes_for(dir: Vector3) -> Array[Vector3]:
	var d: Vector3 = dir.normalized()
	var ref: Vector3 = Vector3.UP
	if absf(d.dot(ref)) > 0.95:
		ref = Vector3.RIGHT
	var right: Vector3 = d.cross(ref).normalized()
	var up: Vector3 = right.cross(d).normalized()
	var out: Array[Vector3] = [right, up]
	return out


func _line_of_sight(origin: Vector3, goal: Vector3) -> bool:
	if _tgt == null:
		return false
	var look: float = clampf(_dist, 1.0, probe_length)
	if _corridor_free(origin, goal, look) < 0.99:
		return false
	var start: Vector3 = origin + goal * drone_radius * 0.5
	return _free_frac(start, _tgt.global_position) >= 0.99


## 0..1 free fraction of a corridor length long from origin along dir,
## tested with 5 offset rays so the whole body fits, not just the centre line.
func _corridor_free(origin: Vector3, dir: Vector3, length: float) -> float:
	var ax := _axes_for(dir)
	var start: Vector3 = origin + dir * drone_radius * 0.5
	var end: Vector3 = origin + dir * length
	var offsets: Array[Vector3] = [
		Vector3.ZERO,
		ax[0] * drone_radius, -ax[0] * drone_radius,
		ax[1] * drone_radius * 0.75, -ax[1] * drone_radius * 0.75,
	]
	var worst: float = 1.0
	for o in offsets: # Worst of up, down, right, left
		worst = minf(worst, _free_frac(start + o, end + o))
	
	return worst

# Creates a single raycast between from and to and returning whether the ray hit something
func _free_frac(from: Vector3, to: Vector3) -> float:
	var span: float = from.distance_to(to)
	if span < 0.001:
		return 1.0
	var q := PhysicsRayQueryParameters3D.create(from, to, obstacle_mask, _exclude)
	var hit: Dictionary = get_world_3d().direct_space_state.intersect_ray(q)

	# Render intersect_ray with extension
	if hit:
		DebugDraw3D.draw_line(q.from, hit.position, Color.GREEN)
		DebugDraw3D.draw_sphere(hit.position, 0.1, Color.YELLOW)
	else:
		DebugDraw3D.draw_line(q.from, q.to, Color.RED)

	# Calculate hit
	if hit.is_empty():
		return 1.0
	return clampf(from.distance_to(hit["position"]) / span, 0.0, 1.0)
