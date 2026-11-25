extends Node3D
## oracle_teacher.gd -- uses the oracle's obstacle-aware decision as a TEACHER.
##
## Unlike oracle_controller.gd this applies no forces: it only produces
## `move_input` / `yaw_input` labels for a CharacterBody3D drone, so it can
## supervise the same action space the policy outputs.
##
## Two uses:
##   teacher_step(drone)                     -> {"move", "yaw", "teaching"}   labels (BC / DAgger)
##   coach_bonus(agent_move, agent_yaw)      -> float                         annealed PPO shaping
##
## Node setup: child of the drone scene, sibling of AIController3D.
## Set `target` (or leave null and it reads drone.seen_target).

signal label_recorded(obs: Array, act: Array)

@export_group("References")
@export var target: StaticBody3D                       # null -> use drone.seen_target

@export_group("Probing")
@export_flags_3d_physics var obstacle_mask: int = 3    # terrain + obstacles
@export var drone_radius := 0.8
@export var probe_length := 6.0

@export_group("Decision")
@export var fan_deg := PackedFloat32Array([20.0, 40.0, 60.0, 80.0, 100.0, 130.0])
@export var w_clearance := 4.0
@export var w_deviation := 1.0
@export var w_side := 0.35
@export var side_commit_time := 1.0
@export var los_confirm_time := 0.35
@export var stuck_timeout := 3.0
@export var progress_epsilon := 0.5
@export var arrive_distance := 2.0

@export_group("Teaching")
## HARD RULE: only label frames where the AGENT can also see the target.
## Otherwise you clone privileged behaviour the observation cannot justify.
@export var gate_on_visibility := true

@export_group("Coaching reward")
@export var coach_weight := 0.02
@export var coach_decay_per_episode := 0.002
@export var coach_min_weight := 0.0
## Only shape while the agent can see its target; silence otherwise.
@export var coach_only_when_visible := true

@export_group("Recording (behaviour cloning)")
@export var record_path := ""                          # e.g. "user://demos.jsonl"

var _desired := Vector3.FORWARD
var _wanted := Vector3.FORWARD
var _side := 1.0
var _side_timer := 0.0
var _los_timer := 0.0
var _best_dist := INF
var _stuck_timer := 0.0
var _frame := 0
var _weight := 0.0
var _last_move := Vector3.ZERO
var _last_yaw := 0.0
var _teaching := false
var _file: FileAccess = null
var _rng := RandomNumberGenerator.new()


func _ready() -> void:
	_rng.seed = 4242
	_weight = coach_weight
	if record_path != "":
		_file = FileAccess.open(record_path, FileAccess.WRITE)
		if _file == null:
			push_warning("oracle_teacher: cannot open %s" % record_path)


func _notification(what: int) -> void:
	if what == NOTIFICATION_PREDELETE and _file != null:
		_file.close()


# --- public API -------------------------------------------------------------

## Call once per physics frame, BEFORE set_input() uses the result / coach_bonus().
func teacher_step(drone: Node3D) -> Dictionary:
	_frame += 1
	_last_move = Vector3.ZERO
	_last_yaw = 0.0
	_teaching = false

	var tgt := _resolve_target(drone)
	if tgt == null or not _may_teach(drone, tgt):
		return {"move": _last_move, "yaw": _last_yaw, "teaching": false}

	var origin: Vector3 = drone.global_position
	var to_t: Vector3 = tgt.global_position - origin
	var dist := to_t.length()
	var goal := to_t.normalized()

	if dist <= arrive_distance:
		_last_move = Vector3.ZERO
		_last_yaw = _yaw_label(drone, goal)
		_teaching = true
		return {"move": _last_move, "yaw": _last_yaw, "teaching": true}

	_update_stuck(drone, dist, goal)

	if _line_of_sight(origin, goal, dist):
		_los_timer += get_physics_process_delta_time()
	else:
		_los_timer = 0.0

	if _frame % 2 == 0:
		_wanted = goal if _los_timer >= los_confirm_time else _choose_dir(origin, goal, dist)

	_steer(get_physics_process_delta_time())
	_last_move = _to_move_input(drone, _desired * _speed_scale(dist))
	_last_yaw = _yaw_label(drone, _desired)
	_teaching = true

	if _file != null and drone.has_method("get_node") and drone.get_node_or_null("AIController3D") != null:
		var ai := drone.get_node("AIController3D")
		var obs: Array = ai.get_obs()["obs"]
		_record(obs, _last_move, _last_yaw)

	return {"move": _last_move, "yaw": _last_yaw, "teaching": true}


## Annealed action-similarity shaping. Decays every episode to 0, so the final
## policy optimises your real reward and can outgrow the teacher.
func coach_bonus(agent_move: Vector3, agent_yaw: float, target_visible: bool) -> float:
	if _weight <= coach_min_weight or not _teaching:
		return 0.0
	if coach_only_when_visible and not target_visible:
		return 0.0
	var m := agent_move.normalized() if agent_move.length_squared() > 1e-6 else Vector3.ZERO
	var t := _last_move.normalized()
	var align := m.dot(t) if t.length_squared() > 1e-6 else 0.0
	var yaw_align := 1.0 - absf(agent_yaw - _last_yaw) * 0.5
	return _weight * (align * 0.8 + yaw_align * 0.2)


func on_episode_end() -> void:
	_weight = maxf(coach_min_weight, _weight - coach_decay_per_episode)


func current_weight() -> float:
	return _weight


func teaching() -> bool:
	return _teaching


# --- this drone's action interface -----------------------------------------

## fly() does: global_basis * Vector3(x, y, -z) * max_speed, so invert that.
func _to_move_input(drone: Node3D, world_vel: Vector3) -> Vector3:
	var local: Vector3 = drone.global_basis.inverse() * world_vel
	return Vector3(local.x, local.y, -local.z).limit_length(1.0)


## fly() does rotate_y(-yaw_input * yaw_speed * delta), so the label is negated.
func _yaw_label(drone: Node3D, world_dir: Vector3) -> float:
	var fwd: Vector3 = -drone.global_basis.z
	var flat := Vector3(world_dir.x, 0.0, world_dir.z)
	if flat.length_squared() < 0.0025:
		return 0.0
	return clampf(-fwd.signed_angle_to(flat.normalized(), Vector3.UP) * 2.0, -1.0, 1.0)


func _speed_scale(dist: float) -> float:
	return clampf(dist / (arrive_distance * 4.0), 0.25, 1.0)


# --- gating -----------------------------------------------------------------

func _resolve_target(drone: Node3D) -> StaticBody3D:
	if target != null and is_instance_valid(target):
		return target
	var seen = drone.get("seen_target")
	if seen is StaticBody3D and is_instance_valid(seen):
		return seen
	return null


func _may_teach(drone: Node3D, tgt: StaticBody3D) -> bool:
	if not gate_on_visibility:
		return true
	# Only when the agent's own observation puts the target in view.
	var seen = drone.get("seen_target")
	return seen != null and is_instance_valid(seen) and seen == tgt


# --- decision (same algorithms as oracle_controller.gd, no force layer) -----

func _choose_dir(origin: Vector3, goal: Vector3, dist: float) -> Vector3:
	if _side_timer <= 0.0:
		_side = _pick_side(origin, goal, dist)
		_side_timer = side_commit_time
	var ax := _axes_for(goal)
	var look := clampf(dist, 1.0, probe_length)
	var best := goal
	var best_cost := INF
	for deg in fan_deg:
		var a := deg_to_rad(deg)
		for s in [1.0, -1.0]:
			var yaw_c: Vector3 = goal.rotated(ax[1], a * s)
			var c := _score(origin, yaw_c, goal, look, s)
			if c < best_cost:
				best_cost = c
				best = yaw_c
			var pitch_c: Vector3 = goal.rotated(ax[0], a * 0.5 * s)
			c = _score(origin, pitch_c, goal, look, s)
			if c < best_cost:
				best_cost = c
				best = pitch_c
	return best.normalized()


func _score(origin: Vector3, cand: Vector3, goal: Vector3, look: float, sign: float) -> float:
	var dev := rad_to_deg(cand.angle_to(goal)) / 90.0
	var cost := dev * w_deviation + (1.0 - _corridor_free(origin, cand, look)) * w_clearance
	if sign != _side:
		cost += w_side
	return cost


func _pick_side(origin: Vector3, goal: Vector3, dist: float) -> float:
	var ax := _axes_for(goal)
	var look := clampf(dist, 1.0, probe_length)
	var l := _corridor_free(origin, goal.rotated(ax[1], PI * 0.5), look)
	var r := _corridor_free(origin, goal.rotated(ax[1], -PI * 0.5), look)
	return 1.0 if l >= r else -1.0


func _steer(delta: float) -> void:
	var ang := _desired.angle_to(_wanted)
	var step := 3.0 * delta
	_desired = _desired.slerp(_wanted, step / ang).normalized() if ang > step and ang > 1e-4 else _wanted


func _update_stuck(drone: Node3D, dist: float, goal: Vector3) -> void:
	if dist < _best_dist - progress_epsilon:
		_best_dist = dist
		_stuck_timer = 0.0
		return
	_stuck_timer += get_physics_process_delta_time()
	if _stuck_timer < stuck_timeout:
		return
	_stuck_timer = 0.0
	_best_dist = dist
	_side = -_side
	_side_timer = side_commit_time * 2.0
	_wanted = goal.rotated(_axes_for(goal)[1], _rng.randf_range(PI * 0.5, PI) * _side).normalized()
	_desired = _wanted


# --- geometry ---------------------------------------------------------------

func _axes_for(dir: Vector3) -> Array[Vector3]:
	var d := dir.normalized()
	var ref := Vector3.UP
	if absf(d.dot(ref)) > 0.95:
		ref = Vector3.RIGHT
	var right := d.cross(ref).normalized()
	var up := right.cross(d).normalized()
	var out: Array[Vector3] = [right, up]
	return out


func _line_of_sight(origin: Vector3, goal: Vector3, dist: float) -> bool:
	var look := clampf(dist, 1.0, probe_length)
	if _corridor_free(origin, goal, look) < 0.99:
		return false
	return _free_frac(origin + goal * drone_radius * 0.5, origin + goal * dist) >= 0.99


func _corridor_free(origin: Vector3, dir: Vector3, length: float) -> float:
	var ax := _axes_for(dir)
	var start := origin + dir * drone_radius * 0.5
	var end := origin + dir * length
	var worst := 1.0
	for o in [Vector3.ZERO, ax[0] * drone_radius, -ax[0] * drone_radius,
			ax[1] * drone_radius * 0.75, -ax[1] * drone_radius * 0.75]:
		worst = minf(worst, _free_frac(start + o, end + o))
	return worst


func _free_frac(from: Vector3, to: Vector3) -> float:
	var span := from.distance_to(to)
	if span < 0.001:
		return 1.0
	var q := PhysicsRayQueryParameters3D.create(from, to, obstacle_mask, [_self_rid()])
	var hit := get_world_3d().direct_space_state.intersect_ray(q)
	return 1.0 if hit.is_empty() else clampf(from.distance_to(hit["position"]) / span, 0.0, 1.0)


func _self_rid() -> RID:
	var p := get_parent()
	return p.get_rid() if p is CollisionObject3D else RID()


# --- recording --------------------------------------------------------------

func _record(obs: Array, move: Vector3, yaw: float) -> void:
	var row := {"obs": Array(obs), "act": [move.x, move.y, move.z, yaw]}
	_file.store_line(JSON.stringify(row))
	label_recorded.emit(obs, row["act"])
