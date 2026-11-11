extends Node3D

# Physics layers used in this project
# 1 = terrain, 2 = obstacles (trees/structures), 4 = drone, 8 = target
const GROUND_MASK: int = 1
const BLOCKING_MASK: int = 1 | 2 | 4 | 8

const ATTEMPTS_PER_POINT: int = 10
const MIN_GROUND_NORMAL_Y: float = 0.6 # Reject extremely steep slopes (cos(53 degrees) ≈ 0.6)
const GROUND_CLEARANCE: float = 0.1 # Small lift so the target doesn't overlap the ground it rests on

@export var bounding_box: CollisionShape3D # Must use a BoxShape3D, defines the spawn volume -> SpawnArea in test_enviro.tscn
@export var Target: PackedScene

var X_MIN_BOUND: float
var X_MAX_BOUND: float
var Y_MIN_BOUND: float
var Y_MAX_BOUND: float
var Z_MIN_BOUND: float
var Z_MAX_BOUND: float

var spawn_points_array: Array[Vector3] = []

var _bounds_ready: bool = false

# Initialize bounding box and bounds
func _ready():
	if bounding_box == null:
		push_warning("bounding_box is not assigned, procedural spawn points are disabled (falling back to TargetSpawns markers).")
		return

	var box_shape = bounding_box.shape as BoxShape3D

	if not box_shape:
		push_error("Bounding box must be a BoxShape3D.")
		return

	# global_basis.get_scale() captures scaling on both this node AND its parents (but no rotation)
	var half_extents: Vector3 = box_shape.size * bounding_box.global_basis.get_scale().abs() / 2.0
	var center: Vector3 = bounding_box.global_position

	X_MIN_BOUND = center.x - half_extents.x
	X_MAX_BOUND = center.x + half_extents.x
	Y_MIN_BOUND = center.y - half_extents.y
	Y_MAX_BOUND = center.y + half_extents.y
	Z_MIN_BOUND = center.z - half_extents.z
	Z_MAX_BOUND = center.z + half_extents.z

	_bounds_ready = true

# Builds a physics shape matching the target's collision shape, with the target's node scale baked in
func _build_target_shape() -> CapsuleShape3D:
	var target_instance = Target.instantiate()
	var collision = target_instance.get_node_or_null("TargetCollision") as CollisionShape3D
	var capsule: CapsuleShape3D = null

	if collision and collision.shape is CapsuleShape3D:
		var source = collision.shape as CapsuleShape3D
		capsule = CapsuleShape3D.new()
		capsule.height = source.height * collision.scale.y
		capsule.radius = source.radius * collision.scale.x

	# The instance was never added to the tree, so free it manually
	target_instance.free()
	return capsule

# Procedurally creates spawn points for the target
func generate_spawn_points(min_x: float, max_x: float, min_z: float, max_z: float, num_points: int = 10) -> Array[Vector3]:
	spawn_points_array.clear()

	if not _bounds_ready:
		push_error("Cannot generate spawn points: bounding_box is missing or invalid.")
		return spawn_points_array

	# Prepare the target shape and the world space
	var target_shape = _build_target_shape()
	if target_shape == null:
		push_error("Target must have a CapsuleShape3D child named TargetCollision.")
		return spawn_points_array

	var half_height: float = target_shape.height / 2.0

	var space_state = get_world_3d().direct_space_state

	var shape_query = PhysicsShapeQueryParameters3D.new()
	shape_query.shape = target_shape
	shape_query.collision_mask = BLOCKING_MASK

	# Each point is given ten attempts to find a valid spawn location
	for i in range(num_points * ATTEMPTS_PER_POINT):
		if spawn_points_array.size() >= num_points:
			break

		var x = randf_range(min_x, max_x)
		var z = randf_range(min_z, max_z)

		# Start with a raycast from the top of the bounding box down to find the ground level
		var origin: Vector3 = Vector3(x, Y_MAX_BOUND, z)
		var end_pos: Vector3 = Vector3(x, Y_MIN_BOUND, z)

		var ray_query = PhysicsRayQueryParameters3D.create(origin, end_pos, GROUND_MASK)
		var ray_result = space_state.intersect_ray(ray_query)

		# Missed everything (empty dictionary), try another column
		if ray_result.is_empty():
			continue

		# Only accept reasonably flat, upward facing ground
		if ray_result.normal.y < MIN_GROUND_NORMAL_Y:
			continue

		var hit_position: Vector3 = ray_result.position
		var candidate: Vector3 = hit_position + Vector3(0, half_height + GROUND_CLEARANCE, 0)

		# Check that the target would not overlap anything (trees, structures, drone, other targets) there
		shape_query.transform = Transform3D(Basis(), candidate)
		var overlaps = space_state.intersect_shape(shape_query, 1)

		if overlaps.is_empty():
			spawn_points_array.append(candidate)

	# Confirm
	print("Generated ", spawn_points_array.size(), " spawn points: ", spawn_points_array)

	# Warn user if we didn't generate enough spawn points
	if spawn_points_array.size() < num_points:
		push_warning("Could only generate " + str(spawn_points_array.size()) + " valid spawn points out of requested " + str(num_points) + ".")

	return spawn_points_array


func spawn_target(use_random_spawn: bool = true):
	var chosen_spawn_position: Vector3

	if use_random_spawn and _bounds_ready:
		# Spawn points are generated once over the full bounding box and then reused
		if spawn_points_array.is_empty():
			generate_spawn_points(X_MIN_BOUND, X_MAX_BOUND, Z_MIN_BOUND, Z_MAX_BOUND, 10) # Default number of points

		if spawn_points_array.is_empty():
			push_error("Could not generate any valid spawn points!")
			return

		# Global position of a random generated spawn point
		chosen_spawn_position = spawn_points_array.pick_random()

	else:
		if use_random_spawn:
			push_warning("Random spawning unavailable (no valid bounding_box), using TargetSpawns markers.")

		#Grab all the TargetSpawn Marker3D nodes
		var markers: Array[Node] = get_tree().get_nodes_in_group("TargetSpawns")

		#Safety check
		if markers.is_empty():
			push_error("No spawn points found in the map!")
			return

		# Get the global position of a random spawn point from the list of markers
		chosen_spawn_position = (markers.pick_random() as Node3D).global_position

	#Instantiate the target and place it
	var target_instance = Target.instantiate()
	add_child(target_instance)

	#Lock it to the chosen global position
	target_instance.global_position = chosen_spawn_position

func _input(event):
	if event.is_action_pressed("ui_accept"):
		spawn_target()

