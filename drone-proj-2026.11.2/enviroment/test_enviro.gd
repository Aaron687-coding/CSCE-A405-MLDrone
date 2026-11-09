extends Node3D
@export var Target: PackedScene

func spawn_target():
	#Grab all the TargetSpawn Marker3D nodes  
	var spawn_points = get_tree().get_nodes_in_group("TargetSpawns")
	
	#Safety check
	if spawn_points.is_empty():
		push_error("No spawn points found in the map!")
		return
		
	#Pick a random marker
	var chosen_spawn = spawn_points.pick_random()
	
	#Instantiate the target and place it
	var target_instance = Target.instantiate()
	add_child(target_instance)
	
	#Lock it to the marker's global position
	target_instance.global_position = chosen_spawn.global_position
	
func _input(event):
	if event.is_action_pressed("ui_accept"):
		spawn_target()
