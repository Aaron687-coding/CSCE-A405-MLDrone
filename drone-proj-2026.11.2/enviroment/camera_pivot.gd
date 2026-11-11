extends Node3D

@export var mouse_sensitivity: float = 0.005
var is_dragging: bool = false

func _unhandled_input(event):
	# Track if the right mouse button is being held down
	if event is InputEventMouseButton:
		if event.button_index == MOUSE_BUTTON_RIGHT:
			is_dragging = event.pressed
			
	# Apply rotation directly to the properties if dragging
	if event is InputEventMouseMotion and is_dragging:
		# Subtract the relative mouse motion directly from the rotation axes
		rotation.y -= event.relative.x * mouse_sensitivity
		rotation.x -= event.relative.y * mouse_sensitivity
		
		# Clamp the up/down rotation so it doesn't flip over
		rotation.x = clamp(rotation.x, deg_to_rad(-80), deg_to_rad(0))
		
		# Lock the Z-axis rotation to 0 to guarantee it never rolls
		rotation.z = 0
