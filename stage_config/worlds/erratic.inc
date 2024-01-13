define block model
(
  size [0.5 0.5 0.5]
  gui_nose 0
)

define topurg ranger
(
	sensor( 			
    range [ 0.0  30.0 ]
    fov 240
    samples 958
  )

  # generic model properties
  color "black"
  size [ 0.05 0.05 0.1 ]
)

define mycamera camera
(
	range [ 0.2 8.0 ]
	resolution [ 640 480 ]
	fov [ 60 49.5 ]
	pantilt [ 0 0 ]
	alwayson 1
)

define erratic position
(
  size [0.3 0.3 0.2]
  origin [0.0 0 0 0]
  gui_nose 1
  drive "diff"
  topurg(pose [ 0.050 0.000 0 0.000 ])
  mycamera(pose [ 0 0 0 90.0 ])
)
