#########################
# HDI control message to SDI
#########################
float64 wheel_angle	# [-5*PI,+5*PI] in radians
float64 gas_pos		# [0.0, 1.0]
float64 brake_pos	# [0.0, 1.0]
int8 gear		# -1 = reverse, 0 = neutral, 1 = forward, 2 = park
float64 vibration	# [0.0, 1.0]
