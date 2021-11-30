# Tuning params note

## Before tuning
#### check robot model
- calibration
- `footprint` or `robot_radius`
#### check sensors
static transform publisher
`raytrace_range`, `obstacle_range`
### check testing cases
* every goals
* avoid people


### costamap params
- `inflation_radius` and `cost_scaling_factor`

### local planner params
`max_vel_x`, `max_trans_vel`, `max_rot_vel`, `acc_lim_x`, 
