include "fd.lua"

TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}
TRAJECTORY_BUILDER_2D.use_imu_data = false
POSE_GRAPH.optimize_every_n_nodes = 20
tracking_frame = "base_link"

return options
