# Mux node status
int8 mode
int8 cmd_source                  # Final selected source (0:cmd_vel, 1:teleop_cmd, -1:none)
bool remote_status               # Remote control allowed
bool nav_active                  # cmd_vel topic active
bool teleop_active               # teleop_cmd topic active
bool final_active                # final_cmd being published
