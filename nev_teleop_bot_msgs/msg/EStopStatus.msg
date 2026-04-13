# Emergency stop status
bool is_estop                    # E-stop active
int8 bridge_flag                 # net_bridge flag (0:normal, 1:server_cmd, 2:socket_error, 3:heartbeat_timeout, 4:control_timeout)
int8 mux_flag                    # mux_node flag (0:normal, 1:remote_mode+nav_active+no_teleop)
