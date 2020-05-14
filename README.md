# ros2_timer_latency_measurement
## Experiments

### Comparison of nanosleep + spin_some and timer callback execution latency in spin
![nanosleep_wakeup_latency_ts_vs_rr](./result/nanosleep_wakeup_latency_histgram.png)

### Comparison of the waking latency of nanosleeps in TS and RR with child threads
![create_wall_timer_callback_latency_spin_some_vs_spin](./result/create_wall_timer_callback_latency_histgram.png)
