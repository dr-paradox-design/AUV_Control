function cfg = config_real_log()
% CONFIG_REAL_LOG  config_estimator with the values the 2026-11-05 recorded log supports.
% Tags: [tf_static] from the log's /tf_static (hand-entered, not a hull measurement);
%       [log] measured from the log; [baseline] same value as the real_data/ Python EKF (variant E);
%       [ASSUMED] not checked against anything. Everything not set here keeps config_estimator's value.
DEG = pi / 180;
cfg = config_estimator();

cfg.imu.rate_hz = 80.0;                      % [log] /imu/data rate
cfg.imu.accel_nd = 20 * cfg.imu.accel_nd;    % [NIS-tuned] effective noise of a hand-moved vehicle, not the
                                             % sensor: x1 gives DVL median NIS 4.93, x15 2.63, x20 2.12, x30 1.43
                                             % (target 2.37). Tuned on NIS only, never on agreement with odom.

cfg.dvl.lever = [-0.180; 0.0; 0.109];        % [tf_static] base_link -> dvl_link, converted FLU -> FRD
cfg.dvl.R_bd  = eye(3);                      % [tf_static] identity rotation
cfg.dvl.sigma = 0.05;                        % [baseline] driver reports a constant 0.1 m/s, uninformative

cfg.depth.lever      = [-0.400; 0.0; 0.0];   % [tf_static] base_link -> pressure_link, FLU -> FRD
cfg.depth.sigma      = 0.080;                % [log] std(first difference)/sqrt(2); includes surface bobbing
cfg.depth.adc_counts = Inf;                  % [ASSUMED] no quantisation term: sensor resolution unknown

cfg.heading.rate_hz = 10.0;                  % [ASSUMED] VN-200 yaw decimated to 10 Hz as the heading aid
cfg.heading.sigma   = 1.3 * DEG;             % [NIS-tuned] 2.0 deg gave median NIS 0.21, 1.3 deg 0.37 (target 0.45).
                                             % VN-200 yaw is its own filter output, so not white noise.

cfg.zero_vel.sigma = 0.1;                    % [baseline] v ~ 0 when DVL silent > 0.5 s (unpowered log only)
cfg.zero_vel.stale_after = 0.5;              % [baseline] s
cfg.zero_vel.period = 0.2;                   % [baseline] s

cfg.init.pos = 0.05;                         % [ASSUMED] start defines the origin; depth from first reading
end
