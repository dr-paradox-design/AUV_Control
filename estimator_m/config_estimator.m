function cfg = config_estimator()
% CONFIG_ESTIMATOR  Estimator study parameters.
%
% EVERY VALUE IN THIS FILE IS ASSUMED until replaced by a measured or datasheet value.
% Each entry notes where the number came from. Replace with:
%   - VN-200 datasheet for the exact variant (rugged / SMD) and an Allan-variance test,
%   - Water Linked A50 datasheet,
%   - the analog pressure sensor datasheet and bench calibration,
%   - measured lever arms and the DVL mounting rotation.
% Same values as the frozen Python reference, estimator/config.py.
% Units: SI (m, s, rad) unless the name says otherwise. Vectors are columns.

DEG = pi / 180;
G0 = 9.80665;                               % standard gravity, only to convert "mg" specs

% --- Earth -------------------------------------------------------------------
cfg.earth.lat_deg  = 20.0;                  % ASSUMED dive-site latitude
cfg.earth.omega_ie = 7.292115e-5;           % Earth rotation rate [rad/s] (WGS-84)

% --- VN-200 IMU --------------------------------------------------------------
cfg.imu.rate_hz          = 100.0;           % ASSUMED filter rate (VN-200 up to 800 Hz)
cfg.imu.gyro_nd          = 0.0035 * DEG;    % [rad/s/sqrt(Hz)] datasheet snippet, UNVERIFIED
cfg.imu.accel_nd         = 0.14e-3 * G0;    % [m/s^2/sqrt(Hz)] datasheet snippet, UNVERIFIED
cfg.imu.gyro_bias_sigma  = 5.0 * DEG / 3600;% [rad/s] in-run bias stability, UNVERIFIED
cfg.imu.accel_bias_sigma = 0.04e-3 * G0;    % [m/s^2] in-run bias stability, UNVERIFIED
cfg.imu.bias_tau         = 300.0;           % [s] Gauss-Markov time constant, ASSUMED

% --- Water Linked DVL A50 ----------------------------------------------------
cfg.dvl.rate_hz      = 10.0;                % ASSUMED (A50 datasheet not yet read)
cfg.dvl.sigma        = 0.01;                % [m/s] per axis, ASSUMED
cfg.dvl.lever        = [0.20; 0.0; 0.15];   % [m] body FRD, ASSUMED
cfg.dvl.R_bd         = eye(3);              % body -> DVL rotation, ASSUMED aligned
cfg.dvl.min_depth    = 1.0;                 % [m] invalid near the surface, ASSUMED
cfg.dvl.seabed_depth = 30.0;                % [m] flat seabed, ASSUMED
cfg.dvl.max_altitude = 50.0;                % [m] bottom-lock limit, ASSUMED (unverified)
cfg.dvl.dropout      = [300.0, 360.0];      % [s] forced loss of bottom lock

% --- Analog pressure (depth) sensor ------------------------------------------
cfg.depth.rate_hz     = 20.0;               % ASSUMED
cfg.depth.full_scale  = 30.0;               % [m] user value
cfg.depth.adc_counts  = 1024;               % 10-bit ADC, user value (full span assumed)
cfg.depth.sigma       = 0.01;               % [m] noise before quantisation, ASSUMED
cfg.depth.lever       = [-0.10; 0.0; 0.05]; % [m] body FRD, ASSUMED
cfg.depth.bias_sigma0 = 0.02;               % [m] residual zero offset, ASSUMED
cfg.depth.bias_rw     = 1e-4;               % [m/sqrt(s)] temperature drift, ASSUMED

% --- VN-200 GNSS (surface only) ----------------------------------------------
cfg.gnss.rate_hz   = 5.0;                   % ASSUMED
cfg.gnss.sigma_pos = [1.0; 1.0; 1.5];       % [m] N E D, snippet, UNVERIFIED
cfg.gnss.sigma_vel = 0.05;                  % [m/s] snippet, UNVERIFIED
cfg.gnss.lever     = [0.0; 0.0; -0.30];     % [m] antenna 0.3 m above IMU, ASSUMED

% --- Magnetometer heading ----------------------------------------------------
cfg.heading.rate_hz = 10.0;                 % ASSUMED
cfg.heading.sigma   = 2.0 * DEG;            % [rad] after calibration, ASSUMED

% --- Initial uncertainty (1 sigma) -------------------------------------------
cfg.init.pos        = 1.0;                  % [m] ASSUMED
cfg.init.vel        = 0.1;                  % [m/s] ASSUMED
cfg.init.roll_pitch = 1.0 * DEG;            % [rad] ASSUMED
cfg.init.yaw        = 5.0 * DEG;            % [rad] ASSUMED

% --- Scenario ----------------------------------------------------------------
cfg.sc.duration     = 600.0;                % [s]
cfg.sc.surge        = 0.8;                  % [m/s]
cfg.sc.dive_depth   = 10.0;                 % [m]
cfg.sc.dive_start   = 60.0;  cfg.sc.dive_len   = 50.0;
cfg.sc.ascent_start = 500.0; cfg.sc.ascent_len = 50.0;
cfg.sc.turn_times   = [150.0, 250.0, 350.0, 450.0];
cfg.sc.turn_signs   = [+1, -1, +1, -1];
cfg.sc.turn_len     = 30.0;                 % [s] each turn is 180 deg

% --- Derived Earth vectors (NED) ---------------------------------------------
lat = cfg.earth.lat_deg * DEG;
s2 = sin(lat)^2;
g = 9.7803253359 * (1 + 0.00193185265241 * s2) / sqrt(1 - 0.00669437999013 * s2);  % WGS-84 normal gravity
cfg.w_ie = cfg.earth.omega_ie * [cos(lat); 0; -sin(lat)];
cfg.g    = [0; 0; g];
end
