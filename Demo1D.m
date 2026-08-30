%% ========================================================================
%  MONTE CARLO 1D SENSOR FUSION DEMONSTRATION
%
%  Vehicle truth:
%       Constant acceleration = 0.5 m/s^2
%
%  Filters:
%       1. INS only
%       2. INS + GPS
%       3. INS + Doppler
%       4. INS + GPS + Doppler
%
%  State:
%
%       X = [ position
%             velocity
%             accelerometer bias ]
%
%  Monte Carlo:
%       50 runs
%
%       For every run:
%
%           bias ~ N(0, sigma_bias^2)
%
%       The bias is CONSTANT during one complete run,
%       but changes between Monte Carlo runs.
%
%  All four filters receive exactly the SAME sensor realization
%  during each Monte Carlo run.
%
%  GNU Octave
% ========================================================================

clear;
close all;
clc;


%% ========================================================================
% RANDOM SEED
% ========================================================================

rand("seed", 10);
randn("seed", 10);


%% ========================================================================
% MONTE CARLO PARAMETERS
% ========================================================================

N_MC = 50;


%% ========================================================================
% SIMULATION PARAMETERS
% ========================================================================

T_sim = 50;                % [s]

imu_rate = 100;            % [Hz]
gps_rate = 1;              % [Hz]
doppler_rate = 10;         % [Hz]

dt = 1 / imu_rate;

time = 0:dt:T_sim;

N = length(time);


%% ========================================================================
% FILTER DEFINITIONS
% ========================================================================

N_FILTERS = 4;

FILTER_INS = 1;
FILTER_GPS = 2;
FILTER_DOPPLER = 3;
FILTER_ALL = 4;

filter_names = {
    "INS only",
    "INS + GPS",
    "INS + Doppler",
    "INS + GPS + Doppler"
};


% Which measurements each filter is allowed to use

use_gps = [
    false;
    true;
    false;
    true
];

use_doppler = [
    false;
    false;
    true;
    true
];


%% ========================================================================
% PERFECT VEHICLE TRUTH
% ========================================================================

a_true = 0.5;              % [m/s^2]

p0_true = 0;               % [m]
v0_true = 0;               % [m/s]


position_true = ...
    p0_true ...
    + v0_true .* time ...
    + 0.5 .* a_true .* time.^2;


velocity_true = ...
    v0_true ...
    + a_true .* time;


acceleration_true = ...
    a_true .* ones(1,N);


%% ========================================================================
% FINAL TRUE VALUES
% ========================================================================

fprintf("\n");
fprintf("=============================================\n");
fprintf(" TRUE VEHICLE TRAJECTORY\n");
fprintf("=============================================\n");

fprintf("Simulation time      : %.1f s\n", T_sim);
fprintf("Acceleration         : %.3f m/s^2\n", a_true);
fprintf("Final velocity       : %.3f m/s\n", velocity_true(end));
fprintf("Final position       : %.3f m\n", position_true(end));

fprintf("=============================================\n\n");


%% ========================================================================
% SENSOR ERROR PARAMETERS
% ========================================================================

% -------------------------------------------------------------------------
% IMU
% -------------------------------------------------------------------------

% Constant turn-on bias distribution
%
% Each Monte Carlo run gets:
%
%   b ~ N(0, sigma_accel_bias^2)
sigma_accel_bias = 0.05;        % [m/s^2]

% Sample-to-sample accelerometer white noise
sigma_accel_noise = 0.02;       % [m/s^2]


% -------------------------------------------------------------------------
% GPS
% -------------------------------------------------------------------------

sigma_gps_position = 2.0;       % [m]
sigma_gps_velocity = 0.15;      % [m/s]

% -------------------------------------------------------------------------
% Doppler range / range-rate sensor
% -------------------------------------------------------------------------

sigma_doppler_range = 0.5;      % [m]
sigma_doppler_rr = 0.05;        % [m/s]


%% ========================================================================
% DOPPLER BEACON
% ========================================================================

% Stationary beacon at a known position.
%
% Vehicle goes from:
%
%       x = 0 m
%
% to
%
%       x = 625 m
%
% during this simulation.
%
% Therefore it never crosses the beacon.

beacon_position = 1000;         % [m]


%% ========================================================================
% SENSOR UPDATE INTERVALS
% ========================================================================

gps_step = round(imu_rate / gps_rate);
doppler_step = round(imu_rate / doppler_rate);

%% ========================================================================
% MONTE CARLO STORAGE
%
% Dimensions:
%
%   [Monte Carlo run, time, filter]
% ========================================================================

position_error = ...
    zeros(N_MC, N, N_FILTERS);

velocity_error = ...
    zeros(N_MC, N, N_FILTERS);

bias_error = ...
    zeros(N_MC, N, N_FILTERS);


estimated_position = ...
    zeros(N_MC, N, N_FILTERS);

estimated_velocity = ...
    zeros(N_MC, N, N_FILTERS);

estimated_bias = ...
    zeros(N_MC, N, N_FILTERS);


% Actual constant bias assigned to each Monte Carlo run

true_bias_mc = zeros(N_MC,1);


%% ========================================================================
% FILTER COVARIANCE STORAGE
% ========================================================================

position_sigma_mc = ...
    zeros(N_MC, N, N_FILTERS);

velocity_sigma_mc = ...
    zeros(N_MC, N, N_FILTERS);

bias_sigma_mc = ...
    zeros(N_MC, N, N_FILTERS);


%% ========================================================================
% EKF PROCESS NOISE
% ========================================================================

% Acceleration measurement noise enters position and velocity.

G_accel = [
    0.5 * dt^2;
    dt;
    0
];


Q_accel = ...
    G_accel ...
    * sigma_accel_noise^2 ...
    * G_accel';


% Truth bias is constant.
%
% Use a very small random walk in the FILTER model so covariance
% does not become unrealistically frozen.

sigma_bias_process = 1e-5;      % [m/s^2 / sqrt(s)]


Q_bias = diag([
    0;
    0;
    sigma_bias_process^2 * dt
]);


Q = Q_accel + Q_bias;


%% ========================================================================
% STATE TRANSITION JACOBIAN
%
% p(k+1) = p + v*dt + 0.5*(a_meas-b)*dt^2
%
% v(k+1) = v + (a_meas-b)*dt
%
% b(k+1) = b
% ========================================================================

F = [
    1, dt, -0.5*dt^2;
    0, 1,  -dt;
    0, 0,   1
];


%% ========================================================================
% MONTE CARLO LOOP
% ========================================================================

for mc = 1:N_MC

    fprintf("Monte Carlo run %2d / %2d\n", mc, N_MC);


    %% ====================================================================
    % DRAW CONSTANT ACCELEROMETER BIAS
    % ====================================================================

    accel_bias_true = ...
        sigma_accel_bias * randn();


    true_bias_mc(mc) = accel_bias_true;


    %% ====================================================================
    % GENERATE ONE COMMON IMU REALIZATION
    %
    % ALL FOUR FILTERS USE THIS SAME IMU.
    % ====================================================================

    accel_noise = ...
        sigma_accel_noise ...
        .* randn(1,N);


    accel_measurement = ...
        acceleration_true ...
        + accel_bias_true ...
        + accel_noise;


    %% ====================================================================
    % GENERATE ONE COMMON GPS REALIZATION
    % ====================================================================

    gps_position = NaN(1,N);

    gps_velocity = NaN(1,N);


    for k = 1:gps_step:N

        gps_position(k) = ...
            position_true(k) ...
            + sigma_gps_position * randn();


        gps_velocity(k) = ...
            velocity_true(k) ...
            + sigma_gps_velocity * randn();

    end


    %% ====================================================================
    % GENERATE ONE COMMON DOPPLER REALIZATION
    % ====================================================================

    doppler_range = NaN(1,N);

    doppler_range_rate = NaN(1,N);


    for k = 1:doppler_step:N

        delta_true = ...
            position_true(k) ...
            - beacon_position;


        true_range = ...
            abs(delta_true);


        if abs(delta_true) > 1e-12

            los_sign_true = sign(delta_true);

        else

            los_sign_true = 1;

        end


        true_range_rate = ...
            los_sign_true ...
            * velocity_true(k);


        doppler_range(k) = ...
            true_range ...
            + sigma_doppler_range * randn();


        doppler_range_rate(k) = ...
            true_range_rate ...
            + sigma_doppler_rr * randn();

    end


    %% ====================================================================
    % INITIALIZE ALL FOUR FILTERS
    %
    % X(:,filter)
    %
    % state:
    %
    %       [ position
    %         velocity
    %         accel bias ]
    %
    % The filters do NOT know the actual bias.
    % ====================================================================

    X = zeros(3, N_FILTERS);


    for f = 1:N_FILTERS

        X(:,f) = [
            0;
            0;
            0
        ];

    end


    %% ====================================================================
    % INITIAL COVARIANCE
    % ====================================================================

    P = zeros(3,3,N_FILTERS);


    P0 = diag([
        10^2;
        2^2;
        sigma_accel_bias^2
    ]);


    for f = 1:N_FILTERS

        P(:,:,f) = P0;

    end


    %% ====================================================================
    % TEMPORARY STORAGE FOR THIS RUN
    % ====================================================================

    position_run = zeros(N_FILTERS,N);

    velocity_run = zeros(N_FILTERS,N);

    bias_run = zeros(N_FILTERS,N);


    pos_sigma_run = zeros(N_FILTERS,N);

    vel_sigma_run = zeros(N_FILTERS,N);

    bias_sigma_run = zeros(N_FILTERS,N);


    for f = 1:N_FILTERS

        position_run(f,1) = X(1,f);

        velocity_run(f,1) = X(2,f);

        bias_run(f,1) = X(3,f);


        pos_sigma_run(f,1) = ...
            sqrt(P(1,1,f));

        vel_sigma_run(f,1) = ...
            sqrt(P(2,2,f));

        bias_sigma_run(f,1) = ...
            sqrt(P(3,3,f));

    end


    %% ====================================================================
    % TIME LOOP
    % ====================================================================

    for k = 2:N


        % ================================================================
        % PROCESS EACH FILTER
        % ================================================================

        for f = 1:N_FILTERS


            %% ============================================================
            % IMU PROPAGATION
            % ============================================================

            a_meas = ...
                accel_measurement(k-1);


            % Correct acceleration using current estimated bias

            a_corrected = ...
                a_meas ...
                - X(3,f);


            % ------------------------------------------------------------
            % State propagation
            % ------------------------------------------------------------

            X_pred = zeros(3,1);


            X_pred(1) = ...
                X(1,f) ...
                + X(2,f)*dt ...
                + 0.5*a_corrected*dt^2;


            X_pred(2) = ...
                X(2,f) ...
                + a_corrected*dt;


            X_pred(3) = ...
                X(3,f);


            % ------------------------------------------------------------
            % Covariance propagation
            % ------------------------------------------------------------

            P_pred = ...
                F ...
                * P(:,:,f) ...
                * F' ...
                + Q;


            X(:,f) = X_pred;

            P(:,:,f) = P_pred;


            %% ============================================================
            % DOPPLER UPDATE
            % ============================================================

            if use_doppler(f) && ...
               ~isnan(doppler_range(k))


                delta_est = ...
                    X(1,f) ...
                    - beacon_position;


                if abs(delta_est) > 1e-12

                    los_sign_est = ...
                        sign(delta_est);

                else

                    los_sign_est = 1;

                end


                % --------------------------------------------------------
                % Predicted Doppler measurement
                % --------------------------------------------------------

                predicted_range = ...
                    abs(delta_est);


                predicted_range_rate = ...
                    los_sign_est ...
                    * X(2,f);


                h = [
                    predicted_range;
                    predicted_range_rate
                ];


                % --------------------------------------------------------
                % Actual measurement
                % --------------------------------------------------------

                z = [
                    doppler_range(k);
                    doppler_range_rate(k)
                ];


                % --------------------------------------------------------
                % Measurement Jacobian
                % --------------------------------------------------------

                H = [
                    los_sign_est, 0,            0;
                    0,            los_sign_est, 0
                ];


                % --------------------------------------------------------
                % Measurement covariance
                % --------------------------------------------------------

                R = diag([
                    sigma_doppler_range^2;
                    sigma_doppler_rr^2
                ]);


                % --------------------------------------------------------
                % Innovation
                % --------------------------------------------------------

                innovation = ...
                    z - h;


                % --------------------------------------------------------
                % Innovation covariance
                % --------------------------------------------------------

                S = ...
                    H ...
                    * P(:,:,f) ...
                    * H' ...
                    + R;


                % --------------------------------------------------------
                % Kalman gain
                % --------------------------------------------------------

                K = ...
                    P(:,:,f) ...
                    * H' ...
                    / S;


                % --------------------------------------------------------
                % State correction
                % --------------------------------------------------------

                X(:,f) = ...
                    X(:,f) ...
                    + K*innovation;


                % --------------------------------------------------------
                % Joseph covariance update
                % --------------------------------------------------------

                I = eye(3);


                P(:,:,f) = ...
                    (I-K*H) ...
                    * P(:,:,f) ...
                    * (I-K*H)' ...
                    + K*R*K';

            end


            %% ============================================================
            % GPS UPDATE
            % ============================================================

            if use_gps(f) && ...
               ~isnan(gps_position(k))


                % --------------------------------------------------------
                % GPS measurement
                % --------------------------------------------------------

                z = [
                    gps_position(k);
                    gps_velocity(k)
                ];


                % --------------------------------------------------------
                % Predicted measurement
                % --------------------------------------------------------

                h = [
                    X(1,f);
                    X(2,f)
                ];


                % --------------------------------------------------------
                % Measurement matrix
                % --------------------------------------------------------

                H = [
                    1, 0, 0;
                    0, 1, 0
                ];


                % --------------------------------------------------------
                % GPS measurement covariance
                % --------------------------------------------------------

                R = diag([
                    sigma_gps_position^2;
                    sigma_gps_velocity^2
                ]);


                % --------------------------------------------------------
                % Innovation
                % --------------------------------------------------------

                innovation = ...
                    z - h;


                % --------------------------------------------------------
                % Innovation covariance
                % --------------------------------------------------------

                S = ...
                    H ...
                    * P(:,:,f) ...
                    * H' ...
                    + R;


                % --------------------------------------------------------
                % Kalman gain
                % --------------------------------------------------------

                K = ...
                    P(:,:,f) ...
                    * H' ...
                    / S;


                % --------------------------------------------------------
                % State correction
                % --------------------------------------------------------

                X(:,f) = ...
                    X(:,f) ...
                    + K*innovation;


                % --------------------------------------------------------
                % Joseph covariance update
                % --------------------------------------------------------

                I = eye(3);


                P(:,:,f) = ...
                    (I-K*H) ...
                    * P(:,:,f) ...
                    * (I-K*H)' ...
                    + K*R*K';

            end


            %% ============================================================
            % SAVE FILTER STATE
            % ============================================================

            position_run(f,k) = ...
                X(1,f);


            velocity_run(f,k) = ...
                X(2,f);


            bias_run(f,k) = ...
                X(3,f);


            pos_sigma_run(f,k) = ...
                sqrt(P(1,1,f));


            vel_sigma_run(f,k) = ...
                sqrt(P(2,2,f));


            bias_sigma_run(f,k) = ...
                sqrt(P(3,3,f));

        end

    end


    %% ====================================================================
    % STORE MONTE CARLO RESULTS
    % ====================================================================

    for f = 1:N_FILTERS


        estimated_position(mc,:,f) = ...
            position_run(f,:);


        estimated_velocity(mc,:,f) = ...
            velocity_run(f,:);


        estimated_bias(mc,:,f) = ...
            bias_run(f,:);


        % ---------------------------------------------------------------
        % Errors
        % ---------------------------------------------------------------

        position_error(mc,:,f) = ...
            position_run(f,:) ...
            - position_true;


        velocity_error(mc,:,f) = ...
            velocity_run(f,:) ...
            - velocity_true;


        bias_error(mc,:,f) = ...
            bias_run(f,:) ...
            - accel_bias_true;


        % ---------------------------------------------------------------
        % Covariance
        % ---------------------------------------------------------------

        position_sigma_mc(mc,:,f) = ...
            pos_sigma_run(f,:);


        velocity_sigma_mc(mc,:,f) = ...
            vel_sigma_run(f,:);


        bias_sigma_mc(mc,:,f) = ...
            bias_sigma_run(f,:);

    end

end


%% ========================================================================
% MONTE CARLO STATISTICS
% ========================================================================

position_rmse = zeros(N,N_FILTERS);

velocity_rmse = zeros(N,N_FILTERS);

bias_rmse = zeros(N,N_FILTERS);


position_mean_error = zeros(N,N_FILTERS);

velocity_mean_error = zeros(N,N_FILTERS);

bias_mean_error = zeros(N,N_FILTERS);


for f = 1:N_FILTERS


    position_rmse(:,f) = ...
        sqrt( ...
            squeeze( ...
                mean(position_error(:,:,f).^2,1) ...
            ) ...
        );


    velocity_rmse(:,f) = ...
        sqrt( ...
            squeeze( ...
                mean(velocity_error(:,:,f).^2,1) ...
            ) ...
        );


    bias_rmse(:,f) = ...
        sqrt( ...
            squeeze( ...
                mean(bias_error(:,:,f).^2,1) ...
            ) ...
        );


    position_mean_error(:,f) = ...
        squeeze( ...
            mean(position_error(:,:,f),1) ...
        );


    velocity_mean_error(:,f) = ...
        squeeze( ...
            mean(velocity_error(:,:,f),1) ...
        );


    bias_mean_error(:,f) = ...
        squeeze( ...
            mean(bias_error(:,:,f),1) ...
        );

end


%% ========================================================================
% MONTE CARLO 1-SIGMA STATISTICS
%
% sigma_mean : filter covariance 1-sigma, averaged over all runs
%
% sigma_emp  : empirical 1-sigma, std of the signed error
%              across all Monte Carlo runs
% ========================================================================

position_sigma_mean = zeros(N,N_FILTERS);

velocity_sigma_mean = zeros(N,N_FILTERS);


position_sigma_emp = zeros(N,N_FILTERS);

velocity_sigma_emp = zeros(N,N_FILTERS);


for f = 1:N_FILTERS


    position_sigma_mean(:,f) = ...
        squeeze( ...
            mean(position_sigma_mc(:,:,f),1) ...
        );


    velocity_sigma_mean(:,f) = ...
        squeeze( ...
            mean(velocity_sigma_mc(:,:,f),1) ...
        );


    position_sigma_emp(:,f) = ...
        squeeze( ...
            std(position_error(:,:,f),0,1) ...
        );


    velocity_sigma_emp(:,f) = ...
        squeeze( ...
            std(velocity_error(:,:,f),0,1) ...
        );

end


%% ========================================================================
% FIGURE 1
% GENERATED ACCELEROMETER BIAS DISTRIBUTION
% ========================================================================

figure;

hist(true_bias_mc,10);

grid on;

xlabel("Constant Accelerometer Bias [m/s^2]");

ylabel("Number of Monte Carlo Runs");

title("Generated Accelerometer Bias Distribution");


%% ========================================================================
% FIGURE 2
% POSITION RMSE COMPARISON
% ========================================================================

figure;

plot( ...
    time, ...
    position_rmse(:,FILTER_INS), ...
    "LineWidth", 2 ...
);

hold on;


plot( ...
    time, ...
    position_rmse(:,FILTER_GPS), ...
    "LineWidth", 2 ...
);


plot( ...
    time, ...
    position_rmse(:,FILTER_DOPPLER), ...
    "LineWidth", 2 ...
);


plot( ...
    time, ...
    position_rmse(:,FILTER_ALL), ...
    "LineWidth", 2 ...
);


grid on;

xlabel("Time [s]");

ylabel("Position RMSE [m]");

title("Monte Carlo Position RMSE");

legend( ...
    filter_names{1}, ...
    filter_names{2}, ...
    filter_names{3}, ...
    filter_names{4}, ...
    "Location", ...
    "northwest" ...
);


%% ========================================================================
% FIGURE 3
% VELOCITY RMSE COMPARISON
% ========================================================================

figure;

plot( ...
    time, ...
    velocity_rmse(:,FILTER_INS), ...
    "LineWidth", 2 ...
);

hold on;


plot( ...
    time, ...
    velocity_rmse(:,FILTER_GPS), ...
    "LineWidth", 2 ...
);


plot( ...
    time, ...
    velocity_rmse(:,FILTER_DOPPLER), ...
    "LineWidth", 2 ...
);


plot( ...
    time, ...
    velocity_rmse(:,FILTER_ALL), ...
    "LineWidth", 2 ...
);


grid on;

xlabel("Time [s]");

ylabel("Velocity RMSE [m/s]");

title("Monte Carlo Velocity RMSE");

legend( ...
    filter_names{1}, ...
    filter_names{2}, ...
    filter_names{3}, ...
    filter_names{4}, ...
    "Location", ...
    "northwest" ...
);


%% ========================================================================
% FIGURE 4
% ACCELEROMETER BIAS RMSE COMPARISON
% ========================================================================

figure;

plot( ...
    time, ...
    bias_rmse(:,FILTER_INS), ...
    "LineWidth", 2 ...
);

hold on;


plot( ...
    time, ...
    bias_rmse(:,FILTER_GPS), ...
    "LineWidth", 2 ...
);


plot( ...
    time, ...
    bias_rmse(:,FILTER_DOPPLER), ...
    "LineWidth", 2 ...
);


plot( ...
    time, ...
    bias_rmse(:,FILTER_ALL), ...
    "LineWidth", 2 ...
);


grid on;

xlabel("Time [s]");

ylabel("Accelerometer Bias RMSE [m/s^2]");

title("Monte Carlo Accelerometer Bias Estimation RMSE");

legend( ...
    filter_names{1}, ...
    filter_names{2}, ...
    filter_names{3}, ...
    filter_names{4}, ...
    "Location", ...
    "northeast" ...
);


%% ========================================================================
% FIGURE 5
% ALL POSITION ERROR TRAJECTORIES
%
% Light grey : all Monte Carlo error traces
% Red        : filter covariance 1-sigma (mean over runs)
% Blue       : empirical 1-sigma (std across runs)
%
% One subplot per estimator
% ========================================================================

grey_color = [0.8 0.8 0.8];

figure;


for f = 1:N_FILTERS

    subplot(2,2,f);

    hold on;


    temp = ...
        squeeze(position_error(:,:,f));


    h_grey = ...
        plot(time, temp', "Color", grey_color);


    h_cov = ...
        plot(time, position_sigma_mean(:,f), "r", "LineWidth", 1.5);


    plot(time, -position_sigma_mean(:,f), "r", "LineWidth", 1.5);


    h_emp = ...
        plot(time, position_sigma_emp(:,f), "b", "LineWidth", 1.5);


    plot(time, -position_sigma_emp(:,f), "b", "LineWidth", 1.5);


    grid on;

    xlabel("Time [s]");

    ylabel("Position Error [m]");

    title(filter_names{f});


    if f == 2

        legend( ...
            [h_grey(1) h_cov h_emp], ...
            "MC runs", ...
            "Filter 1\sigma", ...
            "Empirical 1\sigma", ...
            "Location", "northeast", ...
            "FontSize", 8 ...
        );

    end

end


%% ========================================================================
% FIGURE 6
% ALL VELOCITY ERROR TRAJECTORIES
%
% Light grey : all Monte Carlo error traces
% Red        : filter covariance 1-sigma (mean over runs)
% Blue       : empirical 1-sigma (std across runs)
% ========================================================================

figure;


for f = 1:N_FILTERS

    subplot(2,2,f);

    hold on;


    temp = ...
        squeeze(velocity_error(:,:,f));


    h_grey = ...
        plot(time, temp', "Color", grey_color);


    h_cov = ...
        plot(time, velocity_sigma_mean(:,f), "r", "LineWidth", 1.5);


    plot(time, -velocity_sigma_mean(:,f), "r", "LineWidth", 1.5);


    h_emp = ...
        plot(time, velocity_sigma_emp(:,f), "b", "LineWidth", 1.5);


    plot(time, -velocity_sigma_emp(:,f), "b", "LineWidth", 1.5);


    grid on;

    xlabel("Time [s]");

    ylabel("Velocity Error [m/s]");

    title(filter_names{f});


    if f == 2

        legend( ...
            [h_grey(1) h_cov h_emp], ...
            "MC runs", ...
            "Filter 1\sigma", ...
            "Empirical 1\sigma", ...
            "Location", "northeast", ...
            "FontSize", 8 ...
        );

    end

end


%% ========================================================================
% FIGURE 7
% BIAS ESTIMATION TRAJECTORIES
% ========================================================================

figure;


for f = 1:N_FILTERS

    subplot(2,2,f);

    hold on;


    for mc = 1:N_MC

        plot( ...
            time, ...
            squeeze(estimated_bias(mc,:,f)) ...
        );

    end


    grid on;

    xlabel("Time [s]");

    ylabel("Estimated Bias [m/s^2]");

    title(filter_names{f});

end


%% ========================================================================
% FIGURE 8
% FINAL POSITION ERROR DISTRIBUTION
%
% Using histograms for straightforward GNU Octave compatibility.
% ========================================================================

figure;


for f = 1:N_FILTERS

    subplot(2,2,f);


    final_position_errors = ...
        position_error(:,end,f);


    hist(final_position_errors,10);


    grid on;

    xlabel("Final Position Error [m]");

    ylabel("Runs");

    title(filter_names{f});

end


%% ========================================================================
% FIGURE 9
% EXAMPLE SINGLE MONTE CARLO RUN
%
% Useful for presentation:
% show all four navigation solutions for one run.
% ========================================================================

example_run = 1;


figure;

plot( ...
    time, ...
    position_true, ...
    "k", ...
    "LineWidth", ...
    3 ...
);

hold on;


for f = 1:N_FILTERS

    plot( ...
        time, ...
        squeeze(estimated_position(example_run,:,f)), ...
        "LineWidth", ...
        1.5 ...
    );

end


grid on;

xlabel("Time [s]");

ylabel("Position [m]");

title("Example Monte Carlo Run - Position");

legend( ...
    "Truth", ...
    filter_names{1}, ...
    filter_names{2}, ...
    filter_names{3}, ...
    filter_names{4}, ...
    "Location", ...
    "northwest" ...
);


%% ========================================================================
% FIGURE 10
% EXAMPLE RUN - POSITION ERROR
% ========================================================================

figure;

hold on;


for f = 1:N_FILTERS

    plot( ...
        time, ...
        squeeze(position_error(example_run,:,f)), ...
        "LineWidth", ...
        1.5 ...
    );

end


grid on;

xlabel("Time [s]");

ylabel("Position Error [m]");

title("Example Monte Carlo Run - Position Error");

legend( ...
    filter_names{1}, ...
    filter_names{2}, ...
    filter_names{3}, ...
    filter_names{4}, ...
    "Location", ...
    "northwest" ...
);


%% ========================================================================
% FINAL STATISTICS
% ========================================================================

fprintf("\n");
fprintf("==============================================================\n");
fprintf(" MONTE CARLO SUMMARY\n");
fprintf("==============================================================\n");

fprintf("Number of runs = %d\n\n", N_MC);


fprintf("Generated accelerometer bias:\n");

fprintf( ...
    "  Mean = %.6f m/s^2\n", ...
    mean(true_bias_mc) ...
);

fprintf( ...
    "  Std  = %.6f m/s^2\n\n", ...
    std(true_bias_mc) ...
);


fprintf( ...
    "%-24s %-16s %-16s %-16s\n", ...
    "Filter", ...
    "Pos RMSE [m]", ...
    "Vel RMSE [m/s]", ...
    "Bias RMSE [m/s2]" ...
);

fprintf( ...
    "------------------------------------------------------------------------\n" ...
);


for f = 1:N_FILTERS

    fprintf( ...
        "%-24s %-16.6f %-16.6f %-16.6f\n", ...
        filter_names{f}, ...
        position_rmse(end,f), ...
        velocity_rmse(end,f), ...
        bias_rmse(end,f) ...
    );

end


fprintf("==============================================================\n");
