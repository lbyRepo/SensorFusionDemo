%% ========================================================================
%  TWO-BEACON OBSERVABILITY DEMONSTRATION
%
%  Companion demo for the observability lesson (docs/06).
%
%  Same 2D nonlinear EKF structure as NonLinear2D.m, but:
%
%       - No GPS. Aiding is Doppler range / range-rate beacons only.
%
%       - Three estimators:
%
%             1. INS only
%             2. INS + range/range-rate to beacon 1
%             3. INS + range/range-rate to beacons 1 AND 2
%
%  Purpose:
%
%       A single range/range-rate beacon observes only the
%       line-of-sight (radial) position component and the
%       cross-LOS velocity component. The position estimate can
%       slide along the constant-range circle (tangential
%       direction) without being corrected.
%
%       A second, geometrically separated beacon makes the
%       tangential direction observable: two range circles
%       intersect at a point.
%
%  Extra output:
%
%       Position error decomposed into radial and tangential
%       components relative to beacon 1, with the filter
%       covariance projected onto the same directions.
%
%  Monte Carlo: 50 runs
%
%  GNU Octave
% ========================================================================

clear;
close all;
clc;

rand("seed",10);
randn("seed",10);


%% ========================================================================
% MONTE CARLO
% ========================================================================

N_MC = 50;


%% ========================================================================
% SIMULATION PARAMETERS
% ========================================================================

T_sim = 60;

imu_rate = 100;
doppler_rate = 10;
dt = 1 / imu_rate;
time = 0:dt:T_sim;
N = length(time);
doppler_step = round(imu_rate/doppler_rate);

%% ========================================================================
% FILTER DEFINITIONS
% ========================================================================

N_FILTERS = 3;
FILTER_INS = 1;
FILTER_ONE_BEACON = 2;
FILTER_TWO_BEACONS = 3;

filter_names = {

    "INS only";
    "INS + 1 Beacon";
    "INS + 2 Beacons"

};


use_beacon1 = [

    false;
    true;
    true

];


use_beacon2 = [

    false;
    false;
    true

];


%% ========================================================================
% INITIAL VEHICLE CONDITIONS
% ========================================================================

x0 = 50;                   % m
y0 = 30;                   % m
speed0 = 5;                % m/s
heading0 = 10*pi/180;      % rad

%% ========================================================================
% PERFECT VEHICLE INPUTS
% ========================================================================

a_forward_truth = 0.25 + 0.15*sin(0.25*time);


gyro_truth = (5*pi/180) * sin(0.18*time);


%% ========================================================================
% GENERATE PERFECT SPEED AND HEADING
% ========================================================================

speed_true = zeros(1,N);
heading_true = zeros(1,N);
speed_true(1) = speed0;
heading_true(1) = heading0;


for k = 2:N

    speed_true(k) = speed_true(k-1) + a_forward_truth(k-1)*dt;
    heading_new = heading_true(k-1) + gyro_truth(k-1)*dt;

    % Wrap heading to [-pi, pi]
    heading_true(k) = atan2( sin(heading_new), cos(heading_new) );

end


%% ========================================================================
% BODY-FRAME ACCELERATION
% ========================================================================

ax_body_truth = a_forward_truth;
ay_body_truth = speed_true .* gyro_truth;


%% ========================================================================
% PERFECT VELOCITY AND POSITION
% ========================================================================

vx_true = speed_true .* cos(heading_true);
vy_true = speed_true .* sin(heading_true);


x_true = zeros(1,N);
y_true = zeros(1,N);
x_true(1) = x0;
y_true(1) = y0;

for k = 2:N

    x_true(k) = x_true(k-1) + 0.5 * (vx_true(k-1) + vx_true(k)) * dt;
    y_true(k) = y_true(k-1) + 0.5 * (vy_true(k-1) + vy_true(k)) * dt;

end


%% ========================================================================
% SENSOR CHARACTERISTICS
% ========================================================================

sigma_accel_bias = 0.03;        % m/s^2
sigma_accel_noise = 0.02;       % m/s^2

sigma_gyro_bias = 0.15*pi/180;  % rad/s
sigma_gyro_noise = 0.05*pi/180; % rad/s

sigma_doppler_range = 0.5;      % m
sigma_doppler_rr = 0.05;        % m/s


%% ========================================================================
% DOPPLER BEACONS
%
% Two range/range-rate beacons with well separated
% lines of sight.
% ========================================================================

beacon1_x = 0;
beacon1_y = 0;

beacon2_x = 600;
beacon2_y = -300;


%% ========================================================================
% MONTE CARLO STORAGE
% ========================================================================

position_error = zeros(N_MC,N,N_FILTERS);

x_error = zeros(N_MC,N,N_FILTERS);
y_error = zeros(N_MC,N,N_FILTERS);

x_sigma_mc = zeros(N_MC,N,N_FILTERS);
y_sigma_mc = zeros(N_MC,N,N_FILTERS);


%% Radial / tangential error decomposition (relative to beacon 1)

radial_error = zeros(N_MC,N,N_FILTERS);
tangential_error = zeros(N_MC,N,N_FILTERS);

radial_sigma_mc = zeros(N_MC,N,N_FILTERS);
tangential_sigma_mc = zeros(N_MC,N,N_FILTERS);


%% ========================================================================
% MONTE CARLO LOOP
% ========================================================================

for mc = 1:N_MC

    fprintf( "Monte Carlo Run %d / %d\n", mc, N_MC );

    %% ====================================================================
    % CONSTANT IMU BIASES FOR THIS RUN
    % ====================================================================

    bax_true = sigma_accel_bias*randn();
    bay_true = sigma_accel_bias*randn();
    bg_true = sigma_gyro_bias*randn();


    %% ====================================================================
    % COMMON IMU SENSOR REALIZATION
    % ====================================================================

    ax_meas = ax_body_truth + bax_true + sigma_accel_noise*randn(1,N);
    ay_meas = ay_body_truth + bay_true + sigma_accel_noise*randn(1,N);
    gyro_meas = gyro_truth + bg_true + sigma_gyro_noise*randn(1,N);


    %% ====================================================================
    % COMMON BEACON MEASUREMENT REALIZATIONS
    % ====================================================================

    doppler1_range = NaN(1,N);
    doppler1_rr = NaN(1,N);

    doppler2_range = NaN(1,N);
    doppler2_rr = NaN(1,N);


    for k = 1:doppler_step:N

        % Beacon 1

        dx = x_true(k) - beacon1_x;
        dy = y_true(k) - beacon1_y;
        range_true = sqrt(dx^2 + dy^2);
        range_rate_true = ( dx*vx_true(k) + dy*vy_true(k) ) / range_true;

        doppler1_range(k) = range_true + sigma_doppler_range*randn();
        doppler1_rr(k) = range_rate_true + sigma_doppler_rr*randn();


        % Beacon 2

        dx = x_true(k) - beacon2_x;
        dy = y_true(k) - beacon2_y;
        range_true = sqrt(dx^2 + dy^2);
        range_rate_true = ( dx*vx_true(k) + dy*vy_true(k) ) / range_true;

        doppler2_range(k) = range_true + sigma_doppler_range*randn();
        doppler2_rr(k) = range_rate_true + sigma_doppler_rr*randn();

    end


    %% ====================================================================
    % INITIALIZE THREE EKFs
    %
    % State:
    %
    %       1 x
    %       2 y
    %       3 vx
    %       4 vy
    %       5 heading
    %       6 bax
    %       7 bay
    %       8 bg
    % ====================================================================

    X = zeros(8,N_FILTERS);

    for f = 1:N_FILTERS

        X(:,f) = [
            x0;
            y0;
            speed0*cos(heading0);
            speed0*sin(heading0);
            heading0;
            0;
            0;
            0
        ];

    end


    %% ====================================================================
    % INITIAL COVARIANCE
    % ====================================================================

    P0 = diag([
        10^2;
        10^2;
        2^2;
        2^2;
        (5*pi/180)^2;
        sigma_accel_bias^2;
        sigma_accel_bias^2;
        sigma_gyro_bias^2

    ]);


    P = zeros(8,8,N_FILTERS);

    for f = 1:N_FILTERS

        P(:,:,f) = P0;

    end


    %% ====================================================================
    % RUN STORAGE
    % ====================================================================

    x_est = zeros(N_FILTERS,N);
    y_est = zeros(N_FILTERS,N);


    % k = 1 seeds: errors are zero, sigmas from P0

    rt0 = sqrt( x_true(1)^2 + y_true(1)^2 );
    rx0 = x_true(1) / rt0;
    ry0 = y_true(1) / rt0;

    for f = 1:N_FILTERS

        x_est(f,1) = X(1,f);
        y_est(f,1) = X(2,f);

        x_sigma_mc(mc,1,f) = sqrt(P0(1,1));
        y_sigma_mc(mc,1,f) = sqrt(P0(2,2));

        radial_sigma_mc(mc,1,f) = sqrt( [rx0 ry0]*P0(1:2,1:2)*[rx0;ry0] );
        tangential_sigma_mc(mc,1,f) = sqrt( [-ry0 rx0]*P0(1:2,1:2)*[-ry0;rx0] );

    end


    %% ====================================================================
    % EKF LOOP
    % ====================================================================

    for k = 2:N


        for f = 1:N_FILTERS


            %% ============================================================
            % BIAS-CORRECT IMU
            % ============================================================

            ax_corrected = ax_meas(k-1) - X(6,f);
            ay_corrected = ay_meas(k-1) - X(7,f);
            gyro_corrected = gyro_meas(k-1) - X(8,f);


            %% ============================================================
            % BODY -> NAVIGATION ROTATION
            % ============================================================

            psi = X(5,f);
            c = cos(psi);
            s = sin(psi);
            ax_nav = c*ax_corrected - s*ay_corrected;
            ay_nav = s*ax_corrected + c*ay_corrected;


            %% ============================================================
            % STATE PROPAGATION
            % ============================================================

            X_pred = X(:,f);
            X_pred(1) = X(1,f) + X(3,f)*dt + 0.5*ax_nav*dt^2;
            X_pred(2) = X(2,f) + X(4,f)*dt + 0.5*ay_nav*dt^2;
            X_pred(3) = X(3,f) + ax_nav*dt;
            X_pred(4) = X(4,f) + ay_nav*dt;
            X_pred(5) = X(5,f) + gyro_corrected*dt;

            % Wrap heading
            X_pred(5) = atan2( sin(X_pred(5)), cos(X_pred(5)) );

            X_pred(6) = X(6,f);
            X_pred(7) = X(7,f);
            X_pred(8) = X(8,f);

            %% ============================================================
            % PROCESS MODEL JACOBIAN
            % ============================================================

            daNx_dpsi = -s*ax_corrected -c*ay_corrected;
            daNy_dpsi = c*ax_corrected -s*ay_corrected;
            F = eye(8);

            F(1,3) = dt;
            F(2,4) = dt;

            F(1,5) = 0.5*daNx_dpsi*dt^2;
            F(2,5) = 0.5*daNy_dpsi*dt^2;
            F(3,5) = daNx_dpsi*dt;
            F(4,5) = daNy_dpsi*dt;

            F(1,6) = -0.5*c*dt^2;
            F(2,6) = -0.5*s*dt^2;
            F(3,6) = -c*dt;
            F(4,6) = -s*dt;

            F(1,7) = 0.5*s*dt^2;
            F(2,7) = -0.5*c*dt^2;
            F(3,7) = s*dt;
            F(4,7) = -c*dt;

            F(5,8) = -dt;

            %% ============================================================
            % PROCESS NOISE
            % ============================================================

            G = zeros(8,3);

            G(1,1) = 0.5*c*dt^2;
            G(2,1) = 0.5*s*dt^2;
            G(3,1) = c*dt;
            G(4,1) = s*dt;

            G(1,2) = -0.5*s*dt^2;
            G(2,2) = 0.5*c*dt^2;
            G(3,2) = -s*dt;
            G(4,2) = c*dt;

            G(5,3) = dt;

            Qimu = G * diag([
                    sigma_accel_noise^2;
                    sigma_accel_noise^2;
                    sigma_gyro_noise^2
                ]) * G';

            sigma_accel_bias_process = 1e-5;
            sigma_gyro_bias_process = 1e-6;

            Qbias = zeros(8);
            Qbias(6,6) = sigma_accel_bias_process^2*dt;
            Qbias(7,7) = sigma_accel_bias_process^2*dt;
            Qbias(8,8) = sigma_gyro_bias_process^2*dt;
            Q = Qimu + Qbias;

            %% Covariance propagation

            P_pred = F*P(:,:,f)*F' + Q;
            X(:,f) = X_pred;
            P(:,:,f) = P_pred;


            %% ============================================================
            % BEACON 1 UPDATE
            % ============================================================

            if use_beacon1(f) && ~isnan(doppler1_range(k))


                px = X(1,f) - beacon1_x;
                py = X(2,f) - beacon1_y;
                vx = X(3,f);
                vy = X(4,f);
                r = sqrt(px^2 + py^2);

                if r < 1e-6

                    r = 1e-6;

                end

                q = px*vx + py*vy;
                rr = q/r;

                h = [
                    r;
                    rr
                ];

                z = [
                    doppler1_range(k);
                    doppler1_rr(k)
                ];


                %% Jacobian

                dr_dx = px/r;
                dr_dy = py/r;
                drr_dx = vx/r - px*q/r^3;
                drr_dy = vy/r - py*q/r^3;
                drr_dvx = px/r;
                drr_dvy = py/r;

                H = [
                    dr_dx, dr_dy, 0, 0, 0, 0, 0, 0;
                    drr_dx, drr_dy, drr_dvx, drr_dvy, 0, 0, 0, 0
                ];


                R = diag([
                    sigma_doppler_range^2;
                    sigma_doppler_rr^2
                ]);


                innovation = z-h;
                S = H*P(:,:,f)*H' + R;
                K = P(:,:,f)*H'/S;
                X(:,f) = X(:,f) + K*innovation;

                % Wrap corrected heading
                X(5,f) = atan2( sin(X(5,f)), cos(X(5,f)) );
                I = eye(8);
                P(:,:,f) = (I-K*H) * P(:,:,f) * (I-K*H)' + K*R*K';

            end


            %% ============================================================
            % BEACON 2 UPDATE
            % ============================================================

            if use_beacon2(f) && ~isnan(doppler2_range(k))


                px = X(1,f) - beacon2_x;
                py = X(2,f) - beacon2_y;
                vx = X(3,f);
                vy = X(4,f);
                r = sqrt(px^2 + py^2);

                if r < 1e-6

                    r = 1e-6;

                end

                q = px*vx + py*vy;
                rr = q/r;

                h = [
                    r;
                    rr
                ];

                z = [
                    doppler2_range(k);
                    doppler2_rr(k)
                ];


                %% Jacobian

                dr_dx = px/r;
                dr_dy = py/r;
                drr_dx = vx/r - px*q/r^3;
                drr_dy = vy/r - py*q/r^3;
                drr_dvx = px/r;
                drr_dvy = py/r;

                H = [
                    dr_dx, dr_dy, 0, 0, 0, 0, 0, 0;
                    drr_dx, drr_dy, drr_dvx, drr_dvy, 0, 0, 0, 0
                ];


                R = diag([
                    sigma_doppler_range^2;
                    sigma_doppler_rr^2
                ]);


                innovation = z-h;
                S = H*P(:,:,f)*H' + R;
                K = P(:,:,f)*H'/S;
                X(:,f) = X(:,f) + K*innovation;

                % Wrap corrected heading
                X(5,f) = atan2( sin(X(5,f)), cos(X(5,f)) );
                I = eye(8);
                P(:,:,f) = (I-K*H) * P(:,:,f) * (I-K*H)' + K*R*K';

            end


            %% ============================================================
            % STORE
            % ============================================================

            x_est(f,k) = X(1,f);
            y_est(f,k) = X(2,f);

            x_sigma_mc(mc,k,f) = sqrt(P(1,1,f));
            y_sigma_mc(mc,k,f) = sqrt(P(2,2,f));


            % Radial / tangential decomposition (relative to beacon 1)

            rt = sqrt( x_true(k)^2 + y_true(k)^2 );
            rx = x_true(k) / rt;
            ry = y_true(k) / rt;

            ex = x_est(f,k) - x_true(k);
            ey = y_est(f,k) - y_true(k);

            radial_error(mc,k,f)     =   ex*rx + ey*ry;
            tangential_error(mc,k,f) =  -ex*ry + ey*rx;

            Ppos = P(1:2,1:2,f);

            radial_sigma_mc(mc,k,f) = sqrt( [rx ry]*Ppos*[rx;ry] );
            tangential_sigma_mc(mc,k,f) = sqrt( [-ry rx]*Ppos*[-ry;rx] );

        end

    end


    %% ====================================================================
    % ERRORS
    % ====================================================================

    for f = 1:N_FILTERS

        %% Position magnitude
        position_error(mc,:,f) = sqrt( (x_est(f,:) - x_true).^2 + (y_est(f,:) - y_true).^2 );

        %% Signed per-axis errors
        x_error(mc,:,f) = x_est(f,:) - x_true;
        y_error(mc,:,f) = y_est(f,:) - y_true;

    end

end


%% ========================================================================
% MONTE CARLO STATISTICS
% ========================================================================

position_rmse = zeros(N,N_FILTERS);

x_sigma_mean = zeros(N,N_FILTERS);
y_sigma_mean = zeros(N,N_FILTERS);
x_sigma_emp = zeros(N,N_FILTERS);
y_sigma_emp = zeros(N,N_FILTERS);

radial_sigma_mean = zeros(N,N_FILTERS);
tangential_sigma_mean = zeros(N,N_FILTERS);
radial_sigma_emp = zeros(N,N_FILTERS);
tangential_sigma_emp = zeros(N,N_FILTERS);

for f = 1:N_FILTERS

    position_rmse(:,f) = sqrt( mean( position_error(:,:,f).^2, 1 ) )';

    x_sigma_mean(:,f) = mean( x_sigma_mc(:,:,f), 1 )';
    y_sigma_mean(:,f) = mean( y_sigma_mc(:,:,f), 1 )';
    x_sigma_emp(:,f) = std( x_error(:,:,f), 0, 1 )';
    y_sigma_emp(:,f) = std( y_error(:,:,f), 0, 1 )';

    radial_sigma_mean(:,f) = mean( radial_sigma_mc(:,:,f), 1 )';
    tangential_sigma_mean(:,f) = mean( tangential_sigma_mc(:,:,f), 1 )';
    radial_sigma_emp(:,f) = std( radial_error(:,:,f), 0, 1 )';
    tangential_sigma_emp(:,f) = std( tangential_error(:,:,f), 0, 1 )';

end


%% ========================================================================
% PLOT 1
% VEHICLE TRAJECTORY AND BEACONS
% ========================================================================

figure;
plot( x_true, y_true, "LineWidth",2 );
hold on;
plot( beacon1_x, beacon1_y, "o", "MarkerSize",10 );
plot( beacon2_x, beacon2_y, "s", "MarkerSize",10 );
plot( x_true(1), y_true(1), "ks", "MarkerSize",8 );
grid on;
axis equal;
xlabel("X [m]");
ylabel("Y [m]");
title("Truth Vehicle Trajectory");
legend( "Vehicle", "Beacon 1", "Beacon 2", "Start" );


%% ========================================================================
% PLOT 2
% POSITION RMSE
% ========================================================================

figure;
hold on;

for f = 1:N_FILTERS
    plot( time, position_rmse(:,f), "LineWidth",2 );
end

grid on;
xlabel("Time [s]");
ylabel("2D Position RMSE [m]");
title("Monte Carlo Position RMSE");
legend( filter_names{1}, filter_names{2}, filter_names{3} );


%% ========================================================================
% PLOT 3
% RADIAL VS TANGENTIAL ERROR
%
% Light grey : all Monte Carlo error traces
% Red        : filter covariance 1-sigma projected onto direction
% Blue       : empirical 1-sigma (std across runs)
% ========================================================================

grey_color = [0.8 0.8 0.8];

figure;

col_filters = [FILTER_ONE_BEACON, FILTER_TWO_BEACONS];

for c = 1:2

    fc = col_filters(c);

    %% Radial error

    subplot(2,2,c);
    hold on;

    h_grey = plot( time, squeeze(radial_error(:,:,fc))', "Color", grey_color );

    h_cov = plot( time, radial_sigma_mean(:,fc), "r", "LineWidth", 1.5 );
    plot( time, -radial_sigma_mean(:,fc), "r", "LineWidth", 1.5 );

    h_emp = plot( time, radial_sigma_emp(:,fc), "b", "LineWidth", 1.5 );
    plot( time, -radial_sigma_emp(:,fc), "b", "LineWidth", 1.5 );

    grid on;
    xlabel("Time [s]");
    ylabel("Radial Position Error [m]");
    title(filter_names{fc});

    if c == 2
        legend( ...
            [h_grey(1) h_cov h_emp], ...
            "MC runs", ...
            "Filter 1\sigma", ...
            "Empirical 1\sigma", ...
            "Location", "northeast", ...
            "FontSize", 8 ...
        );
    end

    %% Tangential error

    subplot(2,2,c+2);
    hold on;

    plot( time, squeeze(tangential_error(:,:,fc))', "Color", grey_color );

    plot( time,  tangential_sigma_mean(:,fc), "r", "LineWidth", 1.5 );
    plot( time, -tangential_sigma_mean(:,fc), "r", "LineWidth", 1.5 );

    plot( time,  tangential_sigma_emp(:,fc), "b", "LineWidth", 1.5 );
    plot( time, -tangential_sigma_emp(:,fc), "b", "LineWidth", 1.5 );

    grid on;
    xlabel("Time [s]");
    ylabel("Tangential Position Error [m]");
    title(filter_names{fc});

end


%% ========================================================================
% PLOT 4
% MONTE CARLO POSITION ERROR PER AXIS
% ========================================================================

figure;

for f = 1:N_FILTERS

    %% X position error

    subplot(2,3,f);
    hold on;

    h_grey = plot( time, squeeze(x_error(:,:,f))', "Color", grey_color );

    h_cov = plot( time, x_sigma_mean(:,f), "r", "LineWidth", 1.5 );
    plot( time, -x_sigma_mean(:,f), "r", "LineWidth", 1.5 );

    h_emp = plot( time, x_sigma_emp(:,f), "b", "LineWidth", 1.5 );
    plot( time, -x_sigma_emp(:,f), "b", "LineWidth", 1.5 );

    grid on;
    xlabel("Time [s]");
    ylabel("X Position Error [m]");
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

    %% Y position error

    subplot(2,3,f+3);
    hold on;

    plot( time, squeeze(y_error(:,:,f))', "Color", grey_color );

    plot( time,  y_sigma_mean(:,f), "r", "LineWidth", 1.5 );
    plot( time, -y_sigma_mean(:,f), "r", "LineWidth", 1.5 );

    plot( time,  y_sigma_emp(:,f), "b", "LineWidth", 1.5 );
    plot( time, -y_sigma_emp(:,f), "b", "LineWidth", 1.5 );

    grid on;
    xlabel("Time [s]");
    ylabel("Y Position Error [m]");
    title(filter_names{f});

end


%% ========================================================================
% FINAL PERFORMANCE
% ========================================================================

fprintf("\n");
fprintf("=======================================================================\n");
fprintf(" TWO-BEACON OBSERVABILITY MONTE CARLO PERFORMANCE\n");
fprintf("=======================================================================\n");
fprintf( "%-24s %-16s %-16s %-16s\n", "Filter", "Pos[m]", "Radial[m]", "Tangential[m]" );
fprintf("-----------------------------------------------------------------------\n");

for f = 1:N_FILTERS
    fprintf( "%-24s %-16.4f %-16.4f %-16.4f\n", ...
        filter_names{f}, ...
        position_rmse(end,f), ...
        radial_sigma_emp(end,f), ...
        tangential_sigma_emp(end,f) );
end

fprintf("=======================================================================\n");
