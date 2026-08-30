%% ========================================================================
%  2D INS / GPS / DOPPLER EKF WITH GYROSCOPE
%
%  States:
%
%       X = [ x
%             y
%             vx
%             vy
%             heading
%             accel_bias_x
%             accel_bias_y
%             gyro_bias ]
%
%
%  IMU measurements:
%
%       Body X acceleration
%       Body Y acceleration
%       Z-axis gyro
%
%
%  Aiding:
%
%       GPS:
%           x, y, vx, vy
%
%       Doppler beacon at (0,0):
%           range
%           range rate
%
%
%  Four estimators:
%
%       1. INS only
%       2. INS + GPS
%       3. INS + Doppler
%       4. INS + GPS + Doppler
%
%
%  Monte Carlo:
%
%       50 runs
%
%       Constant accelerometer and gyro biases
%       are redrawn for each run.
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
gps_rate = 1;
doppler_rate = 10;
dt = 1 / imu_rate;
time = 0:dt:T_sim;
N = length(time);
gps_step = round(imu_rate/gps_rate);
doppler_step = round(imu_rate/doppler_rate);

%% ========================================================================
% FILTER DEFINITIONS
% ========================================================================

N_FILTERS = 4;
FILTER_INS = 1;
FILTER_GPS = 2;
FILTER_DOPPLER = 3;
FILTER_ALL = 4;

filter_names = {
    "INS only";
    "INS + GPS";
    "INS + Doppler";
    "INS + GPS + Doppler"
};


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
% INITIAL VEHICLE CONDITIONS
% ========================================================================

x0 = 50;                   % m
y0 = 30;                   % m
speed0 = 5;                % m/s
heading0 = 10*pi/180;      % rad

%% ========================================================================
% PERFECT VEHICLE INPUTS
%
% Longitudinal acceleration is sinusoidal.
%
% Gyroscope yaw rate is also sinusoidal.
% ========================================================================

a_forward_truth = 0.25 + 0.15*sin(0.25*time);


% Z-axis angular rate
%
% Peak = +/- 5 deg/s

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
%
% For planar vehicle motion:
%
%       a_x_body = longitudinal acceleration
%
%       a_y_body = v * yaw_rate
%
% The Y acceleration is the centripetal/lateral acceleration.
% ========================================================================

ax_body_truth = a_forward_truth;
ay_body_truth = speed_true .* gyro_truth;


%% ========================================================================
% PERFECT VELOCITY IN NAVIGATION FRAME
% ========================================================================

vx_true = speed_true .* cos(heading_true);
vy_true = speed_true .* sin(heading_true);


%% ========================================================================
% PERFECT POSITION
% ========================================================================

x_true = zeros(1,N);
y_true = zeros(1,N);
x_true(1) = x0;
y_true(1) = y0;

for k = 2:N

    % Trapezoidal integration
    x_true(k) = x_true(k-1) + 0.5 * (vx_true(k-1) + vx_true(k)) * dt;
    y_true(k) = y_true(k-1) + 0.5 * (vy_true(k-1) + vy_true(k)) * dt;

end


%% ========================================================================
% TRUE NAVIGATION-FRAME ACCELERATION
%
% Useful only for plotting/verification.
% ========================================================================

ax_nav_truth = zeros(1,N);
ay_nav_truth = zeros(1,N);

for k = 1:N

    c = cos(heading_true(k));
    s = sin(heading_true(k));
    ax_nav_truth(k) = c*ax_body_truth(k) - s*ay_body_truth(k);
    ay_nav_truth(k) = s*ax_body_truth(k) + c*ay_body_truth(k);

end


%% ========================================================================
% SENSOR CHARACTERISTICS
% ========================================================================

% Accelerometer constant turn-on bias
sigma_accel_bias = 0.03;        % m/s^2


% Accelerometer white noise
sigma_accel_noise = 0.02;       % m/s^2


% Gyroscope constant turn-on bias
sigma_gyro_bias = 0.15*pi/180;                % rad/s


% Gyroscope white noise
sigma_gyro_noise = 0.05*pi/180;                % rad/s


% GPS
sigma_gps_position = 2.0;       % m
sigma_gps_velocity = 0.15;      % m/s


% Doppler beacon
sigma_doppler_range = 0.5;      % m
sigma_doppler_rr = 0.05;        % m/s


%% ========================================================================
% DOPPLER BEACON
% ========================================================================

beacon_x = 0;
beacon_y = 0;


%% ========================================================================
% MONTE CARLO STORAGE
% ========================================================================

position_error = zeros(N_MC,N,N_FILTERS);
velocity_error = zeros(N_MC,N,N_FILTERS);
heading_error = zeros(N_MC,N,N_FILTERS);
accel_bias_error = zeros(N_MC,N,N_FILTERS);
gyro_bias_error = zeros(N_MC,N,N_FILTERS);
true_bax_mc = zeros(N_MC,1);
true_bay_mc = zeros(N_MC,1);
true_bg_mc = zeros(N_MC,1);


%% Signed per-axis errors

x_error = zeros(N_MC,N,N_FILTERS);
y_error = zeros(N_MC,N,N_FILTERS);
vx_error = zeros(N_MC,N,N_FILTERS);
vy_error = zeros(N_MC,N,N_FILTERS);


%% Per-axis filter covariance 1-sigma

x_sigma_mc = zeros(N_MC,N,N_FILTERS);
y_sigma_mc = zeros(N_MC,N,N_FILTERS);
vx_sigma_mc = zeros(N_MC,N,N_FILTERS);
vy_sigma_mc = zeros(N_MC,N,N_FILTERS);


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
    true_bax_mc(mc) = bax_true;
    true_bay_mc(mc) = bay_true;
    true_bg_mc(mc) = bg_true;


    %% ====================================================================
    % COMMON IMU SENSOR REALIZATION
    %
    % ALL FOUR FILTERS RECEIVE EXACTLY THE SAME DATA.
    % ====================================================================

    ax_meas = ax_body_truth + bax_true + sigma_accel_noise*randn(1,N);
    ay_meas = ay_body_truth + bay_true + sigma_accel_noise*randn(1,N);
    gyro_meas = gyro_truth + bg_true + sigma_gyro_noise*randn(1,N);


    %% ====================================================================
    % COMMON GPS REALIZATION
    % ====================================================================

    gps_x = NaN(1,N);
    gps_y = NaN(1,N);
    gps_vx = NaN(1,N);
    gps_vy = NaN(1,N);

    for k = 1:gps_step:N

        gps_x(k) = x_true(k) + sigma_gps_position*randn();
        gps_y(k) = y_true(k) + sigma_gps_position*randn();
        gps_vx(k) = vx_true(k) + sigma_gps_velocity*randn();
        gps_vy(k) = vy_true(k) + sigma_gps_velocity*randn();

    end


    %% ====================================================================
    % COMMON DOPPLER REALIZATION
    % ====================================================================

    doppler_range = NaN(1,N);
    doppler_rr = NaN(1,N);


    for k = 1:doppler_step:N

        dx = x_true(k) - beacon_x;
        dy = y_true(k) - beacon_y;
        range_true = sqrt(dx^2 + dy^2);
        range_rate_true = ( dx*vx_true(k) + dy*vy_true(k) ) / range_true;
        doppler_range(k) = range_true + sigma_doppler_range*randn();
        doppler_rr(k) = range_rate_true + sigma_doppler_rr*randn();

    end


    %% ====================================================================
    % INITIALIZE FOUR EKFs
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
    vx_est = zeros(N_FILTERS,N);
    vy_est = zeros(N_FILTERS,N);
    heading_est = zeros(N_FILTERS,N);
    bax_est = zeros(N_FILTERS,N);
    bay_est = zeros(N_FILTERS,N);
    bg_est = zeros(N_FILTERS,N);

    for f = 1:N_FILTERS


        x_est(f,1) = X(1,f);
        y_est(f,1) = X(2,f);
        vx_est(f,1) = X(3,f);
        vy_est(f,1) = X(4,f);
        heading_est(f,1) = X(5,f);

        x_sigma_mc(mc,1,f) = sqrt(P0(1,1));
        y_sigma_mc(mc,1,f) = sqrt(P0(2,2));
        vx_sigma_mc(mc,1,f) = sqrt(P0(3,3));
        vy_sigma_mc(mc,1,f) = sqrt(P0(4,4));

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


            % Biases constant
            X_pred(6) = X(6,f);
            X_pred(7) = X(7,f);
            X_pred(8) = X(8,f);

            %% ============================================================
            % PROCESS MODEL JACOBIAN
            % ============================================================

            daNx_dpsi = -s*ax_corrected -c*ay_corrected;
            daNy_dpsi = c*ax_corrected -s*ay_corrected;
            F = eye(8);

            % Position / velocity
            F(1,3) = dt;
            F(2,4) = dt;

            % Heading -> navigation acceleration
            F(1,5) = 0.5*daNx_dpsi*dt^2;
            F(2,5) = 0.5*daNy_dpsi*dt^2;
            F(3,5) = daNx_dpsi*dt;
            F(4,5) = daNy_dpsi*dt;

            % X accelerometer bias

            F(1,6) = -0.5*c*dt^2;
            F(2,6) = -0.5*s*dt^2;
            F(3,6) = -c*dt;
            F(4,6) = -s*dt;

            % Y accelerometer bias
            F(1,7) = 0.5*s*dt^2;
            F(2,7) = -0.5*c*dt^2;
            F(3,7) = s*dt;
            F(4,7) = -c*dt;

            % Gyroscope bias affects heading
            F(5,8) = -dt;

            %% ============================================================
            % IMU PROCESS NOISE MATRIX
            % ============================================================

            G = zeros(8,3);


            % Body X accelerometer noise

            G(1,1) = 0.5*c*dt^2;
            G(2,1) = 0.5*s*dt^2;
            G(3,1) = c*dt;
            G(4,1) = s*dt;

            % Body Y accelerometer noise

            G(1,2) = -0.5*s*dt^2;
            G(2,2) = 0.5*c*dt^2;
            G(3,2) = -s*dt;
            G(4,2) = c*dt;

            % Gyroscope white noise

            G(5,3) = dt;


            Qimu = G * diag([

                    sigma_accel_noise^2;

                    sigma_accel_noise^2;

                    sigma_gyro_noise^2

                ]) * G';


            %% Small bias random walk inside filter

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
            % DOPPLER UPDATE
            % ============================================================

            if use_doppler(f) && ~isnan(doppler_range(k))


                px = X(1,f) - beacon_x;
                py = X(2,f) - beacon_y;
                vx = X(3,f);
                vy = X(4,f);
                r = sqrt(px^2 + py^2);

                if r < 1e-6

                    r = 1e-6;

                end

                q = px*vx + py*vy;
                rr = q/r;


                %% Nonlinear measurement

                h = [

                    r;

                    rr

                ];


                z = [

                    doppler_range(k);

                    doppler_rr(k)

                ];


                %% Doppler Jacobian

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
            % GPS UPDATE
            % ============================================================

            if use_gps(f) && ~isnan(gps_x(k))


                z = [

                    gps_x(k);
                    gps_y(k);
                    gps_vx(k);
                    gps_vy(k)
                ];


                h = [

                    X(1,f);
                    X(2,f);
                    X(3,f);
                    X(4,f)
                ];


                H = [

                    1,0,0,0,0,0,0,0;
                    0,1,0,0,0,0,0,0;
                    0,0,1,0,0,0,0,0;
                    0,0,0,1,0,0,0,0
                ];


                R = diag([

                    sigma_gps_position^2;
                    sigma_gps_position^2;
                    sigma_gps_velocity^2;
                    sigma_gps_velocity^2
                ]);


                innovation = z-h;
                S = H*P(:,:,f)*H' + R;
                K = P(:,:,f)*H'/S;
                X(:,f) = X(:,f) + K*innovation;

                % Wrap heading

                X(5,f) = atan2( sin(X(5,f)), cos(X(5,f)) );
                I = eye(8);
                P(:,:,f) = (I-K*H) * P(:,:,f) * (I-K*H)' + K*R*K';

            end


            %% ============================================================
            % STORE
            % ============================================================

            x_est(f,k) = X(1,f);
            y_est(f,k) = X(2,f);
            vx_est(f,k) = X(3,f);
            vy_est(f,k) = X(4,f);
            heading_est(f,k) = X(5,f);
            bax_est(f,k) = X(6,f);
            bay_est(f,k) = X(7,f);
            bg_est(f,k) = X(8,f);

            x_sigma_mc(mc,k,f) = sqrt(P(1,1,f));
            y_sigma_mc(mc,k,f) = sqrt(P(2,2,f));
            vx_sigma_mc(mc,k,f) = sqrt(P(3,3,f));
            vy_sigma_mc(mc,k,f) = sqrt(P(4,4,f));

        end

    end


    %% ====================================================================
    % ERRORS
    % ====================================================================

    for f = 1:N_FILTERS


        %% Position magnitude
        position_error(mc,:,f) = sqrt( (x_est(f,:) - x_true).^2 + (y_est(f,:) - y_true).^2 );

        %% Velocity magnitude
        velocity_error(mc,:,f) = sqrt( (vx_est(f,:) - vx_true).^2 + (vy_est(f,:) - vy_true).^2 );

        %% Heading error
        heading_difference = heading_est(f,:) - heading_true;
        heading_error(mc,:,f) = atan2( sin(heading_difference), cos(heading_difference) );

        %% Accelerometer bias vector error
        accel_bias_error(mc,:,f) = sqrt( (bax_est(f,:) - bax_true).^2 + (bay_est(f,:) - bay_true).^2 );

        %% Gyroscope bias error
        gyro_bias_error(mc,:,f) = bg_est(f,:) - bg_true;

        %% Signed per-axis errors
        x_error(mc,:,f) = x_est(f,:) - x_true;
        y_error(mc,:,f) = y_est(f,:) - y_true;
        vx_error(mc,:,f) = vx_est(f,:) - vx_true;
        vy_error(mc,:,f) = vy_est(f,:) - vy_true;

    end

end


%% ========================================================================
% MONTE CARLO RMSE
% ========================================================================

position_rmse = zeros(N,N_FILTERS);
velocity_rmse = zeros(N,N_FILTERS);
heading_rmse = zeros(N,N_FILTERS);
accel_bias_rmse = zeros(N,N_FILTERS);
gyro_bias_rmse = zeros(N,N_FILTERS);

for f = 1:N_FILTERS

    position_rmse(:,f) = sqrt( mean( position_error(:,:,f).^2, 1 ) )';
    velocity_rmse(:,f) = sqrt( mean( velocity_error(:,:,f).^2, 1 ) )';
    heading_rmse(:,f) = sqrt( mean( heading_error(:,:,f).^2, 1 ) )';
    accel_bias_rmse(:,f) = sqrt( mean( accel_bias_error(:,:,f).^2, 1 ) )';
    gyro_bias_rmse(:,f) = sqrt( mean( gyro_bias_error(:,:,f).^2, 1 ) )';

end


%% ========================================================================
% MONTE CARLO PER-AXIS 1-SIGMA STATISTICS
%
% sigma_mean : filter covariance 1-sigma, averaged over all runs
%
% sigma_emp  : empirical 1-sigma, std of the signed error
%              across all Monte Carlo runs
% ========================================================================

x_sigma_mean = zeros(N,N_FILTERS);
y_sigma_mean = zeros(N,N_FILTERS);
vx_sigma_mean = zeros(N,N_FILTERS);
vy_sigma_mean = zeros(N,N_FILTERS);

x_sigma_emp = zeros(N,N_FILTERS);
y_sigma_emp = zeros(N,N_FILTERS);
vx_sigma_emp = zeros(N,N_FILTERS);
vy_sigma_emp = zeros(N,N_FILTERS);

for f = 1:N_FILTERS

    x_sigma_mean(:,f) = mean( x_sigma_mc(:,:,f), 1 )';
    y_sigma_mean(:,f) = mean( y_sigma_mc(:,:,f), 1 )';
    vx_sigma_mean(:,f) = mean( vx_sigma_mc(:,:,f), 1 )';
    vy_sigma_mean(:,f) = mean( vy_sigma_mc(:,:,f), 1 )';

    x_sigma_emp(:,f) = std( x_error(:,:,f), 0, 1 )';
    y_sigma_emp(:,f) = std( y_error(:,:,f), 0, 1 )';
    vx_sigma_emp(:,f) = std( vx_error(:,:,f), 0, 1 )';
    vy_sigma_emp(:,f) = std( vy_error(:,:,f), 0, 1 )';

end


%% ========================================================================
% PLOT 1
% VEHICLE TRAJECTORY
% ========================================================================

figure;
plot( x_true, y_true, "LineWidth",2 );
hold on;
plot( beacon_x, beacon_y, "o", "MarkerSize",10 );
plot( x_true(1), y_true(1), "s", "MarkerSize",8 );
grid on;
axis equal;
xlabel("X [m]");
ylabel("Y [m]");
title("Truth Vehicle Trajectory");
legend( "Vehicle", "Doppler Beacon", "Start" );

%% ========================================================================
% PLOT 2
% TRUE IMU INPUTS
% ========================================================================

figure;
subplot(3,1,1);
plot(time,ax_body_truth,"LineWidth",2);
grid on;
ylabel("a_x^b [m/s^2]");
title("Perfect IMU Inputs");
subplot(3,1,2);
plot(time,ay_body_truth,"LineWidth",2);
grid on;
ylabel("a_y^b [m/s^2]");
subplot(3,1,3);
plot( time, gyro_truth*180/pi, "LineWidth",2 );
grid on;
ylabel("\omega_z [deg/s]");
xlabel("Time [s]");


%% ========================================================================
% PLOT 3
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
legend( filter_names{1}, filter_names{2}, filter_names{3}, filter_names{4} );


%% ========================================================================
% PLOT 4
% VELOCITY RMSE
% ========================================================================

figure;
hold on;

for f = 1:N_FILTERS

    plot( time, velocity_rmse(:,f), "LineWidth",2 );

end


grid on;
xlabel("Time [s]");
ylabel("2D Velocity RMSE [m/s]");
title("Monte Carlo Velocity RMSE");
legend( filter_names{1}, filter_names{2}, filter_names{3}, filter_names{4} );


%% ========================================================================
% PLOT 5
% HEADING RMSE
% ========================================================================

figure;
hold on;

for f = 1:N_FILTERS

    plot( time, heading_rmse(:,f)*180/pi, "LineWidth",2 );

end


grid on;
xlabel("Time [s]");
ylabel("Heading RMSE [deg]");
title("Monte Carlo Heading RMSE");
legend( filter_names{1}, filter_names{2}, filter_names{3}, filter_names{4} );


%% ========================================================================
% PLOT 6
% GYRO BIAS RMSE
% ========================================================================

figure;
hold on;

for f = 1:N_FILTERS

    plot( time, gyro_bias_rmse(:,f)*180/pi, "LineWidth",2 );

end


grid on;
xlabel("Time [s]");
ylabel("Gyro Bias RMSE [deg/s]");
title("Monte Carlo Gyroscope Bias RMSE");
legend( filter_names{1}, filter_names{2}, filter_names{3}, filter_names{4} );

%% ========================================================================
% PLOT 7
% ACCELEROMETER BIAS RMSE
% ========================================================================

figure;

hold on;


for f = 1:N_FILTERS

    plot( time, accel_bias_rmse(:,f), "LineWidth",2 );

end


grid on;
xlabel("Time [s]");
ylabel("Accelerometer Bias RMSE [m/s^2]");
title("Monte Carlo Accelerometer Bias RMSE");
legend( filter_names{1}, filter_names{2}, filter_names{3}, filter_names{4} );


%% ========================================================================
% PLOT 8
% MONTE CARLO POSITION ERROR PER AXIS
%
% Light grey : all Monte Carlo error traces
% Red        : filter covariance 1-sigma (mean over runs)
% Blue       : empirical 1-sigma (std across runs)
% ========================================================================

grey_color = [0.8 0.8 0.8];

figure;

for f = 1:N_FILTERS

    %% X position error

    subplot(2,4,f);
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

    subplot(2,4,f+4);
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
% PLOT 9
% MONTE CARLO VELOCITY ERROR PER AXIS
%
% Light grey : all Monte Carlo error traces
% Red        : filter covariance 1-sigma (mean over runs)
% Blue       : empirical 1-sigma (std across runs)
% ========================================================================

figure;

for f = 1:N_FILTERS

    %% Vx velocity error

    subplot(2,4,f);
    hold on;

    h_grey = plot( time, squeeze(vx_error(:,:,f))', "Color", grey_color );

    h_cov = plot( time, vx_sigma_mean(:,f), "r", "LineWidth", 1.5 );
    plot( time, -vx_sigma_mean(:,f), "r", "LineWidth", 1.5 );

    h_emp = plot( time, vx_sigma_emp(:,f), "b", "LineWidth", 1.5 );
    plot( time, -vx_sigma_emp(:,f), "b", "LineWidth", 1.5 );

    grid on;
    xlabel("Time [s]");
    ylabel("Vx Velocity Error [m/s]");
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

    %% Vy velocity error

    subplot(2,4,f+4);
    hold on;

    plot( time, squeeze(vy_error(:,:,f))', "Color", grey_color );

    plot( time,  vy_sigma_mean(:,f), "r", "LineWidth", 1.5 );
    plot( time, -vy_sigma_mean(:,f), "r", "LineWidth", 1.5 );

    plot( time,  vy_sigma_emp(:,f), "b", "LineWidth", 1.5 );
    plot( time, -vy_sigma_emp(:,f), "b", "LineWidth", 1.5 );

    grid on;
    xlabel("Time [s]");
    ylabel("Vy Velocity Error [m/s]");
    title(filter_names{f});

end


%% ========================================================================
% FINAL PERFORMANCE
% ========================================================================

fprintf("\n");
fprintf("=======================================================================\n");
fprintf(" FINAL 2D MONTE CARLO PERFORMANCE\n");
fprintf("=======================================================================\n");
fprintf( "%-24s %-12s %-12s %-12s %-12s\n", "Filter", "Pos[m]", "Vel[m/s]", "Head[deg]", "Bg[deg/s]" );
fprintf("-----------------------------------------------------------------------\n");


for f = 1:N_FILTERS
    fprintf( "%-24s %-12.4f %-12.4f %-12.4f %-12.5f\n", filter_names{f}, position_rmse(end,f), velocity_rmse(end,f), heading_rmse(end,f)*180/pi, gyro_bias_rmse(end,f)*180/pi );
end


fprintf("=======================================================================\n");
