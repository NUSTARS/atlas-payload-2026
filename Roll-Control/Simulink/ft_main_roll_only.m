function ft_main_roll_only(flight_csv)
    sgf_order = 3;
    sgf_framelen = 21;

    data = readtable(string(flight_csv));

    % Find where state == 'main'
    start_idx = find(strcmp(data.state_name, 'main'), 1, 'first');
    if isempty(start_idx)
        error(['No "main" state found in ', flight_csv]);
    end




    % Trim and normalize time
    data_main = data(start_idx:end, :);
    data_main.time = data_main.time - data_main.time(1);

    % Remove duplicate time
    [unique_time, unique_idx] = unique(data_main.time, 'stable');
    data_main = data_main(unique_idx, :);

    % Interpolate gyro data
    dt = 0.01; % desired sample interval (s)
    t_uniform = (0:dt:data_main.time(end))';
    gyro_roll_uniform = interp1(data_main.time, data_main.gyro_roll, t_uniform, 'pchip');
    raw_gyro_roll = timeseries(data_main.gyro_roll, data_main.time)
    % assignin('base', [name, '_raw_gyro_roll'], data_main.gyro_roll);
    % Create timeseries
    gyro_roll = timeseries(gyro_roll_uniform, t_uniform);
    % sgf_gyro_roll = timeseries(sgolayfilt(data_main.gyro_roll, sgf_order, sgf_framelen), data_main.time)
    % gyro_roll = 1;
    
    % Parameters for Savitzky-Golay filter
    % Calculate Savitzky-Golay filter coefficients
    [b, g] = sgolay(sgf_order, sgf_framelen);
    % Compute the sampling interval
    dt = 0.01
    % Compute the first derivative
    p = 1; % First derivative
    dy = -conv(gyro_roll_uniform, factorial(p)/dt^p * g(:, p+1), 'same');

    sgf_gyro_roll = timeseries(dy, t_uniform)

    % Export to base workspace
    [~, name, ~] = fileparts(flight_csv);
    assignin('base', [name, '_raw_gyro_roll'], raw_gyro_roll);
    assignin('base', [name, '_gyro_roll'], gyro_roll);
    assignin('base', [name, '_sgf_dgyro_roll'], sgf_gyro_roll);

end
