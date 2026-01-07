% IMPORTANT
% Preston wrote this and he is less than 50% sure it works so take it with
% many grains of salt


FT1 = readtable("Data\FT1_primary.csv");
FT2 = readtable("Data\FT2_primary.csv");
FT3 = readtable("Data\FT3_primary.csv");
FT4 = readtable("Data\FT4_primary.csv");

FT1_main = FT1(strcmp(FT1.state_name, 'main'), :);
FT2_main = FT2(strcmp(FT2.state_name, 'main'), :);
FT3_main = FT3(strcmp(FT3.state_name, 'main'), :);
FT4_main = FT4(strcmp(FT4.state_name, 'main'), :);

function plotResponses(dataTable, timeHeader, responseHeader)
    % Extract time and response data
    timeData = dataTable.(timeHeader);
    responseData = dataTable.(responseHeader);

    % Compute sampling frequency
    dt = mean(diff(timeData));     % average timestep
    dt
    Fs = 1 / dt;                   % sampling frequency (Hz)
    Fs
    L = length(responseData);      % number of samples

    % Frequency vector (one-sided)
    f = Fs * (0:(L/2)) / L;

    % Compute FFT
    Y = fft(responseData);
    P2 = abs(Y/L);                 % two-sided spectrum
    P1 = P2(1:L/2+1);              % single-sided spectrum
    P1(2:end-1) = 2*P1(2:end-1);   % scale for single-sided amplitude

    % Limit to 0–100 Hz
    maxFreq = 100;
    mask = f <= maxFreq;
    f = f(mask);
    P1 = P1(mask);

    % Create a figure for the subplots
    figure;

    % --- Time-domain plot ---
    subplot(2, 1, 1);
    plot(timeData, responseData);
    title('Time Response');
    xlabel('Time [s]');
    ylabel('Response');
    grid on;

    % --- Frequency-domain plot ---
    subplot(2, 1, 2);
    plot(f, P1);
    title('Frequency Response');
    xlabel('Frequency [Hz]');
    ylabel('|Magnitude|');
    grid on;
end


plotResponses(FT1_main, 'time', 'gyro_roll')

