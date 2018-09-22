# Represents a time series signal from n channels in different positions

# Frame of the sensor array, time stamp of signal beginning
Header header

# Number of channels/sensors
int32 channels

# Number of samples in the signal for each sensor
int32 samples

# Number of samples collected per second by each channel
int32 sample_rate


uint16[] data # C1 C2 C3 C4 C1 C2 C3 C4 ...
