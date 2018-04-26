#!/usr/bin/python
from __future__ import division
import numpy as np
import traceback

class TimeSignal1D(object):
    def __init__(self, samples, sampling_freq=1.0, start_time=0.0, copy=True):
        '''
        Represents a time signal sampled at a constant rate
        samples - 1D numpy array
        sampling_freq - sampling_frequency in Hz
        start_time - start time (in seconds) for the corresponding time values
        '''
        if not isinstance(samples, np.ndarray):
            raise TypeError("'samples' argument must be an np.ndarray")
        if samples.ndim != 1:
            raise ValueError("'samples' must be 1-dimensional")
        self.samples = samples.copy() if copy else samples
        self.sampling_freq = float(sampling_freq)
        self.start_time = start_time

    @classmethod
    def from_function(cls, funct, start_t, end_t, sampling_freq):
        '''
        Creates a TimeSignal1D by sampling from a callback function.
        @param funct Callback that takes in a single float time value and returns a single float output value
        @param start_t Start time in seconds (float)
        @param end_t End time in seconds (float)
        @param sampling_freq Sampling frequency in Hz
        '''
        time_values = np.arange(start_t, end_t, 1 / sampling_freq)
        signal_values = np.empty(time_values.size)
        for i, val in enumerate(time_values):
            signal_values[i] = funct(time_values[i])
        return cls(samples=signal_values,
                   sampling_freq=(time_values.size - 1) / (time_values[-1] - time_values[0]),
                   start_time=start_t,
                   copy=False) # No need to copy since array is temporary

    def __len__(self):
        return len(self.samples)

    def duration(self):
        ''' Returns the duration (in seconds) of the signal '''
        return (len(self.samples) - 1) / self.sampling_freq

    def get_time_values(self):
        ''' Returns an ndarray with the corresponding time values for all signal samples '''
        return np.linspace(self.start_time, self.start_time + self.duration(), len(self.samples), endpoint=True)

    def idx_at_time(self, time):
        ''' Returns the array idx for the value corresponding to a specific time '''
        delta = time - self.start_time
        round_idx = np.round(delta * self.sampling_freq)
        float_idx = delta * self.sampling_freq
        if np.isclose(1.0 - np.fabs(float_idx - round_idx), 1.0):
            return int(round(float_idx))
        else:
            return int(np.floor(float_idx))

    def at_time(self, time):
        ''' Returns the signal's value at a specific time '''
        return self.samples[self.idx_at_time(time)]

    def time_slice(self, start=None, end=None, copy=True):
        ''' Returns a slice of the signal based on start and end times '''
        start = self.start_time if start is None else start
        end = self.start_time + self.duration if end is None else end
        samples = self.samples[self.idx_at_time(start): self.idx_at_time(end) + 1]
        return TimeSignal1D(samples, self.sampling_freq, start_time=start, copy=copy)

    def set_time_slice(self, start, end, signal):
        ''' Writes the values of a TimeSignal1D to a slice of the current one '''
        if len(self.time_slice(start, end)) != len(signal):
            raise RuntimeError("Destination slice has different size from input signal")
        self.samples[self.idx_at_time(start): self.idx_at_time(end) + 1] = signal.samples

    def upsample_linear(self, upsample_factor):
        '''
        Linearly interpolates intermediate values to return an upsampled version of the signal
        upsample_factor - float (factor by which to increase the number of samples
        '''
        new_n = int(round(len(self) * upsample_factor))
        new_time_vals = np.linspace(self.start_time, self.start_time + self.duration(), new_n)
        upsamp_signal = np.interp(new_time_vals, self.get_time_values(), self.samples)
        dt = new_time_vals[1] - new_time_vals[0]
        new_freq = 1.0 / dt
        return TimeSignal1D(upsamp_signal, sampling_freq=new_freq, start_time=self.start_time)

    def plot(self, plotting_function=None, **kwargs):
        '''  Convenience function for plotting the signal using a provided plotting function '''
        args = (self.get_time_values(), self.samples)
        if plotting_function is None:
            try:
                import matplotlib
                matplotlib.pyplot.plot(*args, **kwargs)
            except BaseException:
                print traceback.format_exc()
        else:
            try:
                plotting_function(*args, **kwargs)
            except BaseException:
                print traceback.format_exc()


def make_delayed_signal(reference_signal, delay, total_duration):
    '''
    Creates a TimeSignal1D instance from by shifting a pulse signal
    @param reference_signal TimeSignal1D instance to generate delayed signal from
    @param delay Time (in seconds) to delay the signal
    @param total_duration Desired total_duration (in seconds) of the returned signal
    '''
    if not isinstance(reference_signal, TimeSignal1D):
        raise TypeError("'pulse_signal' must be an instance of TimeSignal1D")
    assert total_duration > 0, 'Durations must be positive values'

    num_samples_float = total_duration * reference_signal.sampling_freq + 1.0
    num_samples_round = np.round(num_samples_float)
    if np.isclose(1.0 - np.fabs(num_samples_float - num_samples_round), 1.0):
        n = int(num_samples_round)
    else:
        n = int(np.floor(num_samples_float))

    out_signal = TimeSignal1D(
        samples=np.zeros(n),
        sampling_freq=reference_signal.sampling_freq,
        start_time=delay + reference_signal.start_time + (reference_signal.duration() - total_duration) / 2.0
    )
    copy_start = max(reference_signal.start_time + delay, out_signal.start_time)
    copy_end = min(
        reference_signal.start_time + reference_signal.duration() + delay,
        out_signal.start_time + total_duration
    )

    out_signal.set_time_slice(copy_start, copy_end, reference_signal)
    return out_signal


def make_delayed_signals_from_DTOA(reference_signal, dtoa, total_duration=None, include_reference=True):
    '''
    Generates a list of TimeSignal1D's based on dtoa measurements
    @param reference_signal - TimeSignal1D instance to generate delayed signals from
    @param dtoa List of differences in time of arrival in seconds
    @param total_duration Desired total_duration (in seconds) of the returned signals
    @param include_reference If True, the reference signal will be prepenced to the output
    '''
    signals = []
    if total_duration is None:
        total_duration = reference_signal.duration()
    if include_reference:
        signals.append(make_delayed_signal(reference_signal, 0, total_duration))

    for delay in dtoa:
        signals.append(make_delayed_signal(reference_signal, delay, total_duration))

    return signals

def plot_signals(signals, plotting_function=None, **kwargs):
    ''' Convenience function to plot a sequence of TimeSignal1D's '''
    if not hasattr(type(signals), '__iter__'):
        raise TypeError("'signals' must be a sequence")
    args = []
    for s in signals:
        if not isinstance(s, TimeSignal1D):
            raise TypeError("'signals' must be a sequence of TimeSignal1D's")
        args.append(s.get_time_values())
        args.append(s.samples)

    if plotting_function is None:
        try:
            import matplotlib
            matplotlib.pyplot.plot(*args, **kwargs)
        except BaseException:
            print traceback.format_exc()
    else:
        try:
            plotting_function(*args, **kwargs)
        except BaseException:
            print traceback.format_exc()
