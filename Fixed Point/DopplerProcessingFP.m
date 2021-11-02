function y = DopplerProcessingFP(x)
%DOPPLERPROCESSINGFP Fixed point Doppler processing
%   Takes range-processed signal as input, returns FFT

%% Set up FFT object

% Instantiate FFT object
ft = dsp.FFT( ...
    'FFTImplementation',        'Radix-2', ...
    'BitReversedOutput',        false, ...
    'Normalize',                false, ...
    'FFTLengthSource',          'Property', ...
    'FFTLength',                1024, ...
    'WrapInput',                true);

%% Perform Doppler FFT

% Take FFT of input signal
X_f = ft(x);

% FFT shift signal
y = fftshift(X_f);

end

