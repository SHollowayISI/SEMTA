function y = RangeProcessingFP(x, x_ref)
%RANGEPROCESSINGFP Fixed point range processing
%   Takes receieved and reference signals as input, returns correlation
%   along first dimension

%% Set up FFT object

% Instantiate FFT object
ft = dsp.FFT( ...
    'FFTImplementation',        'Radix-2', ...
    'BitReversedOutput',        false, ...
    'Normalize',                false, ...
    'FFTLengthSource',          'Property', ...
    'FFTLength',                2048, ...
    'WrapInput',                true);

% Instantiate IFFT object
ift = dsp.IFFT( ...
    'FFTImplementation',        'Radix-2', ...
    'Normalize',                false, ...
    'FFTLengthSource',          'Property', ...
    'FFTLength',                2048, ...
    'WrapInput',                true);

%% Perform correlation

% Take FFT of input signal
X_f = ft(x);

% Multiply by reference signal
X_f = X_f .* x_ref;

% Take inverse FFT
y = ift(X_f);

% Remove out of range bins
y = y(1:1250);


end

