%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                          %
%           Generated by MATLAB 9.8 and Fixed-Point Designer 7.0           %
%                                                                          %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%#codegen
function y = RangeProcessingFP_fixpt(x, x_ref)
%RANGEPROCESSINGFP Fixed point range processing
%   Takes receieved and reference signals as input, returns correlation
%   along first dimension

%% Set up FFT object

% Instantiate FFT object
fm = get_fimath();

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
X_f = fi(ft(x), 1, 32, 43, fm);

% Multiply by reference signal
X_f(:) = X_f .* x_ref;

% Take inverse FFT
y = fi(ift(X_f), 1, 32, 35, fm);

% Remove out of range bins
y = fi(y(1:1250), 1, 32, 35, fm);


end


function fm = get_fimath()
	fm = fimath('RoundingMethod', 'Floor',...
	     'OverflowAction', 'Wrap',...
	     'ProductMode','FullPrecision',...
	     'MaxProductWordLength', 128,...
	     'SumMode','FullPrecision',...
	     'MaxSumWordLength', 128);
end