function y = RangeProcessingFP(x, x_ref)
%RANGEPROCESSINGFP Fixed point range processing
%   Takes receieved and reference signals as input, returns FFT

%% Calculate FFT properties

% Calculate FFT size
N = 2^ceil(log2(length(x)));

% Generate twiddle factors
w_f = fi_radix2twiddles(N);
w_r = conj(w_f);

%% Perform correlation

% Take forward FFT of received signal
y = fi_m_radix2fft_algorithm1_6_2(x, w_f);

% Multiply by windowed, conjguated, FFT'd reference signal
y = y .* x_ref;

% Take reverse FFT for correlation response
y = fi_m_radix2fft_algorithm1_6_2(y, w_r);

end

