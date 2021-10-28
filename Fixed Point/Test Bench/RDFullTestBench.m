
% Bookkeeping
tic;

% Unpack variables
filein = load('Reference/LargeResponse.mat');
x_large = filein.rx_sig;
filein = load('Reference/SmallResponse.mat');
x_small = filein.rx_sig;
filein = load('Reference/ReferenceSignal.mat');
x_ref = filein.x_ref;

% Run for each signal
y_large = RDProcessingFP_wrapper_fixpt_mex('RDProcessingFP_wrapper_fixpt', x_large, x_ref);
y_small = RDProcessingFP_wrapper_fixpt_mex('RDProcessingFP_wrapper_fixpt', x_small, x_ref);

% Bookkeeping
toc;