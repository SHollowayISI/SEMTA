
% Bookkeeping
tic;

% Unpack variables
filein = load('Reference/LargeCube.mat');
y_large = filein.y_large;
filein = load('Reference/SmallCube.mat');
y_small = filein.y_small;
filein = load('Reference/CFAROffsets.mat');
offsetList = filein.offsetList;

% Convert to power
y_small = abs(y_small).^2;
y_large = abs(y_large).^2;

% Set variables
Nguard_rng = 3;
Nguard_dop = 3;
Ntrain_rng = 15;
Ntrain_dop = 2;
Nr = 1250;
Nd = 1024;
Pfa = 1e-6;
maxNumOutputs = 65536;

% Generate index offset list
% NOTE: This should probably be hardcoded for embedded purposes
%{
offsetList = [];

% Top and bottom of square
for ind = 1:Ntrain_dop
    offsetList = [offsetList, ...
        (-R_maxDist - N_rng*(ind + Nguard_dop)):(R_maxDist - N_rng*(ind + Nguard_dop))];
    offsetList = [offsetList, ...
        (-R_maxDist + N_rng*(ind + Nguard_dop)):(R_maxDist + N_rng*(ind + Nguard_dop))];
end

% Left and right of square
for ind = (-Nguard_dop):Nguard_dop
    offsetList = [offsetList, ...
        ind*N_rng - ((Nguard_rng+1):R_maxDist)];
    offsetList = [offsetList, ...
        ind*N_rng + ((Nguard_rng+1):R_maxDist)];
end
%}
    
% Calculate threshold factor
N = length(offsetList);
threshFactor = N * (Pfa ^ (-1/N) - 1);

% Run CFAR test cases
for n = 1:2
    [small_idx{n}, small_pow{n}] = CFARDetectionFP_wrapper_fixpt_mex( ...
        'CFARDetectionFP_wrapper_fixpt', y_small(:,:,n), ...
        offsetList, maxNumOutputs, threshFactor, ...
        Nr, Nd, Ntrain_rng, Ntrain_dop, Nguard_rng, Nguard_dop);
end

for n = 1:2
    [large_idx{n}, large_pow{n}] = CFARDetectionFP_wrapper_fixpt_mex( ...
        'CFARDetectionFP_wrapper_fixpt', y_small(:,:,n), ...
        offsetList, maxNumOutputs, threshFactor, ...
        Nr, Nd, Ntrain_rng, Ntrain_dop, Nguard_rng, Nguard_dop);
end

toc;