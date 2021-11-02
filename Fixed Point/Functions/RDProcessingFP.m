function y = RDProcessingFP(x, x_ref)
%RDPROCESSINGFP Combined fixed-point processing functions
%   Performs range processing followed by doppler processing in fixed
%   point, used for verification of correct operation of individual fixed
%   point functions.

%% Setup

% Generate output matrices
y = complex(zeros(1250, 1024, 2));

%% Range-Doppler processing

% Loop through chirps and perform range processing
for chirp = 1:1024
    for channel = 1:2
        
        y(:,chirp,channel) = RangeProcessingFP(x(:,chirp,channel), x_ref);
        
    end
end

% Loop through range bins and perform Doppler processing
for bin = 1:1250
    for channel = 1:2
        
        y(bin,:,channel) = DopplerProcessingFP(y(bin,:,channel)');
        
    end
end



end

