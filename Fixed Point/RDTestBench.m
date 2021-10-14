
% Bookkeeping
tic;

% Unpack variables
filein = load('Reference/LargeResponse.mat');
x_large = filein.rx_sig;
filein = load('Reference/SmallResponse.mat');
x_small = filein.rx_sig;
filein = load('Reference/ReferenceSignal.mat');
x_ref = filein.x_ref;

% Loop through chirps for each signal
y_large = nan(size(x_large));
y_small = nan(size(x_small));

for ch = 1:size(x_large, 2)
    for rx = 1:2
        
        y_large(:,ch,rx) = RangeProcessingFP(x_large(:,ch,rx), x_ref);
        y_small(:,ch,rx) = RangeProcessingFP(x_small(:,ch,rx), x_ref);
        
    end
end

y_rd_large = nan(size(y_large));
y_rd_small = nan(size(y_small));
for rb = 1:size(x_large, 1)
    for rx = 1:2
        
        y_rd_large(rb,:,rx) = DopplerProcessingFP(y_large(rb,:,rx)');
        y_rd_small(rb,:,rx) = DopplerProcessingFP(y_small(rb,:,rx)');
        
    end
end

% Bookkeeping
toc;