%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                          %
%           Generated by MATLAB 9.8 and Fixed-Point Designer 7.0           %
%                                                                          %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [indexList,powerList] = CFARDetectionFP_wrapper_fixpt(dataCube,offsetList,maxNumOutputs,threshFactor,N_rng,N_dop,Ntrain_rng,Ntrain_dop,Nguard_rng,Nguard_dop)
    fm = get_fimath();
    dataCube_in = fi( dataCube, 0, 32, 20, fm );
    offsetList_in = fi( offsetList, 1, 14, 0, fm );
    maxNumOutputs_in = fi( maxNumOutputs, 0, 32, 0, fm );
    threshFactor_in = fi( threshFactor, 0, 32, 28, fm );
    N_rng_in = fi( N_rng, 0, 11, 0, fm );
    N_dop_in = fi( N_dop, 0, 11, 0, fm );
    Ntrain_rng_in = fi( Ntrain_rng, 0, 4, 0, fm );
    Ntrain_dop_in = fi( Ntrain_dop, 0, 2, 0, fm );
    Nguard_rng_in = fi( Nguard_rng, 0, 2, 0, fm );
    Nguard_dop_in = fi( Nguard_dop, 0, 2, 0, fm );
    [indexList_out,powerList_out] = CFARDetectionFP_fixpt( dataCube_in, offsetList_in, maxNumOutputs_in, threshFactor_in, N_rng_in, N_dop_in, Ntrain_rng_in, Ntrain_dop_in, Nguard_rng_in, Nguard_dop_in );
    indexList = double( indexList_out );
    powerList = double( powerList_out );
end

function fm = get_fimath()
	fm = fimath('RoundingMethod', 'Floor',...
	     'OverflowAction', 'Wrap',...
	     'ProductMode','FullPrecision',...
	     'MaxProductWordLength', 128,...
	     'SumMode','FullPrecision',...
	     'MaxSumWordLength', 128);
end
