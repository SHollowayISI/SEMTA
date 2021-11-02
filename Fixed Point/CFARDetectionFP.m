function [indexList, powerList] = CFARDetectionFP(dataCube, ...
    offsetList, maxNumOutputs, threshFactor, ...
    N_rng, N_dop, Ntrain_rng, Ntrain_dop, Nguard_rng, Nguard_dop)
    
    % Initialize output lists to a maximum size
    indexList = zeros(maxNumOutputs,1);    
    powerList = zeros(maxNumOutputs,1);
    listCounter = int32(1);

    % Calculate index limits
    R_maxDist = Ntrain_rng + Nguard_rng;
    D_maxDist = Ntrain_dop + Nguard_dop;
    R_limits = [R_maxDist + 1, N_rng - R_maxDist];
    D_limits = [D_maxDist + 1, N_dop - D_maxDist];
        
    % Loop through indices to test
    for R_ind = R_limits(1):R_limits(2)
        for D_ind = D_limits(1):D_limits(2)
            
            % Determine linear index of CUT
            linIndex = sub2ind([N_rng, N_dop], R_ind, D_ind);
            
            % Average over power
            threshold = mean(dataCube(linIndex + offsetList)) * threshFactor;
 
            if dataCube(linIndex) > threshold
                
                % Add entry to list
                indexList(listCounter) = linIndex;
                powerList(listCounter) = dataCube(linIndex);
                listCounter = listCounter + 1;
                
                % Break at end of list
                if listCounter > maxNumOutputs
                    return
                end
            end
        end
    end
end

