### Settings ###

# Import statements
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import os
import numpy as np
import scipy.constants as constants
import scipy.io as sio
import csv



### Function declarations ###

# Single unit Kalman filter
def KalmanFilterSingle(trackSingleIn, trackParams, frame, passDirection):

    # Unpack variables
    numFr = trackSingleIn['n_fr']

    # Unpack measurement values
    meas = dict()
    for key in trackSingleIn['meas'].keys():
        meas[key] = trackSingleIn['meas'][key][frame]

    # Check if track needs to be initialized
    if (passDirection == 'forward' and frame == 0) or (passDirection == 'reverse' and frame == numFr-1):

        # Use measurement as state prediction
        X_init = np.matrix([[
            meas['range'] * np.cos(meas['az'] * np.pi / 180),
            meas['vel'] * np.cos(meas['az'] * np.pi / 180),
            0,
            meas['range'] * np.sin(meas['az'] * np.pi / 180),
            meas['vel'] * np.sin(meas['az'] * np.pi / 180),
            0
        ]]).T

        # Use measurement uncertainties as covariance prediction
        P_init = GenerateStateCovariance(meas, trackParams)

        # Save results
        return SaveStepData(trackSingleIn, frame, X_init, P_init, X_init, P_init, meas['time'])

    else:

        # Calculate time step since previous hit
        if passDirection == 'forward':
            lastHitFrame = frame-1
        else:
            lastHitFrame = frame+1
        Tm = trackSingleIn['meas']['time'][frame] - trackSingleIn['meas']['time'][lastHitFrame]


    ## Calculate Model Matrices

    # Measurement covariance matrix
    if meas:
        R_full = np.diag(GenerateStateCovariance(meas, trackParams))
    else:
        meas_temp = dict()
        for key in trackSingleIn['meas'].keys():
            meas_temp[key] = trackSingleIn['meas'][key][lastHitFrame]
        R_full = np.diag(GenerateStateCovariance(meas_temp, trackParams))

    R = np.diag([R_full[i] for i in [0, 3]])

    # Process covariance matrix (NCA model)
    Q_1d = np.array([[(Tm**4)/4, (Tm**3)/2, (Tm**2)/2],
                     [(Tm**3)/2, (Tm**2),   (Tm)],
                     [(Tm**2)/2, (Tm),      1]])
    Q = np.asmatrix(np.concatenate((
                np.concatenate((Q_1d*(trackParams['sigma_v'][0]**2), np.zeros(Q_1d.shape)), axis=1), 
                np.concatenate((np.zeros(Q_1d.shape), Q_1d*(trackParams['sigma_v'][1]**2)), axis=1)), 
                axis=0))

    # Kinematic process matrix (NCA model)
    F_1d = np.array([[1,    (Tm),   (Tm**2)/2],
                     [0,    1,      (Tm)],
                     [0,    0,      1]])
    F = np.asmatrix(np.concatenate((
                np.concatenate((F_1d, np.zeros(F_1d.shape)), axis=1), 
                np.concatenate((np.zeros(F_1d.shape), F_1d), axis=1)), 
                axis=0))


    ## Prediction step

    # Predict kinematic vector
    X_pre = F * np.asmatrix(trackSingleIn['estimate'][lastHitFrame]['state'])

    # Predicted kinematic covariance
    P_pre = (F * (np.asmatrix(trackSingleIn['estimate'][lastHitFrame]['covar'])* F.T)) + Q


    ## Calculate measurement matrices

    # Measurement vector
    Z = np.matrix([
            meas['range'] * np.cos(meas['az'] * np.pi / 180),
            meas['range'] * np.sin(meas['az'] * np.pi / 180)])

    # Measurement matrix
    H = np.matrix([[1, 0, 0, 0, 0, 0],
                  [0, 0, 0, 1, 0, 0]])

    # Measurement residual
    Z_res = Z.T - (H * X_pre)


    ## Estimation Step

    # Measurement residual covariance
    S = H * (P_pre * H.T) + R

    # Kalman matrix
    K = (P_pre * H.T) * S.I

    # Estimated kinematic vector
    X_est = X_pre + (K * Z_res)

    # Estimated kinematic covariance
    P_est = P_pre - (K * (H * P_pre))

    # Save data
    return SaveStepData(trackSingleIn, frame, X_est, P_est, X_pre, P_pre, meas['time'])

# Save data for each Kalman filter step
def SaveStepData(trackIn, frame, X_est, P_est, X_pre, P_pre, time):

    # Save estimates
    trackIn['estimate'][frame] = dict(
        state   = X_est,
        covar   = P_est,
        cart    = [X_est[i] for i in [0, 3]],
        az      = np.arctan(X_est[3] / X_est[0]) * 180 / np.pi,
        range   = np.linalg.norm([X_est[i] for i in [0, 3]]),
        speed   = np.linalg.norm([X_est[i] for i in [1, 4]]),
        time    = time
    )

    # Save predictions
    trackIn['prediction'][frame] = dict(
        state   = X_pre,
        covar   = P_pre,
        cart    = [X_pre[i] for i in [0, 3]],
        az      = np.arctan(X_pre[3] / X_pre[0]) * 180 / np.pi,
        range   = np.linalg.norm([X_pre[i] for i in [0, 3]]),
        speed   = np.linalg.norm([X_pre[i] for i in [1, 4]]),
        time    = time
    )

    # Return data structure
    return trackIn

# Calculate state uncertainty matrix
def GenerateStateCovariance(meas, trackParams):

    # Calculate measurement variances
    sig_R, sig_A, sig_V = CalculateVariance(meas)

    # Convert to radians
    sig_A = sig_A * np.pi / 180

    # Calculate speed variance (assuming symmetric uniform distribution)
    speed_unc = 0.57735 * trackParams['max_vel']
    accel_unc = 0.57735 * trackParams['max_acc']

    # Calculate uncertainty matrix
    az_rad = meas['az'] * np.pi / 180
    P = np.diag([
        (sig_R * np.cos(az_rad))**2 
            + (sig_A * meas['range'] * np.sin(az_rad))**2,
        (sig_V * np.cos(az_rad))**2 
            + (speed_unc * meas['range'] * np.sin(az_rad))**2
            + (sig_A * meas['vel'] * np.sin(az_rad))**2,
        accel_unc,
        (sig_R * np.sin(az_rad))**2 
            + (sig_A * meas['range'] * np.cos(az_rad))**2,
        (sig_V * np.sin(az_rad))**2 
            + (speed_unc * meas['range'] * np.cos(az_rad))**2
            + (sig_A * meas['vel'] * np.cos(az_rad))**2,
        accel_unc
    ])
    return P

# Calculate measurement variance from empirical curves
def CalculateVariance(meas):

    # Unpack variables
    steer = meas['steer']
    range = meas['range']
    theta = meas['az']
    SNR = meas['SNR']
    C = constants.speed_of_light
    

    ## Calculate range variance

    # Calculate fading factor due to incomplete pulse return
    fade = np.min([1,
        2 * range / (C * 10e-6),
        (5e-5 - (2 * range / C)) / 10e-6
    ])

    # Calculate variance accounting only for SNR
    sig_R = 7.8927 * np.power(10, -SNR / 20)

    # Incorporate fading resolution loss
    sig_R /= fade**2


    ## Calculate angle variance

    # Calculate variance accounting only for SNR
    sig_A = (6.335 / 1.4817) / np.sqrt(2 * np.power(10, SNR / 10))

    # Adjust variance for beam steering loss
    sig_A /= np.cos(steer * np.pi / 180)**2

    # Adjust vaiance for offest from beam center
    offset = np.abs(theta - steer)
    sig_A *= np.sqrt(1 + (1.4817 * offset / 6.335)**2)


    ## Calculate velocity variance

    # Variance due only to SNR
    sig_V = np.power(10, -1.2517 - 0.0044602 * SNR)

    # Return tuple of results
    return (sig_R, sig_A, sig_V)

# Single unit, single pass processing
def TrackingSingleUnit(trackSingleIn, trackParams, passDirection):
    
    # Unpack variables
    numFr = trackSingleIn['n_fr']

    # Set up output data structure
    trackOut = dict(
        n_fr        = numFr,
        meas        = trackSingleIn['meas'],
        estimate    = [None]*numFr,
        prediction  = [None]*numFr
    )

    # Generate frame list
    if passDirection == 'forward':
        frList = range(0,numFr,1)
    elif passDirection == 'reverse':
        frList = range(numFr-1,-1,-1)

    # Loop through frames
    for fr in frList:
        trackOut = KalmanFilterSingle(trackOut, trackParams, fr, passDirection)

    # Return data object
    return trackOut

# Single unit, both pass processing
def TrackingSingleUnitBidirectional(trackSingleIn, trackParams):

    # Unpack variables
    numFr = trackSingleIn['n_fr']

    # Set up output data structure
    trackEstimate = [None]*numFr

    # Run forward and backward passes
    trackForward = TrackingSingleUnit(trackSingleIn, trackParams, 'forward')
    trackReverse = TrackingSingleUnit(trackSingleIn, trackParams, 'reverse')

    # Fuse data for each frame
    for fr in range(numFr):

        # Unpack structures
        estF = trackForward['estimate'][fr]
        estR = trackReverse['estimate'][fr]

        # Perform inverse variance weighting
        varSum = np.asmatrix((1 / np.diagonal(estF['covar'])) + (1 / np.diagonal(estR['covar']))).T
        stateSum = (estF['state'] / np.asmatrix(np.diagonal(estF['covar'])).T) + (estR['state'] / np.asmatrix(np.diagonal(estR['covar'])).T)

        varNew = np.diag((1 / varSum.T).tolist()[0])
        stateNew = stateSum / varSum

        # Save data back to struct
        trackEstimate[fr] = dict(
            state = stateNew,
            covar = varNew,
            cart = [stateNew[i] for i in [0, 3]],
            az = np.arctan(stateNew[3] / stateNew[0]) * 180 / np.pi,
            range = np.linalg.norm([stateNew[i] for i in [0, 3]]),
            speed = np.linalg.norm([stateNew[i] for i in [1, 4]])
        )
            
    # Return data structure
    return trackEstimate

# Multi unit data fusion
def DataFusion(trackSingleIn, trackParams):

    # Unpack variables
    numRx = len(trackSingleIn)
    numFr = 0
    for unitData in trackSingleIn:
        numFr = numFr + unitData['n_fr']

    # Set up data structure
    trackingMulti = dict(
        estimate         = [None]*numFr,
        forward          = [None]*numFr,
        reverse          = [None]*numFr,
        prediction       = [None]*numFr,
        meas_estimate    = [None]*numFr,
        meas_prediction  = [None]*numFr,
        bi_hit_list      = [False]*numFr,
        hit_list         = [False]*numFr,
        hit_list_out     = [True]*numFr
    )

    # Loop through frames
    for fr in range(numFr):

        # Initialize single frame data structure
        trackingMulti['meas_estimate'][fr] = dict(
            numDetect = 0,
            state   = None,
            covar     = None)

        # Initialize sums
        numDetect, stateSum, varSum = (0, 0, 0)

        # If limiting number of contributing receivers, make orderedlist
        if trackParams['limitSensorFusion'] and numRx > 1:

            # Loop through receivers to estimate variance
            varianceMeasure = [None]*numRx
            for rx in range(numRx):

                if trackSingleIn[rx]['hit_list'][fr]:
                    var = np.diag(trackSingleIn[rx]['estimate'][fr]['covar'])
                    varianceMeasure[rx] = var[0] + var[3]
                else:
                    varianceMeasure[rx] = np.inf

            # Determine sorted list
            rxList = np.argpartition(varianceMeasure, 2)[:2]
        
        else:
            rxList = range(numRx)

        # Loop through receivers
        for rx in range(numRx):

            # Only add if receiver detected target
            if trackSingleIn[rx]['hit_list'][fr]:

                # Unpack variables
                est = trackSingleIn[rx]['estimate'][fr]
                state = est['state']
                var = np.diag(est['covar'])

                # Adjust for unit position
                state[0] += radarPos[0][rx]
                state[3] += radarPos[1][rx]

                if rx in rxList:

                    # Update running sums
                    numDetect += 1
                    stateSum += state / np.asmatrix(var).T
                    varSum += 1 / var

        # If some data is collected
        if numDetect > 0:

            # Calculate results per frame
            trackingMulti['meas_estimate'][fr]['numDetect'] = numDetect
            trackingMulti['meas_estimate'][fr]['state'] = stateSum / np.asmatrix(varSum).T
            trackingMulti['meas_estimate'][fr]['covar'] = np.diag(1 / varSum)
            trackingMulti['hit_list'][fr] = True

    # Return data structure
    return trackingMulti

# Multi unit, single pass processing
def TrackingMulti(trackMultiIn, trackParams, passDirection):

    # Unpack variables
    numFr = trackParams['numFr']

    # Generate frame list
    if passDirection == 'forward':
        frList = range(0,numFr,1)
    elif passDirection == 'reverse':
        frList = range(numFr-1,-1,-1)
        
    # Set up field
    trackMultiIn[passDirection] = dict(
        estimate     = [None]*numFr,
        prediction   = [None]*numFr,
        hit_list     = [True]*numFr)


    # Loop through frames
    for fr in frList:
        
        # Determine previous successful measurement
        if passDirection == 'forward':
            if fr > 0:
                hit_ind = np.argwhere(trackMultiIn['hit_list'][:fr])
                hit_ind = hit_ind[np.where((fr - hit_ind) <= (trackParams['miss_max']+1))]
            else:
                hit_ind = np.matrix([])
        elif passDirection == 'reverse':
            if fr < numFr-1:
                hit_ind = np.flip(fr + np.argwhere(trackMultiIn['hit_list'][(fr+1):]) + 1)
                hit_ind = hit_ind[np.where((hit_ind - fr) <= (trackParams['miss_max']+1))]
            else:
                hit_ind = np.matrix([])

        # Check if track needs to be initialized
        if hit_ind.size == 0:

            # Save measurement if taken
            if trackMultiIn['hit_list'][fr]:

                # Use measurement as state prediction
                X_init = trackMultiIn['meas_estimate'][fr]['state']
                P_init = trackMultiIn['meas_estimate'][fr]['covar']

                # Save results
                trackMultiIn[passDirection] = SaveStepData(trackMultiIn[passDirection], fr, X_init, P_init, X_init, P_init)
                continue

            else:

                # Set output hit list
                trackMultiIn[passDirection]['hit_list'][fr] = False

        else:

            # Calculate time step since previous hit
            lastHitFrame = hit_ind[-1]
            Tm = trackParams['frame_time'] * (fr - lastHitFrame)


            ## Calculate Model Matrices

            # Measurement covariance matrix
            R = trackMultiIn['meas_estimate'][fr]['covar']

            # Process covariance matrix (NCA model)
            Q_1d = np.array([[(Tm**4)/4, (Tm**3)/2, (Tm**2)/2],
                            [(Tm**3)/2, (Tm**2),   (Tm)],
                            [(Tm**2)/2, (Tm),      1]])
            Q = np.asmatrix(np.concatenate((
                        np.concatenate((Q_1d*(trackParams['sigma_v'][0]**2), np.zeros(Q_1d.shape)), axis=1), 
                        np.concatenate((np.zeros(Q_1d.shape), Q_1d*(trackParams['sigma_v'][1]**2)), axis=1)), 
                        axis=0))

            # Kinematic process matrix (NCA model)
            F_1d = np.array([[1,    (Tm),   (Tm**2)/2],
                            [0,    1,      (Tm)],
                            [0,    0,      1]])
            F = np.asmatrix(np.concatenate((
                        np.concatenate((F_1d, np.zeros(F_1d.shape)), axis=1), 
                        np.concatenate((np.zeros(F_1d.shape), F_1d), axis=1)), 
                        axis=0))


            ## Prediction step

            # Predict kinematic vector
            X_pre = F * np.asmatrix(trackMultiIn[passDirection]['estimate'][lastHitFrame]['state'])

            # Predicted kinematic covariance
            P_pre = (F * (np.asmatrix(trackMultiIn[passDirection]['estimate'][lastHitFrame]['covar'])* F.T)) + Q

            # Save prediction as estimate if measurement not taken
            if not trackMultiIn[passDirection]['hit_list'][fr]:
                trackMultiIn[passDirection] = SaveStepData(trackMultiIn[passDirection], fr, X_pre, P_pre, X_pre, P_pre)
                continue

            ## Calculate measurement matrices

            # Measurement vector
            Z = np.asmatrix(trackMultiIn['meas_estimate'][fr]['state'])

            # Measurement matrix
            H = np.eye(6)

            # Measurement residual
            Z_res = Z - (H * X_pre)


            ## Estimation Step

            # Measurement residual covariance
            S = H * (P_pre * H.T) + R

            # Kalman matrix
            K = (P_pre * H.T) * S.I

            # Estimated kinematic vector
            X_est = X_pre + (K * Z_res)

            # Estimated kinematic covariance
            P_est = P_pre - (K * (H * P_pre))

            # Save data
            trackMultiIn[passDirection] = SaveStepData(trackMultiIn[passDirection], fr, X_est, P_est, X_pre, P_pre)

    return trackMultiIn

# Multi unit, both pass processing
def TrackingMultiBidirectional(trackMultiIn, trackParams):

    # Unpack variables
    numFr = trackParams['numFr']

    # Run forward and backwards pass of tracking
    trackMultiIn = TrackingMulti(trackMultiIn, trackParams, 'forward')
    trackMultiIn = TrackingMulti(trackMultiIn, trackParams, 'reverse')
    hit_list = [f & r for f, r in zip(trackMultiIn['forward']['hit_list'], trackMultiIn['reverse']['hit_list'])]

    # Loop through frames
    for fr in range(numFr):

        # Unpack structures
        estF = trackMultiIn['forward']['estimate'][fr]
        estR = trackMultiIn['reverse']['estimate'][fr]

        if hit_list[fr]:

            # Perform inverse variance weighting
            varSum = np.asmatrix((1 / np.diagonal(estF['covar'])) + (1 / np.diagonal(estR['covar']))).T
            stateSum = (estF['state'] / np.asmatrix(np.diagonal(estF['covar'])).T) + (estR['state'] / np.asmatrix(np.diagonal(estR['covar'])).T)

            varNew = np.diag((1 / varSum.T).tolist()[0])
            stateNew = stateSum / varSum

            # Save data back to struct
            trackMultiIn['estimate'][fr] = dict(
                state = stateNew,
                covar = varNew,
                pos   = [stateNew[i] for i in [0, 3]],
                vel   = [stateNew[i] for i in [1, 4]])

        elif trackMultiIn['forward']['hit_list'][fr]:

            # Save only forward results
            trackMultiIn['estimate'][fr] = dict(
                state = estF['state'],
                covar = estF['covar'],
                pos   = [estF['state'][i] for i in [0, 3]],
                vel   = [estF['state'][i] for i in [1, 4]])

        elif trackMultiIn['reverse']['hit_list'][fr]:

            # Save only forward results
            trackMultiIn['estimate'][fr] = dict(
                state = estR['state'],
                covar = estR['covar'],
                pos   = [estR['state'][i] for i in [0, 3]],
                vel   = [estR['state'][i] for i in [1, 4]])

    # Generate new detection list
    trackMultiIn['bi_hit_list'] = [f | r for f, r in zip(trackMultiIn['forward']['hit_list'], trackMultiIn['reverse']['hit_list'])]
    return trackMultiIn



### Main ###

def ProcessFiles(foldername):

    # Set tracking parameters
    trackParams = dict(
        max_vel = 250,
        max_acc = 1,
        dist_thresh = 13.8,
        miss_max = 5,
        sigma_v = (0.09, 0),
        bi_multi = True,
        bi_single = True
    )


    ### Read input data ###

    # Discover files in directory 
    dataIn = []
    for filename in os.listdir(foldername):
        fullFilename = os.path.join(foldername, filename)
        mat = sio.loadmat(fullFilename)
        dataIn.append(mat['track_out'][0][0])
    numFiles = len(dataIn)

    # Unpack .mat file
    matKeys = {'range', 'vel', 'SNR', 'az', 'steer', 'radar_pos', 'time', 'n_fr'}
    trackData = [dict()]*numFiles

    # Set up new data structure
    for idx, unitData in enumerate(dataIn):
        for key in matKeys:
            trackData[idx][key] = unitData[key]



    ### Single unit tracking ###

    # Set up data structure
    trackSingle = [None]*numFiles

    # Loop through receivers
    for rx in range(numFiles):

        # Set up data structures
        meas = dict()
        for key in matKeys:
            if key != 'n_fr' and key != 'radar_pos':
                meas[key] = np.array([val[0] for val in trackData[rx][key]])

        trackSingle[rx] = dict(
            radar_pos = trackData[rx]['radar_pos'],
            n_fr = trackData[rx]['n_fr'][0][0],
            meas = meas,
            estimate = [])
        
        # Pass to function
        trackSingle[rx]['estimate'] = TrackingSingleUnitBidirectional(trackSingle[rx], trackParams)



    ### Multistatic processing ###

    # Run data fusion
    trackingMulti = DataFusion(trackSingle, trackParams)

    return

    # Multistatic tracking
    trackingMulti = TrackingMultiBidirectional(trackingMulti, trackParams)

    ### Results processing ###

    # Create output folder if it doesn't exist
    dir_path = os.path.dirname(os.path.realpath(__file__))
    saveFolder = dir_path + '/Output'
    if not os.path.exists(saveFolder):
        os.makedirs(saveFolder)

    # Collect filename without path or extension
    rawFilename = filename.replace("\\","/").split('/')[-1]
    rawFilename = rawFilename.split('.')[0]

    # Create folder for CSVs
    saveFolder = saveFolder + '/' + rawFilename
    if not os.path.exists(saveFolder):
        os.makedirs(saveFolder)
    
    # Generate result matrix
    positionEstimate = [None]*numFr
    for fr in range(numFr):
        positionEstimate[fr] = [fr+1] + [trackParams['frame_time']*(fr+0.5)] + [float(el) for el in trackingMulti['estimate'][fr]['pos']]


    # Save multistatic results
    with open(saveFolder + '/multi.csv', mode='w', newline='') as csv_out:
        csvWriter = csv.writer(csv_out, delimiter=',', quotechar='"')
        csvWriter.writerow(('Frame Number', 'Frame Time', 'Cross-Range Position', 'Down-Range Position'))
        csvWriter.writerows(positionEstimate)

    # Plot multistatic results
    data = np.array(positionEstimate)
    xs = data[:,3]
    ys = data[:,2]
    xlims = (np.min(xs), np.max(xs))
    ylims = (np.min(ys), np.max(ys))
    plt.scatter(xs, ys)
    plt.grid()
    plt.xlim(xlims)
    plt.ylim(ylims)
    plt.savefig(saveFolder + '/multi.png')
    plt.close()

    # Save single unit results
    for rx in range(numRx):
        with open(saveFolder + '/single' + str(rx+1) + '.csv', mode='w', newline='') as csv_out:
            
            singleEstimate = [None]*numFr
            for fr in range(numFr):
                
                if trackSingle[rx]['hit_list'][fr]:
                    singleEstimate[fr] = [fr+1] + [trackParams['frame_time']*(fr+0.5)] + [float(trackSingle[rx]['estimate'][fr]['cart'][0])] + [float(trackSingle[rx]['estimate'][fr]['cart'][1])]
                    if np.isnan(singleEstimate[fr][-1]):
                        singleEstimate[fr] = [fr+1, trackParams['frame_time']*(fr+0.5), '', '']
                
                else:
                    singleEstimate[fr] = [fr+1, trackParams['frame_time']*(fr+0.5), '', '']
            
            csvWriter = csv.writer(csv_out, delimiter=',', quotechar='"')
            csvWriter.writerow(('Frame Number', 'Frame Time', 'Cross-Range Position', 'Down-Range Position'))
            csvWriter.writerows(singleEstimate)

            # Convert nan types for plotting
            for frame in singleEstimate:
                if frame[-1] == '':
                    frame[-1] = np.nan
                    frame[-2] = np.nan

            # Plot single unit results
            data = np.array(singleEstimate)
            xs = data[:,3]
            ys = data[:,2]
            plt.scatter(xs, ys)
            plt.grid()
            plt.xlim(xlims)
            plt.ylim(ylims)
            plt.savefig(saveFolder + '/single' + str(rx+1) + '.png')
            plt.close()

if __name__ == '__main__':
    ProcessFiles('/home/sholloway/Documents/GitHub/SEMTA/PostProcessing/Input/AsyncTracking')