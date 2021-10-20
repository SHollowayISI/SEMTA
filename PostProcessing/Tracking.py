### Setup ###

# Import statements
import numpy as np
import scipy.constants as constants
import scipy.io as sio
import pdb


### Function declarations ###

# Single unit Kalman filter
def KalmanFilterSingle(trackSingleIn, trackParams, frame, passDirection):

    # Unpack measurement values
    meas = dict()
    for key in trackSingleIn['meas'].keys():
        meas[key] = trackSingleIn['meas'][key][frame]
    
    # Determine previous successful measurement
    if passDirection == 'forward':
        if frame > 0:
            hit_ind = np.argwhere(trackSingleIn['hit_list'][:(frame-1)])
            hit_ind = hit_ind[np.where((frame - hit_ind) <= (trackParams['miss_max']+1))]
        else:
            hit_ind = np.matrix([])
    elif passDirection == 'reverse':
        if frame < numFr-1:
            hit_ind = len(trackSingleIn['hit_list']) - np.argwhere(trackSingleIn['hit_list'][(frame+1):]) - 1
            hit_ind = hit_ind[np.where((hit_ind - frame) <= (trackParams['miss_max']+1))]
        else:
            hit_ind = np.matrix([])

    # Check if track needs to be initialized
    if hit_ind.size == 0:

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
        return SaveStepData(trackSingleIn, frame, X_init, P_init, X_init, P_init)

    else:

        # Calculate time step since previous hit
        lastHitFrame = hit_ind[-1]
        Tm = trackParams['frame_time'] * (frame - lastHitFrame)


    ## Calculate Model Matrices

    # Measurement covariance matrix
    if not meas:
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

    # Save prediction as estimate if measurement not taken
    if not trackSingleIn['hit_list'][frame]:
        return SaveStepData(trackSingleIn, frame, X_pre, P_pre, X_pre, P_pre)

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
    return SaveStepData(trackSingleIn, frame, X_est, P_est, X_pre, P_pre)

# Save data for each Kalman filter step
def SaveStepData(trackIn, frame, X_est, P_est, X_pre, P_pre):

    # Save estimates
    trackIn['estimate'][frame] = dict(
        state   = X_est,
        covar   = P_est,
        cart    = [X_est[i] for i in [1, 4]],
        az      = np.arctan(X_est[4] / X_est[1]) * 180 / np.pi,
        range   = np.linalg.norm([X_est[i] for i in [0, 3]]),
        speed   = np.linalg.norm([X_est[i] for i in [1, 4]])
    )

    # Save predictions
    trackIn['prediction'][frame] = dict(
        state   = X_pre,
        covar   = P_pre,
        cart    = [X_pre[i] for i in [1, 4]],
        az      = np.arctan(X_pre[4] / X_pre[1]) * 180 / np.pi,
        range   = np.linalg.norm([X_pre[i] for i in [0, 3]]),
        speed   = np.linalg.norm([X_pre[i] for i in [1, 4]])
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
    accel_unc = 0.47735 * trackParams['max_acc']

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
    sig_A = (6.335 / 1.4817) / np.sqrt(2 * np.power(10, -SNR / 10))

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
    
    # Set up output data structure
    trackOut = dict(
        isActive    = False,
        misses      = 0,
        hit_list    = [bool(el) for el in trackSingleIn['hit_list'].tolist()],
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

        # Check for detection
        if trackOut['hit_list'][fr]:

            # Set flags
            trackOut['isActive'] = True
            trackOut['misses'] = 0

        else:

            # Set flags
            trackOut['misses'] += 1
            trackOut['isActive'] &= (trackOut['misses'] < trackParams['miss_max'])
        
        # Perform track filtering
        if trackOut['isActive']:
            trackOut = KalmanFilterSingle(trackOut, trackParams, fr, passDirection)


    # Return data object
    return trackOut

# Single unit, both pass processing
def TrackingSingleUnitBidirectional(trackSingleIn, trackParams):

    # Set up output data structure
    trackEstimate = [None]*numFr

    # Run forward and backward passes
    trackReverse = TrackingSingleUnit(trackSingleIn, trackParams, 'reverse')
    trackForward = TrackingSingleUnit(trackSingleIn, trackParams, 'forward')
    hit_list = [f | r for f, r in zip(trackForward['hit_list'], trackReverse['hit_list'])]

    # Fuse data for each frame
    for fr in range(numFr):

        if hit_list[fr] and trackForward['estimate'][fr] is not None:

            # Unpack structures
            estF = trackForward['estimate'][fr]
            estR = trackReverse['estimate'][fr]

            # Perform inverse variance weighting
            varSum = np.asmatrix((1 / np.diagonal(estF['covar'])) 
            + (1 / np.diagonal(estR['covar']))).T
            stateSum = (estF['state'] / np.asmatrix(np.diagonal(estF['covar'])).T) 
            + (estR['state'] / np.asmatrix(np.diagonal(estR['covar'])).T)

            varNew = np.diag((1 / varSum.T).tolist()[0])
            stateNew = stateSum / varSum

            # Save data back to struct
            trackEstimate[fr] = dict(
                state = stateNew,
                covar = varNew,
                cart = [stateNew[i] for i in [0, 3]],
                az = np.arctan(stateNew[4] / stateNew[1]) * 180 / np.pi,
                range = np.linalg.norm([stateNew[i] for i in [0, 3]]),
                speed = np.linalg.norm([stateNew[i] for i in [1, 4]])
            )
            
    # Return data structure
    return trackEstimate

# Multi unit data fusion
def DataFusion(trackSingleIn, trackParams, radarPos):

    # Set up data structure
    trackingMulti = dict(
        estimate         = [None]*numFr,
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
        for rx in rxList:

            # Only add if receiver detected target
            if trackSingleIn[rx]['hit_list'][fr]:

                # Unpack variables
                est = trackSingleIn[rx]['estimate'][fr]
                state = est['state']
                var = np.diag(est['covar'])

                # Adjust for unit position
                state[0] += radarPos[0][rx]
                state[3] += radarPos[1][rx]

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

    # Generate frame list
    if passDirection == 'forward':
        frList = range(0,numFr,1)
    elif passDirection == 'reverse':
        frList = range(numFr-1,-1,-1)

    # Loop through frames
    for fr in frList:

        # Determine previous successful measurement
        if passDirection == 'forward':
            if fr > 0:
                hit_ind = np.argwhere(trackMultiIn['hit_list'][:(fr-1)])
                hit_ind = hit_ind[np.where((fr - hit_ind) <= (trackParams['miss_max']+1))]
            else:
                hit_ind = np.matrix([])
        elif passDirection == 'reverse':
            if fr < numFr-1:
                hit_ind = len(trackMultiIn['hit_list']) - np.argwhere(trackMultiIn['hit_list'][(fr+1):]) - 1
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
                trackMultiIn = SaveStepData(trackMultiIn, fr, X_init, P_init, X_init, P_init)
                continue

            else:

                # Set output hit list
                trackMultiIn['hit_list_out'][fr] = False

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
            X_pre = F * np.asmatrix(trackMultiIn['meas_estimate'][lastHitFrame]['state'])

            # Predicted kinematic covariance
            P_pre = (F * (np.asmatrix(trackMultiIn['meas_estimate'][lastHitFrame]['covar'])* F.T)) + Q

            # Save prediction as estimate if measurement not taken
            if not trackMultiIn['hit_list'][fr]:
                trackMultiIn = SaveStepData(trackMultiIn, fr, X_pre, P_pre, X_pre, P_pre)
                continue

            ## Calculate measurement matrices

            # Measurement vector
            Z = np.asmatrix(trackMultiIn['meas_estimate'][fr]['state'])

            # Measurement matrix
            H = np.eye(6)

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
            trackMultiIn = SaveStepData(trackMultiIn, fr, X_est, P_est, X_pre, P_pre)

    return trackMultiIn

# Multi unit, both pass processing
def TrackingMultiBidirectional(trackMultiIn, trackParams):

    # Run forward and backwards pass of tracking
    trackForward = TrackingMulti(trackMultiIn, trackParams, 'forward')
    trackReverse = TrackingMulti(trackMultiIn, trackParams, 'reverse')
    hit_list = [f & r for f, r in zip(trackForward['hit_list_out'], trackReverse['hit_list_out'])]

    # Loop through frames
    for fr in range(numFr):

        # Unpack structures
        estF = trackForward['estimate'][fr]
        estR = trackReverse['estimate'][fr]

        if hit_list[fr]:

            # Perform inverse variance weighting
            varSum = np.asmatrix((1 / np.diagonal(estF['covar'])) 
            + (1 / np.diagonal(estR['covar']))).T
            stateSum = (estF['state'] / np.asmatrix(np.diagonal(estF['covar'])).T) 
            + (estR['state'] / np.asmatrix(np.diagonal(estR['covar'])).T)

            varNew = np.diag((1 / varSum.T).tolist()[0])
            stateNew = stateSum / varSum

            # Save data back to struct
            trackMultiIn['estimate'][fr] = dict(
                state = stateNew,
                covar = varNew,
                pos   = [stateNew[i] for i in [0, 3]],
                vel   = [stateNew[i] for i in [1, 4]])

        elif trackForward['hit_list_out'][fr]:

            # Save only forward results
            trackMultiIn['estimate'][fr] = dict(
                state = estF['state'],
                covar = estF['covar'],
                pos   = [estF['state'][i] for i in [0, 3]],
                vel   = [estF['state'][i] for i in [1, 4]])

        elif trackReverse['hit_list_out'][fr]:

            # Save only forward results
            trackMultiIn['estimate'][fr] = dict(
                state = estR['state'],
                covar = estR['covar'],
                pos   = [estR['state'][i] for i in [0, 3]],
                vel   = [estR['state'][i] for i in [1, 4]])


    # Generate new detection list
    trackMultiIn['bi_hit_list'] = [f | r for f, r in zip(trackForward['hit_list'], trackReverse['hit_list'])]
    return trackMultiIn



### Main ###

# Set tracking parameters
trackParams = dict(
    max_vel = 250,
    max_acc = 1,
    dist_thresh = 13.8,
    miss_max = 5,
    sigma_v = (0.09, 0),
    bi_multi = True,
    bi_single = True,
    limitSensorFusion = True,
    frame_time = 0.0512
)

# Set other parameters
filename = 'TrackingTestInitial_101921_1559.mat'

### Read input data ###

# Unpack .mat file
mat = sio.loadmat('Input/' + filename)
matKeys = {'hit_list', 'range', 'vel', 'SNR', 'az', 'steer'}
trackData = dict()

# Set up new data structure
for key in matKeys:
    trackData[key] = mat['track_out'][0][0][key]
radarPos = mat['track_out'][0][0]['radar_pos']

# Determine global parameters
numRx = trackData['hit_list'].shape[0]
numFr = trackData['hit_list'].shape[1]


### Single unit tracking ###

# Set up data structure
trackSingle = [None]*numRx

# Loop through receivers
for rx in range(numRx):

    # Set up data structures
    meas = dict()
    for key in matKeys:
        if key != 'hit_list':
            meas[key] = trackData[key][rx]

    trackSingle[rx] = dict(
        hit_list = trackData['hit_list'][rx],
        meas = meas,
        estimate = [])
    
    # Pass to function
    trackSingle[rx]['estimate'] = TrackingSingleUnitBidirectional(trackSingle[rx], trackParams)


### Multistatic processing ###

# Run data fusion
trackingMulti = DataFusion(trackSingle, trackParams, radarPos)

# Multistatic tracking
trackingMulti = TrackingMultiBidirectional(trackingMulti, trackParams)


### Saving data & plotting ###