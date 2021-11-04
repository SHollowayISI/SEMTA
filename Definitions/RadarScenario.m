% ClassDef File for SEMTA Radar Scenario

classdef RadarScenario < handle
    properties
        multi
        tracking_single
        tracking_single_raw
        tracking_multi
        rcs
        traj
        radarsetup
        simsetup
        sim
        rx_sig
        cube
        detection
        flags = struct('frame', 0);
        timing
    end
    
    methods
        function viewRCSFreq(rs)
            figure('Name', 'RCS vs. Frequency');
            plot(rs.rcs.freq,...
                10*log10(rs.rcs.evaluate(rs.rcs, rs.rcs.ang(ceil(end/2)), rs.rcs.freq)));
            grid on;
            title('RCS vs. Frequency');
            xlabel('Frequency [Hz]','FontWeight','bold');
            ylabel('RCS [dBm^2]','FontWeight','bold');
            set(gca, 'XLim', rs.rcs.freq([1 end]));
            set(gca, 'YLim', [rs.rcs.ave_rcs-20, max(rs.rcs.ave_rcs+10, rs.rcs.peak_rcs+10)]);
        end
        
        function viewRCSAng(rs)
            figure('Name', 'RCS vs. Aspect Angle');
            subplot(121)
            plotRCS = rs.rcs.evaluate(rs.rcs, rs.rcs.ang, rs.rcs.freq(ceil(end/2)));
            plot(rs.rcs.ang,...
                10*log10(plotRCS));
            grid on;
            title('Rectangular');
            xlabel('Aspect Angle (degree)','FontWeight','bold');
            ylabel('RCS [dBm^2]','FontWeight','bold');
            set(gca, 'XLim', [-180, 180]);
            set(gca, 'YLim', [rs.rcs.ave_rcs-20, max(rs.rcs.ave_rcs+10, rs.rcs.peak_rcs+10)]);
            
            subplot(122)
            polarplot(rs.rcs.ang*(pi/180),...
                10*log10(plotRCS));
            title('Polar');
            pax = gca;
            pax.ThetaZeroLocation = 'top';
            pax.ThetaGrid = 'on';
            pax.RLim = [rs.rcs.ave_rcs-20, max(rs.rcs.ave_rcs+10, rs.rcs.peak_rcs+10)];
        end
        
        function viewTraj(rs)
            
            % Calculate time axis
            t = (0:rs.multi.n_fr)*rs.radarsetup.frame_time;
            
            % Calculate trajectory
            positions = rs.traj.pos(rs.traj, t);
            
            figure('Name', 'Trajectory Plot')
            scatter3( ...
                positions(1,:), ...
                positions(2,:), ...
                positions(3,:), 'filled')
            if abs(rs.traj.exc)>0
                xlim([-abs(rs.traj.exc)*1.5, abs(rs.traj.exc)*1.5]);
            end
            if abs(rs.traj.yvel)>0
                ylim([t(1)*rs.traj.yvel, ...
                    t(end)*rs.traj.yvel]);
            end
            if abs(rs.traj.alt)>0
                zlim([0, 2*rs.traj.alt]);
            end
            xlabel('Excursion [m]','FontWeight','bold');
            ylabel('Distance Along Track [m]','FontWeight','bold');
            zlabel('Altitude [m]','FontWeight','bold');
        end
        
        function viewRxSignal(rs, chirp, plotPower)
            
            if ~exist('chirp', 'var')
                chirp = 1;
            end
            
            if ~exist('plotPower', 'var')
                plotPower = true;
            end
            
            figure('Name', 'Time Domain Signal Plot');
            if plotPower
                plot(abs(rs.rx_sig(:,chirp,1)).^2);
                ylabel('Signal Power','FontWeight','bold');
            else
                plot(real(rs.rx_sig(:,chirp,1)));
                hold on;
                plot(imag(rs.rx_sig(:,chirp,1)));
                ylabel('Signal Amplitude', 'FontWeight', 'bold');
            end
            grid on;
            title('Time Domain Signal Plot');
            xlabel('Sample #','FontWeight','bold')
            xlim([0 size(rs.rx_sig, 1)])
        end
        
        function viewCorrelationFFT(rs, chirp, plotPower)
            
            if ~exist('chirp', 'var')
                chirp = 1;
            end
            
            if ~exist('plotPower', 'var')
                plotPower = true;
            end
            
            N_r = 2^ceil(log2(size(rs.rx_sig, 1)));
            
            figure('Name', 'Correlation Signal FFTs');
            if plotPower
                subplot(2,1,1)
                plot(abs(fft(rs.rx_sig(:,chirp,1), N_r, 1)).^2);
                ylabel('Signal Power','FontWeight','bold');
                grid on;
                title('Receive Signal FFT');
                xlabel('Sample #','FontWeight','bold')
                xlim([0 N_r])
                
                subplot(2,1,2)
                plot(abs(fft(rs.sim.waveform(), N_r, 1)).^2);
                ylabel('Signal Power','FontWeight','bold');
                grid on;
                title('Correlation Signal FFT');
                xlabel('Sample #','FontWeight','bold')
                xlim([0 N_r])
            else
                subplot(2,1,1)
                plot(real(fft(rs.rx_sig(:,chirp,1), N_r, 1)));
                hold on;
                plot(imag(fft(rs.rx_sig(:,chirp,1), N_r, 1)));
                ylabel('Signal Amplitude', 'FontWeight', 'bold');
                grid on;
                title('Receive Signal FFT');
                xlabel('Sample #','FontWeight','bold')
                xlim([0 N_r])
                
                subplot(2,1,2)
                plot(real(fft(rs.sim.waveform(), N_r, 1)));
                hold on;
                plot(imag(fft(rs.sim.waveform(), N_r, 1)));
                ylabel('Signal Amplitude', 'FontWeight', 'bold');
                grid on;
                title('Correlation Signal FFT');
                xlabel('Sample #','FontWeight','bold')
                xlim([0 N_r])
            end
        end
        
        function viewRangeTimeDomain(rs, chirp, plotPower)
            
            if ~exist('chirp', 'var')
                chirp = 1;
            end
            
            if ~exist('plotPower', 'var')
                plotPower = true;
            end
            
            figure('Name', 'Time Domain Range Plot');
            if plotPower
                plot(rs.cube.range_axis, abs(rs.cube.range_cube(:,chirp,1)).^2);
                ylabel('Signal Power','FontWeight','bold');
            else
                plot(rs.cube.range_axis, real(rs.cube.range_cube(:,chirp,1)));
                hold on;
                plot(rs.cube.range_axis, imag(rs.cube.range_cube(:,chirp,1)));
                ylabel('Signal Amplitude', 'FontWeight', 'bold');
            end
            grid on;
            title('Time Domain Range Plot');
            xlabel('Range','FontWeight','bold')
            xlim([0 rs.cube.range_axis(end)])
            
        end
        
        function viewRangeCube(rs, graphType, cpi)
            
            if ~exist('cpi', 'var')
                cpi = 1;
            end
            
            if ~exist('graphType', 'var')
                graphType = 'heatmap';
            end
            
            if strcmp(graphType, 'heatmap')
                figure('Name', 'Range-Slow Time Heat Map');
                imagesc(1:rs.radarsetup.n_p, ...
                    rs.cube.range_axis, ...
                    10*log10(sum(abs(rs.cube.range_cube(:,:,cpi,:)).^2, 4)))
                title('Range-Slow Time Heat Map')
                set(gca,'YDir','normal')
                xlabel('Chirp #','FontWeight','bold')
                ylabel('Range [m]','FontWeight','bold')
                colorbar;
            else
                figure('Name', 'surface');
                surfc(1:rs.radarsetup.n_p, ...
                    rs.cube.range_axis, ...
                    10*log10(sum(abs(rs.cube.range_cube(:,:,cpi,:)).^2, 4)), ...
                    'EdgeColor', 'none')
                title('Range-Slow Time Surface')
                set(gca,'YDir','normal')
                xlabel('Chirp #','FontWeight','bold')
                ylabel('Range [m]','FontWeight','bold')
                zlabel('FFT Log Intensity [dB]','FontWeight','bold')
            end
            
        end
        
        function viewRDCube(rs, graphType, cpi)
            
            if ~exist('graphType', 'var')
                graphType = 'heatmap';
            end
            
            if ~exist('cpi', 'var')
                cpi = 1;
            end
            
            if strcmp(graphType, 'heatmap')
                figure('Name', 'Range-Doppler Heat Map');
                imagesc(rs.cube.vel_axis, ...
                    rs.cube.range_axis, ...
                    10*log10(sum(rs.cube.pow_cube(:,:,cpi,:), 4)))
                title('Range-Doppler Heat Map')
                set(gca,'YDir','normal')
                xlabel('Velocity [m/s]','FontWeight','bold')
                ylabel('Range [m]','FontWeight','bold')
                colorbar;
            else
                figure('Name', 'Range-Doppler Surface');
                surfc(rs.cube.vel_axis, ...
                    rs.cube.range_axis, ...
                    10*log10(sum(rs.cube.pow_cube(:,:,cpi,:), 4)), ...
                    'EdgeColor', 'none')
                title('Range-Doppler Surface')
                set(gca,'YDir','normal')
                xlabel('Velocity [m/s]','FontWeight','bold')
                ylabel('Range [m]','FontWeight','bold')
                zlabel('FFT Log Intensity [dB]','FontWeight','bold')
            end
            
        end
        
        function viewIncoherentCube(rs, graphType)
            
            if ~exist('graphType', 'var')
                graphType = 'heatmap';
            end
            
            if strcmp(graphType, 'heatmap')
                figure('Name', 'Range-Doppler Heat Map');
                imagesc(rs.cube.vel_axis, ...
                    rs.cube.range_axis, ...
                    10*log10(sum(rs.cube.pow_cube, [3 4])))
                title('Range-Doppler Heat Map')
                set(gca,'YDir','normal')
                xlabel('Velocity [m/s]','FontWeight','bold')
                ylabel('Range [m]','FontWeight','bold')
                colorbar;
            else
                figure('Name', 'Range-Doppler Surface');
                surfc(rs.cube.vel_axis, ...
                    rs.cube.range_axis, ...
                    10*log10(sum(rs.cube.pow_cube, [3 4])), ...
                    'EdgeColor', 'none')
                title('Range-Doppler Surface')
                set(gca,'YDir','normal')
                xlabel('Velocity [m/s]','FontWeight','bold')
                ylabel('Range [m]','FontWeight','bold')
                zlabel('FFT Log Intensity [dB]','FontWeight','bold')
            end
            
        end
        
        function viewDetections(rs)
            
            detect_cube = zeros(length(rs.cube.range_axis), length(rs.cube.vel_axis));
            detect_cube(rs.detection.combined_ind_list) = 1;
            
            figure('Name', 'Range-Doppler Detections');
            imagesc(rs.cube.vel_axis, ...
                rs.cube.range_axis, ...
                detect_cube)
            title('Range-Doppler Detections')
            set(gca,'YDir','normal')
            xlabel('Velocity [m/s]','FontWeight','bold')
            ylabel('Range [m]','FontWeight','bold')
        end
        
        function viewDetectionsAllFrames(rs)
            
            detect_cube = zeros(length(rs.cube.range_axis), length(rs.cube.vel_axis));
            for n = 1:rs.radarsetup.cpi_fr
                detect_cube(rs.detection.single_ind_list{n}) = 1;
            end
            
            figure('Name', 'Range-Doppler Detections');
            imagesc(rs.cube.vel_axis, ...
                rs.cube.range_axis, ...
                detect_cube)
            title('Range-Doppler Detections')
            set(gca,'YDir','normal')
            xlabel('Velocity [m/s]','FontWeight','bold')
            ylabel('Range [m]','FontWeight','bold')
        end
        
        function viewSNR(rs)
            figure('Name', 'Plot of SNR vs Frame');
            plot(rs.multi.SNR);
            title('Plot of SNR vs. Frame')
            xlabel('Time [Radar Frame]', 'FontWeight', 'bold');
            ylabel('SNR [dB]', 'FontWeight', 'bold');
            grid on;
        end
        
        function viewTrackingSingle(rs, unitsToPlot, showRadar)
            
            % Default variables
            if ~exist('showRadar', 'var')
                showRadar = false;
            end
            if ~exist('unitsToPlot', 'var')
                unitsToPlot = 1:rs.multi.n_re;
            end
            
            % Open figure
            figure('Name', 'Single Unit Tracking Results');
            
            % Collect tracking data from each unit and frame
            for unit = unitsToPlot
                
                meas_coords = nan(2, rs.multi.num_fr(unit));
                est_coords = meas_coords;
                
                for frame = 1:rs.multi.num_fr(unit)
                    if rs.tracking_single{unit}.hit_list(frame)
                        meas_coords(:,frame) = rs.multi.radar_pos(1:2,unit) + rs.tracking_single{unit}.meas{frame}.cart;
                        est_coords(:,frame) = rs.multi.radar_pos(1:2,unit) + rs.tracking_single{unit}.estimate{frame}.cart;
                    end
                end
    
                % Plot results
                scatter(meas_coords(2,:), meas_coords(1,:), 'b');
                hold on;
                plot(est_coords(2,:), est_coords(1,:), 'r');
                hold on;
                if showRadar
                    scatter(rs.multi.radar_pos(2,unit), rs.multi.radar_pos(1,unit), 'r', '.')
                end
            end
            
            % Add labels
            title('Single Unit Tracking Results');
            xlabel('Along-Track Position [m]', 'FontWeight', 'bold');
            ylabel('Cross-Track Position [m]', 'FontWeight', 'bold');
            
            % Adjust plot
            ax = gca;
            midLim = mean(ax.YLim);
            diffLim = diff(ax.XLim)/2;
            ylim(midLim + diffLim*[-1, 1]);
            grid on;
            pbaspect([1 1 1]);
        end
        
        function viewTrackingMulti(rs, showRadar)
            
            % Default variables
            if ~exist('showRadar', 'var')
                showRadar = false;
            end
            
            % Open figure
            figure('Name', 'Tracking Results');
            
            % Collect tracking data from each unit and frame
            est_coords = nan(2, rs.tracking_multi.num_fr);
            for frame = 1:rs.tracking_multi.num_fr
                est_coords(:,frame) = rs.tracking_multi.track_estimate{frame}.pos;
            end
            
            % Plot results
            plot(est_coords(2,:), est_coords(1,:), 'r');
            hold on;
            if showRadar
                scatter(rs.multi.radar_pos(2,unit), rs.multi.radar_pos(1,unit), 'r', '.')
            end
            
            % Add labels
            title('Multistatic Tracking Results');
            xlabel('Along-Track Position [m]', 'FontWeight', 'bold');
            ylabel('Cross-Track Position [m]', 'FontWeight', 'bold');
            
            % Adjust plot
            ax = gca;
            midLim = mean(ax.YLim);
            diffLim = diff(ax.XLim)/2;
            ylim(midLim + diffLim*[-1, 1]);
            grid on;
            pbaspect([1 1 1]);
        end
        
        function readOut(rs)
            
            if rs.detection.detect_logical
                for det_ind = 1:rs.detection.detect_list.num_detect
                    fprintf('\nTarget detected: (%d of %d)\n', ...
                        det_ind, rs.detection.detect_list.num_detect);
                    fprintf('Range: \t\t%0.1f meters\n', ...
                        rs.detection.detect_list.range(det_ind));
                    fprintf('Velocity: \t%0.1f m/s\n', ...
                        rs.detection.detect_list.vel(det_ind));
                    fprintf('AoA: \t\t%0.1f deg\n', ...
                        rs.detection.detect_list.az(det_ind));
                    fprintf('SNR: \t\t%0.1f dB\n', ...
                        rs.detection.detect_list.SNR(det_ind));
                    fprintf('Ideal SNR: \t%0.1f dB\n', ...
                        CalculateSNR(rs));
                end
            else
                fprintf('\nNo target detected.\n');
                fprintf('Ideal SNR: \t%0.1f dB\n', ...
                    CalculateSNR(rs));
            end
        end
        
        function storeMulti(rs)
            
            % Store infromation of multiple targets
            % Save whether a target was detected
            rs.multi.detect{rs.flags.unit}(rs.flags.frame) = ...
                rs.detection.detect_logical;
            
            % Save radar mode
            rs.multi.mode{rs.flags.unit}{rs.flags.frame} = ...
                rs.flags.mode;
            
            % Save elapsed frames
            rs.multi.num_fr(rs.flags.unit) = rs.flags.frame;
            
            % If target is detected, save range and velocity to target
            if rs.detection.detect_logical
                
                % Save target range
                rs.multi.ranges{rs.flags.unit}(rs.flags.frame) = ...
                    rs.detection.detect_list.range(end);
                
                % Save target velocity
                rs.multi.vels{rs.flags.unit}(rs.flags.frame) = ...
                    rs.detection.detect_list.vel(end);
                
                % Save target angle of arrival
                rs.multi.aoa{rs.flags.unit}(rs.flags.frame) = ...
                    rs.detection.detect_list.az(end);
                
                % Save target SNR
                rs.multi.SNR{rs.flags.unit}(rs.flags.frame) = ...
                    rs.detection.detect_list.SNR(end);
                
                % Save timestamp
                rs.multi.time{rs.flags.unit}(rs.flags.frame) = ...
                    rs.detection.detect_list.time;
            end
        end
        
        function unitReset(rs)
            % Reset mode flag and initial angle
            rs.flags.mode = rs.radarsetup.initial_mode;
            rs.radarsetup.int_type = rs.radarsetup.modes.(rs.radarsetup.initial_mode).int_type;
            rs.flags.frame = 0;
        end
        
        function simTimeStart(rs)
            
            % Set beginning, middle, and end frame time
            rs.flags.frameStartTime = rs.multi.start_time(rs.flags.unit);
            rs.flags.frameMidTime = rs.flags.frameStartTime + rs.radarsetup.frame_time/2;
            rs.flags.frameEndTime = rs.flags.frameStartTime + rs.radarsetup.frame_time;
        end
        
        function simTimeUpdate(rs)
            
            % Updates beginning, middle, and end frame time
            rs.flags.frameStartTime = rs.flags.frameEndTime;
            rs.flags.frameMidTime = rs.flags.frameStartTime + rs.radarsetup.frame_time/2;
            rs.flags.frameEndTime = rs.flags.frameStartTime + rs.radarsetup.frame_time;
        end
        
        function frameUpdate(rs, repetition)
            % Repetition value defines how often to send out update
            
            if mod(rs.flags.frame, repetition) == 0
                fprintf('Frame %d complete.\n', ...
                    rs.flags.frame);
                fprintf('%dms of %dms of simulation time complete.\n\n', ...
                    floor(1000*min(rs.flags.frameStartTime,rs.multi.sim_time)), 1000*rs.multi.sim_time);
            end
            
        end
        
        function unitUpdate(rs, repetition)
            % Repetition value defines how often to send out update
            
            if mod(rs.flags.frame, repetition) == 0
                fprintf('Unit %d of %d simulation complete.\n\n', ...
                    rs.flags.unit, ...
                    rs.multi.n_re);
                
                toc;
            end
            
        end
        
        function multiSetup(rs)
            
            % Initialize container for multistatic information
            rs.multi.ranges = ...
                cell(rs.multi.n_re,1);
            rs.multi.vels = ...
                cell(rs.multi.n_re,1);
            rs.multi.detect = ...
                cell(rs.multi.n_re,1);
            rs.multi.SNR = ...
                cell(rs.multi.n_re,1);
            rs.multi.steering_angle = ...
                cell(rs.multi.n_re,1);
            rs.multi.aoa = ...
                cell(rs.multi.n_re,1);
            rs.multi.mode = ...
                cell(rs.multi.n_re,1);
            rs.tracking_single = ...
                cell(rs.multi.n_re,1);
            rs.multi.num_fr = zeros(rs.multi.n_re, 1);
            
        end
        
        function startProgressTimer(rs)
            
            rs.timing.timing_logical = true;
            rs.timing.startTime = tic;
            rs.timing.TimeDate = now;
            rs.timing.timeGate = 0;
            
        end
        
        function updateProgressTimer(rs, repetition, rep_method)
            
            if ~rs.timing.timing_logical
                error('Must use method timeStart() before timeUpdate()');
            end
            
            % Calculate progress through simulation
            units_complete = rs.flags.unit-1 + min(1,rs.flags.frameStartTime/rs.multi.sim_time);
            percent_complete = min(100*units_complete /rs.multi.n_re,100);
            
            % Calculate remaining time in simulation
            nowTimeDate = now;
            elapsedTime = nowTimeDate - rs.timing.TimeDate;
            estComplete = nowTimeDate + ((100/percent_complete)-1)*elapsedTime;
            
            % Form message to display in command window
            message_p = [sprintf('Percent complete: %0.0f', percent_complete), '%'];
            message_t = ['Estimated time of completion: ', datestr(estComplete)];
            
            % Display current progress
            switch rep_method
                case 'frames'
                    if mod(rs.flags.frame, repetition) == 1
                        disp('');
                        disp(message_p);
                        disp(message_t);
                        disp('');
                    end
                    
                case 'time'
                    if ((rs.timing.timeGate == 0) || ...
                            (toc > repetition + rs.timing.timeGate))
                        disp('');
                        disp(message_p);
                        disp(message_t);
                        disp('');
                        rs.timing.timeGate = toc;
                    end
                    
            end
            
        end
        
    end
end






