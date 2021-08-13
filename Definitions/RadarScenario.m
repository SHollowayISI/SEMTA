% ClassDef File for SEMTA Radar Scenario

classdef RadarScenario < handle
    properties
        multi
        rcs
        traj
        radarsetup
        simsetup
        sim
        rx_sig
        cube
        detection
        flags
        timing
        results
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
                xlim([-rs.traj.exc*1.5, rs.traj.exc*1.5]);
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
        
        function viewRangeCube(rs, graphType, cpi)
            
            if ~exist('cpi', 'var')
                cpi = 1;
            end
            
            if ~exist('graphType', 'var')
                graphType = 'heatmap';
            end
            
            if strcmp(graphType, 'heatmap')
                figure('Name', 'Range-Doppler Heat Map');
                imagesc(1:rs.radarsetup.n_p, ...
                    rs.cube.range_axis, ...
                    10*log10(sum(abs(rs.cube.range_cube(:,:,cpi,:)).^2, 4)))
                title('Range-Doppler Heat Map')
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
                title('Range-Doppler Surface')
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
            figure('Name', 'Range-Doppler Detections');
            imagesc(rs.cube.vel_axis, ...
                rs.cube.range_axis, ...
                rs.detection.cfar_detect_cube)
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
                disp('No target detected.');
                fprintf('Ideal SNR: \t%0.1f dB\n', ...
                    CalculateSNR(rs));
            end
        end
        
        function storeMulti(RadarScenario)
            
            %TODO: REVISE
            %PLACEHOLDER: Store infromation of multiple targets
            % Save whether a target was detected
            RadarScenario.multi.detect( ...
                RadarScenario.flags.frame, RadarScenario.flags.unit) = ...
                RadarScenario.detection.detect_logical;
            
            % Save radar mode
            RadarScenario.multi.mode( ...
                RadarScenario.flags.frame, RadarScenario.flags.unit) = ...
                RadarScenario.flags.mode;
            
            % If target is detected, save range and velocity to target
            if RadarScenario.detection.detect_logical
                
                % Save target range
                RadarScenario.multi.ranges( ...
                    RadarScenario.flags.frame, RadarScenario.flags.unit) = ...
                    RadarScenario.detection.detect_list.range(end);
                
                % Save target velocity
                RadarScenario.multi.vels( ...
                    RadarScenario.flags.frame, RadarScenario.flags.unit) = ...
                    RadarScenario.detection.detect_list.vel(end);
                
                % Save target angle of arrival
                RadarScenario.multi.aoa( ...
                    RadarScenario.flags.frame, RadarScenario.flags.unit) = ...
                    RadarScenario.detection.detect_list.az(end);                    
                
                % Save target SNR
                RadarScenario.multi.SNR( ...
                    RadarScenario.flags.frame, RadarScenario.flags.unit) = ...
                    RadarScenario.detection.detect_list.SNR(end);
                
                
            end
        end
        
        function viewMultilateration(RadarScenario)
            %TODO: REVISE
            
            figure('Name', 'Scatter Plot of Multilateration Results');
            scatter( ...
                RadarScenario.multi.lat_points(1,:), ...
                RadarScenario.multi.lat_points(2,:));
            hold on
            
            scatter( ...
                RadarScenario.multi.track_points(1,:), ...
                RadarScenario.multi.track_points(2,:));
            hold on
            
            scatter( ...
                RadarScenario.traj.pos(1,:), ...
                RadarScenario.traj.pos(2,:), '.');
            hold on
            
            scatter( ...
                RadarScenario.multi.radar_pos(1,:), ...
                RadarScenario.multi.radar_pos(2,:), ...
                'red', 'filled');
            
            title('Scatter Plot of Multilateration Results')
            legend('Multilateration Points', 'Tracking Result', ...
                'Real Trajectory', 'Radar Unit Positions')
            xlabel('Excursion [m]', 'FontWeight', 'bold');
            ylabel('Distance Along Track [m]', 'FontWeight', 'bold');
            grid on;
            
        end
        
        function viewErrors(RadarScenario)
            %TODO: REVISE
            figure('Name', 'Plot of Range Error vs Frame');
            plot(RadarScenario.results.range_error);
            title('Plot of Range Error vs. Frame')
            xlabel('Time [Radar Frame]', 'FontWeight', 'bold');
            ylabel('Range Error [m]', 'FontWeight', 'bold');
            grid on;
            
            figure('Name', 'Plot of Trilateration Error vs Frame');
            plot(abs(RadarScenario.results.trilat_error'));
            title('Plot of Trilateration Error vs. Frame')
            xlabel('Time [Radar Frame]', 'FontWeight', 'bold');
            ylabel('Trilateration Error [m]', 'FontWeight', 'bold');
            legend('X-Direction Error', 'Y-Direction Error');
            grid on;
            
            figure('Name', 'Plot of Trilateration Error with Tracking vs Frame');
            plot(abs(RadarScenario.results.track_error'));
            title('Plot of Trilateration Error with Tracking vs. Frame')
            xlabel('Time [Radar Frame]', 'FontWeight', 'bold');
            ylabel('Trilateration Error [m]', 'FontWeight', 'bold');
            legend('X-Direction Error', 'Y-Direction Error');
            grid on;
            
        end
        
        function frameUpdate(rs, repetition)
            % Repetition value defines how often to send out update
            
            if mod(rs.flags.frame, repetition) == 0
                fprintf('Frame %d of %d complete.\n\n', ...
                    rs.flags.frame, ...
                    rs.multi.n_fr);
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
                nan(rs.multi.n_fr, rs.multi.n_re);
            rs.multi.vels = ...
                nan(rs.multi.n_fr, rs.multi.n_re);
            rs.multi.detect = ...
                false(rs.multi.n_fr, rs.multi.n_re);
            rs.multi.SNR = ...
                nan(rs.multi.n_fr, rs.multi.n_re);
            rs.multi.steering_angle = ...
                nan(rs.multi.n_fr, rs.multi.n_re);
            rs.multi.steering_aoa = ...
                nan(rs.multi.n_fr, rs.multi.n_re);
            
        end
        
        function timeStart(rs)
            
            rs.timing.timing_logical = true;
            rs.timing.startTime = tic;
            rs.timing.TimeDate = now;
            rs.timing.numLoops = ...
                rs.multi.n_fr * ...
                rs.multi.n_re;
            rs.timing.timeGate = 0;
            
        end
        
        function timeUpdate(rs, repetition, rep_method)
            
            if ~rs.timing.timing_logical
                error('Must use method timeStart() before timeUpdate()');
            end
            
            % Calculate progress through simulation
            loops_complete = rs.multi.n_fr * (rs.flags.unit-1) + ...
                rs.flags.frame;
            percent_complete = 100*loops_complete/rs.timing.numLoops;
            
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
                    if mod(loops_complete, repetition) == 1
                        disp(message_p);
                        disp(message_t);
                        disp('');
                    end
                    
                case 'time'
                    if ((rs.timing.timeGate == 0) || ...
                            (toc > repetition + rs.timing.timeGate))
                        disp(message_p);
                        disp(message_t);
                        disp('');
                        rs.timing.timeGate = toc;
                    end
                    
            end
            
        end
        
    end
end






