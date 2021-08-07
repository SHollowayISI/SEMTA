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
        function viewRCSFreq(RadarScenario)
            figure('Name', 'RCS vs. Frequency');
            plot(RadarScenario.rcs.freq,...
                10*log10(RadarScenario.rcs.value(ceil(end/2),:)'))
            grid on;
            title('RCS vs. Frequency');
            xlabel('Frequency [Hz]','FontWeight','bold');
            ylabel('RCS [dBm^2]','FontWeight','bold');
            set(gca, 'XLim', RadarScenario.rcs.freq([1 end]));
            set(gca, 'YLim', [RadarScenario.rcs.ave_rcs-20, RadarScenario.rcs.ave_rcs+10]);
        end
        
        function viewRCSAng(RadarScenario)
            figure('Name', 'RCS vs. Aspect Angle');
            subplot(121)
            plot(RadarScenario.rcs.ang,...
                10*log10(RadarScenario.rcs.value(:,ceil(end/2))))
            grid on;
            title('Rectangular');
            xlabel('Aspect Angle (degree)','FontWeight','bold');
            ylabel('RCS [dBm^2]','FontWeight','bold');
            set(gca, 'XLim', [-180, 180]);
            set(gca, 'YLim', [RadarScenario.rcs.ave_rcs-20, RadarScenario.rcs.ave_rcs+10]);
            
            subplot(122)
            polarplot(RadarScenario.rcs.ang*(pi/180),...
                10*log10(RadarScenario.rcs.value(:,ceil(end/2))))
            title('Polar');
            pax = gca;
            pax.ThetaZeroLocation = 'top';
            pax.ThetaGrid = 'on';
            pax.RLim = [RadarScenario.rcs.ave_rcs-20, RadarScenario.rcs.ave_rcs+10];
        end
        
        function viewTraj(RadarScenario)
            figure('Name', 'Trajectory Plot')
            scatter3( ...
                RadarScenario.traj.pos(1,:), ...
                RadarScenario.traj.pos(2,:), ...
                RadarScenario.traj.pos(3,:), 'filled')
            if abs(RadarScenario.traj.exc)>0
                xlim([-RadarScenario.traj.exc*1.5, RadarScenario.traj.exc*1.5]);
            end
            if abs(RadarScenario.traj.yvel)>0
                ylim([RadarScenario.traj.time(1)*RadarScenario.traj.yvel, ...
                    RadarScenario.traj.time(end)*RadarScenario.traj.yvel]);
            end
            if abs(RadarScenario.traj.alt)>0
                zlim([0, 2*RadarScenario.traj.alt]);
            end
            xlabel('Excursion [m]','FontWeight','bold');
            ylabel('Distance Along Track [m]','FontWeight','bold');
            zlabel('Altitude [m]','FontWeight','bold');
        end
        
        function viewRDCube(RadarScenario, graphType)
            if strcmp(graphType, 'heatmap')
                figure('Name', 'Range-Doppler Heat Map');
                imagesc(RadarScenario.cube.vel_axis, ...
                    RadarScenario.cube.range_axis, ...
                    10*log10(RadarScenario.cube.pow_cube))
                title('Range-Doppler Heat Map')
                set(gca,'YDir','normal')
                xlabel('Velocity [m/s]','FontWeight','bold')
                ylabel('Range [m]','FontWeight','bold')
            else
                figure('Name', 'Range-Doppler Surface');
                surfc(RadarScenario.cube.vel_axis, ...
                    RadarScenario.cube.range_axis, ...
                    10*log10(RadarScenario.cube.pow_cube), ...
                    'EdgeColor', 'none')
                title('Range-Doppler Surface')
                set(gca,'YDir','normal')
                xlabel('Velocity [m/s]','FontWeight','bold')
                ylabel('Range [m]','FontWeight','bold')
                zlabel('FFT Log Intensity [dB]','FontWeight','bold')
            end
            
        end
        
        function viewDetections(RadarScenario)
            figure('Name', 'Range-Doppler Detections');
            imagesc(RadarScenario.cube.vel_axis, ...
                RadarScenario.cube.range_axis, ...
                RadarScenario.detection.detect_cube)
            title('Range-Doppler Detections')
            set(gca,'YDir','normal')
            xlabel('Velocity [m/s]','FontWeight','bold')
            ylabel('Range [m]','FontWeight','bold')
        end
        
        function viewSNR(RadarScenario)
            figure('Name', 'Plot of SNR vs Frame');
            plot(RadarScenario.multi.SNR);
            title('Plot of SNR vs. Frame')
            xlabel('Time [Radar Frame]', 'FontWeight', 'bold');
            ylabel('SNR [dB]', 'FontWeight', 'bold');
            grid on;
        end
        
        function readOut(RadarScenario)
            if RadarScenario.detection.detect_logical
                disp('Target detected:');
                disp(sprintf('Range:    %0.1f meters', ...
                    RadarScenario.detection.target_range));
                disp(sprintf('Velocity: %0.1f m/s', ...
                    RadarScenario.detection.target_vel));
                disp(sprintf('SNR:      %0.1f dB', ...
                    RadarScenario.detection.max_SNR));
            else
                disp('No target detected.');
            end
        end
        
        function storeMulti(RadarScenario)
            
            % Save whether a target was detected
            RadarScenario.multi.detect( ...
                RadarScenario.flags.frame, RadarScenario.flags.unit) = ...
                RadarScenario.detection.detect_logical;
            
            % If target is detected, save range and velocity to target
            if RadarScenario.detection.detect_logical
                
                % Save target range
                RadarScenario.multi.ranges( ...
                    RadarScenario.flags.frame, RadarScenario.flags.unit) = ...
                    RadarScenario.detection.target_range;
                
                % Save target velocity
                RadarScenario.multi.vels( ...
                    RadarScenario.flags.frame, RadarScenario.flags.unit) = ...
                    RadarScenario.detection.target_vel;
                
                % Save target SNR
                RadarScenario.multi.SNR( ...
                    RadarScenario.flags.frame, RadarScenario.flags.unit) = ...
                    RadarScenario.detection.max_SNR;
                
                % TEMPORARY??? CENTROID AND MEDIAN POSITIONING
                
                % Save target range
                RadarScenario.multi.ranges_centroid( ...
                    RadarScenario.flags.frame, RadarScenario.flags.unit) = ...
                    RadarScenario.detection.target_range_centroid;
                
                % Save target velocity
                RadarScenario.multi.vels_centroid( ...
                    RadarScenario.flags.frame, RadarScenario.flags.unit) = ...
                    RadarScenario.detection.target_vel_centroid;
                
                % Save target range
                RadarScenario.multi.ranges_median( ...
                    RadarScenario.flags.frame, RadarScenario.flags.unit) = ...
                    RadarScenario.detection.target_range_median;
                
                % Save target velocity
                RadarScenario.multi.vels_median( ...
                    RadarScenario.flags.frame, RadarScenario.flags.unit) = ...
                    RadarScenario.detection.target_vel_median;
                
                
            end
        end
        
        function viewMultilateration(RadarScenario)
            
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
        
        function frameUpdate(RadarScenario, repetition)
            % Repetition value defines how often to send out update
            
            if mod(RadarScenario.flags.frame, repetition) == 0
                fprintf('Frame %d of %d complete.\n\n', ...
                    RadarScenario.flags.frame, ...
                    RadarScenario.multi.n_fr);
            end
            
        end
        
        function unitUpdate(RadarScenario, repetition)
            % Repetition value defines how often to send out update
            
            if mod(RadarScenario.flags.frame, repetition) == 0
                fprintf('Unit %d of %d simulation complete.\n\n', ...
                    RadarScenario.flags.unit, ...
                    RadarScenario.multi.n_re);
                
                toc;
            end
            
        end
        
        function multiSetup(RadarScenario)
            
            % Initialize container for multistatic information
            RadarScenario.multi.ranges = ...
                zeros(RadarScenario.multi.n_fr, RadarScenario.multi.n_re);
            RadarScenario.multi.vels = ...
                zeros(RadarScenario.multi.n_fr, RadarScenario.multi.n_re);
            RadarScenario.multi.detect = ...
                false(RadarScenario.multi.n_fr, RadarScenario.multi.n_re);
            RadarScenario.multi.SNR = ...
                zeros(RadarScenario.multi.n_fr, RadarScenario.multi.n_re);
            
            % TEMPORARY???? SAVING OF CENTROID AND MEDIUM POSITIONING
            RadarScenario.multi.ranges_centroid = ...
                zeros(RadarScenario.multi.n_fr, RadarScenario.multi.n_re);
            RadarScenario.multi.vels_centroid = ...
                zeros(RadarScenario.multi.n_fr, RadarScenario.multi.n_re);
            RadarScenario.multi.ranges_median = ...
                zeros(RadarScenario.multi.n_fr, RadarScenario.multi.n_re);
            RadarScenario.multi.vels_median = ...
                zeros(RadarScenario.multi.n_fr, RadarScenario.multi.n_re);
            
        end
        
        function parameterCheck(RadarScenario)
            t_step = diff(RadarScenario.traj.time(1:2));
            if t_step*RadarScenario.simsetup.prf ~= 1
                error('Trajectory model time step must equal PRI');
            end
            
            t_end = RadarScenario.traj.time(end);
            sim_end = RadarScenario.multi.n_fr * RadarScenario.simsetup.n_p * ...,
                RadarScenario.simsetup.cpi_fr / RadarScenario.simsetup.prf;
            
            if t_end < sim_end
                error('Trajectory final time must be greater than full simulation time');
            end
            
        end
        
        function timeStart(RadarScenario)
            
            RadarScenario.timing.timing_logical = true;
            RadarScenario.timing.startTime = tic;
            RadarScenario.timing.TimeDate = now;
            RadarScenario.timing.numLoops = ...
                RadarScenario.multi.n_fr * ...
                RadarScenario.multi.n_re;
            RadarScenario.timing.timeGate = 0;
            
        end
        
        function timeUpdate(RadarScenario, repetition, rep_method)
            
            if ~RadarScenario.timing.timing_logical
                error('Must use method timeStart() before timeUpdate()');
            end
            
            % Calculate progress through simulation
            loops_complete = RadarScenario.multi.n_fr * (RadarScenario.flags.unit-1) + ...
                RadarScenario.flags.frame;
            percent_complete = 100*loops_complete/RadarScenario.timing.numLoops;
            
            % Calculate remaining time in simulation
            nowTimeDate = now;
            elapsedTime = nowTimeDate - RadarScenario.timing.TimeDate;
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
                    if ((RadarScenario.timing.timeGate == 0) || ...
                            (toc > repetition + RadarScenario.timing.timeGate))
                        disp(message_p);
                        disp(message_t);
                        disp('');
                        RadarScenario.timing.timeGate = toc;
                    end
                    
            end
            
        end
        
    end
end






