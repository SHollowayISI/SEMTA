% 
% % load('MAT Files\Scenario Objects\scenario_Excursion_0kmExc_5kmDist_SEMTA_031420_2149.mat')
% load('MAT Files\Scenario Objects\scenario_Excursion_2kmExc_5kmDist_SEMTA_031420_2220.mat')
% 
% 
% scenario.multi.track_method = 'Kalman Filter';
% scenario.multi.interp = false;
% 
% % 2km Excursion fitting
% scenario.multi.sig_v = 2;
% scenario.multi.sig_z = 1.95;
% 
% % 0km Excursion fitting
% % scenario.multi.sig_v = 10;
% % scenario.multi.sig_z = 2.5;
% 
% % Perform multilateration on list of detected targets
% % scenario.multi = MultistaticProcessing_SEMTA(scenario);
% 
% % Perform smoothing on multilaterated target coordinates
% scenario.multi = TargetTracking_SEMTA(scenario);
% 
% % Visualize multilateration result
% % viewMultilateration(scenario);
% 
% % Estimate error of results
% scenario.results = ErrorEstimation_SEMTA(scenario);
% 
% % View plots of errors
% % viewErrors(scenario);
% 
% real_track_x = scenario.results.track_error(1,~isnan(scenario.results.track_error(1,:)));
% real_track_y = scenario.results.track_error(2,~isnan(scenario.results.track_error(2,:)));
% 
% real_trilat_x = scenario.results.trilat_error(1,~isnan(scenario.results.trilat_error(1,:)));
% real_trilat_y = scenario.results.trilat_error(2,~isnan(scenario.results.trilat_error(2,:)));
% 
% ec = 5;
% 
% track_rms_y = rms(real_track_y((ec+1):(end-ec)));
% track_rms_x = rms(real_track_x((ec+1):(end-ec)));
% track_rms = rms([track_rms_x; track_rms_y])
% 
% trilat_rms_y = rms(real_trilat_y((ec+1):(end-ec)));
% trilat_rms_x = rms(real_trilat_x((ec+1):(end-ec)));
% trilat_rms = rms([trilat_rms_x; trilat_rms_y])
% 
% ratio = track_rms./trilat_rms
% 
% 
% 
% 


traj = scenario.traj;
multi = scenario.multi;
simsetup = scenario.simsetup;

% Calculate points in trajectory
traj_ind = round(simsetup.cpi_fr * simsetup.n_p * ...
    (0:(multi.n_fr-1))+1);
traj_points = traj.pos(:,traj_ind);


close all

track_window = [5, 5];
start_ind = 7;
end_ind = 106;
end_nan = 108;

plot_track = [movmean(scenario.multi.track_points(1,1:end_nan), track_window(1)); ...
    movmean(scenario.multi.track_points(2,1:end_nan), track_window(2))];

track_error = plot_track(1:2,start_ind:end_ind) - traj_points(1:2,start_ind:end_ind);
track_error_rms = rms(track_error,2);
total_error = rms(track_error,1);
rms(total_error)

figure
plot(track_error')

xlabel('Time [Radar Frame]', 'FontWeight', 'bold')
ylabel('Tracking Error [m]', 'FontWeight', 'bold')
title('SEMTA Trajectory Tracking Simulation')
legend('Cross-Track Error', 'Along-Track Error')

xlim([1 length(total_error)])
ylim([-50 50])
grid on


% figure;
% scatter(multi.radar_pos(2,:)/1000, multi.radar_pos(1,:)/1000 + 5, 'red', 'filled')
% hold on
% scatter(multi.lat_points(2,:)/1000, multi.lat_points(1,:)/1000 + 5)
% hold on
% plot(plot_track(2,:)/1000, plot_track(1,:)/1000 + 5)
% 
% legend('Radar Locations', 'Multilateration Points', 'Tracked Trajectory')
% xlabel('Distance Along Track [km]', 'FontWeight', 'bold')
% ylabel('Distance from Radar Line [km]', 'FontWeight', 'bold')
% title('SEMTA Trajectory Tracking Simulation')
% 
% xlim([0 25])
% ylim([0 10])
% grid on
























