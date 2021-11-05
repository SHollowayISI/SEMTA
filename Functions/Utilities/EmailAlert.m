function [] = EmailAlert(address, save_name, attach)
%EMAILALERT Sends email after completion of simulation
%   Takes email address and figure filepath as input. Sends email
%   announcing completion of simulation, and includes zip file of
%   attachments if 'attach' set true.

%% Zip files if chosen, then send email

if attach
    
    % Create zip file directory if non-existent
    if ~exist('Figures/Zip Files', 'dir')
        mkdir('Figures/Zip Files')
    end
    
    % Compress files into Zip File folder
    zip(['Figures/Zip Files/', save_name], ['Figures/', save_name, '/']);
    
    % Determine zip file size
    s = dir(['Figures/Zip Files/', save_name, '.zip']);
    filesize = s.bytes/10e6;
    
    % Send email
    if filesize > 25
        % If file size compatible send attachment
        sendmail(address, ...
            ['Test Complete: ', save_name], ...
            'See attached files for saved figures', ...
            ['Figures/Zip Files/', save_name, '.zip']);
    else
        % Otherwise send email without attachment
        sendmail(address, ...
            ['Test Complete: ', save_name], ...
            'Filesize too large to attach, figures saved on local');
    end
    
else
    
    % Send email without attachment
    sendmail(address, ...
        ['Test Complete: ', save_name], ...
        'See attached files for saved figures');
end

% Pause to ensure no timing errors
pause(5);


end

