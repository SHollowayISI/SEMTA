function [] = EmailSetup()
%EMAILSETUP Sets up email alert system for TRASAT
%   Input destination email address

%% Set up email server

from_email = 'shsimupdate@gmail.com';
username = 'shsimupdate';
password = 'Frisco_5';

setpref('Internet','SMTP_Server','smtp.gmail.com');
setpref('Internet','E_mail',from_email);
setpref('Internet','SMTP_Username',username);
setpref('Internet','SMTP_Password',password);

props = java.lang.System.getProperties;
props.setProperty('mail.smtp.auth','true');
props.setProperty('mail.smtp.socketFactory.class', 'javax.net.ssl.SSLSocketFactory');
props.setProperty('mail.smtp.socketFactory.port','465');

end

