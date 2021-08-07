function [] = SaveFigures(save_name,fig_path, format)
%SAVEFIGURES Saves all open figures
%   Saves all open figures to fig_path directory, with name save_name plus
%   title of figure.

if ~exist(fig_path, 'dir')
    mkdir(fig_path)
end

FigList = findobj(allchild(0), 'flat', 'Type', 'figure');
for iFig = 1:length(FigList)
    FigHandle = FigList(iFig);
    FigName   = get(FigHandle, 'Name');
    saveas(FigHandle, fullfile(fig_path, [save_name, '_', FigName, format]));
end

% Display update to command window
disp(['Figures saved in ', save_name]);

end

