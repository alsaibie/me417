%%
clear all;
%% Snippet I1
diary 'snippet_I1.out'
run('snippet_I1.m')
% saveas(gcf,'snippet_I1.svg');
diary off

%% Snippet I2
diary 'snippet_I2.out'
run('snippet_I2.m')
saveas(gcf,'snippet_I2.svg');
diary off

%% Snippet I3
diary 'snippet_I3.out'
run('snippet_I3.m')
saveas(gcf,'snippet_I3.svg');
diary off

%% Snippet II1
diary 'snippet_II1.out'
run('snippet_II1.m')
% saveas(gcf,'snippet_II1.svg');
diary off

%% Snippet II2
diary 'snippet_II2.out'
run('snippet_II2.m')
saveas(gcf,'snippet_II2.svg');
diary off

%% Snippet II3
diary 'snippet_II3.out'
run('snippet_II3.m')
saveas(gcf,'snippet_II3.svg');
diary off




