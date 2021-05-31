%%
clear all;
set(groot,'defaultLineLineWidth',2.5) 
set(0,'DefaultaxesLineWidth', 1.5) 
set(0,'DefaultaxesFontSize', 18) 
set(0,'DefaultaxesFontWeight', 'bold') 
set(0,'DefaultTextFontSize', 18) 
% set(0,'DefaultaxesFontName', 'Times new Roman') 
% set(0,'DefaultlegendFontName', 'Times new Roman')
set(0,'defaultAxesXGrid','on') 
set(0,'defaultAxesYGrid','on') 

%% Snippet I1
diary 'snippet_I1.out'
run('snippet_I1.m')
% saveas(gcf,'snippet_I1.svg');
diary off

%% Snippet I2
diary 'snippet_I2.out'
run('snippet_I2.m')
set(findall(gcf,'type','line'),'linewidth',2.5);
saveas(gcf,'snippet_I2.svg');
diary off

%% Snippet I3
diary 'snippet_I3.out'
run('snippet_I3.m')
set(findall(gcf,'type','line'),'linewidth',2.5);
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
set(findall(gcf,'type','line'),'linewidth',2.5);
saveas(gcf,'snippet_II2.svg');
diary off

%% Snippet II3
diary 'snippet_II3.out'
run('snippet_II3.m')
set(findall(gcf,'type','line'),'linewidth',2.5);
saveas(gcf,'snippet_II3.svg');
diary off




