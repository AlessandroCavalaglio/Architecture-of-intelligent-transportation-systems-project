% -----------------------
%% Initialization
% -----------------------

close all; clear all; clc;
format long;

% Set LaTeX as default interpreter for axis labels, ticks and legends
set( 0,     'defaulttextinterpreter',          'latex' );
set( groot, 'defaultAxesTickLabelInterpreter', 'latex' );
set( groot, 'defaultLegendInterpreter',        'latex' );

set( 0, 'DefaultFigureWindowStyle', 'docked' );
set( 0, 'defaultAxesFontSize',      18 );
set( 0, 'DefaultLegendFontSize',    18 );

addpath( genpath( 'Lib_Vehicle' ) );
addpath( genpath( 'Lib_Tyre' ) );
addpath( genpath( 'Longit_Controller' ) );
addpath( genpath( 'Lateral_Controllers' ) );
addpath( genpath( 'Scenario' ) );
addpath( genpath( 'Utilities' ) );
addpath( genpath( 'Figures' ) );
addpath( 'Clothoids/matlab' );

% --------------------
%% Open Simulink model
% --------------------
model_name = ['Vehicle_Model_2Track_ABS_ESP_KINOBS_EMPTY'];

% Check if the Simulink model is already opened. Otherwise open it
openModels = find_system( 'SearchDepth', 0 );
if ( isempty( find( strcmp( openModels, model_name ), 1 ) ) )
    load_system( model_name);
    open_system( model_name );
end