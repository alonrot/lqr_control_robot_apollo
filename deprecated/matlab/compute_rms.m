%% Initialization:
clear all;
cd /Network/Servers/wagner/Volumes/wagner/amarcovalle/amd-clmc-ws/workspace/src/catkin/pole_balancing/pole_balancing_apollo;

%% Analysis loop:

display('Analyzing and storing the data...');

% pick_transient = input('Do you want to pick the transient in the analysis? [1] Yes, [0] No: ');
pick_transient = 0;

% Loading SL data:
aux = dir('~/amd-clmc-ws/workspace/src/catkin/pole_balancing/pole_balancing_apollo');
% aux = dir;
data_sets = {}; c = 0;
for i = 1:length(aux)

    if ~isempty(strfind(aux(i).name,'d02'))
        c = c + 1;
        data_sets{c} = strcat(['~/amd-clmc-ws/workspace/src/catkin/pole_balancing/pole_balancing_apollo/' aux(i).name]);
    end

end

display(strcat(['Please, select the file you want to load among the next ones:']));
for i = 1:size(data_sets,2)
    display(strcat(['    [' num2str(i) '] ' data_sets{i}]));
end
file_number = input('    Loading the file: ');
file_name = data_sets{file_number};

% Loading variables:
[D,vars,freq] = clmcplot_convert(file_name);
var_names = {'angle_filt','cart_state_x','cart_state_xd','cart_state_xdd_f'};
measurements_ = clmcplot_getvariables(D, vars, var_names);

%% Variable selection:

t_f = input('Please, introduce the cutting time: ');

% Sampling time:
Ts = 0.001;

% Signals:
angle_filt = measurements_(1:t_f/Ts,1);
cart_state_x = measurements_(1:t_f/Ts,2);
cart_state_xd = measurements_(1:t_f/Ts,3);
cart_state_xdd_f = measurements_(1:t_f/Ts,4);

% Time stamp:
t_ = 0:0.001:(length(theta_real)-1)*0.001;

%% Signal analysis:

display(' ');
display(['RMS(angle_filt)       = ' num2str(rms(angle_filt)*180/pi) ' deg']);
display(['RMS(cart_state_x)     = ' num2str(rms(cart_state_x)) ' m']);
display(['RMS(cart_state_xd)    = ' num2str(rms(cart_state_xd)) ' m/s']);
display(['RMS(cart_state_xdd_f) = ' num2str(rms(cart_state_xdd_f)) ' m/s2']);