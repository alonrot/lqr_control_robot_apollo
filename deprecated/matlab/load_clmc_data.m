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
var_names = {'theta_real'};
measurements_ = clmcplot_getvariables(D, vars, var_names);

%% Plotting:

theta_real = measurements_(:,1);
t_ = 0:0.001:(length(theta_real)-1)*0.001;

der_ = diff(theta_real);

peaks_ = find(der_);
intervals = diff(peaks_);
sample_time_diff = intervals*0.001;


hdl_fig = figure;
hdl_fig.Position = [300 300 1600 1200];
hist(sample_time_diff*1000,60);
xlabel('Sampling time [ms]'); ylabel('Number of ocurrences');

% plot(t_,theta_real,'o--')
% xlabel('time [s]');
% ylabel('dummy data');