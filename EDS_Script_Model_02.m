clear all; close all; clc;

%% Simulation Time
stop_time = 20;                % [sec] simulation stop time

%% Senario
senario = 3;

steering_angle_initial_m =[0;0;0;0;15];        % [deg] (initial position) average steering angle
steering_angle_1_m       =[0;0;-15;15;5];      % [deg] (position 1) average steering angle
time_steering_angle_1    =  8;                 % [sec] (position 1) start time of position 1
steering_angle_2_m       =[0;0;-15;15;-1];     % [deg] (position 2) average steering angle
time_steering_angle_2    = 10;                 % [sec] (position 2) start time of position 2
steering_angle_3_m       =[0;0;-15;15;-10];    % [deg] (position 3) average steering angle
time_steering_angle_3    = 14;                 % [sec] (position 3) start time of position 3
steering_angle_4_m       =[0;0;0;0;0];         % [deg] (position 4) average steering angle
time_steering_angle_4    = 18;                 % [sec] (position 4) start time of position 4

slope_m                  = [0;0.10;0;0;0.02;]; % 10% slope -> 5.74deg 
wheel_kph                = [80;60;80;80;80;];  % [km/h] reference vehicle speed

if senario==1
    ii=1;
elseif senario==2
    ii=2;
elseif senario==3
    ii=3;
elseif senario==4
    ii=4;
elseif senario==5
    ii=5;    
else
end

steering_angle_initial =steering_angle_initial_m(ii);
steering_angle_1       =steering_angle_1_m(ii); 
steering_angle_2       =steering_angle_2_m(ii);  
steering_angle_3       =steering_angle_3_m(ii);     
steering_angle_4       =steering_angle_4_m(ii);

%% Load Torque Calculation
g  = 9.81;           % [m/s^2] Gravitational acceleration
pi = 4*atan(1);      % [-] number pi
d  = 1.5;            % [m] [m] distance between the wheels of the same axle or distance between the back and the front wheel
L  = 2.5;            % [m] wheelbase or the distance between front and rear wheels 

slope = slope_m(ii); % 10% slope -> 5.74deg  
a  = asind(slope);   % [deg] Grade angle of the road
a=a/180*pi;          % [rad] convert deg to rad
fr = 0.01;           % [-] Vehicle friction coefficient
Rw = 0.26;           % [m] Wheel radius
M  = 1200;           % [kg] Vehicle total mass
S  = 1.9;            % [m^2] Vehicle frontal area
density = 1.3;       % [kg/m^3] Air density, At 101.325 kPa (abs) and 15 °C (59 °F), air has a density of approximately 1.225 kg/m3 
Cx = 0.25;           % [-] Aerodynamic drag coefficient

K_tor_all_1 = 1/2; % [-] Torque allocation in wheel 1
K_tor_all_2 = 1/2; % [-] Torque allocation in wheel 1

%% Gearbox
i_gear = 1;          % [-] Gearbox ratio 
n_eff_gear = 1;      % [-] Efficiency of the gearbox 

%% Speed Reference
% w_ref_initial = 80/(3.6*Rw);      % [rad/s] (initial position) the accelerator pedal command sets the common reference speed
% w_ref_1       = w_ref_initial;    % [rad/s] (position 1) the accelerator pedal command sets the common reference speed
% time_w_ref_1  = 5;                % [sec] (position 1) start time of position 1
% w_ref_2       = w_ref_initial;    % [rad/s] (position 2) the accelerator pedal command sets the common reference speed
% time_w_ref_2  = 9;                % [sec] (position 2) start time of position 2
% w_ref = w_ref_2;                  % [rad/s] the accelerator pedal command sets the common reference speed
% % rpm_ref = w_ref*60/(2*pi);      % [rpm] the accelerator pedal command sets the common reference speed

wheel_radps = wheel_kph(ii)/(3.6*Rw);% [rad/s] reference vehicle speed
% accel = 1*115.4567759/7.2;         % motor accel r/s  % 615.3846=accel*5.33   u = a*t 769.2308
accel_time = 5.33;                   % 6.6625 for 100km/h
accel = wheel_radps/accel_time;      % motor accel r/s 

%% 3-Phase Inverter / Universal Bridge
% [1] Work (Youtube)
% Power Electronic device type: IGBT/Diodes
R_snubber = 1e5; % [Ohms] Snubber resistance Rs 
Cs = inf;        % [F] Snubber capacitance
Ron = 1e-3;      % [Ohms]
Vf = 0;          % [V] Forward voltage Device
Vfd = 0;         % [V] Forward voltage Diode

%% DC voltage sourve
Vdc_1 = 500; % [V] DC voltage sourve
Vdc_2 = 500; % [V] DC voltage sourve
% 1 Volt second [Vs] = 1 Weber [Wb] 

slope = slope_m(ii); % 10% slope -> 5.74deg  
a  = asind(slope);   % [deg] Grade angle of the road
a=a/180*pi;          % [rad] convert deg to rad

%% Controllers
Kp_w  = 55/0.024;     % Proportional gain of speed controller
Ki_w  = 2/0.024;      % Integral     gain of speed controller
Kd_w  = 0.5/8;    % Derivative   gain of speed controller
Filter_coefficient = 100/50;
Satur_max_PI_w = 40+30;
Satur_min_PI_w = -40-30;
Kp_iq = 30/12;  % Proportional gain of iq controller
Ki_iq = 10/14;  % Integral     gain of iq controller
Satur_max_PI_iq = Vdc_1*sqrt(1/3);
Satur_min_PI_iq = -Vdc_1*sqrt(1/3);
Kp_id = 5/12;   % Proportional gain of id controller
Ki_id = 0.2/14; % Integral     gain of id controller
Satur_max_PI_id = Vdc_1*sqrt(1/3);
Satur_min_PI_id = -Vdc_1*sqrt(1/3);

%% PMSM
% Number of phases:  3
% Back EMF waveform: Sinusoidal
% Rotor type:        Round
% Mechanical input:  Torque Tm
% % [1] Work (Youtube) Rs meion => a(%) auxnetai
Rs = 4.47-1.47-0.40;    % [Ohm]      % Stator phase resistance
La = 0.00395; % [H]        % Armature inductance
Kt = 2.56+0.24;    % [Nm/Apeak] % Torque constant
J  = 0.07/4;    % [kg.m^2]   % Inertia
F  = 4e-04;   % [N.m.s]    % Viscous damping
p  = 4;       % [-]        % Pole pairs
Tf = 0;       % [N.m]      % Static friction
% [J F p Tf]
% Initial_motor_speed = 5;


%%
% wheel_kph = w_ref*Rw*3.6/i_gear
% wheel_kph = 80;
% w_ref_req = wheel_kph/ (Rw*3.6/i_gear)

% sim('Electronic_Differential_PMSM_DC_Motor_Simulink_Model_02')

% drawnow;
% ppp = 1;
% stop;

% shh = get(0,'ShowHiddenHandles');
% set(0,'ShowHiddenHandles','On');
% set(gcf,'PaperPositionMode','auto');
% set(gcf,'InvertHardcopy','off');
% saveas(gcf,'mypic.jpg','-r600');
% set(0,'ShowHiddenHandles',shh);

% if true
% fig = gcf;
% fig.Color = 'k'; %background color.
% % fig.DockControls = 'off';
% % fig.RendererMode = 'manual';
% % fig.Renderer = 'painters';
% fig.Units = 'centimeters';
% % fig.Resize = 'off';
% % fig.InvertHardcopy = 'on';
% % fig.PaperPositionMode = 'manual';
% fig.PaperUnits = 'centimeters';
% % %Con l'orientazione landscape inverte width e height.
% % fig.PaperOrientation = 'portrait';
% % fig.resolution = 1555;
% fig.PaperSize = [13 8];
% % fig.PaperPosition = [4.205 2.97 75.69 53.46];
% % directory = 'C:\Users\panos\OneDrive\Υπολογιστής\Desktop\Master_Thesis';
% % print(directory, '-painters', '-dtiffn', '-r900');
% % saveas(gcf,'Field_Current4.png','-dpng','-r900');% print(fig,'Field_Current4.png','-dpng','-r900');
% % print(fig,'Field_Current4.png','-dpng','-r900');
% % exportgraphics(gcf,'Field_Current5.png');
% 
% % print(gcf,'foo.png','-dpng','-r600'); //600dpi
% % Print -djpg -r300 picture1
% % set(fig, 'PaperUnits', 'centimeters');
% % set(fig, 'Units', 'centimeters');
% % set(fig, 'Color','k');
% % set(fig, 'PaperSize', [23 18]);
% % set(fig, 'PaperPositionMode', 'manual');
% % set(fig, 'PaperPosition', [0 0 4 2]);
% % % fig = get_param('sldemo_tank','Handle');
% % print(fig,'Field_Current4.png', 'Resolution',1000);
% 
% end

% %% Electric Vehicle Stability with Rear Electronic Differential Traction (PMSM)
% %% & Electronic Differential with Direct Torque Fuzzy Control for Vehicle Propulsion System (PMSM drives)
% % (PMSM drives)
% 
% % The specifications of the vehicle used in simulation
% M = 1200;           % [kg] Vehicle total mass 
% L = 1.5;            % [m] Distance between two wheels and axes
% d = 2.5;            % [m] Distance between the back and the front wheel 
% Rw = 0.26;          % [m] Wheel radius
% S = 1.9;            % [m^2] Vehicle frontal area 
% Cx = 0.25;          % [N/(ms)^2] Aerodynamic drag coefficient
% i_gear = 7.2;       % [-] Gearbox ratio 
% n_ef_gear = 0.98;   % [-] Efficiency of the gearbox 
% % 25.2; % [kWh] Battery energy capacity 
% % 205; % [km] Autonomy
% 
% % The specifications of motors
% motor_Resistance = 0.03;      % [Ω] Resistance
% d_axis_inductance = 0.2;      % [mH] d−axis inductance
% q_axis_inductance = 0.2;      % [mH] q−axis inductance
% permanent_magnet_flux = 0.08; % [Wb] Permanent magnet flux 
% pole_pairs = 4;               % [-] Number of poles
% 
% 
% %% Fuzzy Logic Speed Control Stability Improvement of Lightweight Electric Vehicle Drive
% % (IM)
% 
% % Electric vehicle Parameters
% Te = 247;           % [Nm] Motor traction torque 
% Je = 7.07;          % [Kgm^2] Moment on inertia of the drive train 
% Rw = 0.36;          % [m] Wheel radius 
% i_gear = 10.0;      % [-] Total gear ratio
% n_ef_gear = 0.93;   % [-] Total transmission efficiency
% M = 3904;           % [Kg] Vehicle mass 
% fe = 0.001;         % [-] Bearing friction coefficient
% Cx = 0.46;          % [] Aerodynamic coefficient 
% S = 3.48;           % [m^2] Vehicle frontal area
% fr = 0.01;          % [-] Vehicle friction coefficient 
% Lw = 2.5;           % [m] Distance between two wheels and axes 
% dw = 1.5;           % [m] Distance between the back and the front wheel 
% 
% % Induction Motors Parameters
% Rr = 0.003;     % [Ω] Rotor winding resistance (per phase)
% Rs = 0.0044;    % [Ω] Stator winding resistance (per phase) 
% Ls = 16.1;      % [μH] Stator leakage inductance (per pohase) 
% Lm = 482;       % [μH] Magnetizing inductance (per phase)
% Lr = 12.9;      % [μH] Rotor leakage inductance (per phase)
% fc = 0.0014;    % [-] Friction coefficient 
% P  = 4;         % [-] Number of poles

