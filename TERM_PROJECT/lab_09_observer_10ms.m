%% Lab 0x09: Improved Motor Control with Velocity Observation
% Design A port to the saved 10 ms motor period. The continuous
% model and error-pole selections are retained from Arina's design.

%% 1. Select physical parameters and the firmware sample period

Km = 2.887;        % (rad/s)/V
tau = 0.22;        % s

Ts = 0.01;         % s; confirmed scheduler period in the Step 1 checkpoint
Vdc = 9;           % V; original Simulink setting, not a board setting
battery_reference_V = 7.65;       % Measured Step 1 reference
firmware_voltage_limit_V = 6;     % Symmetric +/-6 V limits on Romi

%% 2. Build the continuous open-loop model, including the disturbance state

A = [-1/tau     0         1;
        1       0         0;
        0       0         0];

B = [Km/tau;
       0;
       0];

C = [0 1 0];
D = 0;

assert(rank(obsv(A,C)) == 3, 'Continuous model is not observable.');

sys = ss(A,B,C,D);

%% 3. Discretize

sysd = c2d(sys,Ts,'zoh');

[Ad,Bd,Cd,Dd] = ssdata(sysd);

assert(rank(obsv(Ad,Cd)) == 3, 'Discrete model is not observable.');

%% 4. Select observer error dynamics explicitly

% Design A

t_set = 0.12;
zeta = 0.99;
s3 = -1;

wn = 4/(zeta*t_set);

poles_s = [-zeta*wn + 1i*wn*sqrt(1-zeta^2);
           -zeta*wn - 1i*wn*sqrt(1-zeta^2);
           s3];

poles_z = exp(poles_s*Ts);

L = place(Ad',Cd',poles_z)';
L_A = L;

%% 4B. Compare a second observer tuning choice

t_set_B = 0.20;
zeta_B = 0.80;
s3_B = -10;

wn_B = 4/(zeta_B*t_set_B);

poles_s_B = [-zeta_B*wn_B + 1i*wn_B*sqrt(1-zeta_B^2);
             -zeta_B*wn_B - 1i*wn_B*sqrt(1-zeta_B^2);
             s3_B];

poles_z_B = exp(poles_s_B*Ts);

L_B = place(Ad',Cd',poles_z_B)';

%% 5. Assemble observer matrices and check error poles

Ao_A = Ad - L_A*Cd;
Bo_A = [Bd-L_A*Dd L_A];
Co_A = eye(3);
Do_A = zeros(3,2);

Ao_B = Ad - L_B*Cd;
Bo_B = [Bd-L_B*Dd L_B];
Co_B = eye(3);
Do_B = zeros(3,2);

assert(all(abs(eig(Ao_A)) < 1), ...
    'Design A error poles must be inside the unit circle.');

assert(all(abs(eig(Ao_B)) < 1), ...
    'Design B error poles must be inside the unit circle.');

assert(max(min(abs(eig(Ao_A)-poles_z.'),[],2)) < 1e-8);
assert(max(min(abs(eig(Ao_B)-poles_z_B.'),[],2)) < 1e-8);

tuning_comparison = table( ...
    [t_set;t_set_B], ...
    [zeta;zeta_B], ...
    [s3;s3_B], ...
    [Ts;Ts], ...
    'VariableNames', ...
    {'SettlingTime_s','DampingRatio','ThirdPole','SamplePeriod_s'}, ...
    'RowNames',{'Design_A','Design_B'});

disp(tuning_comparison);

Ao = Ao_A;
Bo = Bo_A;
Co = Co_A;
Do = Do_A;

fprintf('\nfrom ulab import numpy as np\n\n');

printArray('Ao',Ao);
printArray('Bo',Bo);
printArray('Co',Co);
printArray('Do',Do);

%% 6B. Numerical one-step reference calculation

x_check = [1; 0.2; -0.5];
w_check = [2; 0.21];

y_check = Co*x_check + Do*w_check;
x_next_check = Ao*x_check + Bo*w_check;

format long g

disp('MATLAB current output:');
disp(y_check);

disp('MATLAB next state:');
disp(x_next_check);

%% 7. Select PI gains and create a working model from the template

Kp = 0.46;          % V/(rad/s)
Ki_cont = 2.7;      % V/rad

Ki_sample = Ki_cont*Ts;

omega_set = 1.5;    % rad/s
t_stop = 1.0;       % s

template_file = fullfile(fileparts(mfilename('fullpath')), ...
    'lab09_observer_controller.sltx');

model_name = 'lab09_observer_controller';

if ~bdIsLoaded(model_name)
    Simulink.createFromTemplate(template_file,'Name',model_name);
end

%% 7B. Simulate Design A

Ao = Ao_A;
Bo = Bo_A;
Co = Co_A;
Do = Do_A;

A_o = Ao;
B_o = Bo;
C_o = Co;
D_o = Do;

out_A = sim(model_name,'StopTime',num2str(t_stop));

v_true = out_A.logsout.getElement('lab09_true_velocity').Values;
v_obs_A = out_A.logsout.getElement('lab09_observed_velocity').Values;
v_command = out_A.logsout.getElement('lab09_voltage').Values;

%% 7C. Simulate Design B

Ao = Ao_B;
Bo = Bo_B;
Co = Co_B;
Do = Do_B;

A_o = Ao;
B_o = Bo;
C_o = Co;
D_o = Do;

out_B = sim(model_name,'StopTime',num2str(t_stop));

v_obs_B = out_B.logsout.getElement('lab09_observed_velocity').Values;

%% 7D. Compare observer tuning choices

figure('Name','Lab 09 Observer Tuning Comparison');

subplot(2,1,1);

plot(v_true.Time,v_true.Data,'k','DisplayName','True velocity');
hold on;

stairs(v_obs_A.Time,v_obs_A.Data,'b','DisplayName','Design A');
stairs(v_obs_B.Time,v_obs_B.Data,'r','DisplayName','Design B');

yline(omega_set,'--','DisplayName','Setpoint');

ylabel('Wheel velocity (rad/s)');
title('Observer velocity comparison');
legend('Location','best');
grid on;

subplot(2,1,2);

stairs(v_command.Time,v_command.Data);

ylabel('Limited voltage (V)');
xlabel('Time (s)');
grid on;

%% 7E. Restore selected Design A

Ao = Ao_A;
Bo = Bo_A;
Co = Co_A;
Do = Do_A;

A_o = Ao;
B_o = Bo;
C_o = Co;
D_o = Do;

%% 8. Firmware interface and design documentation

% State order:
% [wheel velocity (rad/s); displacement (rad); disturbance (rad/s^2)]
%
% Input order:
% [limited motor voltage (V); encoder displacement (rad)]
%
% Co selects all three estimated states.
% The first output is estimated velocity.
% Cd selects measured encoder displacement.
%
% Encoder displacement must come from cumulative encoder counts
% converted to radians using the existing encoder calibration.
%
% Romi uses the signed voltage returned by MotorDriver.set_voltage().
% It includes clipping and integer PWM duty quantization.
%
% Convert existing velocity measurements and setpoints to rad/s.
% Convert duty percentage to volts using the Lab 0x05 battery reference.
%
% Ts must match the actual motor-task execution period.
%
% Design A has the faster complex pair and the slower third pole.
% Check both transition speed and noise in the comparison plot.
% Faster poles have increased sensitivity to encoder quantization.
% Slower poles have estimation delay.
%
% Case study:
% https://charlierefvem.github.io/ME4305/case-studies/case_motor_observer.html

%% 9. Numerical verification

y_reference = Co*x_check + Do*w_check;
x_reference = Ao*x_check + Bo*w_check;

y_simulated = Co_A*x_check + Do_A*w_check;
x_simulated = Ao_A*x_check + Bo_A*w_check;

output_error = max(abs(y_reference-y_simulated));
state_error = max(abs(x_reference-x_simulated));

simulation_tolerance = 1e-10;

assert(output_error < simulation_tolerance);
assert(state_error < simulation_tolerance);

%% Reproduce the 10 ms board check
% The reported board check in the original file used Ts = 0.02 s.
% Use the firmware boot printout for the new board result. The vectors
% below are calculated references, not new measurements from Romi.
expected_output_10ms = [1; 0.2; -0.5];
expected_state_10ms = [1.2721924517745482; 0.21638478801573713; -0.4165700690188203];
board_tolerance = 1e-5;

assert(max(abs(y_check - expected_output_10ms)) < board_tolerance);
assert(max(abs(x_next_check - expected_state_10ms)) < board_tolerance);

save('lab09_observer_design_10ms.mat', ...
    'Km','tau','Ts','Vdc', ...
    'A','B','C','D', ...
    'Ad','Bd','Cd','Dd', ...
    'Ao_A','Bo_A','Co_A','Do_A', ...
    'Ao_B','Bo_B','Co_B','Do_B', ...
    'L_A','L_B', ...
    't_set','zeta','s3', ...
    't_set_B','zeta_B','s3_B', ...
    'poles_s','poles_z','poles_s_B','poles_z_B', ...
    'tuning_comparison', ...
    'Kp','Ki_cont','Ki_sample', ...
    'x_check','w_check','y_check','x_next_check', ...
    'output_error','state_error', ...
    'expected_output_10ms','expected_state_10ms', ...
    'battery_reference_V','firmware_voltage_limit_V');

%% Local function

function printArray(name,M)

    fprintf('%s = np.array([\n',name);

    for row = 1:size(M,1)

        fprintf('    [');

        for col = 1:size(M,2)

            if col > 1
                fprintf(', ');
            end

            fprintf('%.17g',M(row,col));

        end

        fprintf('],\n');

    end

    fprintf('])\n\n');

end
