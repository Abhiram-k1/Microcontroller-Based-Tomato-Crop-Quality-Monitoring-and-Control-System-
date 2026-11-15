%% TOMATO CROP MONITORING AND CONTROL - SIMULINK BUILDER ALIGNED WITH main.m
% Creates Tomato_Crop_Control_Main.slx: a Simulink twin of main.m
% (Microcontroller-Based-Tomato-Crop-Quality-Monitoring-and-Control-System).
%
% What "aligned with main.m" means here:
%   - Same 11-state nonlinear plant as main.m Tomato_Dynamics, including the
%     selectable soil type (Loamy / Sandy / Clay), K relaxation target 190,
%     evaporation-dependent tank refill and the pump water-level interlock.
%   - Same discrete-time execution as main.m: dt = 0.01 h, forward-Euler
%     plant update with physical state bounds, 2401 samples over 24 h.
%   - Same three controllers, with gains designed by main.m's own routines
%     (copied unchanged below): Decentralized PID, Pole Placement, LQR-I.
%     P.fixLqriAW = true applies the corrected LQR-I anti-windup; use it with the
%     patched main.m so both still match sample-for-sample.
%   - Same sensor layer: Gaussian noise, same sigmas, same sensor ranges,
%     same random stream (rng(42) + the Pole-Placement draws), so a MATLAB
%     run reproduces main.m sample-for-sample.
%   - Same growth-stage setpoints, FAO/UC Davis ranges, Crop Condition
%     Index and benchmark metrics (ISE, IAE, total variation, mean CCI).
%
% How to use:
%   1. Put this file in the project folder (next to main.m).
%   2. Run: build_tomato_simulink_main
%   Requires Simulink (MATLAB Function blocks). No other toolboxes.
%
% Controller selection inside the model: model-workspace variable ctrlType
%   1 = Decentralized PID, 2 = Pole Placement, 3 = LQR-I
%
% Simulation time is in HOURS. The 24 h timeline is a compressed
% demonstration of growth-stage setpoints, not a real tomato life cycle.

clearvars;
clc;

%% ---------------- User parameters (edit here) ----------------
model          = 'Tomato_Crop_Control_Main';
P.soilType     = 'Loamy';   % 'Loamy' (main.m default), 'Sandy' or 'Clay'
P.ctrlType     = 3;         % controller shown when the model is opened (1/2/3)
P.noiseOn      = 1;         % 1 = sensor noise as in main.m, 0 = ideal sensors
P.fixLqriAW    = true;      % true: corrected LQR-I anti-windup (use with the patched main.m)
                            % false: original main.m logic (temperature integrator locks the fan at 100%)
runAfterBuild  = true;      % simulate after building
runAllCtrls    = true;      % true: run PID, PP and LQR-I like main.m
compareToMain  = true;      % compare with results/master_simulation_results.mat if found

P.Ts   = 0.01;              % h  (main.m dt)
P.Tend = 24;                % h
ctrlNames = {'Decentralized PID', 'Pole Placement', 'LQR-I'};

%% ---------------- 1. Offline design (identical to main.m steps 1-5) ----------------
rng(42);                                           % main.m line 27
db        = getOnlineTomatoDatabaseLocal();
params    = getParametersLocal(db, P.soilType);
trimPoint = findTrimLocal(params);
[A, B, C_full] = linearizeLocal(trimPoint.x0, trimPoint.u0, trimPoint.d0, params);
C_primary = zeros(4, 11);
C_primary(1, 1) = 1; C_primary(2, 2) = 1; C_primary(3, 3) = 1; C_primary(4, 4) = 1;
pidCtrl  = designPIDLocal(params);
ppCtrl   = designPolePlacementLocal(A, B);         % calls rng(42) internally, as in main.m
lqriCtrl = designLQRILocal(A, B, C_primary);

fprintf('Soil type: %s\n', params.soil.type);
fprintf('Trim actuators: Pump=%.1f%%, Fan=%.1f%%, Mist=%.1f%%, Light=%.1f%%\n', trimPoint.u0 * 100);
fprintf('Pole placement max |K| = %.4f | LQR-I CARE residual = %.2e\n', ppCtrl.maxGain, lqriCtrl.residual);

% Sensor-noise sequences: main.m draws them in this order after the PP design
Nsamp = round(P.Tend / P.Ts) + 1;
tVec  = (0:Nsamp-1)' * P.Ts;
sc    = getSensorConfigLocal();
sigma = [sc.noiseStd]';
noiseSeq = cell(1, 3);
for c = 1:3
    n = zeros(Nsamp, 11);
    for k = 1:Nsamp
        for i = 1:11
            n(k, i) = sigma(i) * randn();          % same call pattern as simulateSensorLayerLocal
        end
    end
    noiseSeq{c} = struct('time', tVec, ...
        'signals', struct('values', n, 'dimensions', 11));
end

%% ---------------- 2. Create model ----------------
if bdIsLoaded(model)
    close_system(model, 0);
end
new_system(model);
open_system(model);

set_param(model, ...
    'StartTime', '0', ...
    'StopTime', num2str(P.Tend), ...
    'SolverType', 'Fixed-step', ...
    'Solver', 'FixedStepDiscrete', ...
    'FixedStep', num2str(P.Ts), ...
    'SaveTime', 'on', ...
    'TimeSaveName', 'tout', ...
    'SaveOutput', 'on', ...
    'OutputSaveName', 'yout', ...
    'SignalLogging', 'on', ...
    'SignalLoggingName', 'logsout');

mw = get_param(model, 'ModelWorkspace');
assignin(mw, 'Ts', P.Ts);
assignin(mw, 'x0', params.initialState);
assignin(mw, 'ctrlType', P.ctrlType);
assignin(mw, 'sensorNoise', noiseSeq{P.ctrlType});

%% ---------------- 3. Top-level blocks ----------------
add_block('simulink/Sources/Digital Clock', [model '/Simulation_Clock'], ...
    'SampleTime', 'Ts', 'Position', [35 65 75 95]);
add_block('simulink/Sources/Constant', [model '/Controller_Select'], ...
    'Value', 'ctrlType', 'SampleTime', 'Ts', 'Position', [345 280 425 310]);

add_block('simulink/Ports & Subsystems/Subsystem', [model '/01_Growth_Stage'], ...
    'Position', [145 35 300 115]);
add_block('simulink/Ports & Subsystems/Subsystem', [model '/02_Disturbance_Generator'], ...
    'Position', [145 155 300 225]);
add_block('simulink/Ports & Subsystems/Subsystem', [model '/03_Sensor_Layer'], ...
    'Position', [510 35 690 125]);
add_block('simulink/Ports & Subsystems/Subsystem', [model '/04_Controller_PID_PP_LQRI'], ...
    'Position', [510 175 690 275]);
add_block('simulink/Ports & Subsystems/Subsystem', [model '/05_Safety_Actuators'], ...
    'Position', [765 175 930 265]);
add_block('simulink/Ports & Subsystems/Subsystem', [model '/06_Tomato_Plant'], ...
    'Position', [1035 80 1220 190]);
add_block('simulink/Ports & Subsystems/Subsystem', [model '/07_Crop_Condition_Index'], ...
    'Position', [765 330 945 425]);

%% ---------------- 4. Build subsystems ----------------
buildFcnSubsystem([model '/01_Growth_Stage'], 'Growth_Stage_Function', growthCode(db), ...
    {'time_h'}, {'setpoints', 'stage_index'});
buildFcnSubsystem([model '/02_Disturbance_Generator'], 'Disturbance_Function', disturbanceCode(), ...
    {'time_h'}, {'disturbance_vector'});
buildSensorSubsystem([model '/03_Sensor_Layer'], sensorCode(sc, P.noiseOn));
buildFcnSubsystem([model '/04_Controller_PID_PP_LQRI'], 'Tri_Hybrid_Controller', ...
    controllerCode(P.Ts, pidCtrl, ppCtrl, lqriCtrl, trimPoint, P.fixLqriAW), ...
    {'controller_type', 'setpoint_vector', 'sensor_vector'}, {'actuator_command'});
buildFcnSubsystem([model '/05_Safety_Actuators'], 'Safety_Interlock_Function', ...
    safetyCode(params.safety.minWaterLevel), ...
    {'actuator_command', 'true_plant_states'}, {'safe_actuator_vector', 'low_water_warning'});
buildPlantSubsystem([model '/06_Tomato_Plant'], plantStepCode(params, P.Ts), P.Ts);
buildFcnSubsystem([model '/07_Crop_Condition_Index'], 'Crop_Condition_Function', qualityCode(db), ...
    {'sensor_vector', 'stage_index'}, {'quality_score_vector', 'high_voc_warning'});

%% ---------------- 5. Top-level wiring (same signal flow as main.m loop) ----------------
add_line(model, 'Simulation_Clock/1', '01_Growth_Stage/1', 'autorouting', 'on');
add_line(model, 'Simulation_Clock/1', '02_Disturbance_Generator/1', 'autorouting', 'on');

add_line(model, '06_Tomato_Plant/1', '03_Sensor_Layer/1', 'autorouting', 'on');

add_line(model, 'Controller_Select/1', '04_Controller_PID_PP_LQRI/1', 'autorouting', 'on');
add_line(model, '01_Growth_Stage/1', '04_Controller_PID_PP_LQRI/2', 'autorouting', 'on');
add_line(model, '03_Sensor_Layer/1', '04_Controller_PID_PP_LQRI/3', 'autorouting', 'on');

add_line(model, '04_Controller_PID_PP_LQRI/1', '05_Safety_Actuators/1', 'autorouting', 'on');
add_line(model, '06_Tomato_Plant/1', '05_Safety_Actuators/2', 'autorouting', 'on');   % main.m interlock uses x(10)

add_line(model, '05_Safety_Actuators/1', '06_Tomato_Plant/1', 'autorouting', 'on');
add_line(model, '02_Disturbance_Generator/1', '06_Tomato_Plant/2', 'autorouting', 'on');

add_line(model, '03_Sensor_Layer/1', '07_Crop_Condition_Index/1', 'autorouting', 'on');
add_line(model, '01_Growth_Stage/2', '07_Crop_Condition_Index/2', 'autorouting', 'on');

%% ---------------- 6. Monitoring and logging ----------------
addDisplayScope(model, 'Sensor_Readings', '03_Sensor_Layer/1', [735 35 820 75], [850 25 900 75]);
addDisplayScope(model, 'Actuator_Commands', '05_Safety_Actuators/1', [955 205 1030 245], [955 265 1005 305]);
addDisplayScope(model, 'Crop_Condition', '07_Crop_Condition_Index/1', [975 350 1050 390], [1080 345 1130 395]);

add_block('simulink/Sinks/Display', [model '/Low_Water_Warning_Display'], 'Position', [955 445 1030 480]);
add_line(model, '05_Safety_Actuators/2', 'Low_Water_Warning_Display/1', 'autorouting', 'on');
add_block('simulink/Sinks/Display', [model '/High_VOC_Warning_Display'], 'Position', [1080 445 1155 480]);
add_line(model, '07_Crop_Condition_Index/2', 'High_VOC_Warning_Display/1', 'autorouting', 'on');

addToWorkspace(model, 'Log_Plant_States',      'plant_states',      [1270 35 1370 65]);
addToWorkspace(model, 'Log_Sensor_Readings',   'sensor_readings',   [1270 85 1370 115]);
addToWorkspace(model, 'Log_Actuator_Commands', 'actuator_commands', [1270 135 1370 165]);
addToWorkspace(model, 'Log_Quality_Scores',    'quality_scores',    [1270 185 1370 215]);
addToWorkspace(model, 'Log_Setpoints',         'setpoints',         [1270 235 1370 265]);

add_line(model, '06_Tomato_Plant/1',          'Log_Plant_States/1',      'autorouting', 'on');
add_line(model, '03_Sensor_Layer/1',          'Log_Sensor_Readings/1',   'autorouting', 'on');
add_line(model, '05_Safety_Actuators/1',      'Log_Actuator_Commands/1', 'autorouting', 'on');
add_line(model, '07_Crop_Condition_Index/1',  'Log_Quality_Scores/1',    'autorouting', 'on');
add_line(model, '01_Growth_Stage/1',          'Log_Setpoints/1',         'autorouting', 'on');

Simulink.Annotation(model, sprintf( ...
    'TOMATO CROP MONITORING & CONTROL (aligned with main.m) | 11 states | 11 sensors | 4 actuators | 5 stages | soil: %s', ...
    params.soil.type));
Simulink.Annotation(model, 'ctrlType (model workspace): 1 = Decentralized PID, 2 = Pole Placement, 3 = LQR-I. Time in hours.');

%% ---------------- 7. Check, save ----------------
try
    set_param(model, 'SimulationCommand', 'update');
    disp('Diagram update OK: no dimension or algebraic-loop errors.');
catch ME
    warning('Diagram update reported a problem:\n%s', ME.message);
end
set_param(model, 'ZoomFactor', 'FitSystem');
save_system(model, [model '.slx']);
fprintf('\nCreated model: %s.slx  (Ts = %g h, stop = %g h)\n', model, P.Ts, P.Tend);

%% ---------------- 8. Simulate (same benchmark as main.m) ----------------
if runAfterBuild
    if runAllCtrls, ctrlList = 1:3; else, ctrlList = P.ctrlType; end
    res = cell(1, 3);
    for c = ctrlList
        assignin(mw, 'ctrlType', c);
        assignin(mw, 'sensorNoise', noiseSeq{c});
        simOut = sim(model);
        res{c} = collectResults(simOut, P.Ts);
        fprintf('  Simulated: %s\n', ctrlNames{c});
    end
    assignin(mw, 'ctrlType', P.ctrlType);              % leave the chosen default in the model
    assignin(mw, 'sensorNoise', noiseSeq{P.ctrlType});
    save_system(model);

    printBenchmark(res, ctrlNames);
    plotResults(res, ctrlNames, params.soil.type);
    save('simulink_main_results.mat', 'res', 'ctrlNames', 'params', 'trimPoint', ...
         'A', 'B', 'pidCtrl', 'ppCtrl', 'lqriCtrl');

    if compareToMain
        compareWithMain(res);
    end
end

%% ========================================================================
% MODEL-BUILDING HELPERS
% ========================================================================

function buildFcnSubsystem(sys, fcnName, code, inNames, outNames)
    clearSubsystem(sys);
    for i = 1:numel(inNames)
        addIn(sys, inNames{i}, [25 40+60*(i-1) 55 60+60*(i-1)]);
    end
    for j = 1:numel(outNames)
        addOut(sys, outNames{j}, [470 45+60*(j-1) 500 65+60*(j-1)]);
    end
    h = 40 + 60 * max(numel(inNames), numel(outNames));
    addMATLAB(sys, fcnName, code, [180 35 360 35+h]);
    for i = 1:numel(inNames)
        add_line(sys, [inNames{i} '/1'], sprintf('%s/%d', fcnName, i), 'autorouting', 'on');
    end
    for j = 1:numel(outNames)
        add_line(sys, sprintf('%s/%d', fcnName, j), [outNames{j} '/1'], 'autorouting', 'on');
    end
end

function buildSensorSubsystem(sys, code)
    % true state + main.m Gaussian noise sequence (From Workspace) -> range limits
    clearSubsystem(sys);
    addIn(sys, 'true_plant_states', [25 45 55 65]);
    addOut(sys, 'sensor_vector', [470 75 500 95]);
    add_block('simulink/Sources/From Workspace', [sys '/Sensor_Noise_Sequence'], ...
        'VariableName', 'sensorNoise', 'SampleTime', 'Ts', 'Interpolate', 'off', ...
        'OutputAfterFinalValue', 'Holding final value', 'Position', [25 115 135 145]);
    addMATLAB(sys, 'Sensor_Model', code, [200 45 360 135]);
    add_line(sys, 'true_plant_states/1', 'Sensor_Model/1', 'autorouting', 'on');
    add_line(sys, 'Sensor_Noise_Sequence/1', 'Sensor_Model/2', 'autorouting', 'on');
    add_line(sys, 'Sensor_Model/1', 'sensor_vector/1', 'autorouting', 'on');
end

function buildPlantSubsystem(sys, code, Ts)
    % x(k+1) = bound( x(k) + f(x(k),u(k),d(k))*dt ) : main.m forward-Euler loop
    clearSubsystem(sys);
    addIn(sys, 'actuator_vector', [25 55 55 75]);
    addIn(sys, 'disturbance_vector', [25 145 55 165]);
    addOut(sys, 'plant_state_vector', [560 85 590 105]);
    add_block('simulink/Discrete/Unit Delay', [sys '/State_Memory_x_k'], ...
        'InitialCondition', 'x0', 'SampleTime', num2str(Ts), 'Position', [420 75 460 115]);
    addMATLAB(sys, 'Tomato_Plant_Euler_Step', code, [170 55 330 175]);
    add_line(sys, 'State_Memory_x_k/1', 'Tomato_Plant_Euler_Step/1', 'autorouting', 'on');
    add_line(sys, 'actuator_vector/1', 'Tomato_Plant_Euler_Step/2', 'autorouting', 'on');
    add_line(sys, 'disturbance_vector/1', 'Tomato_Plant_Euler_Step/3', 'autorouting', 'on');
    add_line(sys, 'Tomato_Plant_Euler_Step/1', 'State_Memory_x_k/1', 'autorouting', 'on');
    add_line(sys, 'State_Memory_x_k/1', 'plant_state_vector/1', 'autorouting', 'on');
end

function clearSubsystem(sys)
    try, delete_line(sys, 'In1/1', 'Out1/1'); end %#ok<TRYNC>
    try, delete_block([sys '/In1']); end %#ok<TRYNC>
    try, delete_block([sys '/Out1']); end %#ok<TRYNC>
end

function addIn(sys, name, pos)
    n = numel(find_system(sys, 'SearchDepth', 1, 'BlockType', 'Inport')) + 1;
    b = add_block('simulink/Ports & Subsystems/In1', [sys '/' name], 'Position', pos);
    set_param(b, 'Port', num2str(n));
end

function addOut(sys, name, pos)
    n = numel(find_system(sys, 'SearchDepth', 1, 'BlockType', 'Outport')) + 1;
    b = add_block('simulink/Ports & Subsystems/Out1', [sys '/' name], 'Position', pos);
    set_param(b, 'Port', num2str(n));
end

function addMATLAB(sys, name, code, pos)
    path = [sys '/' name];
    add_block('simulink/User-Defined Functions/MATLAB Function', path, 'Position', pos);
    rt = sfroot;
    chart = rt.find('-isa', 'Stateflow.EMChart', 'Path', path);
    if isempty(chart)
        error('Could not find MATLAB Function chart at %s. Check Stateflow installation.', path);
    end
    chart.Script = code;
end

function addToWorkspace(model, blockName, variableName, pos)
    add_block('simulink/Sinks/To Workspace', [model '/' blockName], ...
        'VariableName', variableName, 'SaveFormat', 'Timeseries', 'Position', pos);
end

function addDisplayScope(model, label, src, dispPos, scopePos)
    add_block('simulink/Sinks/Display', [model '/' label '_Display'], 'Position', dispPos);
    add_block('simulink/Sinks/Scope',   [model '/' label '_Scope'],   'Position', scopePos);
    add_line(model, src, [label '_Display/1'], 'autorouting', 'on');
    add_line(model, src, [label '_Scope/1'],   'autorouting', 'on');
end

%% ========================================================================
% POST-PROCESSING (same metrics as main.m simulateClosedLoopLocal)
% ========================================================================

function r = collectResults(simOut, dt)
    [t, X]  = getTimeData(simOut.plant_states);
    [~, Y]  = getTimeData(simOut.sensor_readings);
    [~, U]  = getTimeData(simOut.actuator_commands);
    [~, Q]  = getTimeData(simOut.quality_scores);
    [~, SP] = getTimeData(simOut.setpoints);
    r.time = t; r.states = X; r.sensors = Y; r.inputs = U; r.setpoints = SP;
    r.quality.environmental = Q(:, 1); r.quality.soil = Q(:, 2);
    r.quality.nutrient = Q(:, 3);      r.quality.overall = Q(:, 4);
    errs = X(:, 1:4) - SP;
    r.metrics.totalISE = sum(sum(errs.^2, 1) * dt);
    r.metrics.totalIAE = sum(sum(abs(errs), 1) * dt);
    r.metrics.totalTV  = sum(sum(abs(diff(U, 1, 1)), 1));
    r.metrics.meanCCI  = mean(Q(:, 4));
    % unit-free tracking error: mean |error| as % of setpoint, per loop [M T RH L]
    r.metrics.errPct   = 100 * mean(abs(errs) ./ abs(SP), 1);
end

function [t, D] = getTimeData(signal)
    t = signal.Time(:);
    D = signal.Data;
    n = numel(t);
    sz = size(D);
    timeDim = find(sz == n, 1, 'last');
    if isempty(timeDim)
        error('Cannot match the time vector (%d samples) with the logged signal.', n);
    end
    order = [timeDim, setdiff(1:ndims(D), timeDim, 'stable')];
    D = reshape(permute(D, order), n, []);
end

function printBenchmark(res, names)
    fprintf('\n===================================================================\n');
    fprintf('  QUANTITATIVE PERFORMANCE BENCHMARK MATRIX (Simulink, main.m twin)\n');
    fprintf('===================================================================\n');
    fprintf('  %-32s', 'Metric'); fprintf('%20s', names{:}); fprintf('\n');
    rows = {'Total ISE (Tracking Error):', 'totalISE', '%20.2f';
            'Total IAE:',                  'totalIAE', '%20.2f';
            'Total Actuator Variation:',   'totalTV',  '%20.2f';
            'Mean Crop Condition Index:',  'meanCCI',  '%19.2f%%'};
    for r = 1:size(rows, 1)
        fprintf('  %-32s', rows{r, 1});
        for c = 1:3
            if isempty(res{c}), fprintf('%20s', '-');
            else, fprintf(rows{r, 3}, res{c}.metrics.(rows{r, 2})); end
        end
        fprintf('\n');
    end
    lbl = {'Moisture', 'Temperature', 'Humidity', 'Illumination'};
    fprintf('  Mean |error| as %% of setpoint (unit-free, raw ISE is dominated by lux^2):\n');
    for j = 1:4
        fprintf('  %-32s', ['  ' lbl{j} ':']);
        for c = 1:3
            if isempty(res{c}), fprintf('%20s', '-');
            else, fprintf('%19.2f%%', res{c}.metrics.errPct(j)); end
        end
        fprintf('\n');
    end
    fprintf('===================================================================\n\n');
end

function plotResults(res, names, soilType)
    cols = [0.85 0.20 0.15; 0.90 0.55 0.10; 0.10 0.50 0.85];
    idx = find(~cellfun(@isempty, res));
    r0 = res{idx(1)};
    yNames = {'Soil Moisture (%)', 'Temperature (^oC)', 'Relative Humidity (%)', 'Illumination (lux)'};
    aNames = {'Irrigation Pump (u_1)', 'Ventilation Fan (u_2)', 'Misting System (u_3)', 'Grow Light (u_4)'};

    figure('Name', 'Simulink twin: state tracking', 'Color', 'w');
    for i = 1:4
        subplot(2, 2, i);
        plot(r0.time, r0.setpoints(:, i), 'k--', 'LineWidth', 1.6); hold on;
        for c = idx, plot(res{c}.time, res{c}.states(:, i), 'Color', cols(c, :), 'LineWidth', 1.2); end
        grid on; xlabel('Time (h)'); ylabel(yNames{i});
        legend([{'Target'}, names(idx)], 'Location', 'best', 'FontSize', 8);
        markStages();
    end
    sgtitle(sprintf('State tracking across growth stages (soil: %s)', soilType));

    figure('Name', 'Simulink twin: actuators', 'Color', 'w');
    for j = 1:4
        subplot(2, 2, j);
        for c = idx, plot(res{c}.time, 100 * res{c}.inputs(:, j), 'Color', cols(c, :), 'LineWidth', 1.1); hold on; end
        grid on; ylim([-5 105]); xlabel('Time (h)'); ylabel('Duty cycle (%)'); title(aNames{j});
        legend(names(idx), 'Location', 'best', 'FontSize', 8);
        markStages();
    end

    figure('Name', 'Simulink twin: crop condition index', 'Color', 'w');
    f = {'environmental', 'soil', 'nutrient', 'overall'};
    t4 = {'Environmental', 'Soil (pH & EC)', 'Nutrient (NPK)', 'Composite CCI'};
    for q = 1:4
        subplot(2, 2, q);
        for c = idx, plot(res{c}.time, res{c}.quality.(f{q}), 'Color', cols(c, :), 'LineWidth', 1.1); hold on; end
        grid on; ylim([65 108]); xlabel('Time (h)'); ylabel('Score (0-100)'); title(t4{q});
        legend(names(idx), 'Location', 'southwest', 'FontSize', 8);
        markStages();
    end
end

function markStages()
    yl = ylim;
    for s = [2 8 12 18]
        plot([s s], yl, 'k:', 'HandleVisibility', 'off');
    end
    ylim(yl);
end

function compareWithMain(res)
    cands = {fullfile(pwd, 'results', 'master_simulation_results.mat'), ...
             fullfile(fileparts(mfilename('fullpath')), 'results', 'master_simulation_results.mat')};
    f = '';
    for i = 1:numel(cands)
        if isfile(cands{i}), f = cands{i}; break; end
    end
    if isempty(f)
        fprintf('main.m results not found (looked in %s). Put this file next to main.m and run main.m first.\n', cands{1});
        return;
    end
    M = load(f);
    tags = {'resultsPID', 'resultsPP', 'resultsLQRI'};
    fprintf('Comparison with main.m (%s):\n', f);
    for c = 1:3
        if isempty(res{c}) || ~isfield(M, tags{c}), continue; end
        ref = M.(tags{c});
        n = min(size(ref.states, 1), size(res{c}.states, 1));
        dX = max(abs(ref.states(1:n, :) - res{c}.states(1:n, :)), [], 'all');
        dU = max(abs(ref.inputs(1:n, :) - res{c}.inputs(1:n, :)), [], 'all');
        fprintf('  %-12s max|x_sim - x_main| = %.3g   max|u_sim - u_main| = %.3g   CCI %.4f vs %.4f\n', ...
            tags{c}, dX, dU, res{c}.metrics.meanCCI, ref.metrics.meanCCI);
    end
end

%% ========================================================================
% MATLAB FUNCTION BLOCK CODE GENERATORS
% (numbers are taken from main.m's database / design, printed at full precision)
% ========================================================================

function s = num(v)
    s = mat2str(v, 17);
end

function code = growthCode(db)
    L = {'function [sp, stage] = GrowthStage(t)'
         '% sp = [moisture; temperature; humidity; light] (main.m db.phase setpoints)'};
    th = [2 8 12 18];
    for p = 1:5
        ph = db.phase(p);
        spv = num([ph.M_sp; ph.T_sp; ph.H_sp; ph.L_sp]);
        if p == 1, L{end+1} = sprintf('if t < %g', th(p));
        elseif p < 5, L{end+1} = sprintf('elseif t < %g', th(p));
        else, L{end+1} = 'else'; end
        L{end+1} = sprintf('    stage = %d; sp = %s;', p, spv);
    end
    L{end+1} = 'end';
    L{end+1} = 'end';
    code = strjoin(L, newline);
end

function code = disturbanceCode()
    code = strjoin({
    'function d = DisturbanceGenerator(t)'
    '% main.m getDisturbancesLocal'
    't_day = mod(t, 24);'
    'extT = 23.5 + 5.5 * sin(2 * pi * (t_day - 9.5) / 24);'
    'if t_day >= 6.0 && t_day <= 19.5'
    '    natL = max(0, 22000.0 * sin(pi * (t_day - 6.0) / 13.5));'
    'else'
    '    natL = 0.0;'
    'end'
    'evap = 0.35 + 0.60 * (natL / 22000.0) + 0.03 * max(extT - 22.0, 0);'
    'd = [extT; natL; evap];'
    'end'}, newline);
end

function code = sensorCode(sc, noiseOn)
    code = strjoin({
    'function y = SensorModel(x, n)'
    '% main.m simulateSensorLayerLocal: y = sat(x + sigma*randn), randn sequence fed in as n'
    sprintf('lo = %s;', num([sc.minVal]'))
    sprintf('hi = %s;', num([sc.maxVal]'))
    sprintf('noiseOn = %d;', noiseOn)
    'y = zeros(11, 1);'
    'for i = 1:11'
    '    sensed = x(i) + noiseOn * n(i);'
    '    y(i) = max(lo(i), min(hi(i), sensed));'
    'end'
    'end'}, newline);
end

function code = controllerCode(Ts, pid, pp, lq, trim, fixAW)
    Kp = [pid.moisture.Kp; pid.temperature.Kp; pid.humidity.Kp; pid.light.Kp];
    Ki = [pid.moisture.Ki; pid.temperature.Ki; pid.humidity.Ki; pid.light.Ki];
    Kd = [pid.moisture.Kd; pid.temperature.Kd; pid.humidity.Kd; pid.light.Kd];
    if fixAW
        % Freeze integrator j only if integrating y_err(j) would push u(j) further into
        % saturation. du(j)/dz(j) = -Kz(j,j); its sign differs for the temperature loop.
        awLine = {'        push = -Kz(j,j) * y_err(j);', ...
                  '        if ~((u_raw(j) >= 1 && push > 0) || (u_raw(j) <= 0 && push < 0))'};
    else
        awLine = {'        % original main.m anti-windup', ...
                  '        if ~((u_raw(j) >= 1 && y_err(j) < 0) || (u_raw(j) <= 0 && y_err(j) > 0))'};
    end
    code = strjoin({
    'function u_cmd = TriHybridController(ctype, sp, y)'
    '% ctype: 1 = Decentralized PID, 2 = Pole Placement, 3 = LQR-I (main.m step 6 switch)'
    'persistent intg pe z_int'
    sprintf('dt = %s;', num(Ts))
    sprintf('Kp = %s;', num(Kp))
    sprintf('Ki = %s;', num(Ki))
    sprintf('Kd = %s;', num(Kd))
    sprintf('u0 = %s;', num(trim.u0))
    sprintf('x0t = %s;', num(trim.x0))
    sprintf('Kpp = %s;', num(pp.K))
    sprintf('Kx = %s;', num(lq.Kx))
    sprintf('Kz = %s;', num(lq.Ki))
    'if isempty(intg),  intg  = zeros(4, 1); end'
    'if isempty(pe),    pe    = zeros(4, 1); end'
    'if isempty(z_int), z_int = zeros(4, 1); end'
    'yv = y(:); spv = sp(:);'
    'u_cmd = zeros(4, 1);'
    '% tracking errors from SENSED values (temperature reversed: fan cools)'
    'e = [spv(1) - yv(1); yv(2) - spv(2); spv(3) - yv(3); spv(4) - yv(4)];'
    'xtarget = x0t; xtarget(1:4) = spv;'
    'if ctype == 1'
    '    for j = 1:4'
    '        ur = Kp(j)*e(j) + Ki(j)*intg(j) + Kd(j)*(e(j) - pe(j))/dt;'
    '        u_cmd(j) = max(0, min(1, ur));'
    '        if ~((ur >= 1 && e(j) > 0) || (ur <= 0 && e(j) < 0))'
    '            intg(j) = intg(j) + e(j) * dt;'
    '        end'
    '        pe(j) = e(j);'
    '    end'
    'elseif ctype == 2'
    '    u_cmd = u0 - Kpp * (yv - xtarget);'
    'else'
    '    u_raw = u0 - Kx * (yv - xtarget) - Kz * z_int;'
    '    u_cmd = max(0, min(1, u_raw));'
    '    y_err = yv(1:4) - spv(1:4);'
    '    for j = 1:4'
    awLine{1}
    awLine{2}
    '            z_int(j) = z_int(j) + y_err(j) * dt;'
    '        end'
    '    end'
    'end'
    'end'}, newline);
end

function code = safetyCode(minWater)
    code = strjoin({
    'function [u_actual, lowWater] = SafetyLogic(u_cmd, x)'
    '% main.m: actuator saturation, then pump cut-off when TRUE tank level < minimum'
    'u_actual = max(0.0, min(1.0, u_cmd(:)));'
    sprintf('lowWater = double(x(10) < %s);', num(minWater))
    'if lowWater > 0'
    '    u_actual(1) = 0.0;'
    'end'
    'end'}, newline);
end

function code = plantStepCode(p, Ts)
    s = p.soil;
    code = strjoin({
    'function xNext = TomatoPlantStep(x, u, d)'
    '% main.m Tomato_Dynamics + forward Euler + enforcePhysicalBoundsLocal'
    sprintf('%% Soil type: %s', s.type)
    sprintf('dt = %s;', num(Ts))
    sprintf('minWater = %s;', num(p.safety.minWaterLevel))
    sprintf('drainageGain = %s; irrigationGain = %s; evapFactor = %s;', num(s.drainageGain), num(s.irrigationGain), num(s.evapFactor))
    sprintf('drainageThreshold = %s; leachFac = %s;', num(s.drainageThreshold), num(s.leachFactor))
    'xv = x(:);'
    'pump      = max(0, min(1, u(1)));'
    'fan       = max(0, min(1, u(2)));'
    'mist      = max(0, min(1, u(3)));'
    'growLight = max(0, min(1, u(4)));'
    'if xv(10) < minWater'
    '    pump = 0.0;'
    'end'
    'extT = d(1); natL = d(2); evap = d(3);'
    'M = xv(1); T = xv(2); L = xv(4); pH = xv(5); EC = xv(6);'
    'N = xv(7); P = xv(8); K = xv(9); VOC = xv(11);'
    'drainage = drainageGain * max(M - drainageThreshold, 0);'
    'moistureLoss = evapFactor * evap * (1 + 0.01 * max(T - 25, 0));'
    'dM = irrigationGain * pump - moistureLoss - drainage;'
    'dT = 0.15 * (extT - T) + 0.00008 * L - 3.0 * fan;'
    'dH = 8.0 * mist - 4.0 * fan - 0.15 * max(T - 25, 0);'
    'dL = 0.5 * (natL - L) + 30000.0 * growLight;'
    'transpiration = (0.50 + 0.50 * (L / 25000.0)) * (1.0 + 0.02 * max(T - 22, 0));'
    'dpH = 0.04 * (6.35 - pH) + 0.02 * (6.45 - pH) * pump - 0.0002 * max(N - 140, 0);'
    'dEC = 0.55 * pump - 0.05 * (evap / 0.8) - 0.03 * leachFac * drainage * EC + 0.02 * (2.4 - EC);'
    'dN = 22.0 * pump - 3.8 * transpiration - 0.05 * leachFac * drainage * (N / 100.0) + 0.03 * (130.0 - N);'
    'dP = 7.5 * pump - 1.2 * transpiration - 0.02 * leachFac * drainage * (P / 50.0) + 0.02 * (48.0 - P);'
    'dK = 28.0 * pump - 4.8 * transpiration - 0.04 * leachFac * drainage * (K / 150.0) + 0.03 * (190.0 - K);'
    'refillNominal = 0.5 * evap + 0.2 * 0.15;'
    'q_refill = 0.05 * (80.0 - xv(10)) + refillNominal;'
    'dWater = q_refill - (0.5 * evap + 0.2 * pump);'
    'dVOC = 0.05 * (100.0 - VOC) + 0.10 * max(T - 30, 0);'
    'dx = [dM; dT; dH; dL; dpH; dEC; dN; dP; dK; dWater; dVOC];'
    'xNext = xv + dx * dt;'
    'lo = [0; -10; 0; 0; 0; 0; 0; 0; 0; 0; 0];'
    'hi = [100; 60; 100; 100000; 14; 15; 500; 500; 500; 100; 500];'
    'xNext = max(lo, min(hi, xNext));'
    'end'}, newline);
end

function code = qualityCode(db)
    L = {'function [q, vocWarning] = CropQuality(y, stage)'
         '% main.m calcQualityOnlineLocal; q = [environmental; soil; nutrient; overall]'
         'st = max(1, min(5, round(stage)));'
         'switch st'};
    for p = 1:5
        ph = db.phase(p);
        if p < 5, L{end+1} = sprintf('case %d', p); else, L{end+1} = 'otherwise'; end
        L{end+1} = sprintf('    Tr=%s; Hr=%s; Mr=%s; Lr=%s;', num(ph.T_range), num(ph.H_range), num(ph.M_range), num(ph.L_range));
        L{end+1} = sprintf('    pHr=%s; ECr=%s; Nr=%s; Pr=%s; Kr=%s;', num(ph.pH_range), num(ph.EC_range), num(ph.N_range), num(ph.P_range), num(ph.K_range));
    end
    L = [L; {
    'end'
    'qEnv  = (rSc(y(2), Tr(1), Tr(2)) + rSc(y(3), Hr(1), Hr(2)) + rSc(y(1), Mr(1), Mr(2)) + rSc(y(4), Lr(1), Lr(2))) / 4.0;'
    'qSoil = (rSc(y(5), pHr(1), pHr(2)) + rSc(y(6), ECr(1), ECr(2))) / 2.0;'
    'qNutr = (rSc(y(7), Nr(1), Nr(2)) + rSc(y(8), Pr(1), Pr(2)) + rSc(y(9), Kr(1), Kr(2))) / 3.0;'
    'qTot  = max(0, min(100, 0.50 * qEnv + 0.25 * qSoil + 0.25 * qNutr));'
    'q = [qEnv; qSoil; qNutr; qTot];'
    'vocWarning = double(y(11) > 200);   % extra alarm (not part of main.m CCI)'
    'end'
    ''
    'function s = rSc(val, low, high)'
    'if val >= low && val <= high'
    '    s = 100.0;'
    'elseif val < low'
    '    s = max(0.0, 100.0 - ((low - val) / max(low * 0.40, 1.0)) * 100.0);'
    'else'
    '    s = max(0.0, 100.0 - ((val - high) / max(high * 0.40, 1.0)) * 100.0);'
    'end'
    'end'}];
    code = strjoin(L, newline);
end

%% ========================================================================
% main.m DESIGN ROUTINES (copied unchanged so the gains are identical)
% ========================================================================

function db = getOnlineTomatoDatabaseLocal()
    db.cropName = 'Tomato (Solanum lycopersicum)';
    db.phase(1).name = 'Germination'; db.phase(1).tSpan = [0, 2];
    db.phase(1).T_sp = 24.0; db.phase(1).T_range = [20, 28];
    db.phase(1).H_sp = 72.5; db.phase(1).H_range = [65, 80];
    db.phase(1).M_sp = 70.0; db.phase(1).M_range = [60, 80];
    db.phase(1).L_sp = 10000; db.phase(1).L_range = [5000, 15000];
    db.phase(1).pH_range = [6.0, 6.6]; db.phase(1).EC_range = [1.6, 2.5];
    db.phase(1).N_range = [90, 140]; db.phase(1).P_range = [30, 55]; db.phase(1).K_range = [120, 180];

    db.phase(2).name = 'Vegetative'; db.phase(2).tSpan = [2, 8];
    db.phase(2).T_sp = 25.0; db.phase(2).T_range = [21, 27];
    db.phase(2).H_sp = 65.0; db.phase(2).H_range = [55, 75];
    db.phase(2).M_sp = 60.0; db.phase(2).M_range = [50, 70];
    db.phase(2).L_sp = 18000; db.phase(2).L_range = [12000, 25000];
    db.phase(2).pH_range = [6.0, 6.6]; db.phase(2).EC_range = [1.8, 2.7];
    db.phase(2).N_range = [115, 165]; db.phase(2).P_range = [35, 60]; db.phase(2).K_range = [140, 200];

    db.phase(3).name = 'Flowering'; db.phase(3).tSpan = [8, 12];
    db.phase(3).T_sp = 23.5; db.phase(3).T_range = [20, 26];
    db.phase(3).H_sp = 62.5; db.phase(3).H_range = [50, 70];
    db.phase(3).M_sp = 58.0; db.phase(3).M_range = [50, 65];
    db.phase(3).L_sp = 22500; db.phase(3).L_range = [15000, 30000];
    db.phase(3).pH_range = [6.1, 6.7]; db.phase(3).EC_range = [2.0, 2.9];
    db.phase(3).N_range = [110, 155]; db.phase(3).P_range = [40, 68]; db.phase(3).K_range = [155, 220];

    db.phase(4).name = 'Fruit Dev.'; db.phase(4).tSpan = [12, 18];
    db.phase(4).T_sp = 25.0; db.phase(4).T_range = [21, 28];
    db.phase(4).H_sp = 60.0; db.phase(4).H_range = [50, 70];
    db.phase(4).M_sp = 65.0; db.phase(4).M_range = [55, 75];
    db.phase(4).L_sp = 22500; db.phase(4).L_range = [15000, 30000];
    db.phase(4).pH_range = [6.1, 6.7]; db.phase(4).EC_range = [2.1, 3.1];
    db.phase(4).N_range = [105, 150]; db.phase(4).P_range = [38, 62]; db.phase(4).K_range = [165, 230];

    db.phase(5).name = 'Maturity'; db.phase(5).tSpan = [18, 24];
    db.phase(5).T_sp = 22.5; db.phase(5).T_range = [18, 26];
    db.phase(5).H_sp = 55.0; db.phase(5).H_range = [45, 65];
    db.phase(5).M_sp = 55.0; db.phase(5).M_range = [45, 65];
    db.phase(5).L_sp = 20000; db.phase(5).L_range = [12000, 28000];
    db.phase(5).pH_range = [6.0, 6.6]; db.phase(5).EC_range = [2.2, 3.2];
    db.phase(5).N_range = [95, 140]; db.phase(5).P_range = [32, 55]; db.phase(5).K_range = [150, 210];
end

function p = getParametersLocal(db, soilType)
    if nargin < 2 || isempty(soilType)
        soilType = 'Loamy';
    end
    p.crop.name = db.cropName;
    p.initialState = [60; 25; 65; 15000; 6.35; 2.4; 130; 48; 180; 80; 100];
    p.initialInput = [0; 0; 0; 0];
    p.disturbance.externalTemperature = 25.0;
    p.disturbance.naturalLight = 15000.0;
    p.disturbance.evaporation = 0.8;
    p.sim.startTime = 0;
    p.sim.endTime   = 24;
    p.safety.minWaterLevel = 10.0;
    p.soil.type = soilType;
    switch lower(soilType)
        case 'sandy'
            p.soil.drainageGain = 0.045; p.soil.irrigationGain = 14.0; p.soil.evapFactor = 2.60;
            p.soil.drainageThreshold = 40.0; p.soil.leachFactor = 1.60;
        case 'clay'
            p.soil.drainageGain = 0.008; p.soil.irrigationGain = 9.5; p.soil.evapFactor = 1.40;
            p.soil.drainageThreshold = 65.0; p.soil.leachFactor = 0.50;
        otherwise % 'loamy'
            p.soil.drainageGain = 0.020; p.soil.irrigationGain = 12.0; p.soil.evapFactor = 2.00;
            p.soil.drainageThreshold = 50.0; p.soil.leachFactor = 1.00;
    end
end

function sc = getSensorConfigLocal()
    sc = [
        struct('stateName', 'x1: Soil Moisture', 'noiseStd', 0.50, 'minVal', 0,   'maxVal', 100), ...
        struct('stateName', 'x2: Temperature',   'noiseStd', 0.20, 'minVal', -40, 'maxVal', 85), ...
        struct('stateName', 'x3: Humidity',      'noiseStd', 0.80, 'minVal', 0,   'maxVal', 100), ...
        struct('stateName', 'x4: Illumination',  'noiseStd', 50.0, 'minVal', 0,   'maxVal', 65535), ...
        struct('stateName', 'x5: Soil pH',       'noiseStd', 0.05, 'minVal', 0,   'maxVal', 14), ...
        struct('stateName', 'x6: Soil EC',       'noiseStd', 0.02, 'minVal', 0,   'maxVal', 10), ...
        struct('stateName', 'x7: Nitrogen',      'noiseStd', 1.00, 'minVal', 0,   'maxVal', 1999), ...
        struct('stateName', 'x8: Phosphorus',    'noiseStd', 0.50, 'minVal', 0,   'maxVal', 1999), ...
        struct('stateName', 'x9: Potassium',     'noiseStd', 1.00, 'minVal', 0,   'maxVal', 1999), ...
        struct('stateName', 'x10: Water Level',  'noiseStd', 0.50, 'minVal', 0,   'maxVal', 100), ...
        struct('stateName', 'x11: Gas / VOC',    'noiseStd', 1.00, 'minVal', 0,   'maxVal', 500) ...
    ];
end

function dx = Tomato_Dynamics(x, u, d, params)
    pump = max(0, min(1, u(1))); fan = max(0, min(1, u(2)));
    mist = max(0, min(1, u(3))); growLight = max(0, min(1, u(4)));
    if x(10) < params.safety.minWaterLevel
        pump = 0.0;
    end
    extT = d(1); natL = d(2); evap = d(3);
    soil = params.soil;
    M = x(1); T = x(2); L = x(4); pH = x(5); EC = x(6); N = x(7);
    P = x(8); K = x(9); VOC = x(11);
    drainage = soil.drainageGain * max(M - soil.drainageThreshold, 0);
    moistureLoss = soil.evapFactor * evap * (1 + 0.01 * max(T - 25, 0));
    dM = soil.irrigationGain * pump - moistureLoss - drainage;
    dT = 0.15 * (extT - T) + 0.00008 * L - 3.0 * fan;
    dH = 8.0 * mist - 4.0 * fan - 0.15 * max(T - 25, 0);
    dL = 0.5 * (natL - L) + 30000.0 * growLight;
    transpiration = (0.50 + 0.50 * (L / 25000.0)) * (1.0 + 0.02 * max(T - 22, 0));
    leachFac = 1.0;
    if isfield(soil, 'leachFactor'), leachFac = soil.leachFactor; end
    dpH = 0.04 * (6.35 - pH) + 0.02 * (6.45 - pH) * pump - 0.0002 * max(N - 140, 0);
    dEC = 0.55 * pump - 0.05 * (evap / 0.8) - 0.03 * leachFac * drainage * EC + 0.02 * (2.4 - EC);
    dN = 22.0 * pump - 3.8 * transpiration - 0.05 * leachFac * drainage * (N / 100.0) + 0.03 * (130.0 - N);
    dP = 7.5 * pump - 1.2 * transpiration - 0.02 * leachFac * drainage * (P / 50.0) + 0.02 * (48.0 - P);
    dK = 28.0 * pump - 4.8 * transpiration - 0.04 * leachFac * drainage * (K / 150.0) + 0.03 * (190.0 - K);
    refillNominal = 0.5 * evap + 0.2 * 0.15;
    if isfield(params, 'water') && isfield(params.water, 'refillNominal')
        refillNominal = params.water.refillNominal;
    end
    q_refill = 0.05 * (80.0 - x(10)) + refillNominal;
    dWater = q_refill - (0.5 * evap + 0.2 * pump);
    dVOC = 0.05 * (100.0 - VOC) + 0.10 * max(T - 30, 0);
    dx = [dM; dT; dH; dL; dpH; dEC; dN; dP; dK; dWater; dVOC];
end

function trim = findTrimLocal(p)
    d0 = [p.disturbance.externalTemperature; p.disturbance.naturalLight; p.disturbance.evaporation];
    soil = p.soil;
    M_star = 60.0; T_star = 25.0; H_star = 65.0; L_star = 15000.0;
    extT = d0(1); natL = d0(2); evap = d0(3);
    drainage = soil.drainageGain * max(M_star - soil.drainageThreshold, 0);
    moistureLoss = soil.evapFactor * evap * (1 + 0.01 * max(T_star - 25, 0));
    u1_star = (moistureLoss + drainage) / soil.irrigationGain;
    u2_star = max(0, (0.15 * (extT - T_star) + 0.00008 * L_star) / 3.0);
    u3_star = max(0, (4.0 * u2_star - (-0.15 * max(T_star - 25, 0))) / 8.0);
    if natL < L_star
        u4_star = min(1.0, max(0.0, (L_star - natL) / 30000.0));
    else
        u4_star = 0.0;
    end
    u0 = [max(0, min(1, u1_star)); max(0, min(1, u2_star)); max(0, min(1, u3_star)); max(0, min(1, u4_star))];
    transpiration = (0.50 + 0.50 * (L_star / 25000.0)) * (1.0 + 0.02 * max(T_star - 22, 0));
    leachFac = 1.0;
    if isfield(soil, 'leachFactor'), leachFac = soil.leachFactor; end
    N_star  = (22.0 * u0(1) - 3.8 * transpiration + 0.03 * 130.0) / (0.03 + 0.05 * leachFac * (drainage / 100.0));
    P_star  = (7.5 * u0(1) - 1.2 * transpiration + 0.02 * 48.0) / (0.02 + 0.02 * leachFac * (drainage / 50.0));
    K_star  = (28.0 * u0(1) - 4.8 * transpiration + 0.03 * 190.0) / (0.03 + 0.04 * leachFac * (drainage / 150.0));
    pH_star = (0.04 * 6.35 + 0.02 * 6.45 * u0(1) - 0.0002 * max(N_star - 140, 0)) / (0.04 + 0.02 * u0(1));
    EC_star = (0.55 * u0(1) - 0.05 * (evap / 0.8) + 0.02 * 2.4) / (0.02 + 0.03 * leachFac * drainage);
    x0 = [M_star; T_star; H_star; L_star; pH_star; EC_star; N_star; P_star; K_star; 80.0; 100.0];
    p.water.refillNominal = 0.5 * evap + 0.2 * u0(1);
    dx0 = Tomato_Dynamics(x0, u0, d0, p);
    trim.x0 = x0; trim.u0 = u0; trim.d0 = d0; trim.dx0 = dx0;
    trim.maxEnvDeriv = max(abs(dx0(1:4)));
    trim.maxAllDeriv = max(abs(dx0));
end

function [A, B, C, D] = linearizeLocal(x0, u0, d0, p)
    nx = length(x0); nu = length(u0); h = 1e-6;
    f0 = Tomato_Dynamics(x0, u0, d0, p);
    A = zeros(nx, nx); B = zeros(nx, nu);
    for i = 1:nx
        xp = x0; xm = x0;
        xp(i) = xp(i) + h; xm(i) = xm(i) - h;
        A(:, i) = (Tomato_Dynamics(xp, u0, d0, p) - Tomato_Dynamics(xm, u0, d0, p)) / (2 * h);
    end
    for j = 1:nu
        up = u0;
        if u0(j) + h <= 1.0
            up(j) = up(j) + h;
            B(:, j) = (Tomato_Dynamics(x0, up, d0, p) - f0) / h;
        else
            up(j) = up(j) - h;
            B(:, j) = (f0 - Tomato_Dynamics(x0, up, d0, p)) / h;
        end
    end
    C = eye(nx); D = zeros(nx, nu);
end

function Co = pureCtrbLocal(A, B)
    n = size(A, 1); m = size(B, 2);
    Co = zeros(n, n * m); Co(:, 1:m) = B;
    for k = 2:n
        Co(:, (k-1)*m + 1 : k*m) = A * Co(:, (k-2)*m + 1 : (k-1)*m);
    end
end

function [T, Acc, Bcc, Auu, r] = pureStaircaseLocal(A, B)
    n = size(A, 1);
    Co = pureCtrbLocal(A, B);
    [U_svd, S_svd, ~] = svd(Co, 'econ');
    s_vals = diag(S_svd);
    tol = max(size(Co)) * eps(max(s_vals));
    r = sum(s_vals > max(tol, 1e-10 * s_vals(1)));
    if r == 0
        T = eye(n);
    elseif r == n
        T = U_svd(:, 1:n);
    else
        Uc = U_svd(:, 1:r);
        [Q_full, ~] = qr(Uc);
        T = [Uc, Q_full(:, r+1:n)];
    end
    Abar = T' * A * T; Bbar = T' * B;
    Acc = Abar(1:r, 1:r); Bcc = Bbar(1:r, :);
    if r < n, Auu = Abar(r+1:n, r+1:n); else, Auu = []; end
end

function ctrl = designPIDLocal(~)
    ctrl.moisture.Kp    = 0.080;   ctrl.moisture.Ki    = 0.015;   ctrl.moisture.Kd    = 0.005;
    ctrl.temperature.Kp = 0.200;   ctrl.temperature.Ki = 0.020;   ctrl.temperature.Kd = 0.010;
    ctrl.humidity.Kp    = 0.100;   ctrl.humidity.Ki    = 0.015;   ctrl.humidity.Kd    = 0.005;
    ctrl.light.Kp       = 0.00005; ctrl.light.Ki       = 0.00001; ctrl.light.Kd       = 0.00001;
end

function results = designPolePlacementLocal(A, B)
    n = size(A, 1); m = size(B, 2);
    A_env = A(1:4, 1:4); B_env = B(1:4, :);
    desiredPoles = [-0.60; -0.80; -1.00; -1.20];
    bestCond = Inf; bestG = zeros(m, 4); bestX = zeros(4, 4);
    rng(42);
    for trial = 1:30
        if trial == 1
            G = repmat(eye(m), 1, ceil(4/m)); G = G(:, 1:4);
        else
            G = randn(m, 4);
            for j = 1:4, G(:, j) = G(:, j) / max(norm(G(:, j)), 1e-6); end
        end
        X_cand = zeros(4, 4);
        for j = 1:4
            Ash = A_env - desiredPoles(j) * eye(4);
            if rcond(Ash) < 1e-13, Ash = Ash - 1e-7 * eye(4); end
            X_cand(:, j) = Ash \ (B_env * G(:, j));
        end
        c = cond(X_cand);
        if ~isnan(c) && c < bestCond
            bestCond = c; bestG = G; bestX = X_cand;
        end
    end
    K_env = real(bestG / bestX);
    K = [K_env, zeros(m, n - 4)];
    results.K = K;
    results.desiredPoles = desiredPoles;
    results.closedLoopPoles = eig(A - B * K);
    results.maxGain = max(abs(K(:)));
end

function results = designLQRILocal(A, B, C_meas)
    n = size(A, 1); m = size(B, 2); p = 4;
    C_track = C_meas(1:p, :);
    Aa = [A, zeros(n, p); C_track, zeros(p, p)];
    Ba = [B; zeros(p, m)];
    na = size(Aa, 1);
    stateScale = [60; 25; 65; 15000; 6.4; 2.5; 150; 50; 180; 80; 100];
    intScale   = [10; 5; 10; 5000];
    Sa = blkdiag(diag(stateScale), diag(intScale));
    Su = eye(m);
    Aa_s = Sa \ Aa * Sa;
    Ba_s = Sa \ Ba * Su;
    [Ta, Aca, Bca, ~, ra] = pureStaircaseLocal(Aa_s, Ba_s);
    Qa_diag = [25; 35; 25; 15; 0.5*ones(7, 1); 60; 80; 50; 30];
    Qa_s = diag(Qa_diag);
    Qca = Ta(:, 1:ra)' * Qa_s * Ta(:, 1:ra);
    Qca = (Qca + Qca') / 2;
    R = diag([1.5, 1.2, 1.5, 0.8]);
    G_ric = Bca * (R \ Bca'); G_ric = (G_ric + G_ric') / 2;
    Ham = [Aca, -G_ric; -Qca, -Aca'];
    [U_h, T_h] = schur(Ham, 'real');
    [U_h, ~]   = ordschur(U_h, T_h, 'lhp');
    V1 = U_h(1:ra, 1:ra); V2 = U_h(ra+1:2*ra, 1:ra);
    P_care = real((V2 / V1 + (V2 / V1)') / 2);
    resCARE = norm(Aca' * P_care + P_care * Aca - P_care * G_ric * P_care + Qca, 'fro');
    Kca = R \ (Bca' * P_care);
    Ka_s = [Kca, zeros(m, na - ra)] * Ta';
    Ka = Su * Ka_s / Sa;
    results.Kx = Ka(:, 1:n);
    results.Ki = Ka(:, n+1:end);
    results.Ka = Ka;
    results.closedLoopPoles = eig(Aca - Bca * Kca);
    results.residual = resCARE;
    results.maxGain = max(abs(Ka(:)));
end
