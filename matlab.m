%% ECW26063: COMBINED ALL-IN-ONE REGENERATIVE BRAKING SIMULATION
clear; clc; close all;

fprintf('===========================================================\n');
fprintf('  ECW26063: COMBINED REGENERATIVE BRAKING DASHBOARD SYSTEM  \n');
fprintf('===========================================================\n');

%% 1. SIMULATION PARAMETERS & INITIALIZATION
dt = 0.01;            % Time step size (s)
t_final = 25;         % Total simulation time (s)
t = 0:dt:t_final;     % Time array
N = length(t);        % Total steps

% Vehicle & Electrical Specifications
m_total = 1200;       % Total EV Mass (kg)
r_wheel = 0.31;       % Wheel Radius (m)
V_nom = 350;          % Battery Nominal Voltage (V)
Capacity_Ah = 60;     % Battery Capacity (Ah)
SOC_init = 60;        % Initial Battery SOC (%)
decel_target = 3.0;   % Target Deceleration Rate (m/s^2)

% Preallocate Data Arrays for Speed & Efficiency
v_actual = zeros(1, N); 
v_target = zeros(1, N); 
T_demand = zeros(1, N);
T_regen = zeros(1, N); 
T_friction = zeros(1, N); 
P_regen_kW = zeros(1, N);
E_regen_Wh = zeros(1, N); 
SOC = zeros(1, N);

% Initial Conditions
v_actual(1) = 80 / 3.6; % Initial Speed: 80 km/h converted to m/s
SOC(1) = SOC_init;      % Start at 60% SOC

%% 2. MAIN INTEGRATED SIMULATION LOOP
for i = 1:N-1
    % -------------------------------------------------------------------
    % BLOCK 1: DRIVE PROFILE & DRIVER BRAKE DEMAND
    % -------------------------------------------------------------------
    v_cruise = 80 / 3.6; % 80 km/h in m/s
    if t(i) < 5
        v_target(i) = v_cruise;
        T_demand(i) = 0;
    else
        v_target(i) = max(0, v_cruise - decel_target * (t(i) - 5));
        speed_error = v_target(i) - v_actual(i);
        Kp_driver = 120; % Driver gain
        T_demand(i) = min(0, Kp_driver * speed_error * m_total * r_wheel);
    end

    % -------------------------------------------------------------------
    % BLOCK 2: DYNAMIC TORQUE ALLOCATION (REGEN VS FRICTION)
    % -------------------------------------------------------------------
    v_cutoff = 1.39;      % 5 km/h low-speed cutoff in m/s
    SOC_max = 90.0;       % Max allowable SOC (%)
    T_regen_max = -220;   % Max motor braking torque limit (Nm)
    
    % Battery Overcharge Protection (SOC Derating Logic)
    if SOC(i) >= SOC_max
        soc_factor = 0.0;
    elseif SOC(i) > 80.0
        soc_factor = (SOC_max - SOC(i)) / 10.0; % Linear derate from 80% to 90%
    else
        soc_factor = 1.0; % Full regen below 80% SOC
    end
    
    % Torque Splitting Logic
    if (T_demand(i) < 0) && (v_actual(i) > v_cutoff)
        T_regen_available = T_regen_max * soc_factor;
        T_regen(i) = max(T_demand(i), T_regen_available);
        T_friction(i) = T_demand(i) - T_regen(i);
    else
        T_regen(i) = 0;
        T_friction(i) = T_demand(i);
    end

    % -------------------------------------------------------------------
    % BLOCK 3: MOTOR & BATTERY ENERGY RECOVERY ENGINE
    % -------------------------------------------------------------------
    eta_overall = 0.96 * 0.90; % Net Efficiency (~86.4%: Motor 90% * Inverter 96%)
    
    if abs(T_regen(i)) > 0 && v_actual(i) > 0.1
        omega_wheel = v_actual(i) / r_wheel;
        P_mech_kW = (abs(T_regen(i)) * omega_wheel) / 1000;
        P_regen_kW(i) = P_mech_kW * eta_overall;
        
        dE_Wh = (P_regen_kW(i) * 1000 * dt) / 3600;
        dSOC = (dE_Wh / (V_nom * Capacity_Ah)) * 100;
    else
        P_regen_kW(i) = 0;
        dE_Wh = 0;
        dSOC = 0;
    end
    
    % Update Cumulative Energy and Battery State of Charge
    E_regen_Wh(i+1) = E_regen_Wh(i) + dE_Wh;
    SOC(i+1) = SOC(i) + dSOC;

    % -------------------------------------------------------------------
    % BLOCK 4: VEHICLE DYNAMICS (PHYSICS ENGINE)
    % -------------------------------------------------------------------
    Cd = 0.28;          % Drag Coefficient
    A_front = 2.2;      % Frontal Area (m^2)
    rho_air = 1.225;    % Air density (kg/m^3)
    C_rr = 0.015;       % Rolling Resistance Coefficient
    g = 9.81;           % Acceleration due to gravity (m/s^2)
    
    if v_actual(i) > 0.01
        F_aero = 0.5 * rho_air * Cd * A_front * (v_actual(i)^2);
        F_rr = m_total * g * C_rr;
        F_resist = F_aero + F_rr;
        
        F_brake_total = (T_regen(i) + T_friction(i)) / r_wheel;
        a_actual = (F_brake_total - F_resist) / m_total;
        
        v_actual(i+1) = max(0, v_actual(i) + a_actual * dt);
    else
        v_actual(i+1) = 0;
    end
end

% Align last values for graphing accuracy
v_target(end) = v_target(end-1); 
T_demand(end) = T_demand(end-1);
T_regen(end) = T_regen(end-1); 
T_friction(end) = T_friction(end-1);
P_regen_kW(end) = P_regen_kW(end-1);

%% 3. PRINT PERFORMANCE METRICS TO COMMAND WINDOW
fprintf('\n---------------- SIMULATION RESULTS ----------------\n');
fprintf('Final Speed:              %.2f km/h\n', v_actual(end) * 3.6);
fprintf('Total Energy Recovered:   %.2f Wh\n', E_regen_Wh(end));
fprintf('Final Battery SOC:        %.4f %%\n', SOC(end));
fprintf('Net SOC Increase:         +%.4f %%\n', SOC(end) - SOC_init);
fprintf('-----------------------------------------------------\n\n');

%% 4. GRAPHICAL DASHBOARD DISPLAY
figure('Name', 'ECW26063 Dashboard', 'Color', [1 1 1], 'Position', [100 100 1000 650]);

% Subplot 1: Speed Tracking
subplot(2,2,1);
plot(t, v_target*3.6, '--r', 'LineWidth', 1.5); hold on;
plot(t, v_actual*3.6, 'b', 'LineWidth', 1.8); hold off;
grid on;
title('Block 1: Vehicle Speed Tracking');
xlabel('Time (s)'); ylabel('Speed (km/h)');
legend('Target Speed', 'Actual Speed', 'Location', 'northeast');

% Subplot 2: Torque Split
subplot(2,2,2);
plot(t, T_regen, 'g', 'LineWidth', 1.8); hold on;
plot(t, T_friction, 'r', 'LineWidth', 1.8); hold off;
grid on;
title('Block 2: Dynamic Torque Allocation');
xlabel('Time (s)'); ylabel('Torque (Nm)');
legend('Regen Torque', 'Friction Torque', 'Location', 'southeast');

% Subplot 3: Power Output
subplot(2,2,3);
plot(t, P_regen_kW, 'm', 'LineWidth', 1.8);
grid on;
title('Block 3: Regenerative Power Output');
xlabel('Time (s)'); ylabel('Power (kW)');
legend('Electric Power', 'Location', 'northeast');

% Subplot 4: Energy & Battery SOC
subplot(2,2,4);
yyaxis left
plot(t, E_regen_Wh, '-b', 'LineWidth', 1.8);
ylabel('Recovered Energy (Wh)', 'Color', 'b');
xlabel('Time (s)');

yyaxis right
plot(t, SOC, '--k', 'LineWidth', 1.5);
ylabel('Battery SOC (%)', 'Color', 'k');

grid on;
title('Block 4: Energy Recovered & Battery SOC');
legend('Energy (Wh)', 'Battery SOC (%)', 'Location', 'southeast');

sgtitle('ECW26063: EV Regenerative Braking System Performance', 'FontSize', 14, 'FontWeight', 'bold');