%% ARS408-21 CAN Log Analysis — MATLAB
% Decode candump-2026-06-24_154259.log from MiviaCar
% MIVIA Lab, UNISA — Abdelmoutalib DOUADI
%
% Usage:
%   1. Set LOG_FILE path below
%   2. Run the script
%   3. Figures will appear automatically

clear; clc; close all;

%% ── Configuration ────────────────────────────────────────────────────────
LOG_FILE = 'candump-2026-06-24_154259.log';  % <-- change path if needed

% ARS408 DBC constants
FACTOR_DIST_LONG = 0.2;   OFFSET_DIST_LONG = -500.0;   % m
FACTOR_DIST_LAT  = 0.2;   OFFSET_DIST_LAT  = -204.6;   % m
FACTOR_VREL_LONG = 0.25;  OFFSET_VREL_LONG = -128.0;   % m/s
FACTOR_VREL_LAT  = 0.25;  OFFSET_VREL_LAT  = -64.0;    % m/s
FACTOR_RCS       = 0.5;   OFFSET_RCS       = -64.0;     % dBm²

CLASS_NAMES = {'POINT','CAR','TRUCK','PEDESTRIAN','MOTO','BICYCLE','WIDE','RESERVED'};
DYN_NAMES   = {'MOVING','STATIONARY','ONCOMING','CROSS_L','CROSS_R','UNKNOWN','?','STOPPED'};

%% ── Read and parse log file ──────────────────────────────────────────────
fprintf('Reading %s ...\n', LOG_FILE);
fid = fopen(LOG_FILE, 'r');
if fid < 0
    error('Cannot open file: %s', LOG_FILE);
end

% Storage
objects = struct('ts',{},'meas',{},'id',{},'x',{},'y',{},...
                 'vx',{},'vy',{},'rcs',{},'dyn',{},'cls',{},...
                 'len',{},'wid',{},'prob',{});

current_meas = -1;
obj_buffer   = containers.Map('KeyType','int32','ValueType','any');
ts_t0        = -1;
n_cycles     = 0;
obj_count    = 0;

while ~feof(fid)
    line = fgetl(fid);
    if ~ischar(line), break; end

    % Parse: (timestamp) interface ID#DATA
    tok = regexp(line, '\((\d+\.\d+)\)\s+\w+\s+([0-9A-Fa-f]+)#([0-9A-Fa-f]+)', 'tokens');
    if isempty(tok), continue; end
    tok = tok{1};
    ts     = str2double(tok{1});
    can_id = hex2dec(tok{2});
    hex_data = tok{3};

    if ts_t0 < 0, ts_t0 = ts; end
    t = ts - ts_t0;

    % Convert hex string to bytes
    n_bytes = length(hex_data)/2;
    data = zeros(1, n_bytes, 'uint8');
    for b = 1:n_bytes
        data(b) = hex2dec(hex_data(2*b-1:2*b));
    end

    switch can_id
        case 0x60A  % Obj_0_Status
            % Flush previous cycle
            if current_meas >= 0 && obj_buffer.Count > 0
                keys_list = keys(obj_buffer);
                for k = 1:length(keys_list)
                    o = obj_buffer(keys_list{k});
                    obj_count = obj_count + 1;
                    objects(obj_count) = o;
                end
                n_cycles = n_cycles + 1;
            end
            current_meas = double(bitshift(uint32(data(2)),8)) + double(data(3));
            obj_buffer   = containers.Map('KeyType','int32','ValueType','any');

        case 0x60B  % Obj_1_General
            if current_meas < 0, continue; end
            oid = double(data(1));

            % DistLong: 13-bit Motorola MSB@bit15
            raw_dl = double(bitshift(uint16(data(2)),5)) + ...
                     double(bitshift(uint8(data(3)),-3));
            x = raw_dl * FACTOR_DIST_LONG + OFFSET_DIST_LONG;

            % DistLat: 11-bit Motorola MSB@bit18
            raw_lat = double(bitshift(uint16(bitand(data(3),7)),8)) + double(data(4));
            y = raw_lat * FACTOR_DIST_LAT + OFFSET_DIST_LAT;

            % VrelLong: 10-bit Motorola MSB@bit39
            raw_vl = double(bitshift(uint16(data(5)),2)) + ...
                     double(bitshift(data(6),-6));
            vx = raw_vl * FACTOR_VREL_LONG + OFFSET_VREL_LONG;

            % VrelLat: 9-bit Motorola MSB@bit45
            raw_va = double(bitshift(uint16(bitand(data(6),63)),3)) + ...
                     double(bitshift(data(7),-5));
            vy = raw_va * FACTOR_VREL_LAT + OFFSET_VREL_LAT;

            % DynProp: B6[2:0]
            dyn = double(bitand(data(7), 7)) + 1;  % +1 for MATLAB index

            % RCS: B7
            rcs = double(data(8)) * FACTOR_RCS + OFFSET_RCS;

            o.ts   = t;
            o.meas = current_meas;
            o.id   = oid;
            o.x    = x;
            o.y    = y;
            o.vx   = vx;
            o.vy   = vy;
            o.rcs  = rcs;
            o.dyn  = dyn;
            o.cls  = 1;   % default POINT
            o.len  = 0;
            o.wid  = 0;
            o.prob = 0;

            obj_buffer(int32(oid)) = o;

        case 0x60C  % Obj_2_Quality
            if current_meas < 0 || ~isKey(obj_buffer,int32(data(1))), continue; end
            o = obj_buffer(int32(data(1)));
            o.prob = double(bitshift(data(7),-5));
            obj_buffer(int32(data(1))) = o;

        case 0x60D  % Obj_3_Extended
            if current_meas < 0 || ~isKey(obj_buffer,int32(data(1))), continue; end
            o = obj_buffer(int32(data(1)));
            o.cls = double(bitand(data(4),7)) + 1;  % +1 for MATLAB index
            o.len = double(data(7)) * 0.2;
            o.wid = double(data(8)) * 0.2;
            obj_buffer(int32(data(1))) = o;
    end
end
fclose(fid);
fprintf('Parsed %d cycles, %d object detections\n', n_cycles, obj_count);

%% ── Organize data ─────────────────────────────────────────────────────────
ts_arr  = [objects.ts]';
id_arr  = [objects.id]';
x_arr   = [objects.x]';
y_arr   = [objects.y]';
vx_arr  = [objects.vx]';
vy_arr  = [objects.vy]';
rcs_arr = [objects.rcs]';
cls_arr = [objects.cls]';
prob_arr= [objects.prob]';

range_arr  = sqrt(x_arr.^2 + y_arr.^2);
azimuth_arr = atan2d(y_arr, x_arr);

%% ── Figure 1 : Bird-Eye View (scatter plot) ───────────────────────────────
figure('Name','ARS408 — Bird-Eye View','Position',[100 100 800 600]);

cls_colors = [
    0.8 0.8 0.2;   % POINT    — yellow
    0.2 0.6 1.0;   % CAR      — blue
    1.0 0.4 0.0;   % TRUCK    — orange
    1.0 0.4 0.8;   % PEDESTRIAN — pink
    0.6 0.2 1.0;   % MOTO     — purple
    0.2 0.9 0.4;   % BICYCLE  — green
    0.5 0.9 0.9;   % WIDE     — cyan
    0.5 0.5 0.5;   % RESERVED — gray
];

hold on; grid on; box on;
for c = 1:8
    mask = cls_arr == c;
    if any(mask)
        scatter(x_arr(mask), y_arr(mask), ...
            max(20, (rcs_arr(mask)+30)*2), ...  % size = f(RCS)
            cls_colors(c,:), 'filled', 'MarkerFaceAlpha', 0.4, ...
            'DisplayName', CLASS_NAMES{c});
    end
end

% Radar position
plot(0, 0, 'r^', 'MarkerSize', 12, 'MarkerFaceColor', 'r', ...
     'DisplayName', 'Radar (SensorID=0)');

xlabel('X — Distance longitudinale (m)', 'FontSize', 12);
ylabel('Y — Distance latérale (m)', 'FontSize', 12);
title('ARS408-21 MiviaCar — Vue Plan (Bird-Eye)', 'FontSize', 14);
legend('Location', 'northwest', 'FontSize', 10);
xlim([0 25]); ylim([-10 10]);
set(gca, 'Color', [0.1 0.1 0.1]);
set(gcf, 'Color', [0.15 0.15 0.15]);
set(gca, 'XColor', 'w', 'YColor', 'w', 'GridColor', [0.3 0.3 0.3]);
title(gca, 'ARS408-21 MiviaCar — Vue Plan', 'Color', 'w', 'FontSize', 14);

%% ── Figure 2 : Timeline NofObjects ───────────────────────────────────────
figure('Name','ARS408 — NofObjects Timeline','Position',[100 150 900 350]);

% Count objects per cycle
[unique_ts, ~, ic] = unique(ts_arr);
nof_per_cycle = accumarray(ic, ones(size(ic)));

plot(unique_ts, nof_per_cycle, 'c-', 'LineWidth', 1.5);
hold on;
area(unique_ts, nof_per_cycle, 'FaceColor', [0.0 0.5 0.8], ...
     'FaceAlpha', 0.3, 'EdgeColor', 'none');

xlabel('Temps (s)', 'FontSize', 12);
ylabel('Nombre d objets', 'FontSize', 12);
title('ARS408-21 — NofObjects par cycle (13.7 Hz)', 'FontSize', 13);
ylim([0 max(nof_per_cycle)+2]);
grid on;
set(gca, 'Color', [0.1 0.1 0.1], 'XColor','w','YColor','w','GridColor',[0.3 0.3 0.3]);
set(gcf, 'Color', [0.15 0.15 0.15]);

%% ── Figure 3 : RCS Distribution ──────────────────────────────────────────
figure('Name','ARS408 — RCS Distribution','Position',[200 200 700 400]);

edges = -25:2:35;
histogram(rcs_arr, edges, 'FaceColor', [0.3 0.7 1.0], ...
          'EdgeColor', 'none', 'FaceAlpha', 0.8);

hold on;
xline(0,  '--y', 'LineWidth', 1.5, 'Label', '0 dBm²');
xline(10, '--g', 'LineWidth', 1.5, 'Label', '+10 dBm² (CAR)');

xlabel('RCS (dBm²)', 'FontSize', 12);
ylabel('Nombre de détections', 'FontSize', 12);
title('Distribution RCS — ARS408-21 MiviaCar', 'FontSize', 13);
grid on;
set(gca,'Color',[0.1 0.1 0.1],'XColor','w','YColor','w','GridColor',[0.3 0.3 0.3]);
set(gcf,'Color',[0.15 0.15 0.15]);

%% ── Figure 4 : Objets stables — Position vs temps ────────────────────────
figure('Name','ARS408 — Stable Objects','Position',[300 100 900 600]);

stable_ids = [0, 32, 56];  % from analysis
colors_stable = {[1 0.5 0], [0.2 0.6 1], [1 1 0.2]};
labels_stable = {'ID=0 (WIDE ~17.6m)', 'ID=32 (CAR ~11.4m)', 'ID=56 (POINT ~5.6m)'};

subplot(2,1,1); hold on; grid on;
for i = 1:length(stable_ids)
    mask = id_arr == stable_ids(i);
    if any(mask)
        plot(ts_arr(mask), x_arr(mask), '-', ...
             'Color', colors_stable{i}, 'LineWidth', 1.5, ...
             'DisplayName', labels_stable{i});
    end
end
ylabel('DistLong X (m)', 'FontSize', 11);
title('Objets stables — Distance longitudinale vs temps', 'FontSize', 12);
legend('Location', 'northeast', 'FontSize', 9);
ylim([0 25]);
set(gca,'Color',[0.1 0.1 0.1],'XColor','w','YColor','w','GridColor',[0.3 0.3 0.3]);
set(gcf,'Color',[0.15 0.15 0.15]);

subplot(2,1,2); hold on; grid on;
for i = 1:length(stable_ids)
    mask = id_arr == stable_ids(i);
    if any(mask)
        plot(ts_arr(mask), rcs_arr(mask), '-', ...
             'Color', colors_stable{i}, 'LineWidth', 1.5, ...
             'DisplayName', labels_stable{i});
    end
end
xlabel('Temps (s)', 'FontSize', 11);
ylabel('RCS (dBm²)', 'FontSize', 11);
title('Objets stables — RCS vs temps', 'FontSize', 12);
legend('Location', 'northeast', 'FontSize', 9);
set(gca,'Color',[0.1 0.1 0.1],'XColor','w','YColor','w','GridColor',[0.3 0.3 0.3]);

%% ── Figure 5 : Polar Plot (Range / Azimuth) ──────────────────────────────
figure('Name','ARS408 — Polar View','Position',[400 100 600 600]);

polarscatter(deg2rad(azimuth_arr), range_arr, ...
    max(10, (rcs_arr+30)*1.5), rcs_arr, 'filled', 'MarkerFaceAlpha', 0.5);

colormap('jet');
cb = colorbar;
cb.Label.String = 'RCS (dBm²)';
cb.Color = 'w';
title('ARS408-21 — Vue Polaire (Range / Azimuth)', 'FontSize', 13, 'Color', 'w');
set(gcf,'Color',[0.15 0.15 0.15]);
rlim([0 25]);

%% ── Summary stats ────────────────────────────────────────────────────────
fprintf('\n=== SUMMARY ===\n');
fprintf('Duration     : %.1f s\n', max(ts_arr));
fprintf('Cycles       : %d (%.1f Hz)\n', n_cycles, n_cycles/max(ts_arr));
fprintf('Detections   : %d\n', obj_count);
fprintf('Mean objects : %.1f per cycle\n', obj_count/n_cycles);
fprintf('\nClass distribution:\n');
for c = 1:8
    cnt = sum(cls_arr == c);
    if cnt > 0
        fprintf('  %-12s : %4d (%.1f%%)\n', CLASS_NAMES{c}, cnt, cnt/obj_count*100);
    end
end
fprintf('\nStable objects:\n');
for sid = [0 32 56]
    mask = id_arr == sid;
    if any(mask)
        fprintf('  ID=%3d  X=%.2fm  Y=%.2fm  RCS=%.1fdBm²  n=%d\n', ...
            sid, mean(x_arr(mask)), mean(y_arr(mask)), ...
            mean(rcs_arr(mask)), sum(mask));
    end
end
