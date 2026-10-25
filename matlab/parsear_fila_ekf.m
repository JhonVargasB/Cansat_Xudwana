function [valores, es_ekf] = parsear_fila_ekf(linea)
% PARSEAR_FILA_EKF Lee el resumen serial; tambien admite el formato anterior.
% Orden: timestamp, sequence, roll, pitch, yaw, accel_D, valid,
%        height (relativa al arranque), velocity_up, vertical_valid.
    valores = [];
    linea = strtrim(string(linea));
    es_ekf = startsWith(linea, "EKF,");
    if ~es_ekf
        return;
    end
    campos = split(linea, ',');
    if ~ismember(numel(campos), [8 11])
        return;
    end
    v = str2double(campos(2:end)).';
    if any(~isfinite(v)) || any(v(1:2) < 0) ...
            || any(v(1:2) ~= fix(v(1:2))) || ~ismember(v(7), [0 1])
        return;
    end
    if numel(v) == 7
        v = [v NaN NaN 0]; % Firmware antiguo: no hay estimacion vertical.
    elseif ~ismember(v(10), [0 1])
        return;
    end
    valores = v;
end
