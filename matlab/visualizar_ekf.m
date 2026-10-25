% Captura del resumen EKF por serial. Compatible con MATLAB R2021a/R2021b.
% Ejecutar con Run. Cerrar idf.py monitor antes de abrir el mismo puerto.
% Consultar puertos disponibles: serialportlist("available")
%
% Aceleracion NED positiva hacia abajo; altura y velocidad positivas arriba.
% La altura es relativa al punto de arranque del ESP32 (referencia cero).
% EKF,timestamp_us,sequence,roll_deg,pitch_deg,yaw_deg,linear_ad_m_s2,valid,
%     height_m,vertical_velocity_m_s,vertical_valid

puerto = "COM9";             % Cambiar por el puerto del ESP32.
baudios = 115200;
duracion_s = Inf;            % Inf: hasta pulsar Detener o cerrar la ventana.
ventana_s = 30;              % Intervalo visible en las graficas.
carpeta_salida = fullfile(fileparts(mfilename('fullpath')), 'capturas');

datos_ekf = capturar_ekf(puerto, baudios, duracion_s, ventana_s, carpeta_salida);

function datos = capturar_ekf(puerto, baudios, duracion_s, ventana_s, carpeta)
    validateattributes(duracion_s, {'numeric'}, {'scalar', 'positive'});
    validateattributes(ventana_s, {'numeric'}, {'scalar', 'positive', 'finite'});
    if ~isfolder(carpeta)
        mkdir(carpeta);
    end
    % tempname evita sobrescribir otra captura iniciada el mismo segundo.
    [~, identificador] = fileparts(tempname(carpeta));
    base = fullfile(carpeta, ['ekf_' datestr(now, 'yyyymmdd_HHMMSS') '_' identificador]);

    muestras = zeros(1000, 12);
    n = 0;
    descartadas = 0;
    segmento = 1;
    origen_us = NaN;
    ultimo_us = NaN;
    ultima_secuencia = NaN;
    desplazamiento_s = 0;
    fig = [];
    titulo = [];
    serial = [];
    % Se ejecuta tambien al interrumpir con Ctrl+C o si ocurre un error.
    limpieza = onCleanup(@() finalizar());

    fprintf('Abriendo %s a %d baudios...\n', puerto, baudios);
    serial = serialport(puerto, baudios, 'Timeout', 0.5);
    configureTerminator(serial, "LF");
    flush(serial, "input");

    fig = figure('Name', 'EKF - CanSat', 'NumberTitle', 'off', ...
                 'Color', 'w', 'CloseRequestFcn', @cerrar);
    setappdata(fig, 'detener', false);
    layout = tiledlayout(fig, 4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    ax1 = nexttile(layout);
    hold(ax1, 'on'); grid(ax1, 'on');
    curvas(1) = animatedline(ax1, 'Color', [0.85 0.15 0.10], ...
                            'DisplayName', 'Roll', 'MaximumNumPoints', 5000);
    curvas(2) = animatedline(ax1, 'Color', [0.10 0.55 0.20], ...
                            'DisplayName', 'Pitch', 'MaximumNumPoints', 5000);
    curvas(3) = animatedline(ax1, 'Color', [0.10 0.30 0.85], ...
                            'DisplayName', 'Yaw relativo', 'MaximumNumPoints', 5000);
    ylabel(ax1, 'Angulo (grados)');
    legend(ax1, 'Location', 'northwest');
    ax2 = nexttile(layout);
    hold(ax2, 'on'); grid(ax2, 'on');
    curvas(4) = animatedline(ax2, 'Color', [0.55 0.15 0.70], ...
                            'MaximumNumPoints', 5000);
    yline(ax2, 0, '--', 'Color', [0.5 0.5 0.5]);
    ylabel(ax2, 'Aceleracion D (m/s^2)');
    ax3 = nexttile(layout);
    hold(ax3, 'on'); grid(ax3, 'on');
    curvas(5) = animatedline(ax3, 'Color', [0.10 0.55 0.65], ...
                            'MaximumNumPoints', 5000);
    ylabel(ax3, 'Altura relativa (m)');
    ax4 = nexttile(layout);
    hold(ax4, 'on'); grid(ax4, 'on');
    curvas(6) = animatedline(ax4, 'Color', [0.85 0.45 0.10], ...
                            'MaximumNumPoints', 5000);
    yline(ax4, 0, '--', 'Color', [0.5 0.5 0.5]);
    ylabel(ax4, 'Velocidad arriba (m/s)');
    xlabel(ax4, 'Tiempo desde primera muestra (s)');
    linkaxes([ax1 ax2 ax3 ax4], 'x');
    titulo = title(layout, 'Esperando filas EKF,...');
    uicontrol(fig, 'Style', 'pushbutton', 'String', 'Detener y guardar', ...
              'Units', 'normalized', 'Position', [0.76 0.94 0.22 0.045], ...
              'Callback', @(~, ~) setappdata(fig, 'detener', true));

    fprintf('Esperando EKF,... (TELEM y mensajes ESP_LOG se ignoran).\n');
    fprintf('Destino: %s.csv / .mat\n', base);
    reloj = tic;
    while isgraphics(fig) && ~getappdata(fig, 'detener') && toc(reloj) < duracion_s
        % Limitar el lote mantiene la ventana y el boton accesibles.
        for k = 1:200
            if serial.NumBytesAvailable == 0
                break;
            end
            linea = strtrim(readline(serial));
            [v, es_ekf] = parsear_fila_ekf(linea);
            if ~es_ekf
                continue;
            end
            if isempty(v)
                descartadas = descartadas + 1;
                continue;
            end
            if isnan(origen_us)
                origen_us = v(1);
            elseif v(1) < ultimo_us || v(2) < ultima_secuencia
                % Tras un reinicio no conectar dos sesiones con una linea.
                segmento = segmento + 1;
                origen_us = v(1);
                desplazamiento_s = muestras(n, 11);
                for j = 1:6
                    addpoints(curvas(j), desplazamiento_s, NaN);
                end
                fprintf('Reinicio detectado: segmento %d. Yaw vuelve a ser relativo.\n', segmento);
            end
            tiempo_s = desplazamiento_s + (v(1) - origen_us) * 1e-6;
            ultimo_us = v(1);
            ultima_secuencia = v(2);
            n = n + 1;
            if n > size(muestras, 1)
                muestras(end+1000, 12) = 0; %#ok<AGROW> Crecimiento por bloques.
            end
            muestras(n, :) = [v tiempo_s segmento];

            valores = [v(3:6) v(8:9)];
            % Conservar datos invalidos en archivo pero mostrar huecos.
            if v(7) == 0
                valores(1:4) = NaN;
            end
            if v(10) == 0
                valores(5:6) = NaN;
            end
            for j = 1:6
                addpoints(curvas(j), tiempo_s, valores(j));
            end
        end
        if n > 0
            limite = max(ventana_s, muestras(n, 11));
            xlim(ax1, [max(0, limite - ventana_s), limite]);
            titulo.String = sprintf('EKF | muestras=%d | seq=%.0f | actitud=%d | vertical=%d | descartadas=%d', ...
                                    n, muestras(n, 2), muestras(n, 7), muestras(n, 10), descartadas);
        end
        drawnow limitrate;
        pause(0.01);
    end
    datos = crear_tabla();
    clear limpieza; % Cerrar y guardar una sola vez, tambien en salida normal.

    function tabla = crear_tabla()
        tabla = array2table(muestras(1:n, :), 'VariableNames', ...
            {'timestamp_us', 'sequence', 'roll_deg', 'pitch_deg', 'yaw_deg', ...
             'linear_ad_m_s2', 'valid', 'height_m', 'vertical_velocity_m_s', ...
             'vertical_valid', 'time_s', 'segment'});
    end

    function finalizar()
        if ~isempty(serial)
            delete(serial);
        end
        if n > 0
            datos_guardados = crear_tabla();
            try
                writetable(datos_guardados, [base '.csv']);
                save([base '.mat'], 'datos_guardados', 'puerto', 'baudios', 'descartadas');
                fprintf('Guardadas %d muestras EKF en:\n%s.csv\n%s.mat\n', n, base, base);
            catch error_guardado
                warning('EKF:Guardado', 'No se pudo guardar la captura: %s', error_guardado.message);
            end
        else
            fprintf('No se recibieron muestras EKF validamente formadas.\n');
        end
        if ~isempty(fig) && isgraphics(fig)
            set(fig, 'CloseRequestFcn', 'closereq');
            if ~isempty(titulo) && isgraphics(titulo)
                titulo.String = sprintf('Captura terminada: %d muestras EKF', n);
            end
        end
    end

    function cerrar(~, ~)
        setappdata(fig, 'detener', true);
        delete(fig);
    end
end
