# SONIC BONGO — Dataset Schema v1

Firmware target: v17 DATASET MATRIX

## Objetivo

Separar los datos continuos de sensores, los eventos derivados y la configuración activa para poder:

- reproducir y comparar sesiones de calibración;
- construir ventanas temporales para aprendizaje;
- entrenar modelos sin confundir las decisiones del detector actual con etiquetas reales;
- conservar exactamente qué configuración produjo cada muestra.

## Matriz SAMPLES

Una fila por cada actualización válida del punto fusionado.

Columnas exportadas:

1. `session_id`
2. `t_ms`
3. `point_seq`
4. `config_revision`
5. `s1_raw`
6. `s1_fast`
7. `s1_stable`
8. `s1_velocity`
9. `s2_raw`
10. `s2_fast`
11. `s2_stable`
12. `s2_velocity`
13. `y_candidate`
14. `y_point`
15. `y_stable`
16. `velocity`
17. `acceleration`
18. `adaptive_baseline`
19. `displacement`
20. `coherence`
21. `jitter`
22. `candidate_score`
23. `age_s1_ms`
24. `age_s2_ms`
25. `source_mask`
26. `phase_code`
27. `flags`
28. `human_label`

### source_mask

- `1`: S1
- `2`: S2
- `3`: S1 + S2

### phase_code

- `0`: espera/desconocido
- `1`: descenso
- `2`: subiendo
- `3`: rebote
- `4`: candidato
- `5`: roll

### flags

Bitmask:

- bit 0: S1 fresh
- bit 1: S2 fresh
- bit 2: ambos frescos
- bit 3: descenso activo
- bit 4: roll activo
- bit 5: evento emitido en esta muestra
- bit 6: baseline adaptativo congelado

### human_label

Reservado para supervisión futura. No se completa automáticamente.

- `-1`: sin etiqueta
- `0`: rechazar/no golpe
- `1`: golpe confirmado
- `2`: roll confirmado

## Matriz EVENTS

Una fila por cada candidato automático confirmado por la función actual.

Columnas:

- `session_id`
- `event_seq`
- `t_ms`
- `config_revision`
- `point_seq`
- `descent_duration_ms`
- `y_at_event`
- `adaptive_baseline`
- `displacement`
- `amplitude`
- `peak_down_speed`
- `velocity_at_event`
- `acceleration_at_event`
- `period_ms`
- `roll_hz`
- `energy`
- `coherence`
- `jitter`
- `source_mask`
- `roll_count`
- `midi`
- `auto_class`
- `human_label`

`auto_class` y `human_label` son conceptos distintos. El primero describe lo que decidió el algoritmo actual; el segundo queda reservado para verdad supervisada.

## Matriz CONFIGS

Cada cambio de configuración genera una nueva `config_revision`.

Columnas:

- `session_id`
- `revision`
- `t_ms`
- `baseline_cm`
- `max_range_cm`
- `hit_plane_cm`
- `cone_half_angle_deg`
- `target_hz_per_sensor`
- `smoothing_pct`
- `intensity_ref_cm_s`
- `func_min_down_speed`
- `func_min_amplitude`
- `func_reversal_speed`
- `func_min_interval_ms`
- `func_roll_max_gap_ms`

Cada fila de SAMPLES y EVENTS incluye `config_revision` para poder reconstruir el contexto exacto.

## Baseline adaptativo experimental

v17 calcula `adaptive_baseline` sólo como feature de investigación. No modifica todavía el detector musical.

Se actualiza lentamente cuando la mano está relativamente estable y se congela durante movimiento rápido/descenso. La diferencia:

`displacement = y_point - adaptive_baseline`

queda registrada para evaluar después si mejora la generalización respecto de usar altura absoluta.

## Forma prevista para ML

El CSV de SAMPLES es una tabla de almacenamiento. Para entrenamiento se seleccionará un subconjunto de features y se convertirá a ventanas temporales:

`X = [batch, time_steps, features]`

Ejemplos de targets futuros, todavía no fijados:

- golpe / no golpe;
- instante de impacto dentro de una ventana;
- intensidad;
- roll / no roll;
- frecuencia del roll;
- predicción de la siguiente fase de Y(t).

No entrenar usando `auto_class` como ground truth sin etiquetado o validación independiente.

## Endpoints v17

- `/api/data/meta`
- `/api/data/samples?n=...`
- `/api/data/events?n=...`
- `/api/data/configs?n=...`
- `POST /api/data/clear`

Las exportaciones de samples/events/configs son CSV.
