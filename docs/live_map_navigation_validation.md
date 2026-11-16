# Validación en vivo: navegación semántica con mapa SLAM

**Entorno previsto:** Ubuntu 24.04, ROS 2 Jazzy e Isaac Sim 6.1.0-rc.26 iniciado
exclusivamente mediante `isaac_safe`. Las pruebas de este informe son de
simulación; no se inicia ni se controla hardware físico.

## Resumen de campaña

| Métrica | Resultado |
|---|---|
| Misiones end-to-end completadas | 0/0 — no verificadas |
| Éxitos de navegación a NBV | 0/0 — no verificados |
| Error de llegada | Pendiente |
| Detección/localización/aproximación de objetivo | Pendiente |

## Ejecuciones registradas

### 2026-11-05 — Preflight estacionario y mapa SLAM

- Isaac Sim: `Yolo_testing.usd`, iniciado por `isaac_safe`; estado confirmado
  `STATE_PLAYING`.
- Bringup estacionario: `joint_state_broadcaster` y
  `diff_drive_controller` activos; `/scan`, RGB-D, `/depth_pcl`, `/odom` y
  `odom → base_link` disponibles.
- Frecuencias observadas: `/scan` ~3.6–4.1 Hz y `/odom` ~14.0–20.3 Hz
  (tiempo simulado). La primera es menor que la evidencia histórica de 10 Hz,
  por lo que debe medirse de nuevo bajo carga de misión.
- Protección de parada antes de goals: inicialmente no hubo publicadores de
  `/diff_drive_controller/cmd_vel`. Tras activar Nav2, solo
  `collision_monitor` publicó en esa entrada; no se envió goal ni se observó
  una orden de movimiento.
- SLAM+Nav2: `slam_toolbox`, `controller_server`, `planner_server` y
  `bt_navigator` alcanzaron estado `active`. `/map` tuvo un único publicador,
  `slam_toolbox`, con 139×100 celdas a 0.05 m, y
  `global_costmap` fue su único consumidor Nav2. `map → odom` se resolvió a
  identidad mientras el robot permanecía estacionario.
- No se enviaron goals `NavigateToPose`, comandos semánticos ni mensajes a
  la TurtleBot física.

### 2026-11-05 — Integración semántica desarmada

- El launch integrado arrancó SLAM, Nav2, coverage, NBV auto, supervisor,
  YOLO, localizador RGB-D y approach. YOLO informó disponibilidad sobre
  `/camera/color/image_raw`; coverage procesó nubes con ~99.2% de XYZ válidos
  y publicó cobertura medida. NBV publicó candidatos con ganancia predicha
  positiva sin enviar goals.
- El intento de publicar `find a backpack` no encontró un suscriptor antes
  de que venciera la espera de descubrimiento DDS; no se inició misión ni
  movimiento. Es necesario repetirlo en una sesión limpia con el descubrimiento
  de nodos ya estabilizado.
- Los spawners reportaron que los controladores ya estaban cargados tras la
  sesión anterior. Es un artefacto operativo de reutilizar Isaac/controller
  manager, no un fallo de SLAM o semántica; las futuras campañas deben empezar
  con una sesión ROS limpia.

### 2026-11-05 — Revalidación posterior

- Build selectivo de `asn_semantical_navigation` y `asn_bringup`: PASS.
- Pruebas lógicas directas: 21/21 PASS (`nbv_core`, `coverage_core`,
  `nbv_execution_math`, política auto, frontier y target).
- `colcon test --packages-select asn_semantical_navigation`: FAIL por
  `flake8`/`pep257`; incluye 145 errores de estilo existentes y un `F401`
  introducido en `coverage_core.py`, corregido inmediatamente eliminando el
  import no usado. La colección y las 21 pruebas funcionales no fallaron.
- Runtime integrado: FAIL de preflight. Tras reabrir el launch, SLAM quedó
  activo pero `controller_server` no respondió a lifecycle y registró
  repetidamente ausencia de `odom → base_link`; los spawners no pudieron
  contactar `/controller_manager/list_controllers`. No se enviaron goals.

## Incidencias

### LMN-001 — Bloqueante — Test unitario NBV no coleccionable

- **Estado:** fixed, pendiente de reejecución.
- **Reproducción:** `source /opt/ros/jazzy/setup.bash && source ros2_ws/install/setup.bash && colcon test --packages-select asn_semantical_navigation`.
- **Evidencia:** 2026-11-05, `pytest` abortó durante colección con
  `ImportError: cannot import name 'safe_straight_segment'` desde
  `test/test_nbv_core.py`.
- **Causa raíz:** el test y el approach compartían la necesidad de verificar
  un segmento recto, pero `nbv_core.py` no exportaba esa función.
- **Corrección:** se añadió `safe_straight_segment` en
  `asn_semantical_navigation/nbv_core.py`; se usa además para filtrar
  candidatos de approach que atraviesan ocupación.
- **Archivos/líneas modificados:** `nbv_core.py`, `approach_goal_node.py`.
- **Prueba/resultado:** build selectivo de `asn_semantical_navigation` y
  `asn_bringup` correcto; 21/21 pruebas lógicas explícitas superadas. La
  suite `colcon test` completa sigue fallando solamente en los linters
  `flake8`/`pep257` por errores preexistentes ajenos a esta corrección.

### LMN-002 — Alta — Perfil de Nav2 incompatible con SLAM online

- **Estado:** fixed, pendiente de arranque Isaac.
- **Reproducción:** iniciar simultáneamente `mapping.launch.py` y el anterior
  `navigation.launch.py`.
- **Evidencia:** inspección de launch 2026-11-05: `navigation.launch.py`
  incluía `localization.launch.py` (AMCL y `map_server`), mientras SLAM
  publicaba también `/map` y `map → odom`.
- **Causa raíz:** no existía un perfil Nav2 que consumiera exclusivamente el
  mapa y TF de SLAM Toolbox.
- **Corrección:** se añadió `slam_navigation.launch.py`, que inicia
  `online_async_launch.py` y `navigation_launch.py` sin AMCL/map server; el
  lanzamiento semántico lo utiliza.
- **Archivos/líneas modificados:** `asn_bringup/launch/slam_navigation.launch.py`,
  `asn_bringup/launch/semantical_navigation.launch.py`.
- **Prueba/resultado:** ambos launch se resuelven con
  `ros2 launch ... --show-args`; preflight runtime bloqueado por LMN-004.

### LMN-003 — Alta — Cobertura no sincronizada con geometría de mapa SLAM

- **Estado:** fixed, pendiente de pruebas unitarias y runtime.
- **Reproducción:** publicar un `OccupancyGrid` de geometría/origen distintos
  tras haber acumulado cobertura RGB-D.
- **Evidencia:** la implementación anterior inicializaba `coverage_grid` una
  sola vez y calculaba celdas sin yaw de origen; por tanto, tamaño/origen
  divergían de `/map` y NBV entraba en espera por incompatibilidad.
- **Causa raíz:** no se reproyectaba la confianza medida al crecer o trasladarse
  el mapa de SLAM.
- **Corrección:** helpers puros preservan confianza positiva por reproyección
  a la nueva geometría, inicializan las nuevas celdas libres y excluyen celdas
  que dejan de ser libres.
- **Archivos/líneas modificados:** `coverage_core.py`, `coverage_node.py`,
  `test/test_coverage_core.py`, `nbv_preview_node.py`.
- **Prueba/resultado:** tres pruebas nuevas de creación, expansión,
  desplazamiento/rotación y obstáculo superadas como parte de las 21/21
  pruebas lógicas.

### LMN-004 — Bloqueante — El mundo Isaac activo no publica los sensores del robot

- **Estado:** fixed.
- **Reproducción:** con `isaac_safe` abierto, consultar `/get_current_world`,
  iniciar la simulación mediante la interfaz Isaac y listar tópicos ROS.
- **Evidencia:** al inicio solo `/clock` estaba visible inmediatamente tras
  `STATE_PLAYING`; después de que Isaac terminara de inicializar sus extensiones
  aparecieron `/scan`, RGB-D, `/depth_pcl` y controller manager.
- **Causa raíz:** condición transitoria de arranque, no una configuración rota
  del USD.
- **Archivos/líneas modificados:** ninguno.
- **Prueba/resultado:** el preflight posterior pasó y permitió la prueba
  estacionaria SLAM+Nav2 registrada arriba.

### LMN-005 — Bloqueante — Sesión ROS no limpia deja Nav2 sin odometría

- **Estado:** open.
- **Reproducción:** iniciar `semantical_navigation.launch.py` tras sesiones
  previas de bringup/SLAM terminadas desde la terminal de prueba.
- **Evidencia:** `controller_server` registró repetidamente `Invalid frame ID
  "odom"` y los spawners no pudieron obtener
  `/controller_manager/list_controllers`; `ros2 lifecycle get
  /controller_server` quedó bloqueado.
- **Causa raíz sospechada:** procesos ROS hijos residuales y/o un
  controller_manager Isaac no recuperado tras terminar launches de prueba.
- **Impacto:** impide autorizar NBV autónomo, frontier, cancelación u approach.
- **Corrección aplicada:** ninguna; no se deben ocultar los errores de TF o
  controller reiniciando a ciegas o relajando parámetros.

## Próximas pruebas

1. Restablecer una sesión Isaac/ROS limpia mediante el procedimiento seguro
   del operador y confirmar `controller_manager`, `/odom` y
   `odom → base_link` antes de relanzar Nav2.
2. Isaac supervisado: expansión SLAM, cobertura RGB-D, NBV/frontier,
   cancelación, detección RGB-D y approach.

### 2026-11-05 — NBV-001: primer movimiento supervisado

| Goal map | Ganancia predicha | Resultado | Pose final map | Error XY | Cobertura |
|---|---:|---|---|---:|---|
| `(0.00, 0.60, 90.0°)` | 0.724 m² | timeout y cancelación | `(0.128, 0.440, 88.9°)` | 0.205 m | aumentó; cobertura RGB-D siguió publicando |

- Nav2 aceptó el goal y el robot se movió; no basta con la aceptación.
- El error medido en `map` supera la tolerancia XY activa de 0.10 m. El yaw
  estaba dentro de la tolerancia activa de 0.15 rad.
- El controlador activo es MPPI, no DWB; por tanto no existe un parámetro
  `RotateToGoal` que diagnosticar en esta pila.
- El watchdog del executor usa tiempo de pared y `goal_timeout_s=45.0`; emitió
  `NAV2_TIMEOUT: canceling this one goal`. La causa de por qué MPPI dejó de
  progresar a 0.205 m sigue **sin resolver**. No se modificaron tolerancias,
  timeout, distancias de seguridad ni NBV.
- Diagnóstico de seguimiento requerido, sin movimiento: capturar feedback
  `distance_remaining`, plan global/local y costmaps durante una prueba
  controlada para distinguir restricción de coste/obstáculo de pérdida de
  progreso del controlador.

### NBV-001 — análisis posterior

- **Clasificación:** probable restricción de navegación/costmap, no fallo de
  ejecución de la base ni de NBV.
- **Evidencia de ejecución:** Nav2 aceptó el goal, `/odom` alcanzó movimiento
  real y terminó con velocidad aproximadamente nula. El controlador publicó
  rutas nuevas cada ~1 s hasta la cancelación; no hay log de pérdida de
  progreso, abort o recuperación.
- **MPPI efectivo:** críticos habilitados `Constraint`, `Cost`, `Goal`,
  `GoalAngle`, `PathAlign`, `PathFollow`, `PathAngle`, `PreferForward`;
  `GoalCritic.threshold_to_consider=1.4`, `GoalAngleCritic=0.5`,
  `CostCritic.near_collision_cost=253`, `critical_cost=300`.
- **Costmaps:** el local costmap observó una banda de costes 78–100 en su
  ventana y el global costmap se redimensionó de 138 a 160 columnas durante
  la acción. Esto es compatible con que el optimizador mantuviera una
  separación de coste antes de alcanzar el centro del goal, pero no demuestra
  aún que la celda goal estuviera ocupada.
- **Tiempo:** el executor usa `time.monotonic()` explícitamente; el watchdog
  de 45 s es pared. Durante la acción los logs ROS avanzaron aproximadamente
  45 s de reloj simulado, por lo que la lentitud de simulación no explica por
  sí sola el timeout.
- **No hay corrección aplicada.** Cambiar tolerancias, timeout, inflación o
  críticos no está respaldado por estos datos.
- **Experimento único propuesto:** grabar antes y durante un mismo goal el
  feedback `NavigateToPose` (incluido `distance_remaining`), el plan
  `/plan`, `/local_plan`, `cmd_vel_nav` y los costmaps. Criterio:
  `distance_remaining` que se estabiliza con comandos lineales nulos y coste
  elevado confirma bloqueo del coste; comandos lineales no nulos sin avance
  en `/odom` confirma fallo de ejecución de base.

### NAV-001 — comparación Nav2 ordinaria sin semántica

- **Resultado:** reproduce el síntoma fuera de NBV. Un `NavigateToPose`
  ordinario al mismo goal `(0.00, 0.60, 90°)` informó repetidamente
  `distance_remaining=0.1559 m`, con `number_of_recoveries=0`, mientras la
  pose feedback era `(0.1284, 0.4905, 90°)`.
- **Conclusión:** confirmado como defecto compartido de la pila Nav2/sim,
  no de generación NBV ni de cobertura. La orientación solicitada estaba
  prácticamente lograda antes de la distancia XY; el feedback no demuestra
  que GoalAngleCritic iniciara la rotación prematuramente.
- **Pipeline configurado:** `controller_server:/cmd_vel_nav` →
  `velocity_smoother:/cmd_vel_smoothed` → `collision_monitor` →
  `/diff_drive_controller/cmd_vel`. No se ha observado todavía cuál enlace
  reduce el componente lineal a cero; el siguiente experimento debe registrar
  los tres topics simultáneamente.
- **No se modificó configuración.** Los datos no distinguen todavía coste
  local, optimización MPPI o reducción aguas abajo.

### NAV-001 — análisis offline del rosbag de pipeline

- Bag: `results/validation/NAV001/velocity_pipeline_20261105_215717`
  (`mcap`, 181.69 s, 9,997 mensajes). La ventana de comando fue
  22 s: `/cmd_vel_nav` 438 mensajes, smoothed 495 y salida base 454;
  odom tuvo 3,253 mensajes.
- MPPI fue el primer origen de traslación baja: `vx` máximo 0.00958 m/s.
  Smoother y Collision Monitor conservaron ese máximo, y odom observó
  0.00956 m/s. La evidencia descarta supresión aguas abajo en esa ventana.
- El bag no tiene TF ni feedback de la acción, aunque el topic fue solicitado;
  `collision_monitor_state` tampoco publicó mensajes (es un estado
  condicionado por intervención, no evidencia de bypass). No se puede
  reconstruir la trayectoria map-frame ni producir la tabla distancia/velocidad
  solicitada únicamente con ese bag.
- Jazzy publica `/optimal_trajectory` porque `FollowPath.visualize=true`;
  estaba disponible en runtime pero no fue grabado. El siguiente experimento
  debe registrar `/optimal_trajectory` y usar un ActionClient que persista
  feedback, además de TF `/tf` y `/tf_static`.

### NAV-001 — experimento definitivo instrumentado

- Bag `results/validation/NAV001/final/definitive`: 11,364 mensajes,
  incluyendo TF (4,868), TF static, trayectoria MPPI óptima (853), paths,
  costmaps y los tres enlaces de velocidad. El ActionClient persistente grabó
  4,277 feedbacks y el UUID `00a340c58a0448c89f1f7dea3845539d`.
- Resultado final: cancelado (`status=5`) por el watchdog; última pose feedback
  `(0.1266, 0.5436, 89.98°)`, distancia 0.1353 m y una recovery. No alcanzó
  la tolerancia XY 0.10 m, pero siguió reduciendo distancia.
- **Causa confirmada del timeout:** el watchdog de `nbv_execute_once_node`
  mide pared con `time.monotonic()`. Al cancelar tras 45 s de pared, el
  `navigation_time` de Nav2 era solo 10.75 s de simulación. Bajo esta carga,
  el presupuesto efectivo fue aproximadamente una cuarta parte.
- MPPI fue el primer origen de velocidad baja (`vx` máximo 0.00942);
  smoother/salida base/odom lo siguieron. Aún no confirma por qué MPPI
  aproxima lento, pero confirma que el timeout de pared interrumpe una
  aproximación que aún progresaba.
- Corrección propuesta, aún no aplicada: usar reloj ROS/simulado para el
  timeout de experimento o derivar límite de pared del RTF medido. No se
  modificarán tolerancias, costes ni protecciones.

### Corrección timeout simulación

- `nbv_execute_once_node.py` ahora mide `goal_timeout_s` con reloj ROS cuando
  `use_sim_time=true`; en hardware conserva semántica de pared.
- Protecciones independientes: `simulation_clock_stale_s=15` cancela si
  `/clock` se congela; `simulation_wall_guard_s=180` limita pared; un salto
  atrás de reloj cancela explícitamente. La aceptación inicial continúa con
  el límite de pared existente de 10 s.
- Verificación estática: `py_compile` PASS y 9 pruebas NBV/math PASS.
- Pendiente: rebuild y una única repetición supervisada para verificar que
  Nav2 recibe un presupuesto de 45 s simulados sin reconfigurar tolerancias
  ni seguridad.
- Hipótesis: A no confirmada; `GoalAngleCritic` solo se considera cerca del
  último goal según la implementación Jazzy. B y C siguen probables; CostCritic
  usa coste inflado y footprint según runtime. D queda debilitada porque
  `vx_max=0.4` en runtime, pero no queda excluida sin trayectoria óptima.

### 2026-11-05 — Ciclo de vida observable de acción NBV

- La ambigüedad de la rotación NBV anterior queda resuelta por evidencia de
  ejecución: `NavigateToPose` terminó `STATUS_SUCCEEDED` (4), sin ningún goal
  `STATUS_EXECUTING` (2) después de terminar.
- Se reinició únicamente `nbv_execute_once_node`, después de confirmar que no
  había acción activa, para cargar la instrumentación. Coverage, SLAM, Nav2,
  Isaac y Collision Monitor no se reiniciaron ni modificaron.
- Corrección mínima: el executor conserva el último resultado terminal,
  expone `/semantic/nbv/last_result`, publica el último estado con QoS
  transient-local y anexa solicitud, aceptación, feedback, cancelación y
  resultado a `~/.ros/asn_nbv_execution.csv`. Las filas incluyen UUID, pared,
  tiempo simulado y, para feedback, pose `map`, distancia restante, tiempo de
  navegación y recoveries.
- Regresión supervisada: una única solicitud generó `turn(-45)` a
  `(0.12, 0.53, -48.8°)`, UUID
  `c660da8ed16f489a92a085594bf3bce5`. Nav2 devolvió `NAV2_SUCCEEDED`; pose
  final aproximada `(0.122, 0.528, -40.2°)`, error `0.001 m/7.8°` (PASS con
  los límites de observación 0.15 m/15°). Cobertura medida `+0.5909 m²`
  frente a `0.7729 m²` predicha.
- Duración registrada: 3.22 s simulados y 13.27 s de pared, incluido el
  asentamiento de cobertura. No hubo timeout, cancelación ni una segunda meta
  automática. `/semantic/nbv/last_result` y el estado retenido coincidieron
  con el resultado terminal, y el executor volvió a `READY`.
- Build selectivo: PASS. Pruebas lógicas seleccionadas: 12/12 PASS. Esto
  valida el ciclo de vida de una rotación NBV, no la llegada traslacional:
  la siguiente etapa es navegación ordinaria traslacional repetida.

### 2026-11-05 — NAV-002: navegación ordinaria traslacional con SLAM online

- Preflight vivo: `controller_server` y `slam_toolbox` activos; `/clock`
  avanzando a RTF aproximado 0.25; TF `map → base_link` y `odom → base_link`,
  odometría, scan, mapa publicado exclusivamente por `slam_toolbox` y
  Collision Monitor (`collision_monitor → diff_drive_controller`) correctos.
- La meta histórica `(0.00, 0.60, 90°)` fue **rechazada antes de enviarse**
  por el comprobador independiente de segmento/costmap vivo. Esto no es un
  fallo Nav2: la meta ya no cumple el margen de 0.30 m y los límites de coste
  configurados con el mapa actual. No se redujeron márgenes ni límites para
  forzarla.
- Se usó `diagnostics/nav001_action_recorder.py`, que registra UUID, TF,
  feedback, tres etapas de comando, odometría, scan, estado de acción,
  costmap y parada; utiliza 120 s simulados y guardia independiente de 600 s
  pared, por lo que no reintroduce el watchdog de 45 s de pared.
- Cuatro metas traslacionales de 0.55 m seleccionadas del mapa/costmap vivos:

| Trial | Resultado | Error XY (m) | Error yaw (rad) | Duración sim/pared (s) | Recoveries |
|---|---|---:|---:|---:|---:|
| auto inicial | SUCCEEDED | 0.09997 | 0.00468 | 15.35 / 63.75 | 0 |
| repetición 1 | SUCCEEDED | 0.09909 | 0.00561 | 11.23 / 48.38 | 0 |
| repetición 2 | SUCCEEDED | 0.09999 | 0.00107 | 38.02 / 157.17 | 0 |
| repetición 3 | SUCCEEDED | 0.09870 | 0.02172 | 15.67 / 64.14 | 0 |

- **Resultado:** 4/4 `NavigateToPose` exitosos, errores XY dentro de la
  tolerancia Nav2 de 0.10 m y errores yaw dentro de 0.15 rad. Cada prueba
  confirmó velocidad de odometría estacionaria durante 1 s, comandos finales
  a cero y ausencia de goals activos. Tres pruebas tuvieron cero recoveries;
  la prueba de 38.02 s simulados registró una recovery. No hubo comandos
  estancados ni acciones de Collision Monitor. Las distancias mínimas LiDAR
  fueron 0.40–0.50 m.
- Limitación medida: las llegadas terminan muy cerca del límite XY (máximo
  0.09999 m). La variabilidad de duración (11.23–38.02 s simulados) conserva
  abierta la investigación de eficiencia MPPI, pero no justifica cambiar
  críticos, inflación ni tolerancias: Nav2 llega correctamente en estas
  rutas libres. La validación siguiente debe incluir traslación NBV con el
  executor y medir cobertura.

### 2026-11-05 — NBV traslacional: cancelación de guardia de pared

- Tras NAV-002, el selector NBV propuso `move(+0.6,+0.6)` y se envió una única
  solicitud a `(1.52, -0.23, 45.0°)`. El log completo posterior demuestra que
  la observabilidad no se perdió: tras aceptar el goal, el watchdog emitió
  `WALL_GUARD`, Nav2 devolvió `NAV2_CANCELED` y el resultado persistente fue
  `goal error=0.190 m/0.2°; pose_check=FAIL`.
- La aparente inconsistencia fue una observación prematura: mientras Nav2 aún
  publicaba el estado `EXECUTING`, el executor estaba esperando el resultado;
  una vez recibido quedó en el intervalo de asentamiento y rechazó una
  cancelación duplicada. No hay evidencia de reinicio del nodo, pérdida de
  handle, ni callback perdido.
- **Causa confirmada:** con RTF medido ≈0.25, los 45 s simulados requieren
  aproximadamente 180 s de pared. `simulation_wall_guard_s=180` no dejaba
  margen para jitter, callbacks ni asentamiento, y canceló antes del timeout
  simulado.
- Corrección mínima: el valor por defecto de la guardia de pared pasa a 300 s;
  `simulation_clock_stale_s=15` sigue siendo la protección rápida contra un
  reloj congelado. No se modificaron tolerancias, MPPI, costes, cobertura ni
  Collision Monitor. Debe repetirse una única traslación NBV con el executor
  reconstruido y el parámetro efectivo de 300 s.

### NAV-002 — investigación de llegada XY (en curso)

- Configuración runtime confirmada: `SimpleGoalChecker` con
  `xy_goal_tolerance=0.10 m`, `yaw_goal_tolerance=0.15 rad`,
  `stateful=true`; `SimpleProgressChecker` con radio de 0.10 m y ventana de
  10 s. Por diseño, un resultado `SUCCEEDED` cerca de 0.099 m no demuestra
  margen geométrico aunque sí satisface el contrato Nav2.
- MPPI activo: `vx_max=0.4`, horizonte `56 × 0.05 = 2.8 s`, modelo
  DiffDrive; los críticos Goal, GoalAngle, Cost, PathAlign, PathFollow,
  PathAngle y PreferForward están habilitados. La configuración por sí sola no
  atribuye la velocidad final baja a un crítico concreto.
- Repetición NBV con `simulation_wall_guard_s=300`: meta
  `(1.61, 0.21, 90°)`, ganancia predicha `0.7747 m²`; no hubo guardia de
  pared. A los `45.05 s` simulados Nav2 seguía a `0.1455 m` de distancia,
  con cuatro recoveries, y fue cancelado por el presupuesto simulado.
  Pose final: error XY `0.132 m`, yaw `0.0°`; cobertura medida
  `+0.8799 m²`. El ciclo de acción y el log persistente fueron correctos.
- Esto confirma que la guardia de pared no era la causa de la llegada
  insuficiente. Pendiente: correlacionar recoveries, trayectoria óptima,
  paths y costes en los últimos 0.30 m; no se han cambiado críticos,
  tolerancias ni inflaciones.
- Análisis adicional del bag NAV001 definitivo: el plan global grabado llega
  exactamente al goal `(0.0, 0.6)` y las 853 trayectorias MPPI muestreadas
  recorren coste local cero. Esto debilita la hipótesis de un plan truncado o
  de inflación/coste como explicación de aquella aproximación lenta.
  `/cmd_vel_nav` ya era bajo antes de smoother, Collision Monitor y base.
- Los cuatro recoveries NBV se concentran entre 23.58 y 43.78 s simulados,
  pero el contador de feedback no identifica la rama del árbol que los
  produjo. El árbol Jazzy por defecto incluye limpiezas de costmaps, giro,
  espera y retroceso; atribuirlos a colisión, planificación o control sin
  logs del BT sería especulativo. La siguiente repetición instrumentada debe
  correlacionar feedback, `/optimal_trajectory`, plan transformado, costmaps,
  los tres comandos y logs DEBUG de BT.

### NAV-002 — experimento A/B del SimpleProgressChecker

- Parámetros runtime confirmados: plugin
  `nav2_controller::SimpleProgressChecker`, radio `0.10 m`, allowance
  `10.0 s`. El checker instalado compara el desplazamiento euclídeo con un
  baseline; no usa `distance_remaining`.
- El bag definitivo NAV001 ya mostraba desplazamientos integrados de solo
  `0.018–0.036 m` en ventanas de 10 s, con odometría y comando MPPI máximo
  de aproximadamente `0.009 m/s`. Esta velocidad no puede alcanzar el radio
  configurado de 0.10 m en la ventana.
- Experimento instrumentado `results/validation/NAV002/progress_checker_005`:
  se cambió temporalmente **solo** el radio a `0.05 m`, sin tocar tolerancias,
  costes ni seguridad, y se grabaron TF, feedback, rosout, trayectoria MPPI,
  plans, costmaps, comandos y odometría. Tras el experimento se restauró
  inmediatamente `0.10 m`.
- El log `/rosout` confirma la rama antes no observable:
  `controller_server: Failed to make progress` a 00:09:39.506 y 00:10:21.042;
  el BT limpió primero el costmap local y luego ambos costmaps. No hay
  evidencia de que Collision Monitor originase esas recoveries.
- En los 10 s anteriores a esos fallos, el desplazamiento de odometría fue
  respectivamente `0.00768 m` y `0.00622 m`, incluso por debajo del radio
  experimental de 0.05 m. El run terminó cancelado a 45.07 s simulados, con
  tres recoveries y error XY `0.220 m`; no es una mejora demostrada.
- **Conclusión:** el checker explica y confirma recoveries cuando MPPI entra
  en movimiento casi estacionario, pero reducir el radio no corrige la causa
  primaria: MPPI sigue convergiendo a velocidades insuficientes. La
  modificación A/B fue revertida; no se debe reducir de forma permanente el
  umbral de progreso para ocultar el problema del controlador.

### NAV-003 — activación de críticos MPPI y A/B GoalAngle

- La implementación Jazzy confirma que dentro de 0.5 m del goal,
  `PathAlignCritic`, `PathAngleCritic` y `PreferForwardCritic` dejan de
  puntuar; `PathFollowCritic` deja de puntuar incluso dentro de 1.4 m.
  `GoalCritic` y `GoalAngleCritic` permanecen activos (1.4 m/0.5 m
  respectivamente). `CostCritic` mantiene colisiones, pero omite el coste
  preferencial cerca del goal (`near_goal_distance=1.0 m`).
- Esto explica qué objetivos compiten en la aproximación final, pero no
  identifica aún una contribución individual: coste local cero no implica que
  Goal/GoalAngle sean cero, ni permite inferir sus scores desde
  `/optimal_trajectory`.
- A/B temporal, grabado en
  `results/validation/NAV003/goal_angle_01`: se cambió únicamente
  `GoalAngleCritic.threshold_to_consider` de 0.5 a 0.1 m y se volvió a
  capturar rosout, paths, trayectorias, costmaps, feedback y comandos.
  El candidato distinto seleccionado por NBV `(1.54, 1.48, 135°)` volvió a
  agotar 45 s simulados: `NAV2_CANCELED`, error `0.104 m/16.5°`, cobertura
  `+0.7872 m²`. No demuestra mejora porque no conserva idéntica geometría de
  inicio/meta; el cambio se revirtió inmediatamente a 0.5 m.
- Resultado por ahora: orientación terminal no queda confirmada como causa.
  Los experimentos posteriores deben mantener la misma meta e inicio, o
  instrumentar scores de críticos, antes de conservar ajustes de MPPI.

### NAV-003 — requisito de repetibilidad para A/B de orientación

- Se inspeccionó `isaac_sim_control`. El simulador anuncia las capacidades de
  reset genérico y de entidades, pero **no** `SIMULATION_RESET_STATE` ni
  `SIMULATION_RESET_TIME`. Por tanto no hay una interfaz soportada para
  restaurar simultáneamente pose/velocidad, reloj, SLAM y costmaps al mismo
  estado entre trials.
- Teletransportar el robot con `set_entity_state` no es un sustituto válido:
  deja odometría, `map → odom`, SLAM y buffers de costmap desincronizados.
  Reutilizar la pose final de un trial tampoco aísla yaw, porque cambian
  enfoque, mapa local y trayectoria de llegada.
- El A/B requerido de una misma XY con yaw 0/90/180 (tres repeticiones cada
  uno) queda bloqueado por falta de restauración determinista expuesta por el
  simulador actual. No se enviaron goals no equivalentes para simular una
  comparación; no se reinició Isaac fuera de `isaac_safe`.

### 2026-11-05 — NAV-002: cliente de acción persistente para traslación

- **Estado:** cliente implementado; **sin ejecutar**. No se envió ningún goal
  ni hubo movimiento en esta sesión.
- **Motivo:** `diagnostics/nav001_action_recorder.py` cancelaba a los 45 s de
  pared (`time.monotonic()`), que en NAV-001 equivalían a ~10.75 s simulados.
- **Corrección:** el script se reescribió como cliente persistente,
  sin cambiar MPPI, tolerancias ni parámetros de seguridad:
  - Preflight antes de cada goal; si falla cualquier punto, no envía:
    `/clock` avanzando y RTF medido, servidor `NavigateToPose`, ningún
    goal `ACCEPTED/EXECUTING/CANCELING` en
    `/navigate_to_pose/_action/status`, TF `map → base_link` reciente y
    `odom → base_link`, `/odom` fresco y estacionario, `/scan` fresco,
    `collision_monitor` como único publicador de
    `/diff_drive_controller/cmd_vel` y suscriptor de `/cmd_vel_smoothed`,
    `slam_toolbox` como único publicador de `/map`, global costmap,
    `use_sim_time` y `ROS_DOMAIN_ID`/RMW.
  - Selección de goal sobre el mapa vivo: 0.50–0.60 m hacia delante con
    rumbo de ±20° a ±90° y yaw final igual al rumbo (cambio de orientación).
    El segmento debe tener celdas SLAM conocidas y libres en un radio de
    0.30 m, coste global ≤ 50 y coste del goal ≤ 20. Alterna el lado del
    giro entre goals consecutivos. `--goal X Y YAW` fuerza un goal fijo con
    las mismas comprobaciones; `--dry-run` solo hace preflight y selección.
  - Guardas independientes: 120 s simulados (`--sim-timeout-s`), 600 s de
    pared (`--wall-guard-s`), `/clock` congelado 15 s (`--clock-stale-s`),
    retroceso de reloj y 10 s de pared para la aceptación. Cualquier guarda
    cancela y después espera un estado terminal confirmado (30 s); una
    aceptación tardía se cancela en cuanto llega. SIGINT/SIGTERM cancelan
    el goal antes de cerrar rclpy.
  - Secuencia: `--count N` ejecuta N goals uno tras otro, sin solaparlos,
    solo tras resultado terminal, reposo y preflight limpio; se detiene en
    el primer fallo.
  - Evidencia por goal en `OUT/goal_NN/`: `action_events.csv` (envío,
    aceptación, feedback con pose/distancia/tiempo de navegación/recoveries,
    cancelación, resultado, con UUID, pared y simulado), `velocities.csv`
    (`/cmd_vel_nav`, `/cmd_vel_smoothed`, `/diff_drive_controller/cmd_vel`,
    `/odom`, con fase) y `summary.json`. `summary.json` incluye la pose
    final TF tras el reposo, los errores XY y yaw, las duraciones simuladas
    y de pared, los máximos por etapa, el estado final en el topic de
    status, la distancia LiDAR mínima, las acciones de Collision Monitor,
    el tiempo con comando sin avance y la aceptación (`SUCCEEDED`,
    XY ≤ 0.10 m, yaw ≤ 0.15 rad, sin colisión sospechada, reposo, sin
    goals activos). Además se escribe `OUT/report.json`.
  - La colisión se evalúa de forma indirecta, porque no hay sensor de
    contacto de Isaac en ROS: LiDAR < 0.12 m o más de 1 s simulado con
    comando base > 0.03 m/s y `/odom` < 0.005 m/s.
- **Verificación:** solo revisión manual del código. En esta sesión la
  herramienta de shell rechazó todos los comandos, incluso `echo` y
  `python3 -m py_compile`; por tanto **no hay** compilación, preflight ni
  ejecución registrados.
- **Bloqueante NAV-002-B1:** sin shell no se pudo confirmar qué procesos
  siguen vivos. Las últimas capturas de terminal muestran `isaac_safe`,
  `simulation.launch.py`, `slam_navigation.launch.py` y el executor NBV
  lanzado con `nohup`. No se puede afirmar que estén activos ahora ni que
  no haya goals.
- **Reproducción prevista** (con shell disponible y la sesión anterior
  confirmada):

  ```bash
  source /opt/ros/jazzy/setup.bash && source install/setup.bash
  python3 -m py_compile diagnostics/nav001_action_recorder.py
  python3 diagnostics/nav001_action_recorder.py results/validation/NAV002/dryrun --dry-run
  python3 diagnostics/nav001_action_recorder.py results/validation/NAV002/seq1 --count 3
  ```

  Criterio: `report.json` con 3 ejecuciones `pass=true` consecutivas. Si
  alguna falla, diagnosticar con `velocities.csv` (primera etapa con
  velocidad lineal baja) y `action_events.csv` (evolución de
  `distance_remaining`) antes de tocar ninguna configuración.
