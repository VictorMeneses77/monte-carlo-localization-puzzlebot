# Monte Carlo Localization — Puzzlebot (TE3003B)

Localización autónoma de un robot diferencial **Puzzlebot** (Manchester Robotics MCR²) usando el **Algoritmo de Monte Carlo (MCL / Particle Filter)** implementado desde cero en ROS 2 Humble.

---

## Tabla de contenidos

1. [Descripción general](#descripción-general)
2. [Requisitos](#requisitos)
3. [Estructura del paquete](#estructura-del-paquete)
4. [Arquitectura del sistema](#arquitectura-del-sistema)
5. [Fases del algoritmo MCL](#fases-del-algoritmo-mcl)
   - [Fase A — Carga del mapa (MapLoader)](#fase-a--carga-del-mapa-maploader)
   - [Fase B — Inicialización de partículas](#fase-b--inicialización-de-partículas)
   - [Fase C — Suscripción a sensores](#fase-c--suscripción-a-sensores)
   - [Fase D — Loop principal (10 Hz)](#fase-d--loop-principal-10-hz)
   - [Fase G — Predicción por odometría (Motion Model)](#fase-g--predicción-por-odometría-motion-model)
   - [Fase E — Corrección por LiDAR (Sensor Model)](#fase-e--corrección-por-lidar-sensor-model)
   - [Fase F — Resampleo adaptativo](#fase-f--resampleo-adaptativo)
   - [Fase H — Estimación de pose](#fase-h--estimación-de-pose)
   - [Detector de saltos imposibles](#detector-de-saltos-imposibles)
6. [Nodos auxiliares](#nodos-auxiliares)
7. [Tópicos y frames TF](#tópicos-y-frames-tf)
8. [Parámetros configurables](#parámetros-configurables)
9. [Cómo ejecutar](#cómo-ejecutar)
10. [Generación del mapa](#generación-del-mapa)

---

## Descripción general

La **Localización de Monte Carlo** (también llamada *Particle Filter Localization*) es un algoritmo probabilístico que estima la pose del robot dentro de un mapa conocido. En lugar de mantener una distribución de probabilidad cerrada, usa un conjunto de **partículas** (hipótesis de pose) que evolucionan con el movimiento y se ponderan con las observaciones del sensor. Con el tiempo, las partículas convergen hacia la pose real del robot.

Este paquete implementa el algoritmo completo con las siguientes mejoras:
- **Modelo de movimiento de dos rotaciones** (rot1 + traslación + rot2) para mayor precisión.
- **Raycasting** sobre el mapa occupancy para el modelo de sensor.
- **Anclaje suave a odometría** para evitar "teletransporte" en zonas ambiguas.
- **Resampleo adaptativo** con tasa variable de partículas aleatorias (recuperación ante secuestro).
- **Media circular ponderada** para estimar el ángulo sin discontinuidad en ±π.
- **Detector de saltos imposibles** que fuerza reinicialización cuando la nube se dispersa.

---

## Requisitos

| Dependencia | Versión |
|---|---|
| ROS 2 | Humble Hawksbill |
| Python | ≥ 3.10 |
| numpy | cualquiera |
| Pillow (PIL) | cualquiera |
| rclpy, sensor_msgs, nav_msgs, geometry_msgs, tf2_ros | incluidos en ROS 2 |

Instalar dependencias Python si no están presentes:
```bash
pip install numpy Pillow
```

---

## Estructura del paquete

```
puzzlebot_localization/
├── config/
│   └── mcl.rviz                      # Configuración de RViz con partículas y pose estimada
├── launch/
│   └── mcl.launch.py                 # Launch file del nodo MCL + RViz
├── maps/
│   ├── obstacles.pgm                 # Mapa occupancy grid en formato PGM
│   └── obstacles.yaml                # Metadatos del mapa (resolución, origen, umbrales)
├── puzzlebot_localization/
│   ├── __init__.py
│   ├── monte_carlo_localization.py   # Nodo principal — algoritmo MCL completo
│   ├── map_loader.py                 # Carga y utilidades del mapa occupancy
│   ├── transform_utils.py            # Conversiones cuaternión <-> Euler sin deps externas
│   ├── fake_scan_publisher.py        # Nodo auxiliar: scan sintético desde odometría
│   └── teleop_key_auto_stop.py       # Teleop con auto-parada al soltar tecla
├── scripts/
│   └── generate_map.py               # Script standalone para regenerar el mapa PGM
├── package.xml
├── setup.py
└── setup.cfg
```

---

## Arquitectura del sistema

```
                    ┌─────────────────────┐
  /cmd_vel ──────►  │   twist_relay       │ ──► /puzzlebot_controller/cmd_vel
                    └─────────────────────┘
                              │
                    ┌─────────▼───────────┐
                    │   simple_controller  │
                    │  (cinemática + odom) │
                    └──────┬──────────────┘
                           │
          /puzzlebot_controller/odom   /scan (LiDAR)
                           │                │
                    ┌──────▼────────────────▼──────┐
                    │   MonteCarloLocalization      │
                    │                               │
                    │  init_particles()             │
                    │  ┌── predict()       (G)      │
                    │  ├── compute_weights() (E)    │
                    │  ├── resample()       (F)     │
                    │  └── estimate_pose()  (H)     │
                    └──────────────────────────────┘
                           │              │
              /particle_cloud     /estimated_pose
              TF: map → odom
```

---

## Fases del algoritmo MCL

### Fase A — Carga del mapa (MapLoader)

**Archivo:** `puzzlebot_localization/map_loader.py`

El mapa se representa como una imagen PGM (escala de grises) acompañada de un archivo YAML con metadatos. El `MapLoader` realiza las siguientes operaciones:

1. **Lectura del YAML**: extrae `resolution` (m/pixel), `origin` (x, y, yaw del píxel [0,0] en coordenadas mundo) y los umbrales `occupied_thresh` / `free_thresh`.

2. **Lectura del PGM**: la imagen se convierte a una matriz NumPy `uint8`. Cada píxel representa la probabilidad de ocupación escalada a 0–255 (convención ROS: **blanco = libre, negro = ocupado**).

3. **Construcción de la matriz de ocupación** (`self.occupancy`):
   - Valor `100` → celda ocupada (píxel > `occupied_thresh × 255`)
   - Valor `0` → celda libre (píxel < `free_thresh × 255`)
   - Valor `-1` → desconocido

4. **Conversión de coordenadas**:
   - `world_to_map(x, y)` → `(row, col)`: de metros mundo a índices de celda.
   - `map_to_world(row, col)` → `(x, y)`: de índices al centro de la celda en metros.

5. **Consultas de ocupación**:
   - `is_free(x, y)` → `True` si la celda es libre (partícula válida).
   - `is_occupied(x, y)` → `True` si hay obstáculo.

El mapa se publica una vez al inicio en el tópico `/map` como `OccupancyGrid` con QoS `TRANSIENT_LOCAL` para que RViz lo reciba aunque se conecte tarde.

---

### Fase B — Inicialización de partículas

**Método:** `init_particles()` en `monte_carlo_localization.py`

Se crean `num_particles` (default 300) hipótesis de pose inicial:

```
Pose ~ Normal(initial_pose, initial_pose_std)
```

Si se proporciona `initial_pose = [x₀, y₀, θ₀]` e `initial_pose_std = [σx, σy, σθ]`, las partículas se muestrean con distribución gaussiana centrada en la pose inicial. Solo se aceptan partículas cuya posición caiga en una celda **libre** del mapa.

Si no se proporciona pose inicial (o todos los intentos fallan), se hace un muestreo **global**: se elijen filas y columnas aleatorias del mapa hasta encontrar celdas libres, con ángulo uniforme en `[−π, π]`.

Cada partícula tiene:
- `x`, `y`: posición en metros (frame `map`)
- `theta`: orientación en radianes
- `weight`: peso inicial uniforme `1/N`

---

### Fase C — Suscripción a sensores

El nodo se suscribe a dos fuentes de información:

| Tópico | Tipo | Uso |
|---|---|---|
| `/puzzlebot_controller/odom` | `nav_msgs/Odometry` | Movimiento incremental (modelo de movimiento) |
| `/scan` | `sensor_msgs/LaserScan` | Observaciones del entorno (modelo de sensor) |

Los mensajes se almacenan en `self.current_odom` y `self.latest_scan` para ser procesados en el loop principal.

---

### Fase D — Loop principal (10 Hz)

**Método:** `update()` — disparado por un timer de 0.1 s.

En cada iteración:
1. Se extrae el delta de odometría `(dx, dy, dθ)` respecto a la iteración anterior.
2. Si hay movimiento significativo (|dx|>0.1 mm o |dθ|>0.1 mrad), se ejecuta la **predicción**.
3. Se calculan los **pesos** de cada partícula con el scan.
4. Se ejecuta el **resampleo**.
5. Se calcula la **pose estimada**.
6. Se detectan saltos imposibles.
7. Se publican partículas, pose estimada y TF `map → odom`.

---

### Fase G — Predicción por odometría (Motion Model)

**Método:** `predict(dx, dy, dtheta)`

Se usa el **modelo odométrico de dos rotaciones**, estándar en MCL:

```
δ_rot1  = atan2(dy, dx) − θ_prev   # giro inicial hacia la dirección del movimiento
δ_trans = √(dx² + dy²)             # traslación
δ_rot2  = dθ − δ_rot1              # giro final de ajuste de orientación
```

Para cada partícula se añade **ruido gaussiano** antes de aplicar el movimiento:

```python
dr1_noisy   = δ_rot1  + N(0, motion_noise_theta)
trans_noisy = δ_trans + N(0, motion_noise_xy)
dr2_noisy   = δ_rot2  + N(0, motion_noise_theta)

p.theta += dr1_noisy
p.x     += trans_noisy * cos(p.theta)
p.y     += trans_noisy * sin(p.theta)
p.theta += dr2_noisy
```

Si la partícula cae en una celda ocupada tras el movimiento, su peso se reduce a `1e-6` (casi cero pero no exactamente, para que siga participando en el resampleo con probabilidad mínima).

**Parámetros clave:**
- `motion_noise_xy = 0.05 m` (ruido en traslación)
- `motion_noise_theta = 0.05 rad` (ruido en rotación)

---

### Fase E — Corrección por LiDAR (Sensor Model)

**Método:** `compute_weights()`

Para cada partícula se evalúa qué tan bien explicaría el scan observado si el robot estuviese en esa pose. El proceso es:

#### 1. Submuestreo del scan
De los ~360 rayos del LiDAR se toman cada `scan_subsample` rayos (default cada 20, es decir 18 rayos) para reducir el costo computacional.

#### 2. Raycasting sintético
Para cada rayo seleccionado, se lanza un rayo desde la posición de la partícula en la dirección `p.theta + angle_rayo`. Se avanza de celda en celda (paso = `resolution = 0.05 m`) hasta encontrar una celda ocupada o salir del mapa. La distancia recorrida es `r_expected`.

#### 3. Puntuación por diferencia de rangos
```
avg_diff = mean(|r_expected − r_real|)  para todos los rayos válidos
score_scan = 1 / (1 + avg_diff)
```
Un `avg_diff = 0` da `score_scan = 1.0` (perfecto). Un `avg_diff = 1 m` da `score_scan = 0.5`.

#### 4. Anclaje suave a odometría
Para evitar que el filtro "teletransporte" la nube a zonas del mapa con geometría similar (paredes paralelas, simetría), se penaliza partículas demasiado alejadas de la posición de la odometría:

```
dist = √((p.x − odom_x)² + (p.y − odom_y)²)
score_odom = exp(−dist² / (2 × σ_odom²))    σ_odom = 1.0 m
```

#### 5. Peso final
```
p.weight = score_scan × score_odom
```

---

### Fase F — Resampleo adaptativo

**Método:** `resample()`

#### Resampleo sistemático (Low-Variance Resampling)
Se seleccionan `n_resample` partículas proporcional a su peso usando el método de **resampleo sistemático**, que garantiza menor varianza que el resampleo multinomial:

```
step = 1/N
r ~ U(0, step)
Para cada nueva partícula: avanzar r en la CDF de pesos
```

A cada partícula resampleada se le añade un **jitter** pequeño (`2× motion_noise`) para evitar degeneración (*sample impoverishment*).

#### Tasa adaptativa de partículas aleatorias
La fracción de partículas aleatorias inyectadas en cada iteración varía según la calidad de localización, medida por el **peso promedio suavizado con EMA** (Exponential Moving Average):

```
avg_weight_ema = α × avg_weight + (1−α) × avg_weight_ema_anterior
```

La tasa adaptativa interpola linealmente entre umbrales:

| Estado del filtro | `avg_weight_ema` | Tasa de partículas aleatorias |
|---|---|---|
| Bien localizado | ≥ `weight_threshold_good = 0.5` | `random_particles_min_rate = 0.003` (0.3%) |
| Perdido / secuestrado | ≤ `weight_threshold_bad = 0.1` | `random_particles_rate = 0.05` (5%) |
| Intermedio | entre umbrales | Interpolación lineal |

Esto implementa el comportamiento del **AMCL** (Adaptive MCL): pocas partículas aleatorias cuando el robot está localizado, muchas cuando se pierde.

#### Degeneración total
Si la suma de pesos cae por debajo de `1e-12` (todos los pesos son prácticamente cero), se llama a `_reinit_around_odom()` que reinicializa las partículas en gaussianas concéntricas alrededor de la odometría actual.

---

### Fase H — Estimación de pose

**Método:** `estimate_pose()`

La pose final se calcula como la **media ponderada** de todas las partículas:

```
x_est = Σ(wᵢ × xᵢ)
y_est = Σ(wᵢ × yᵢ)
```

Para el ángulo se usa la **media circular ponderada** para manejar correctamente la discontinuidad en ±π:

```
sin_mean = Σ(wᵢ × sin(θᵢ))
cos_mean = Σ(wᵢ × cos(θᵢ))
θ_est = atan2(sin_mean, cos_mean)
```

La pose estimada se publica como `PoseWithCovarianceStamped` en `/estimated_pose`.

#### TF map → odom
Se calcula la transformación que lleva de `map` a `odom` como:
```
T_map_odom = T_map_base × inv(T_odom_base)
```
donde `T_map_base` es la pose estimada y `T_odom_base` es la pose de la odometría. Esto actualiza dinámicamente el frame `map` en el árbol de transformaciones de ROS.

---

### Detector de saltos imposibles

Después de cada estimación se verifica que el cambio de pose del MCL no sea físicamente imposible dado el movimiento de la odometría:

```python
max_step = max(max_jump_min, max_jump_factor × odom_step)
if est_step > max_step:
    _reinit_around_odom()
```

Parámetros:
- `max_jump_factor = 5.0` — el MCL puede moverse hasta 5× más que el odom en un paso
- `max_jump_min = 0.3 m` — tolerancia mínima absoluta

Cuando el filtro "teletransporta" (salta a otra zona del mapa con geometría similar), este detector lo detecta y fuerza una reinicialización alrededor de la odometría.

---

## Nodos auxiliares

### `fake_scan_publisher.py`
Publica un `LaserScan` sintético en `/scan` a partir de la posición de la odometría y el mapa occupancy. Útil para probar el MCL en simulación sin Gazebo (solo con el controlador). Realiza raycasting desde la pose de la odometría con ruido configurable.

**Parámetros:**
- `samples = 360` — número de rayos
- `noise_stddev = 0.0 m` — desviación estándar del ruido en los rangos

### `teleop_key_auto_stop.py`
Teleop por teclado que envía comandos de velocidad y para automáticamente el robot al soltar la tecla. Publica en `/cmd_vel` (Twist).

---

## Tópicos y frames TF

| Tópico | Tipo | Dirección | Descripción |
|---|---|---|---|
| `/puzzlebot_controller/odom` | `Odometry` | Entrada | Odometría del controlador diferencial |
| `/scan` | `LaserScan` | Entrada | Scan del LiDAR (real o sintético) |
| `/particle_cloud` | `PoseArray` | Salida | Nube de partículas (visualización RViz) |
| `/estimated_pose` | `PoseWithCovarianceStamped` | Salida | Pose estimada del robot en frame `map` |
| `/map` | `OccupancyGrid` | Salida | Mapa occupancy (publicado una vez al inicio) |

| TF | Descripción |
|---|---|
| `map` → `odom` | Corrección MCL: diferencia entre pose estimada y odometría |
| `odom` → `base_footprint` | Publicado por `simple_controller` (odometría pura) |

---

## Parámetros configurables

Todos los parámetros se declaran y pueden sobreescribirse desde el launch file o CLI.

| Parámetro | Default | Descripción |
|---|---|---|
| `map_yaml_path` | `maps/obstacles.yaml` | Ruta al YAML del mapa |
| `num_particles` | `300` | Número total de partículas |
| `motion_noise_xy` | `0.05` | Ruido gaussiano en traslación (m) |
| `motion_noise_theta` | `0.05` | Ruido gaussiano en rotación (rad) |
| `scan_subsample` | `20` | Tomar 1 de cada N rayos del scan |
| `resample_rate` | `0.5` | (Reservado para uso futuro) |
| `random_particles_rate` | `0.05` | Tasa máxima de partículas aleatorias (perdido) |
| `random_particles_min_rate` | `0.003` | Tasa mínima de partículas aleatorias (localizado) |
| `weight_threshold_good` | `0.5` | Umbral de peso promedio para "bien localizado" |
| `weight_threshold_bad` | `0.1` | Umbral de peso promedio para "perdido" |
| `weight_ema_alpha` | `0.3` | Factor de suavizado EMA del peso promedio |
| `initial_pose` | `[0.0, 0.0, 0.0]` | Pose inicial `[x, y, θ]` en metros/rad |
| `initial_pose_std` | `[1.0, 1.0, 1.0]` | Desviación estándar de la inicialización |
| `max_jump_factor` | `5.0` | Factor máximo de salto MCL vs odom |
| `max_jump_min` | `0.3` | Tolerancia mínima de salto (m) |

---

## Cómo ejecutar

### 1. Compilar el paquete (workspace ROS 2)

```bash
# Desde la raíz del workspace
colcon build --packages-select puzzlebot_localization
source install/setup.bash
```

### 2. Solo MCL con scan sintético (sin Gazebo)

```bash
# Terminal 1: controlador diferencial
ros2 launch puzzlebot_controller controller.launch.py use_simple_controller:=true

# Terminal 2: publicador de scan sintético
ros2 run puzzlebot_localization fake_scan_publisher --ros-args \
  -p map_yaml_path:=/ruta/a/maps/obstacles.yaml

# Terminal 3: MCL + RViz
ros2 launch puzzlebot_localization mcl.launch.py
```

### 3. Simulación completa con Gazebo

```bash
ros2 launch puzzlebot_bringup full_sim_mcl.launch.py world_name:=obstacles use_rviz:=true
```

Este launch file orquesta:
1. **Gazebo** con el mundo `obstacles.world` y el modelo MCR²
2. **Controlador diferencial** (`simple_controller` + `twist_relay`)
3. **MCL** (con delay de 8 s para que Gazebo arranque)

### 4. Teleop para mover el robot

```bash
ros2 run puzzlebot_localization teleop_key_auto_stop
```

---

## Generación del mapa

El mapa `obstacles.pgm` fue generado programáticamente con `scripts/generate_map.py`. El script dibuja en PIL:

| Obstáculo | Posición | Tamaño/Radio |
|---|---|---|
| Pared Norte | (0, 5) | 10 m × 0.2 m |
| Pared Sur | (0, −5) | 10 m × 0.2 m |
| Pared Este | (5, 0) | 10 m × 0.2 m |
| Pared Oeste | (−5, 0) | 10 m × 0.2 m |
| Caja 1 | (2, 2) | 0.5 m × 0.5 m |
| Caja 2 (rotada 45°) | (−2, −1) | 0.6 m × 0.6 m |
| Cilindro | (−1, 3) | r = 0.3 m |
| Pared interna | (1, −2) | 3 m × 0.15 m, rot. 0.3 rad |

Para regenerar el mapa:
```bash
python3 scripts/generate_map.py
```

El resultado se guarda automáticamente en `maps/obstacles.pgm` y `maps/obstacles.yaml`.

---

## Créditos

Desarrollado para el curso **TE3003B — Integración de Robótica y Sistemas Inteligentes**, Tecnológico de Monterrey, FJ2026.
