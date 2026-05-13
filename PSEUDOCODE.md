# Pseudocódigo — Monte Carlo Localization (Puzzlebot)

Implementación del algoritmo MCL con Beam Model de mezcla (Thrun, Probabilistic Robotics Cap. 6.3).

---

## Algoritmo completo

```
INICIALIZACIÓN:
  Cargar mapa occupancy (PGM + YAML)
    → matriz: celda = LIBRE (0), OCUPADA (100), DESCONOCIDA (-1)

  Para i = 1..N:
    (x, y, θ) ~ Normal(pose_inicial, σ_inicial)
    Si celda(x, y) == LIBRE:
      agregar Partícula(x, y, θ, peso = 1/N)

  Publicar mapa en /map (QoS TRANSIENT_LOCAL)
  Publicar TF estático map → odom (identidad inicial)

══════════════════════════════════════════════════════════
LOOP principal (10 Hz):
══════════════════════════════════════════════════════════

  Leer odometría actual: (x_odom, y_odom, θ_odom)
  Calcular delta respecto a iteración anterior:
    dx     = x_odom  − x_prev
    dy     = y_odom  − y_prev
    dθ     = θ_odom  − θ_prev   (normalizado a [-π, π])

  ┌─────────────────────────────────────────────────────┐
  │  G. PREDICCIÓN — Motion Model (Dead Reckoning)      │
  └─────────────────────────────────────────────────────┘

  Descomponer movimiento en dos rotaciones + traslación:
    δ_rot1  = atan2(dy, dx) − θ_prev   // giro hacia la dirección del movimiento
    δ_trans = √(dx² + dy²)             // distancia recorrida
    δ_rot2  = dθ − δ_rot1              // giro de ajuste final de orientación

  Para cada partícula p:
    // Añadir ruido gaussiano independiente a cada componente
    dr1   = δ_rot1  + N(0, σ_θ)
    trans = δ_trans + N(0, σ_xy)
    dr2   = δ_rot2  + N(0, σ_θ)

    // Aplicar movimiento ruidoso
    p.θ  = p.θ + dr1
    p.x  = p.x + trans · cos(p.θ)
    p.y  = p.y + trans · sin(p.θ)
    p.θ  = p.θ + dr2                   (normalizado a [-π, π])

    // Partícula en pared → peso mínimo (no cero)
    Si celda(p.x, p.y) == OCUPADA:
      p.peso = 1×10⁻⁶

  ┌─────────────────────────────────────────────────────┐
  │  E. CORRECCIÓN — Beam Model (Thrun cap. 6.3)        │
  └─────────────────────────────────────────────────────┘

  // Constantes del modelo de mezcla
  hit_norm    = 1 / √(2π · σ_hit²)
  rand_density = 1 / r_max

  Para cada partícula p:
    Si celda(p.x, p.y) == OCUPADA:
      log_l[p] = -∞;  continuar

    log_l = 0

    Para cada rayo i (submuestreado cada K):
      r_real = scan.ranges[i]
      Si r_real es inválido (inf, nan, fuera de rango): continuar

      ángulo = p.θ + scan.angle[i]   (normalizado a [-π, π])

      // Raycasting: avanzar celda a celda hasta obstáculo
      r_esperado = r_max
      d = resolución
      Mientras d ≤ r_max:
        (rx, ry) = (p.x + d·cos(ángulo),  p.y + d·sin(ángulo))
        Si (rx, ry) fuera del mapa o celda(rx,ry) == OCUPADA:
          r_esperado = d;  break
        d += resolución

      // Likelihood de mezcla para este rayo
      err   = r_real − r_esperado
      p_hit = hit_norm · exp(−err² / (2·σ_hit²))
      p_ray = z_hit · p_hit  +  z_rand · rand_density

      log_l += log(p_ray)      // acumular en log-space

    log_likelihoods[p] = log_l

  // Normalización con max-shift (evita underflow)
  max_log = max(log_likelihoods)
  Para cada partícula p:
    Si log_l[p] == -∞:
      p.peso = 1×10⁻⁹               // mínimo no-cero
    Si no:
      p.peso = exp(log_l[p] − max_log)

  ┌─────────────────────────────────────────────────────┐
  │  F. RESAMPLEO ADAPTATIVO                            │
  └─────────────────────────────────────────────────────┘

  // Actualizar EMA del peso promedio (medida de calidad)
  avg_peso     = Σ(p.peso) / N
  avg_peso_ema = α · avg_peso  +  (1−α) · avg_peso_ema_anterior

  // Tasa adaptativa de partículas aleatorias
  Si avg_peso_ema ≥ umbral_bueno:   tasa = tasa_min        // bien localizado
  Si avg_peso_ema ≤ umbral_malo:    tasa = tasa_max        // perdido
  Si no: tasa = interpolación lineal entre tasa_min y tasa_max

  n_random   = round(N · tasa)
  n_resample = N − n_random

  // Low-variance (systematic) resampling
  Normalizar pesos → CDF
  r ~ U(0, 1/n_resample)
  Para i = 1..n_resample:
    Seleccionar partícula j donde CDF[j] ≥ r
    Copiar p[j] con jitter: x,y += N(0, 2·σ_xy);  θ += N(0, 2·σ_θ)
    r += 1/n_resample

  // Inyectar partículas aleatorias en espacio libre (recovery)
  Para i = 1..n_random:
    Muestrear celda libre aleatoria del mapa
    Agregar Partícula(x, y, θ ~ U(-π, π), peso = 1/N)

  Si Σ(pesos) < 1×10⁻¹²:           // degeneración total
    Reinicializar partículas ~ Normal(odom_actual, 0.3 m)

  ┌─────────────────────────────────────────────────────┐
  │  H. ESTIMACIÓN DE POSE                              │
  └─────────────────────────────────────────────────────┘

  Normalizar: wᵢ = p.peso / Σ(p.peso)

  x_est = Σ(wᵢ · pᵢ.x)
  y_est = Σ(wᵢ · pᵢ.y)

  // Media circular ponderada (maneja salto en ±π sin artefactos)
  θ_est = atan2( Σ(wᵢ · sin(pᵢ.θ)),  Σ(wᵢ · cos(pᵢ.θ)) )

  Publicar:
    /particle_cloud    → nube de partículas (PoseArray)
    /estimated_pose    → pose estimada (PoseWithCovarianceStamped)
    TF map → odom:     T_map_odom = T_map_base × inv(T_odom_base)
```

---

## Parámetros del algoritmo

| Símbolo | Parámetro ROS | Valor | Descripción |
|---|---|---|---|
| N | `num_particles` | 300 | Número de partículas |
| σ_xy | `motion_noise_xy` | 0.01 m | Ruido gaussiano en traslación |
| σ_θ | `motion_noise_theta` | 0.02 rad | Ruido gaussiano en rotación |
| K | `scan_subsample` | 20 | Tomar 1 de cada K rayos |
| σ_hit | `sigma_hit` | 0.25 m | Ancho del componente gaussiano por rayo |
| z_hit | `z_hit` | 0.9 | Peso del componente gaussiano (mezcla) |
| z_rand | `z_rand` | 0.1 | Peso del componente uniforme (piso anti-outlier) |
| α | `weight_ema_alpha` | 0.3 | Factor de suavizado EMA |
| umbral_bueno | `weight_threshold_good` | 0.5 | EMA por encima → tasa mínima de aleatorias |
| umbral_malo | `weight_threshold_bad` | 0.1 | EMA por debajo → tasa máxima de aleatorias |
| tasa_max | `random_particles_rate` | 0.02 | Fracción máxima de partículas aleatorias |
| tasa_min | `random_particles_min_rate` | 0.003 | Fracción mínima de partículas aleatorias |

---

## Por qué mezcla Gaussiano + Uniforme

El modelo gaussiano puro (`p_rand = 0`) es frágil ante rayos atípicos. Con tan solo 2 rayos malos (frecuente por discretización del mapa a 0.05 m/px o diferencias entre el mundo Gazebo y el mapa), el cociente de verosimilitudes entre la pose correcta y una pose errónea puede colapsar:

| Escenario | Gaussiano puro | Mezcla (z_hit=0.9, z_rand=0.1) |
|---|---|---|
| 1 outlier | 2.4 × 10⁶ | 3.4 × 10⁷ |
| 2 outliers | **25×** ← frágil | **200,000×** ← robusto |
| Pose totalmente mala | 10⁻³⁰ | 10⁻²⁵ |

Un ratio de 25× no sobrevive al resampleo en pocas iteraciones. La mezcla eleva ese margen a 200,000×, suficiente para que la pose correcta domine consistentemente.

El término `z_rand · (1/r_max)` actúa como **piso de probabilidad**: garantiza que ningún rayo aislado pueda anular completamente el likelihood de una partícula, modelando mediciones atípicas sin necesitar los componentes `p_short` y `p_max` del Beam Model completo de Thrun.
