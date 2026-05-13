#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import PoseArray, Pose, PoseWithCovarianceStamped
from tf2_ros import TransformBroadcaster, TransformStamped, StaticTransformBroadcaster
from puzzlebot_localization.transform_utils import quaternion_from_euler, euler_from_quaternion

from puzzlebot_localization.map_loader import MapLoader


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def pose_to_xytheta(pose):
    """Extrae (x, y, theta) de una geometry_msgs/Pose."""
    q = pose.orientation
    theta = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
    return pose.position.x, pose.position.y, theta


def xytheta_to_pose(x, y, theta):
    """Crea una geometry_msgs/Pose desde (x, y, theta)."""
    p = Pose()
    p.position.x = float(x)
    p.position.y = float(y)
    q = quaternion_from_euler(0, 0, theta)
    p.orientation.x = q[0]
    p.orientation.y = q[1]
    p.orientation.z = q[2]
    p.orientation.w = q[3]
    return p


def transform_inv(x, y, theta):
    """Inversa de una transformación 2D."""
    c = math.cos(theta)
    s = math.sin(theta)
    x_inv = -(x * c + y * s)
    y_inv = x * s - y * c
    theta_inv = -theta
    return x_inv, y_inv, theta_inv


def transform_mul(x1, y1, t1, x2, y2, t2):
    """Composición de dos transforms 2D: T1 * T2."""
    c1 = math.cos(t1)
    s1 = math.sin(t1)
    x = x1 + x2 * c1 - y2 * s1
    y = y1 + x2 * s1 + y2 * c1
    t = normalize_angle(t1 + t2)
    return x, y, t


class Particle:
    __slots__ = ('x', 'y', 'theta', 'weight')

    def __init__(self, x, y, theta, weight=1.0):
        self.x = x
        self.y = y
        self.theta = theta
        self.weight = weight

    def copy(self):
        return Particle(self.x, self.y, self.theta, self.weight)


class MonteCarloLocalization(Node):
    def __init__(self):
        super().__init__('monte_carlo_localization')

        # Parámetros
        self.declare_parameter('map_yaml_path', 'src/puzzlebot_localization/maps/obstacles.yaml')
        self.declare_parameter('num_particles', 300)
        self.declare_parameter('motion_noise_xy', 0.01)
        self.declare_parameter('motion_noise_theta', 0.02)
        self.declare_parameter('scan_subsample', 20)
        self.declare_parameter('resample_rate', 0.5)
        self.declare_parameter('random_particles_rate', 0.05)  # tasa máxima (cuando la localización es mala)
        self.declare_parameter('random_particles_min_rate', 0.003)  # tasa mínima (cuando localiza bien)
        self.declare_parameter('weight_threshold_good', 0.5)  # peso promedio que se considera "bien localizado"
        self.declare_parameter('weight_threshold_bad', 0.1)   # peso promedio que se considera "perdido"
        self.declare_parameter('weight_ema_alpha', 0.3)       # suavizado EMA del peso promedio
        self.declare_parameter('initial_pose', [0.0, 0.0, 0.0])
        self.declare_parameter('initial_pose_std', [1.0, 1.0, 1.0])

        map_yaml = self.get_parameter('map_yaml_path').get_parameter_value().string_value
        self.num_particles = self.get_parameter('num_particles').get_parameter_value().integer_value
        self.motion_noise_xy = self.get_parameter('motion_noise_xy').get_parameter_value().double_value
        self.motion_noise_theta = self.get_parameter('motion_noise_theta').get_parameter_value().double_value
        self.scan_subsample = self.get_parameter('scan_subsample').get_parameter_value().integer_value
        self.resample_rate = self.get_parameter('resample_rate').get_parameter_value().double_value
        self.random_particles_rate_max = self.get_parameter('random_particles_rate').get_parameter_value().double_value
        self.random_particles_rate_min = self.get_parameter('random_particles_min_rate').get_parameter_value().double_value
        self.weight_thr_good = self.get_parameter('weight_threshold_good').get_parameter_value().double_value
        self.weight_thr_bad  = self.get_parameter('weight_threshold_bad').get_parameter_value().double_value
        self.weight_ema_alpha = self.get_parameter('weight_ema_alpha').get_parameter_value().double_value
        self.avg_weight_ema = 0.0  # peso promedio suavizado (estado del filtro)

        self.get_logger().info(f'Cargando mapa: {map_yaml}')
        self.map_loader = MapLoader(map_yaml)
        self.get_logger().info(f'Mapa cargado: {self.map_loader.width}x{self.map_loader.height} px, '
                               f'res={self.map_loader.resolution} m/px')

        # Suscriptores
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scanCallback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/puzzlebot_controller/odom', self.odomCallback, 10)

        # Publicadores
        self.particle_pub = self.create_publisher(PoseArray, '/particle_cloud', 10)
        self.estimated_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/estimated_pose', 10)
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', map_qos)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        # Estado
        self.particles = []
        self.latest_scan = None
        self.prev_odom = None
        self.current_odom = None
        self.initialized = False
        self.iteration_count = 0
        self.prev_est = None  # (x, y, theta) anterior para detector de saltos
        self.declare_parameter('max_jump_factor', 5.0)  # MCL puede saltar hasta N× el delta del odom
        self.declare_parameter('max_jump_min', 0.3)     # tolerancia mínima absoluta en metros
        self.max_jump_factor = self.get_parameter('max_jump_factor').get_parameter_value().double_value
        self.max_jump_min = self.get_parameter('max_jump_min').get_parameter_value().double_value

        self.init_particles()
        self.publish_map()
        self.publish_initial_tf()

        # Timer para ejecutar MCL a 10 Hz
        self.timer = self.create_timer(0.1, self.update)

    def publish_initial_tf(self):
        """Publica TF estático map -> odom como identidad para que RViz tenga el frame map disponible."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(t)

    def publish_map(self):
        """Publica el mapa como OccupancyGrid en /map."""
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = 'map'
        grid.info.resolution = self.map_loader.resolution
        grid.info.width = self.map_loader.width
        grid.info.height = self.map_loader.height
        grid.info.origin.position.x = self.map_loader.origin_x
        grid.info.origin.position.y = self.map_loader.origin_y
        grid.info.origin.position.z = 0.0
        q = quaternion_from_euler(0, 0, self.map_loader.origin_yaw)
        grid.info.origin.orientation.x = q[0]
        grid.info.origin.orientation.y = q[1]
        grid.info.origin.orientation.z = q[2]
        grid.info.origin.orientation.w = q[3]

        # occupancy es int8: -1 unknown, 0 free, 100 occupied
        flat = self.map_loader.occupancy.flatten().tolist()
        grid.data = flat
        self.map_pub.publish(grid)

    def init_particles(self):
        """Inicializa partículas en celdas libres del mapa, cerca de initial_pose si se proporciona."""
        initial_pose = self.get_parameter('initial_pose').get_parameter_value().double_array_value
        initial_std = self.get_parameter('initial_pose_std').get_parameter_value().double_array_value

        self.particles = []
        attempts = 0
        max_attempts = self.num_particles * 10

        while len(self.particles) < self.num_particles and attempts < max_attempts:
            attempts += 1
            if initial_pose and len(initial_pose) >= 3 and initial_std and len(initial_std) >= 3:
                x = np.random.normal(initial_pose[0], initial_std[0])
                y = np.random.normal(initial_pose[1], initial_std[1])
                theta = normalize_angle(np.random.normal(initial_pose[2], initial_std[2]))
            else:
                # fallback: aleatorio global en celdas libres
                row = np.random.randint(0, self.map_loader.height)
                col = np.random.randint(0, self.map_loader.width)
                if self.map_loader.occupancy[row, col] != 0:
                    continue
                x, y = self.map_loader.map_to_world(row, col)
                theta = np.random.uniform(-math.pi, math.pi)

            if self.map_loader.is_free(x, y):
                self.particles.append(Particle(x, y, theta, 1.0 / self.num_particles))

        if not self.particles:
            self.get_logger().error('No se pudieron inicializar partículas en celdas libres!')
            return

        self.get_logger().info(f'Inicializadas {len(self.particles)} partículas')
        self.initialized = True

        # Publicar inmediatamente para que RViz muestre algo
        est_x, est_y, est_theta = self.estimate_pose()
        self.publish_particles()
        self.publish_estimated_pose(est_x, est_y, est_theta)
        self.publish_initial_tf()

    def odomCallback(self, msg):
        self.current_odom = msg

    def scanCallback(self, msg):
        self.latest_scan = msg

    def predict(self, dx, dy, dtheta):
        """Paso G: predice el movimiento de todas las partículas usando dead reckoning con ruido."""
        delta_trans = math.hypot(dx, dy)
        delta_rot1 = 0.0
        if delta_trans > 0.001:
            delta_rot1 = math.atan2(dy, dx) - self.prev_odom_theta
        delta_rot2 = dtheta - delta_rot1

        for p in self.particles:
            # Añadir ruido gaussiano
            dr1_noisy = delta_rot1 + np.random.normal(0.0, self.motion_noise_theta)
            trans_noisy = delta_trans + np.random.normal(0.0, self.motion_noise_xy)
            dr2_noisy = delta_rot2 + np.random.normal(0.0, self.motion_noise_theta)

            p.theta = normalize_angle(p.theta + dr1_noisy)
            p.x += trans_noisy * math.cos(p.theta)
            p.y += trans_noisy * math.sin(p.theta)
            p.theta = normalize_angle(p.theta + dr2_noisy)

            # Partícula en celda ocupada: peso mínimo (no cero) para que siga participando en resample
            if not self.map_loader.is_free(p.x, p.y):
                p.weight = 1e-6

    def compute_weights(self):
        """Paso E: score basado en coincidencia de distancias de rayos + anclaje suave a odometría.
        El anclaje evita que el error de cuantización del raycasting arrastre la nube lejos.
        """
        if self.latest_scan is None or self.current_odom is None:
            return

        scan = self.latest_scan
        ranges = np.array(scan.ranges)
        angles = np.arange(len(ranges)) * scan.angle_increment + scan.angle_min
        step = max(1, self.scan_subsample)
        indices = np.arange(0, len(ranges), step)

        odom_x, odom_y, odom_theta = pose_to_xytheta(self.current_odom.pose.pose)
        sigma_odom = 1.0  # anclaje moderado: evita "teletransporte" cerca de zonas ambiguas (paredes)

        for p in self.particles:
            if p.weight < 1e-9:
                continue

            total_diff = 0.0
            valid_rays = 0

            for idx in indices:
                r_real = ranges[idx]
                if math.isinf(r_real) or math.isnan(r_real) or r_real < scan.range_min or r_real > scan.range_max:
                    continue

                angle = normalize_angle(p.theta + angles[idx])

                # Raycasting para obtener rango ESPERADO
                r_expected = scan.range_max
                step_size = self.map_loader.resolution
                d = step_size
                while d <= scan.range_max:
                    rx_exp = p.x + d * math.cos(angle)
                    ry_exp = p.y + d * math.sin(angle)
                    row_exp, col_exp = self.map_loader.world_to_map(rx_exp, ry_exp)
                    if row_exp < 0 or row_exp >= self.map_loader.height or col_exp < 0 or col_exp >= self.map_loader.width:
                        r_expected = d
                        break
                    if self.map_loader.occupancy[row_exp, col_exp] == 100:
                        r_expected = d
                        break
                    d += step_size

                total_diff += abs(r_expected - r_real)
                valid_rays += 1

            if valid_rays > 0:
                avg_diff = total_diff / valid_rays
                score_scan = 1.0 / (1.0 + avg_diff)
            else:
                score_scan = 0.0

            # Anclaje suave a odometría: penaliza partículas muy lejanas de la odometría
            dist_to_odom = math.hypot(p.x - odom_x, p.y - odom_y)
            score_odom = math.exp(-(dist_to_odom ** 2) / (2 * sigma_odom ** 2))

            p.weight = score_scan * score_odom

    def resample(self):
        """Paso F: low-variance resampling (systematic resampling).
        Selecciona partículas proporcional a su peso, luego inyecta una fracción
        ADAPTATIVA de aleatorias: pocas cuando el filtro está localizado, más
        cuando el peso promedio cae (recuperación ante secuestro o drift).
        """
        weights = np.array([p.weight for p in self.particles], dtype=np.float64)
        total = weights.sum()
        if total < 1e-12:
            self._reinit_around_odom()
            self.avg_weight_ema = 0.0
            return

        # Actualizar EMA del peso promedio (medida de calidad de localización)
        avg_weight = total / len(self.particles)
        self.avg_weight_ema = (
            self.weight_ema_alpha * avg_weight
            + (1.0 - self.weight_ema_alpha) * self.avg_weight_ema
        )

        # Tasa adaptativa: lineal entre thr_bad → max y thr_good → min
        good, bad = self.weight_thr_good, self.weight_thr_bad
        if self.avg_weight_ema >= good:
            adaptive_rate = self.random_particles_rate_min
        elif self.avg_weight_ema <= bad:
            adaptive_rate = self.random_particles_rate_max
        else:
            t = (good - self.avg_weight_ema) / (good - bad)  # 0 cuando bien, 1 cuando mal
            adaptive_rate = (
                self.random_particles_rate_min
                + t * (self.random_particles_rate_max - self.random_particles_rate_min)
            )

        weights /= total

        n = self.num_particles
        n_random = max(0, int(round(n * adaptive_rate)))
        n_resample = n - n_random
        if n_resample <= 0:
            n_resample = 1
            n_random = n - 1

        # Low-variance (systematic) resampling + jitter para evitar clumping
        new_particles = []
        cumsum = np.cumsum(weights)
        step = 1.0 / n_resample
        r = np.random.uniform(0.0, step)
        j = 0
        jitter_xy = self.motion_noise_xy * 2.0
        jitter_th = self.motion_noise_theta * 2.0
        for _ in range(n_resample):
            while r > cumsum[j]:
                j = min(j + 1, len(cumsum) - 1)
            p = self.particles[j].copy()
            p.x += np.random.normal(0.0, jitter_xy)
            p.y += np.random.normal(0.0, jitter_xy)
            p.theta = normalize_angle(p.theta + np.random.normal(0.0, jitter_th))
            p.weight = 1.0 / n
            new_particles.append(p)
            r += step

        # Partículas aleatorias en espacio libre (recuperación adaptativa)
        if n_random > 0:
            attempts = 0
            added = 0
            while added < n_random and attempts < n_random * 30:
                attempts += 1
                row = np.random.randint(0, self.map_loader.height)
                col = np.random.randint(0, self.map_loader.width)
                if self.map_loader.occupancy[row, col] == 0:
                    x, y = self.map_loader.map_to_world(row, col)
                    theta = np.random.uniform(-math.pi, math.pi)
                    new_particles.append(Particle(x, y, theta, 1.0 / n))
                added += 1

        self.particles = new_particles

    def _reinit_around_odom(self):
        """Reinicializa partículas alrededor de la odometría cuando los pesos degeneran.
        Garantiza num_particles usando espacio libre global como fallback.
        """
        odom_x, odom_y, odom_theta = (0.0, 0.0, 0.0)
        if self.current_odom is not None:
            odom_x, odom_y, odom_theta = pose_to_xytheta(self.current_odom.pose.pose)

        new_particles = []
        for std in [0.3, 0.6, 1.2]:  # aumentar radio si el área local tiene pocas celdas libres
            attempts = 0
            while len(new_particles) < self.num_particles and attempts < self.num_particles * 30:
                attempts += 1
                x = np.random.normal(odom_x, std)
                y = np.random.normal(odom_y, std)
                theta = normalize_angle(np.random.normal(odom_theta, 0.5))
                if self.map_loader.is_free(x, y):
                    new_particles.append(Particle(x, y, theta, 1.0 / self.num_particles))
            if len(new_particles) >= self.num_particles:
                break

        # Fallback global si aun faltan partículas
        while len(new_particles) < self.num_particles:
            row = np.random.randint(0, self.map_loader.height)
            col = np.random.randint(0, self.map_loader.width)
            if self.map_loader.occupancy[row, col] == 0:
                x, y = self.map_loader.map_to_world(row, col)
                theta = np.random.uniform(-math.pi, math.pi)
                new_particles.append(Particle(x, y, theta, 1.0 / self.num_particles))

        self.particles = new_particles
        self.get_logger().warn('Partículas reinicializadas alrededor de la odometría')

    def estimate_pose(self):
        """Estima la pose como media ponderada de las partículas.
        Usa media circular para theta para manejar el salto ±π correctamente.
        """
        if not self.particles:
            if self.current_odom is not None:
                return pose_to_xytheta(self.current_odom.pose.pose)
            return 0.0, 0.0, 0.0

        weights = np.array([p.weight for p in self.particles], dtype=np.float64)
        total = weights.sum()
        if total < 1e-12:
            weights = np.ones(len(self.particles)) / len(self.particles)
        else:
            weights /= total

        xs = np.array([p.x for p in self.particles])
        ys = np.array([p.y for p in self.particles])
        thetas = np.array([p.theta for p in self.particles])

        est_x = float(np.dot(weights, xs))
        est_y = float(np.dot(weights, ys))
        # Media circular ponderada para theta
        sin_mean = float(np.dot(weights, np.sin(thetas)))
        cos_mean = float(np.dot(weights, np.cos(thetas)))
        est_theta = math.atan2(sin_mean, cos_mean)

        return est_x, est_y, est_theta

    def publish_particles(self):
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.poses = [xytheta_to_pose(p.x, p.y, p.theta) for p in self.particles]
        self.particle_pub.publish(msg)

    def publish_estimated_pose(self, x, y, theta):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose = xytheta_to_pose(x, y, theta)
        self.estimated_pose_pub.publish(msg)

    def publish_tf(self, est_x, est_y, est_theta):
        """Publica TF map -> odom basado en la diferencia entre pose estimada y odometría actual."""
        if self.current_odom is None:
            return

        odom_x, odom_y, odom_theta = pose_to_xytheta(self.current_odom.pose.pose)
        # T_map_odom = T_map_base * inverse(T_odom_base)
        inv_x, inv_y, inv_theta = transform_inv(odom_x, odom_y, odom_theta)
        map_odom_x, map_odom_y, map_odom_theta = transform_mul(est_x, est_y, est_theta, inv_x, inv_y, inv_theta)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.translation.x = map_odom_x
        t.transform.translation.y = map_odom_y
        q = quaternion_from_euler(0, 0, map_odom_theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

    def update(self):
        if not self.initialized or self.current_odom is None:
            return

        # Obtener odometría actual
        odom_x, odom_y, odom_theta = pose_to_xytheta(self.current_odom.pose.pose)

        if self.prev_odom is None:
            self.prev_odom = (odom_x, odom_y, odom_theta)
            self.publish_particles()
            return

        dx = odom_x - self.prev_odom[0]
        dy = odom_y - self.prev_odom[1]
        dtheta = normalize_angle(odom_theta - self.prev_odom[2])
        self.prev_odom_theta = self.prev_odom[2]  # guardar ANTES de actualizar
        self.prev_odom = (odom_x, odom_y, odom_theta)

        # G. Predicción (Dead Reckoning)
        if abs(dx) > 0.0001 or abs(dy) > 0.0001 or abs(dtheta) > 0.0001:
            self.predict(dx, dy, dtheta)

        # E. Corrección (score por suma de píxeles)
        self.compute_weights()

        # F. Filtrado / Resampling
        self.resample()

        # H. Estimación de pose
        est_x, est_y, est_theta = self.estimate_pose()

        # Detector de salto imposible: si la estimación se mueve mucho más
        # que el odom, es porque el filtro "teletransportó" a otra zona ambigua.
        # Forzar reinit alrededor del odom para recuperarse.
        if self.prev_est is not None:
            est_step = math.hypot(est_x - self.prev_est[0], est_y - self.prev_est[1])
            odom_step = math.hypot(dx, dy)
            max_step = max(self.max_jump_min, self.max_jump_factor * odom_step)
            if est_step > max_step:
                self.get_logger().warn(
                    f'Salto detectado: MCL={est_step:.2f} m vs odom={odom_step:.3f} m. Forzando reinit.'
                )
                self._reinit_around_odom()
                est_x, est_y, est_theta = self.estimate_pose()

        self.prev_est = (est_x, est_y, est_theta)

        # Publicaciones
        self.publish_particles()
        self.publish_estimated_pose(est_x, est_y, est_theta)
        self.publish_tf(est_x, est_y, est_theta)


def main():
    rclpy.init()
    node = MonteCarloLocalization()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
