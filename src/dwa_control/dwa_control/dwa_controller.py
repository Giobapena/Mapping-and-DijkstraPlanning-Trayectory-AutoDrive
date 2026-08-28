#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
dwa_controller.py — Dynamic Window Approach para F1TENTH
Proyecto 2do Parcial (Parte 2): Seguimiento de trayectorias y control

Caracteristicas:
  - Deteccion AUTOMATICA de topicos (AutoDRIVE o f1tenth_gym_ros)
  - Calibracion AUTOMATICA del mapeo velocidad -> traccion
  - Contador de vueltas + cronometro por vuelta impresos en terminal
  - Exporta los tiempos a CSV como evidencia

Autor: Giovanny Andres Bano Pena
"""

import csv
import glob
import math
import os

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                       DurabilityPolicy)

from std_msgs.msg import Float32, Int32
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry, Path

try:
    from ackermann_msgs.msg import AckermannDriveStamped
    HAVE_ACKERMANN = True
except ImportError:
    HAVE_ACKERMANN = False


# --------------------------------------------------------------------------- #
def yaw_from_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def find_csv(preferred):
    """Devuelve el CSV de trayectoria: el indicado, o el mas reciente que exista."""
    p = os.path.expanduser(preferred)
    if os.path.isfile(p):
        return p
    pats = ["~/Mapping-and-DijkstraPlanning-Trayectory-AutoDrive/**/*.csv",
            "~/**/waypoints/*.csv", "~/autodrive_ws/**/*.csv", "~/*.csv"]
    cands = []
    for pat in pats:
        cands += glob.glob(os.path.expanduser(pat), recursive=True)
    key = ("smooth", "waypoint", "dijkstra", "path", "trayect")
    cands = [c for c in cands if any(k in os.path.basename(c).lower() for k in key)]
    if not cands:
        raise RuntimeError(
            f"No se encontro el CSV de trayectoria.\n"
            f"  Buscado: {p}\n"
            f"  Pasa la ruta correcta con:  -p path_csv:=/ruta/a/tu.csv")
    return max(cands, key=os.path.getmtime)


def load_path_csv(filename):
    """Carga waypoints (x,y). Tolera cabecera, separador ',' o ';' y columnas extra."""
    pts = []
    with open(filename, "r") as f:
        sample = f.read(4096)
        f.seek(0)
        delim = ";" if sample.count(";") > sample.count(",") else ","
        for row in csv.reader(f, delimiter=delim):
            if len(row) < 2:
                continue
            try:
                pts.append((float(row[0]), float(row[1])))
            except ValueError:
                continue
    if len(pts) < 3:
        raise RuntimeError(f"CSV sin waypoints validos: {filename}")
    p = np.asarray(pts, dtype=np.float64)
    # elimina duplicados consecutivos
    keep = np.concatenate([[True], np.linalg.norm(np.diff(p, axis=0), axis=1) > 1e-6])
    return p[keep]


# --------------------------------------------------------------------------- #
class DWAController(Node):

    def __init__(self):
        super().__init__("dwa_controller")
        p = self.declare_parameter

        # --- Entradas -----------------------------------------------------
        p("path_csv", "~/Mapping-and-DijkstraPlanning-Trayectory-AutoDrive/src/global_planner/waypoints/dijkstra_waypoints_smooth.csv")
        p("sim_backend", "auto")        # auto | autodrive | gym
        p("vehicle_ns", "")             # vacio -> autodeteccion
        p("control_rate", 25.0)

        # --- Vehiculo -----------------------------------------------------
        p("wheelbase", 0.324)
        p("max_steer", 0.5236)
        p("robot_radius", 0.18)
        p("lidar_offset_x", 0.10)
        p("steering_sign", 1.0)

        # --- Ventana dinamica ---------------------------------------------
        p("v_min", 0.0)          # 0 = puede frenar/parar; >0 nunca permite detenerse
        p("v_max", 2.0)
        p("w_max", 3.5)
        p("accel_max", 4.0)
        p("dw_max", 6.0)
        p("v_samples", 9)
        p("w_samples", 21)
        p("predict_time", 1.0)
        p("sim_dt", 0.10)

        # --- Pesos de costo -----------------------------------------------
        p("w_path", 4.0)
        p("w_goal", 1.5)
        p("w_obs", 2.5)
        p("w_speed", 0.8)
        p("lookahead_min", 0.8)
        p("lookahead_gain", 0.25)
        p("lookahead_max", 2.0)
        p("obs_max_range", 4.0)
        p("lidar_stride", 3)
        p("w_head", 3.0)            # alineacion con la tangente de la ruta
        p("obs_hard", False)        # False -> LiDAR como costo, no como veto
        p("obs_min_range", 0.40)     # descarta autorretornos del chasis
        p("fov_deg", 200.0)          # sector frontal considerado

        # --- Control longitudinal (solo AutoDRIVE) ------------------------
        p("throttle_kp", 0.25)
        p("throttle_ki", 0.35)
        p("v_full_init", 8.0)           # estimacion inicial, se autoajusta
        p("brake_gain", 0.5)

        # --- Vueltas ------------------------------------------------------
        p("total_laps", 10)
        p("min_lap_time", 3.0)
        p("results_csv", "~/dwa_lap_times.csv")

        # --- Diagnostico / seguridad ---------------------------------------
        p("diag_period", 20)         # cada cuantos ciclos se imprime el estado
        p("stall_speed", 0.08)       # v por debajo de esto cuenta como "detenido"
        p("stall_timeout", 1.5)      # s detenido pidiendo avance -> frena y avisa

        g = lambda n: self.get_parameter(n).value
        self.L = g("wheelbase")
        self.max_steer = g("max_steer")
        self.robot_radius = g("robot_radius")
        self.lidar_off_x = g("lidar_offset_x")
        self.steer_sign = g("steering_sign")
        self.v_min, self.v_max = g("v_min"), g("v_max")
        self.w_max = g("w_max")
        self.a_max, self.dw_max = g("accel_max"), g("dw_max")
        self.nv, self.nw = int(g("v_samples")), int(g("w_samples"))
        self.T, self.dt_sim = g("predict_time"), g("sim_dt")
        self.k_path, self.k_goal = g("w_path"), g("w_goal")
        self.k_obs, self.k_speed = g("w_obs"), g("w_speed")
        self.ld_min, self.ld_k, self.ld_max = (g("lookahead_min"),
                                               g("lookahead_gain"),
                                               g("lookahead_max"))
        self.obs_max_range = g("obs_max_range")
        self.stride = max(1, int(g("lidar_stride")))
        self.k_head = g("w_head")
        self.obs_hard = bool(g("obs_hard"))
        self.obs_min_range = g("obs_min_range")
        self.fov = math.radians(g("fov_deg")) / 2.0
        self.scan_logged = False
        self.kp_thr, self.ki_thr = g("throttle_kp"), g("throttle_ki")
        self.v_full = float(g("v_full_init"))
        self.brake_gain = g("brake_gain")
        self.total_laps = int(g("total_laps"))
        self.min_lap_time = g("min_lap_time")
        self.results_csv = os.path.expanduser(g("results_csv"))
        self.backend_req = str(g("sim_backend")).lower()
        self.ns_req = g("vehicle_ns")
        self.rate = g("control_rate")
        self.diag_period = max(1, int(g("diag_period")))
        self.stall_speed = g("stall_speed")
        self.stall_timeout = g("stall_timeout")
        self.stall_since = None

        # --- Trayectoria ---------------------------------------------------
        csv_file = find_csv(g("path_csv"))
        self.path = load_path_csv(csv_file)
        self.N = len(self.path)
        per = float(np.linalg.norm(
            np.diff(self.path, axis=0, append=self.path[:1]), axis=1).sum())
        self.ds = max(per / self.N, 1e-3)     # espaciado medio entre waypoints
        self.diag = 0
        self.get_logger().info(
            f"Trayectoria: {self.N} waypoints | perimetro {per:.1f} m | "
            f"espaciado {self.ds*100:.1f} cm | {csv_file}")

        # --- Estado --------------------------------------------------------
        self.x = self.y = self.yaw = None
        self.v = 0.0
        self.w = 0.0
        self.obs = np.zeros((0, 2))
        self.idx = 0
        self.prev_idx = 0
        self.have_pose = False
        self.thr_i = 0.0
        self.last_thr = 0.0
        self.have_speed_topic = False
        self._lxy = None
        self._lt = None

        self.dir_checked = False
        self.lap = 0
        self.lap_start = None
        self.race_start = None
        self.lap_times = []
        self.best_lap = None
        self.finished = False
        self.progress = 0.0     # avance neto (en waypoints) desde el ultimo cruce

        # --- Publicadores comunes -----------------------------------------
        self.pub_lap = self.create_publisher(Int32, "/lap_count", 10)
        self.pub_lap_t = self.create_publisher(Float32, "/lap_time", 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_path = self.create_publisher(Path, "/global_path", latched)

        # --- Autodeteccion de la interfaz ----------------------------------
        self.backend = None
        self.ready = False
        self.tries = 0
        self.create_timer(1.0, self.detect)

    # ---------------------- Deteccion de topicos --------------------------- #
    def detect(self):
        if self.ready:
            return
        self.tries += 1
        topics = {n: t for n, t in self.get_topic_names_and_types()}

        def find(typ, *keys, exclude=()):
            hits = [n for n, ts in topics.items()
                    if typ in ts and not any(e in n for e in exclude)]
            for k in keys:
                for h in hits:
                    if k in h:
                        return h
            return hits[0] if hits else None

        scan = find("sensor_msgs/msg/LaserScan", "lidar", "scan", exclude=("opp_",))
        thr = find("std_msgs/msg/Float32", "throttle_command")
        strg = find("std_msgs/msg/Float32", "steering_command")
        ips = find("geometry_msgs/msg/Point", "ips")
        imu = find("sensor_msgs/msg/Imu", "imu", exclude=("opp_",))
        odom = find("nav_msgs/msg/Odometry", "ego_racecar/odom", "odom",
                    exclude=("opp_",))
        drive = find("ackermann_msgs/msg/AckermannDriveStamped", "drive",
                     exclude=("opp_",))

        f = self.backend_req
        use_ad = (f == "autodrive") or (f == "auto" and bool(thr and strg))
        use_gym = (f == "gym") or (f == "auto" and not use_ad and bool(drive))

        missing = []
        if scan is None:
            missing.append("LiDAR")
        if use_ad:
            if not (thr and strg):
                missing.append("throttle_command/steering_command")
            if not (ips or odom):
                missing.append("pose (ips u odom)")
        elif use_gym:
            if not drive:
                missing.append("/drive")
            if not odom:
                missing.append("odom")
        else:
            missing.append("comandos del vehiculo")

        if missing:
            if self.tries in (5, 15, 30):
                self.get_logger().warn(
                    f"Esperando al simulador. Falta: {', '.join(missing)}")
            if self.tries == 45:
                self.get_logger().error(
                    "Sin topicos tras 45 s. Verifica con: ros2 topic list")
            return

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(LaserScan, scan, self.cb_lidar, qos)

        if use_ad:
            self.backend = "autodrive"
            self.pub_thr = self.create_publisher(Float32, thr, 10)
            self.pub_str = self.create_publisher(Float32, strg, 10)
            if ips and imu:
                self.create_subscription(Point, ips, self.cb_ips, qos)
                self.create_subscription(Imu, imu, self.cb_imu, qos)
                pose_src = f"{ips} + {imu}"
            else:
                self.create_subscription(Odometry, odom, self.cb_odom, qos)
                pose_src = odom
            spd = find("std_msgs/msg/Float32", "/speed")
            if spd and spd.endswith("/speed"):
                self.create_subscription(Float32, spd, self.cb_speed, qos)
            self.get_logger().info("Backend: AutoDRIVE")
            self.get_logger().info(f"  traccion  : {thr}")
            self.get_logger().info(f"  direccion : {strg}")
            self.get_logger().info(f"  pose      : {pose_src}")
            self.get_logger().info(f"  lidar     : {scan}")
        else:
            if not HAVE_ACKERMANN:
                self.get_logger().error(
                    "Falta ackermann_msgs: "
                    "sudo apt install ros-humble-ackermann-msgs")
                return
            self.backend = "gym"
            self.pub_drive = self.create_publisher(AckermannDriveStamped, drive, 10)
            self.create_subscription(Odometry, odom, self.cb_odom, qos)
            self.get_logger().info("Backend: f1tenth_gym_ros")
            self.get_logger().info(f"  drive: {drive} | odom: {odom} | scan: {scan}")

        self.publish_global_path()
        self.create_timer(1.0 / self.rate, self.control_loop)
        self.ready = True
        self.get_logger().info("Control DWA activo. Esperando primera pose...")

    # ----------------------------- Callbacks ------------------------------- #
    def cb_ips(self, msg: Point):
        t = self.get_clock().now().nanoseconds * 1e-9
        if self._lxy is not None and self._lt is not None:
            dt = t - self._lt
            if dt > 1e-3 and not self.have_speed_topic:
                d = math.hypot(msg.x - self._lxy[0], msg.y - self._lxy[1])
                self.v = 0.75 * self.v + 0.25 * (d / dt)
        self._lxy, self._lt = (msg.x, msg.y), t
        self.x, self.y = msg.x, msg.y
        self.have_pose = self.yaw is not None

    def cb_speed(self, msg: Float32):
        self.have_speed_topic = True
        self.v = float(msg.data)

    def cb_imu(self, msg: Imu):
        q = msg.orientation
        self.yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        self.w = msg.angular_velocity.z
        self.have_pose = self.x is not None

    def cb_odom(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        self.v = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        self.w = msg.twist.twist.angular.z
        self.have_pose = True

    def cb_lidar(self, msg: LaserScan):
        n = len(msg.ranges)
        r = np.asarray(msg.ranges[::self.stride], dtype=np.float64)
        ang = msg.angle_min + np.arange(0, n, self.stride) * msg.angle_increment
        m = min(len(r), len(ang))
        r, ang = r[:m], ang[:m]
        if not self.scan_logged:
            self.scan_logged = True
            rr = np.asarray(msg.ranges, dtype=np.float64)
            rr = rr[np.isfinite(rr) & (rr > 0)]
            self.get_logger().info(
                f"LiDAR: {n} rayos | FOV "
                f"[{math.degrees(msg.angle_min):.0f}, "
                f"{math.degrees(msg.angle_max):.0f}] deg | "
                f"rango [{msg.range_min:.2f}, {msg.range_max:.2f}] | "
                f"medido min={rr.min():.2f} max={rr.max():.2f} m")
        ok = (np.isfinite(r) & (r > max(msg.range_min, self.obs_min_range))
              & (r < self.obs_max_range) & (np.abs(ang) < self.fov))
        r, ang = r[ok], ang[ok]
        self.obs = (np.zeros((0, 2)) if r.size == 0 else
                    np.stack([r * np.cos(ang) + self.lidar_off_x,
                              r * np.sin(ang)], axis=1))

    # --------------------------- Ruta / objetivo --------------------------- #
    def nearest_index(self):
        back = max(3, int(1.0 / self.ds))     # 1 m atras
        fwd = max(10, int(4.0 / self.ds))     # 4 m adelante
        idxs = (self.idx + np.arange(-back, fwd)) % self.N
        d = np.hypot(self.path[idxs, 0] - self.x, self.path[idxs, 1] - self.y)
        if d.min() > 3.0:                      # rescate: busqueda global
            dall = np.hypot(self.path[:, 0] - self.x, self.path[:, 1] - self.y)
            return int(np.argmin(dall))
        return int(idxs[int(np.argmin(d))])

    def local_goal(self):
        ld = min(self.ld_max, self.ld_min + self.ld_k * self.v)
        acc, j = 0.0, self.idx
        while acc < ld:
            k = (j + 1) % self.N
            acc += float(np.hypot(*(self.path[k] - self.path[j])))
            j = k
            if j == self.idx:
                break
        dx, dy = self.path[j, 0] - self.x, self.path[j, 1] - self.y
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return c * dx + s * dy, -s * dx + c * dy

    def local_path_window(self):
        # debe extenderse mas alla de lo que simula el DWA
        n = min(self.N, max(10, int((self.ld_max + self.v_max * self.T + 1.0)
                                    / self.ds)))
        step = max(1, n // 60)                # submuestreo: ~60 puntos
        idxs = (self.idx + np.arange(-2, n, step)) % self.N
        dx = self.path[idxs, 0] - self.x
        dy = self.path[idxs, 1] - self.y
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return np.stack([c * dx + s * dy, -s * dx + c * dy], axis=1)

    # -------------------------------- DWA ---------------------------------- #
    def dwa_plan(self):
        dtc = 1.0 / self.rate
        v_lo = max(self.v_min, self.v - self.a_max * dtc)
        v_hi = max(v_lo, min(self.v_max, self.v + self.a_max * dtc))
        w_lo = max(-self.w_max, self.w - self.dw_max * dtc)
        w_hi = min(self.w_max, self.w + self.dw_max * dtc)

        # |w| <= v*tan(delta_max)/L : no simular giros que el Ackermann no puede
        vs = np.linspace(v_lo, v_hi, self.nv)
        w_kin = np.minimum(self.w_max, vs * math.tan(self.max_steer) / self.L)
        frac = np.linspace(-1.0, 1.0, self.nw)
        W = np.clip((w_kin[:, None] * frac[None, :]).ravel(), w_lo, w_hi)
        V = np.repeat(vs, self.nw)
        S = V.size
        K = max(2, int(self.T / self.dt_sim))

        xs = np.zeros((S, K)); ys = np.zeros((S, K)); ths = np.zeros((S, K))
        th = np.zeros(S); px = np.zeros(S); py = np.zeros(S)
        for k in range(K):
            th = th + W * self.dt_sim
            px = px + V * np.cos(th) * self.dt_sim
            py = py + V * np.sin(th) * self.dt_sim
            xs[:, k], ys[:, k], ths[:, k] = px, py, th
        traj = np.stack([xs, ys], axis=2)

        # Huella: eje trasero y eje delantero, no un punto
        if self.obs.shape[0]:
            dmin = np.full(S, 1e3)
            for off in (0.0, self.L):
                body = np.stack([xs + off * np.cos(ths),
                                 ys + off * np.sin(ths)], axis=2)
                d = np.linalg.norm(body[:, :, None, :] - self.obs[None, None, :, :],
                                   axis=3)
                dmin = np.minimum(dmin, d.min(axis=(1, 2)))
        else:
            dmin = np.full(S, 1e3)
        collide = dmin < self.robot_radius
        c_obs = 1.0 / np.clip(dmin, 1e-3, None)

        pw = self.local_path_window()
        c_path = np.linalg.norm(traj[:, :, None, :] - pw[None, None, :, :],
                                axis=3).min(axis=2).mean(axis=1)

        gx, gy = self.local_goal()
        c_goal = np.hypot(xs[:, -1] - gx, ys[:, -1] - gy)

        # Rumbo de la ruta en el punto mas cercano al final de cada trayectoria
        end = np.stack([xs[:, -1], ys[:, -1]], axis=1)
        j = np.argmin(np.linalg.norm(end[:, None, :] - pw[None, :, :], axis=2),
                      axis=1)
        j = np.minimum(j, len(pw) - 2)
        tang = pw[j + 1] - pw[j]
        c_head = np.abs(np.arctan2(
            np.sin(ths[:, -1] - np.arctan2(tang[:, 1], tang[:, 0])),
            np.cos(ths[:, -1] - np.arctan2(tang[:, 1], tang[:, 0]))))
        c_speed = (self.v_max - V) / max(self.v_max, 1e-3)

        nrm = lambda a: (a - a.min()) / (np.ptp(a) + 1e-9)
        cost = (self.k_path * nrm(c_path) + self.k_goal * nrm(c_goal)
                + self.k_head * nrm(c_head)
                + self.k_obs * nrm(c_obs) + self.k_speed * nrm(c_speed))
        if self.obs_hard:
            cost[collide] = np.inf
        else:
            # La ruta global ya es libre de colisiones: el LiDAR solo penaliza
            cost = cost + 5.0 * collide.astype(float)
        self._free = int(np.isfinite(cost).sum())
        self._dmax = float(dmin.max())
        if not np.isfinite(cost).any():
            # Nada libre: frena de verdad, sin piso de v_min (emergencia)
            i = int(np.argmax(dmin))          # la menos mala, despacio
            return float(0.4 * V[i]), float(W[i])
        i = int(np.argmin(cost))
        return float(V[i]), float(W[i])

    # ------------------------ Vueltas y cronometro ------------------------- #
    def update_laps(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.race_start is None:
            self.race_start = self.lap_start = now
            self.get_logger().info("Cronometro iniciado.")

        # Avance neto en waypoints desde el ultimo cruce, no un booleano de
        # "prev al final, actual al inicio": una oscilacion justo en la
        # costura (p. ej. atascado en la parte mas angosta de la pista,
        # que coincide con la linea de meta) cruzaba ese umbral varias
        # veces en pocos segundos sin haber recorrido la vuelta completa,
        # contando vueltas falsas de ~5-6 s.
        delta = self.idx - self.prev_idx
        if delta < -self.N / 2:
            delta += self.N          # avanzo cruzando la costura
        elif delta > self.N / 2:
            delta -= self.N          # retrocedio cruzando la costura
        self.progress = max(0.0, self.progress + delta)

        if self.progress >= 0.95 * self.N and (now - self.lap_start) > self.min_lap_time:
            self.progress -= self.N
            t = now - self.lap_start
            self.lap += 1
            self.lap_times.append(t)
            self.best_lap = t if self.best_lap is None else min(self.best_lap, t)
            self.lap_start = now
            self.get_logger().info(
                f"VUELTA {self.lap:2d}/{self.total_laps} | Tiempo: {t:7.3f} s"
                f" | Mejor: {self.best_lap:7.3f} s"
                f" | Total: {now - self.race_start:8.3f} s")
            self.pub_lap.publish(Int32(data=self.lap))
            self.pub_lap_t.publish(Float32(data=float(t)))
            if self.lap >= self.total_laps and not self.finished:
                self.finished = True
                self.report(now)
        self.prev_idx = self.idx

    def report(self, now):
        prom = sum(self.lap_times) / len(self.lap_times)
        log = self.get_logger()
        log.info("=" * 64)
        log.info(f"  CARRERA FINALIZADA - {self.lap} vueltas sin colision")
        log.info(f"  Mejor vuelta    : {self.best_lap:.3f} s")
        log.info(f"  Vuelta promedio : {prom:.3f} s")
        log.info(f"  Tiempo total    : {now - self.race_start:.3f} s")
        log.info("=" * 64)
        try:
            with open(self.results_csv, "w", newline="") as f:
                wtr = csv.writer(f)
                wtr.writerow(["vuelta", "tiempo_s"])
                for i, t in enumerate(self.lap_times, 1):
                    wtr.writerow([i, f"{t:.3f}"])
                wtr.writerow(["mejor", f"{self.best_lap:.3f}"])
                wtr.writerow(["promedio", f"{prom:.3f}"])
            log.info(f"  Tiempos guardados en {self.results_csv}")
        except OSError as e:
            log.warn(f"No se pudo escribir {self.results_csv}: {e}")

    # -------------------------------- Bucle -------------------------------- #
    def control_loop(self):
        if not self.have_pose or self.yaw is None:
            return
        if not self.dir_checked:
            self.check_direction()
            return
        self.idx = self.nearest_index()
        self.update_laps()

        if self.k_obs <= 0.0:
            self.pure_follow()
            return
        v_cmd, w_cmd = self.dwa_plan()
        self.diag += 1
        if self.diag % self.diag_period == 0:
            lid = float(np.linalg.norm(self.obs, axis=1).min()) if len(self.obs) else -1
            cte = float(np.hypot(self.path[self.idx, 0] - self.x,
                                 self.path[self.idx, 1] - self.y))
            self.get_logger().info(
                f"v={self.v:.2f} cmd={v_cmd:.2f} idx={self.idx}/{self.N} "
                f"libres={getattr(self,'_free',0)}/{self.nv*self.nw} "
                f"holgura={getattr(self,'_dmax',0):.2f}m lidar_min={lid:.2f}m "
                f"error_lateral={cte:.2f}m")
        if v_cmd is None:
            self.send(0.0, 0.0, brake=True)
            self.get_logger().warn("Sin trayectoria libre: frenando",
                                   throttle_duration_sec=1.0)
            return
        delta = math.atan2(w_cmd * self.L, max(v_cmd, 0.3))
        self.send(v_cmd, max(-self.max_steer, min(self.max_steer, delta)))

    def pure_follow(self):
        """Seguimiento geometrico puro del CSV. Ignora el LiDAR."""
        ld = min(self.ld_max, max(self.ld_min, self.ld_min + self.ld_k * self.v))
        acc, j = 0.0, self.idx
        while acc < ld:
            k = (j + 1) % self.N
            acc += float(np.hypot(*(self.path[k] - self.path[j])))
            j = k
            if j == self.idx:
                break
        dx, dy = self.path[j, 0] - self.x, self.path[j, 1] - self.y
        c, sn = math.cos(self.yaw), math.sin(self.yaw)
        lx, ly = c * dx + sn * dy, -sn * dx + c * dy
        Ld = max(math.hypot(lx, ly), 1e-3)
        delta = math.atan2(2.0 * self.L * ly, Ld * Ld)      # pure pursuit
        delta = max(-self.max_steer, min(self.max_steer, delta))

        # frenar en curva: cuanto mas giro, menos velocidad
        v_cmd = self.v_max * (1.0 - 0.7 * abs(delta) / self.max_steer)
        v_cmd = max(self.v_min, v_cmd)

        self.diag += 1
        if self.diag % self.diag_period == 0:
            cte = float(np.hypot(self.path[self.idx, 0] - self.x,
                                 self.path[self.idx, 1] - self.y))
            self.get_logger().info(
                f"v={self.v:.2f} cmd={v_cmd:.2f} dir={math.degrees(delta):+.0f}d "
                f"idx={self.idx}/{self.N} error_lateral={cte:.2f}m")
        self.send(v_cmd, delta)

    def check_direction(self):
        """Invierte la ruta si su sentido es opuesto al del vehiculo al arrancar."""
        d = np.hypot(self.path[:, 0] - self.x, self.path[:, 1] - self.y)
        i = int(np.argmin(d))
        tang = self.path[(i + 1) % self.N] - self.path[i]
        head = np.array([math.cos(self.yaw), math.sin(self.yaw)])
        if float(np.dot(tang, head)) < 0:
            self.path = np.ascontiguousarray(self.path[::-1])
            i = self.N - 1 - i
            self.get_logger().info(
                "Trayectoria invertida: su sentido era opuesto al del vehiculo")
            self.publish_global_path()
        self.path = np.roll(self.path, -i, axis=0)   # waypoint 0 = linea de meta
        self.idx = self.prev_idx = 0
        self.dir_checked = True
        self.get_logger().info(
            f"Linea de meta fijada en x={self.x:.3f} y={self.y:.3f}")

    def send(self, v_cmd, delta, brake=False):
        delta = float(self.steer_sign * delta)
        self._check_stall(v_cmd, brake)
        if self.backend == "gym":
            m = AckermannDriveStamped()
            m.header.stamp = self.get_clock().now().to_msg()
            m.drive.speed = 0.0 if brake else float(v_cmd)
            m.drive.steering_angle = delta
            self.pub_drive.publish(m)
            return

        if brake:
            thr = 0.0
            self.thr_i = 0.0
        else:
            # Autocalibracion del mapeo traccion <-> velocidad
            if self.v > 1.0 and self.last_thr > 0.15:
                est = self.v / self.last_thr
                self.v_full = float(np.clip(0.995 * self.v_full + 0.005 * est,
                                            2.0, 40.0))
            e = v_cmd - self.v
            self.thr_i = float(np.clip(self.thr_i + self.ki_thr * e / self.rate,
                                       -0.5, 0.5))
            thr = v_cmd / self.v_full + self.kp_thr * e + self.thr_i
        thr = float(np.clip(thr, -1.0, 1.0))
        self.last_thr = thr
        self.pub_thr.publish(Float32(data=thr))
        self.pub_str.publish(Float32(data=float(delta / self.max_steer)))

    def _check_stall(self, v_cmd, brake):
        """Avisa si se pide avanzar pero la velocidad medida no responde:
        el vehiculo esta empotrado contra algo que el LiDAR no ve (p. ej.
        muy cerca, filtrado por obs_min_range) o quedo atascado."""
        now = self.get_clock().now().nanoseconds * 1e-9
        if brake or v_cmd < self.stall_speed:
            self.stall_since = None
            return
        if self.v < self.stall_speed:
            if self.stall_since is None:
                self.stall_since = now
            elif now - self.stall_since > self.stall_timeout:
                self.get_logger().warn(
                    f"ATASCADO: se pide v_cmd={v_cmd:.2f} m/s pero v medida="
                    f"{self.v:.2f} m/s desde hace {now - self.stall_since:.1f} s. "
                    f"Probable colision fuera del rango que ve el LiDAR "
                    f"(obs_min_range={self.obs_min_range:.2f} m). Revisa/reposiciona "
                    f"el vehiculo en el simulador.",
                    throttle_duration_sec=3.0)
        else:
            self.stall_since = None

    def publish_global_path(self):
        m = Path()
        m.header.frame_id = "map"
        m.header.stamp = self.get_clock().now().to_msg()
        for x, y in self.path:
            ps = PoseStamped()
            ps.header = m.header
            ps.pose.position.x, ps.pose.position.y = float(x), float(y)
            ps.pose.orientation.w = 1.0
            m.poses.append(ps)
        self.pub_path.publish(m)

    def stop(self):
        try:
            if self.backend == "gym":
                m = AckermannDriveStamped()
                m.header.stamp = self.get_clock().now().to_msg()
                self.pub_drive.publish(m)
            elif self.backend == "autodrive":
                self.pub_thr.publish(Float32(data=0.0))
                self.pub_str.publish(Float32(data=0.0))
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = DWAController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node.lap_times and not node.finished:
            node.report(node.get_clock().now().nanoseconds * 1e-9)
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
