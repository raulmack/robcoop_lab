"""Ir a un punto con evitación reactiva (proporcionado, no hay que modificarlo).

Se usa el mismo PUNTO DE CONTROL que en la P1 (10 cm delante del eje, ver kinematics.py):

  1. Atracción hacia el objetivo:   u_obj = k_obj · (objetivo - q), saturada a V_MAX.
  2. Evitación con el LiDAR: los puntos del scan cercanos empujan al robot hacia fuera
     (repulsión) y le hacen rodear el obstáculo (componente tangencial). Si el obstáculo está
     justo delante, siempre por la DERECHA: dos robots que se cruzan de frente se esquivan
     sin hablar entre ellos. Si no, por el lado que acerca al objetivo.
  3. u = u_obj + u_evit, saturada, y conversión a (v, ω) como en la P1.
  4. Si el robot deja de acercarse al objetivo durante unos segundos (atrapado detrás de
     otro robot parado), prueba a rodearlo por el otro lado durante un rato; si tampoco
     sirve (bloqueo entre varios), retrocede un poco y espera según su prioridad.

El filtro de seguridad (safety.py) sigue actuando después: parada si hay algo a < 0.20 m.
"""
import math

from .kinematics import D_CONTROL, V_MAX, control_point, integrator_to_unicycle, saturate

K_GOAL = 0.6           # 1/s, ganancia de atracción
GOAL_TOL = 0.08        # m, distancia del punto de control al objetivo para darlo por alcanzado
R_AVOID = 0.45         # m, radio de influencia de los obstáculos (medido desde el LiDAR)
R_CLOSE = 0.30         # m, detrás o al lado solo cuentan los obstáculos más cercanos
K_REP = 0.10           # ganancia de repulsión
K_TAN = 0.08           # m/s, velocidad tangencial máxima
FRONT = math.radians(100)   # semiancho del sector delantero
AHEAD = math.radians(30)    # obstáculo «justo delante»: siempre por la derecha
STALL_TIME = 4.0       # s sin acercarse al objetivo -> se cambia de lado
DETOUR_TIME = 5.0      # s rodeando por el lado contrario
PROGRESS = 0.05        # m que hay que mejorar para considerar que avanza
BACKUP_TIME = 2.0      # s marcha atrás si cambiar de lado no basta (bloqueo entre varios)
BACKUP_V = -0.06       # m/s
YIELD_TIME = 1.5       # s de espera por cada puesto de prioridad tras la marcha atrás

RIGHT, LEFT = 1, -1


def scan_points(scan, max_range=R_AVOID, step=2):
    """Puntos (x, y) del scan en el marco del robot que están a menos de max_range.

    step: se usa uno de cada 'step' rayos (360 rayos -> 180 puntos, suficiente).
    """
    if scan is None:
        return []
    pts = []
    rmin = max(scan.range_min, 0.05) + 0.005      # lecturas en el mínimo: no válidas
    for i in range(0, len(scan.ranges), step):
        r = scan.ranges[i]
        if math.isfinite(r) and rmin < r < max_range:
            a = scan.angle_min + i * scan.angle_increment
            pts.append((r * math.cos(a), r * math.sin(a)))
    return pts


def avoidance_velocity(theta, obstacles, goal_dir=(0.0, 0.0), side=None, r_avoid=R_AVOID):
    """Velocidad de evitación en 'map' a partir de los obstáculos (marco del robot).

    goal_dir: vector unitario hacia el objetivo en 'map'.
    side: None (automático), RIGHT o LEFT (forzado).
    Devuelve (u_x, u_y, lado usado o None si no hay obstáculos delante).
    """
    awx = awy = 0.0            # dirección media de alejamiento (ponderada por cercanía)
    rep = tan = 0.0            # intensidad de la repulsión y de la componente tangencial
    d_near, a_near = math.inf, 0.0
    c, s = math.cos(theta), math.sin(theta)
    for ox, oy in obstacles:
        d = math.hypot(ox, oy)
        a = math.atan2(oy, ox)
        front = abs(a) <= FRONT
        # delante: todo el radio de influencia; detrás o al lado: solo si está muy cerca
        if d < 1e-3 or d >= r_avoid or (not front and d >= R_CLOSE):
            continue
        ax, ay = -ox / d, -oy / d                    # alejarse del obstáculo (marco robot)
        closeness = 1.0 - d / r_avoid
        awx += closeness * (c * ax - s * ay)         # ... en 'map'
        awy += closeness * (s * ax + c * ay)
        # el punto más cercano manda: un robot visto con 6 rayos no empuja 6 veces más
        rep = max(rep, K_REP * (1.0 / d - 1.0 / r_avoid))
        if front:
            tan = max(tan, K_TAN * closeness)
            if d < d_near:
                d_near, a_near = d, a
    n = math.hypot(awx, awy)
    if n < 1e-9:
        return 0.0, 0.0, None
    wx, wy = awx / n, awy / n
    # «por la derecha» = alejamiento girado +90° (con el obstáculo delante, apunta a la
    # derecha del robot)
    tx, ty = -wy, wx
    if side is None:
        side = RIGHT
        if abs(a_near) > AHEAD and tx * goal_dir[0] + ty * goal_dir[1] < -0.1:
            side = LEFT
    if side == LEFT:
        tx, ty = -tx, -ty
    ux, uy = saturate(rep * wx + tan * tx, rep * wy + tan * ty, K_TAN + V_MAX)
    return ux, uy, (side if tan > 0 else None)


def go_to_point(pose, goal, obstacles=(), side=None, k=K_GOAL, vmax=V_MAX, d=D_CONTROL):
    """Orden (v, ω) para llevar el punto de control al objetivo.

    pose: (x, y, θ) en 'map'; goal: (x, y); obstacles: puntos del scan en el marco del robot.
    Devuelve (v, ω, distancia que falta, lado por el que esquiva o None).
    """
    x, y, th = pose
    qx, qy = control_point(x, y, th, d)
    ex, ey = goal[0] - qx, goal[1] - qy
    dist = math.hypot(ex, ey)
    ux, uy = saturate(k * ex, k * ey, vmax)
    gdir = (ex / dist, ey / dist) if dist > 1e-6 else (0.0, 0.0)
    ax, ay, used = avoidance_velocity(th, obstacles, gdir, side)
    # cerca del objetivo la evitación pesa menos (si no, un robot parado al lado lo impide)
    w_av = min(1.0, dist / 0.3)
    ux, uy = saturate(ux + w_av * ax, uy + w_av * ay, vmax)
    v, w = integrator_to_unicycle(ux, uy, th, d)
    return v, w, dist, used


def _rear_blocked(obstacles, dist=0.35):
    """¿Hay algo detrás (mitad trasera) a menos de 'dist'? Entonces no se retrocede."""
    return any(math.hypot(x, y) < dist and x < 0 for x, y in obstacles)


class GoToPoint:
    """go_to_point con memoria: detecta si el robot se ha quedado atascado y cambia de lado.

        nav = GoToPoint(priority=i)                     # i: número del robot
        nav.set_goal((x, y))
        v, w, dist = nav.command(pose, obstacles, t)     # t en segundos

    Si dos cambios de lado seguidos no sirven (varios robots bloqueados entre sí), el robot
    retrocede un poco y espera un tiempo que depende de su prioridad, para romper la simetría.
    """

    def __init__(self, priority=0, stall_time=STALL_TIME, detour_time=DETOUR_TIME):
        self.priority = priority
        self.stall_time = stall_time
        self.detour_time = detour_time
        self.detours = 0                 # veces que ha tenido que cambiar de lado
        self.backups = 0                 # veces que ha tenido que retroceder
        self.set_goal((0.0, 0.0))

    def set_goal(self, goal):
        self.goal = (float(goal[0]), float(goal[1]))
        self.best = math.inf
        self.t_best = None
        self.forced = None               # lado forzado mientras dura el desvío
        self.forced_until = None
        self.last_side = RIGHT
        self.failed = 0                  # cambios de lado seguidos sin avanzar
        self.backup_until = self.wait_until = None

    def command(self, pose, obstacles, t):
        if self.backup_until is not None:            # maniobra de escape en curso
            dist = math.dist(control_point(*pose), self.goal)
            if t < self.backup_until:
                return (0.0 if _rear_blocked(obstacles) else BACKUP_V), 0.0, dist
            if t < self.wait_until:
                return 0.0, 0.0, dist
            self.backup_until = None
            self.best, self.t_best = dist, t
        side = None
        if self.forced is not None:
            if t < self.forced_until:
                side = self.forced
            else:
                self.forced = None
        v, w, dist, used = go_to_point(pose, self.goal, obstacles, side)
        if used is not None:
            self.last_side = used
        if self.t_best is None or dist < self.best - PROGRESS:
            self.best, self.t_best = dist, t
            self.failed = 0
        elif t - self.t_best > self.stall_time and self.forced is None:
            self.best, self.t_best = dist, t
            if self.failed >= 2:
                # bloqueo entre varios robots: marcha atrás y espera según la prioridad
                self.backup_until = t + BACKUP_TIME
                self.wait_until = self.backup_until + YIELD_TIME * self.priority
                self.failed = 0
                self.backups += 1
                return (0.0 if _rear_blocked(obstacles) else BACKUP_V), 0.0, dist
            # atascado: rodear por el otro lado durante un rato
            self.forced = -self.last_side
            self.forced_until = t + self.detour_time
            self.failed += 1
            self.detours += 1
        return v, w, dist

    @property
    def state(self):
        """'normal', 'desvio' (rodeando por el otro lado) o 'escape' (marcha atrás/espera)."""
        if self.backup_until is not None:
            return 'escape'
        return 'desvio' if self.forced is not None else 'normal'
