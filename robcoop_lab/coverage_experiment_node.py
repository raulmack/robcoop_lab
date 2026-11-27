"""Director y registro de un experimento de cobertura (proporcionado).

Es un OBSERVADOR externo: no participa en el control (cada robot calcula su celda), solo
  1. espera a que todos los robots tengan pose y publiquen su posición;
  2. comprueba que salen de una posición de salida conocida (marcas o agrupados);
  3. activa la cobertura (/coverage/enable = True);
  4. registra a 10 Hz la pose de cada robot, el coste de cobertura H (con VUESTRA
     coverage_cost) y el error  e(t) = max_i |C_i − p_i|;
  5. (opcional) cuando el equipo ya se ha desplegado, avería un robot para ver cómo se
     recolocan los demás;
  6. al desplegarse (e < tol durante 'hold' s) o al agotar 'timeout', para todo y guarda
        <out_dir>/<nombre>.csv   y   <out_dir>/<nombre>.json  (resumen y métricas)
Publica además /coverage/markers (RViz): celdas de Voronoi, centroides y zona de interés.
"""
import csv
import json
import math
import os
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PointStamped, Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from . import coverage_core as core
from . import coverage_law
from .coverage_node import ENABLE_QOS
from .kinematics import control_point
from .robot_api import RobotInterface

COLORS = [(0.90, 0.10, 0.29), (0.24, 0.71, 0.29), (0.00, 0.51, 0.78), (0.96, 0.51, 0.19)]
MARKER_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL)
MARKER_RES = 0.10                       # m: casillas con las que se dibujan las celdas en RViz
MARK_TOL = 0.15                         # m: margen para dar por buena la posición de salida


class CoverageExperiment(Node):

    def __init__(self):
        super().__init__('coverage_experiment')
        p = self.declare_parameter
        p('robots', ['tb3_0', 'tb3_1', 'tb3_2'])
        p('density', 'uniforme')
        p('center', list(core.ZONE_CENTER))
        p('sigma', core.ZONE_SIGMA)
        p('peak', core.ZONE_PEAK)
        p('k', core.K_DEFAULT)
        p('mode', 'sim')
        p('start', 'auto')                  # auto (marcas o agrupados) | libre (sin comprobar)
        p('name', '')
        p('out_dir', '')
        p('timeout', 120.0)
        p('tol', core.TOL)
        p('hold', core.HOLD)
        p('fail', '')                       # robot que se avería tras el despliegue ('' = ninguno)
        g = lambda n: self.get_parameter(n).value  # noqa: E731

        self.robots = list(g('robots'))
        self.timeout = float(g('timeout'))
        self.tol, self.hold = float(g('tol')), float(g('hold'))
        self.mode = g('mode')
        self.start = g('start')
        if self.start not in ('auto', 'libre'):
            raise ValueError('start debe ser auto o libre')
        self.fail = g('fail').strip().strip('/')
        if self.fail and self.fail not in self.robots:
            raise ValueError(f'fail={self.fail} no es uno de los robots {self.robots}')
        if self.fail and len(self.robots) < 2:
            raise ValueError('Para averiar un robot hacen falta al menos dos')
        self.points, self.cell_area = core.make_grid()
        zone = (tuple(float(v) for v in g('center')), float(g('sigma')), float(g('peak')))
        self.weights = core.density(g('density'), self.points, *zone)
        self.mpoints, _ = core.make_grid(res=MARKER_RES)
        self.meta = {'robots': self.robots, 'density': g('density'), 'k': g('k'),
                     'mode': self.mode, 'tol': self.tol, 'hold': self.hold,
                     'limit': core.LIMIT, 'res': core.RES, 'fail': self.fail}
        if g('density') == 'zona':
            self.meta.update({'center': list(zone[0]), 'sigma': zone[1], 'peak': zone[2]})

        self.ifaces = [RobotInterface(self, r, command=False) for r in self.robots]
        self.cmd_pubs = [self.create_publisher(Twist, f'/{r}/cmd_vel', 10) for r in self.robots]
        self.last_state = {}                # robot -> instante del último latido
        for r in self.robots:
            self.create_subscription(PointStamped, f'/{r}/coverage/state',
                                     lambda m, r=r: self._on_state(r), 10)
        self.enable_pub = self.create_publisher(Bool, '/coverage/enable', ENABLE_QOS)
        self.create_subscription(Bool, '/coverage/enable', self._on_enable, ENABLE_QOS)
        self.fail_pub = None
        if self.fail:
            self.fail_pub = self.create_publisher(Bool, f'/{self.fail}/coverage/fail', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/coverage/markers', MARKER_QOS)

        self.phase = 'espera'
        self.t0 = None
        self.ready_since = None
        self.stable = 0.0
        self.t_conv = self.t_fail = self.t_reconv = None
        self.broken = set()                 # robots averiados por este nodo
        self.prev_active = None
        self.lost = {}                      # robots que han dejado de publicar -> instante
        self.cost_deployed = self.cost_after_fail = None
        self.q_at_fail = None
        self.q0 = None
        self.rows = []
        self.last = None                    # último (q, activos) para los marcadores
        self.min_dist = math.inf
        self.done = False
        self.enable_pub.publish(Bool(data=False))
        self.create_timer(0.1, self._tick)
        self.create_timer(0.5, self._publish_markers)
        self.get_logger().info(
            f'Experimento de cobertura: robots {self.robots}, densidad {g("density")}, '
            f'k={g("k")}' + (f', avería de {self.fail}' if self.fail else '')
            + '. Esperando a los robots...')

    # --- Utilidades -----------------------------------------------------------------------
    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_state(self, r):
        self.last_state[r] = self._now()

    def _on_enable(self, msg):
        if not msg.data and not self.done and self.phase != 'espera':
            self.get_logger().warn('PARADA recibida: se termina el experimento')
            self._finish('parada de emergencia')

    def _active(self, now):
        """Robots que siguen en el equipo: no averiados y con latido reciente."""
        return [r for r in self.robots if r not in self.broken
                and now - self.last_state.get(r, -math.inf) < core.STALE + 0.5]

    def _cost(self, q):
        """(H con vuestra coverage_cost, H de referencia)."""
        ref = core.ref_cost(self.points, self.weights, q, self.cell_area)
        try:
            mine = float(coverage_law.coverage_cost(self.points, self.weights,
                                                    [np.array(x) for x in q], self.cell_area))
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'coverage_cost da error: {e!r}', throttle_duration_sec=10.0)
            mine = float('nan')
        return mine, ref

    # --- Posición de salida ---------------------------------------------------------------
    def _detect_start(self, poses):
        """Nombre de la salida en la que están los robots y, si no hay, lo que falta."""
        worst = {}
        for name in core.STARTS:
            off = []
            for r, p in zip(self.robots, poses):
                try:
                    i = int(r.rsplit('_', 1)[1])
                    mark = core.STARTS[name][i]
                except (IndexError, ValueError):
                    off.append((r, math.inf))
                    continue
                d = math.hypot(p[0] - mark[0], p[1] - mark[1])
                if d > MARK_TOL:
                    off.append((r, d))
            if not off:
                return name, []
            worst[name] = off
        return None, worst['marcas']

    def _abort_off_marks(self, off):
        """Los resultados no se podrían comparar con la referencia ni con los robots reales."""
        self.done = True
        lista = ', '.join(f'{r} a {d:.2f} m de su marca' for r, d in off)
        if self.mode == 'real':
            fix = ('Colocad los robots en sus marcas, Ctrl+C en la terminal de p3_real y '
                   'relanzadla (el anclaje se hace al arrancar).')
        else:
            fix = ('Probablemente siguen donde acabó el experimento anterior: Ctrl+C en la '
                   'terminal de p3_sim, relanzadla y repetid este experimento.')
        self.get_logger().error(
            f'Los robots no están en una posición de salida ({lista}). No se hace el '
            f'experimento ni se guarda nada. {fix} (Para salir a propósito desde donde estén: '
            'salida:=libre)')

    # --- Bucle principal ------------------------------------------------------------------
    def _tick(self):
        if self.done:
            return
        poses = [i.pose() for i in self.ifaces]
        now = self._now()

        if self.phase == 'espera':
            missing = [r for r, p in zip(self.robots, poses)
                       if p is None or r not in self.last_state]
            if missing:
                self.ready_since = None
                self.get_logger().info(f'Esperando a: {", ".join(missing)}',
                                       throttle_duration_sec=3.0)
                return
            if self.ready_since is None:
                self.ready_since = now
            if now - self.ready_since < 1.0:
                return
            start = 'libre'
            if self.start == 'auto':
                start, off = self._detect_start(poses)
                if start is None:
                    self._abort_off_marks(off)
                    return
            self.meta['start'] = start
            g = lambda n: self.get_parameter(n).value  # noqa: E731
            stamp = time.strftime('%Y%m%d_%H%M%S')
            self.name = g('name') or (f'{self.mode}_{start}_{g("density")}_k{g("k")}'
                                      + ('_fallo' if self.fail else '') + f'_{stamp}')
            self.out_dir = g('out_dir') or core.default_out_dir()
            os.makedirs(self.out_dir, exist_ok=True)
            self.t0 = now
            self.q0 = {r: control_point(*p) for r, p in zip(self.robots, poses)}
            self.phase = 'cobertura'
            self.enable_pub.publish(Bool(data=True))
            self.get_logger().info(f'>>> Cobertura ACTIVADA (salida: {start}; experimento '
                                   f'"{self.name}")')
            # Aviso si el nombre dice otra salida (p. ej. sim_agrupados lanzado desde las marcas)
            self.name_warning = None
            other = [s for s in ('marcas', 'agrupados') if s != start and s in self.name.lower()]
            if other and start != 'libre':
                self.name_warning = (f'el nombre "{self.name}" dice «{other[0]}», pero los robots '
                                     f'salen de «{start}». ¿Falta salida:={other[0]} al lanzar '
                                     'p3_sim?')
                self.get_logger().warn('OJO: ' + self.name_warning)

        t = now - self.t0
        if any(p is None for p in poses):
            self.get_logger().warn('Se ha perdido la TF de algún robot', throttle_duration_sec=2.0)
            if t > self.timeout:
                self._finish('tiempo agotado')
            return
        q = {r: control_point(*p) for r, p in zip(self.robots, poses)}
        active = self._active(now)
        if not active:
            self.get_logger().warn('Ningún robot publica su posición', throttle_duration_sec=2.0)
            if t > self.timeout:
                self._finish('tiempo agotado')
            return
        if self.prev_active is not None and active != self.prev_active:
            for r in sorted(set(self.prev_active) - set(active) - self.broken):
                self.lost[r] = round(t, 2)
                self.get_logger().warn(f'{r} ha dejado de publicar su posición (t = {t:.1f} s): '
                                       'se le da por averiado')
            for r in sorted(set(active) - set(self.prev_active)):
                self.get_logger().info(f'{r} vuelve a publicar su posición (t = {t:.1f} s)')
        self.prev_active = active
        qa = [q[r] for r in active]
        cost, cost_ref = self._cost(qa)
        err = core.ref_error(self.points, self.weights, qa)
        self.last = (q, active, cost_ref, err)
        if self.broken and len(self.rows) % 10 == 0:      # se repite por si se pierde
            self.fail_pub.publish(Bool(data=True))
        for i in range(len(poses)):
            for j in range(i + 1, len(poses)):
                self.min_dist = min(self.min_dist, math.dist(poses[i][:2], poses[j][:2]))
        row = [round(t, 3), round(cost, 4), round(cost_ref, 4), round(err, 4), self.phase,
               len(active)]
        for r, p in zip(self.robots, poses):
            row += [round(p[0], 4), round(p[1], 4), round(p[2], 4), int(r in active)]
        self.rows.append(row)

        self.stable = self.stable + 0.1 if err < self.tol else 0.0
        if self.stable >= self.hold:
            reached = round(t - self.stable + 0.1, 2)
            if self.phase == 'cobertura':
                self.t_conv = reached
                self.cost_deployed = round(cost_ref, 4)
                self.get_logger().info(f'>>> Desplegados en {self.t_conv:.1f} s  '
                                       f'(H = {cost_ref:.2f} m⁴)')
                if self.fail_pub is not None:
                    self.phase = 'recuperacion'
                    self.stable = 0.0
                    self.t_fail = round(t, 2)
                    self.q_at_fail = dict(q)
                    self.broken.add(self.fail)
                    self.fail_pub.publish(Bool(data=True))
                    rest = [q[r] for r in active if r != self.fail]
                    self.cost_after_fail = round(core.ref_cost(
                        self.points, self.weights, rest, self.cell_area), 4)
                    self.get_logger().info(
                        f'>>> AVERÍA de {self.fail}: H pasa de {self.cost_deployed:.2f} a '
                        f'{self.cost_after_fail:.2f} m⁴; los demás tienen que recolocarse')
                    return
            else:
                self.t_reconv = round(reached - self.t_fail, 2)
                self.get_logger().info(f'>>> Recolocados {self.t_reconv:.1f} s después de la '
                                       f'avería  (H = {cost_ref:.2f} m⁴)')
            self._finish('desplegado')
        elif t > self.timeout:
            self._finish('tiempo agotado')

    # --- Marcadores para RViz -------------------------------------------------------------
    def _publish_markers(self):
        if self.last is None or self.done:
            return
        q, active, cost, err = self.last
        arr = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        def marker(mid, mtype, ns):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = stamp
            m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
            m.pose.orientation.w = 1.0
            return m

        colors = [COLORS[self.robots.index(r) % len(COLORS)] for r in active]
        qa = [q[r] for r in active]
        owner = core.ref_owner(self.mpoints, qa)
        cells = marker(0, Marker.CUBE_LIST, 'celdas')
        cells.scale.x = cells.scale.y = MARKER_RES
        cells.scale.z = 0.005
        for (x, y), o in zip(self.mpoints, owner):
            cells.points.append(Point(x=float(x), y=float(y), z=0.0))
            c = colors[int(o)]
            cells.colors.append(ColorRGBA(r=c[0], g=c[1], b=c[2], a=0.35))
        arr.markers.append(cells)

        C, _ = core.ref_centroids(self.points, self.weights, qa)
        for i, r in enumerate(self.robots):
            m = marker(i, Marker.SPHERE, 'centroides')
            if r in active and np.all(np.isfinite(C[active.index(r)])):
                cx, cy = C[active.index(r)]
                m.pose.position.x, m.pose.position.y, m.pose.position.z = float(cx), float(cy), 0.03
                m.scale.x = m.scale.y = m.scale.z = 0.09
                m.color.r, m.color.g, m.color.b = COLORS[i % len(COLORS)]
                m.color.a = 1.0
            else:
                m.action = Marker.DELETE
            arr.markers.append(m)

        # Robot averiado (excluido por los demás): rótulo rojo encima de su última posición
        for i, r in enumerate(self.robots):
            m = marker(i, Marker.TEXT_VIEW_FACING, 'averiado')
            if r not in active and r in q:
                m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(q[r][0]),
                                                                           float(q[r][1]), 0.35)
                m.scale.z = 0.14
                m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.2, 0.2, 1.0
                m.text = 'AVERIADO'
            else:
                m.action = Marker.DELETE
            arr.markers.append(m)

        L = core.LIMIT
        border = marker(0, Marker.LINE_STRIP, 'zona a cubrir')
        border.scale.x = 0.02
        border.color.r = border.color.g = border.color.b = border.color.a = 1.0
        for x, y in ((-L, -L), (L, -L), (L, L), (-L, L), (-L, -L)):
            border.points.append(Point(x=x, y=y, z=0.01))
        arr.markers.append(border)

        ring = marker(0, Marker.LINE_STRIP, 'zona de interes')
        if self.meta['density'] == 'zona':
            (cx, cy), s = self.meta['center'], self.meta['sigma']
            ring.scale.x = 0.03
            ring.color.r, ring.color.g, ring.color.b, ring.color.a = 1.0, 0.9, 0.0, 1.0
            for a in np.linspace(0.0, 2 * math.pi, 41):
                ring.points.append(Point(x=cx + s * math.cos(a), y=cy + s * math.sin(a), z=0.02))
        else:
            ring.action = Marker.DELETE
        arr.markers.append(ring)

        # RViz dibuja cada espacio casi tan ancho como cuatro letras: un rótulo por dato, sin
        # espacios, en ASCII y en posiciones fijas encima de la zona
        tokens = (self.phase, f'H={cost:.2f}m4', f'e={100 * err:.0f}cm')
        for k, (x, t) in enumerate(zip((-1.0, 0.15, 1.15), tokens)):
            txt = marker(k, Marker.TEXT_VIEW_FACING, 'texto')
            txt.pose.position.x, txt.pose.position.y, txt.pose.position.z = x, L + 0.25, 0.1
            txt.scale.z = 0.13
            txt.color.r = txt.color.g = txt.color.b = txt.color.a = 1.0
            txt.text = t
            arr.markers.append(txt)
        self.marker_pub.publish(arr)

    # --- Final ----------------------------------------------------------------------------
    def stop_all(self):
        self.enable_pub.publish(Bool(data=False))
        for pub in self.cmd_pubs:
            pub.publish(Twist())

    def _travelled(self):
        dist = {r: 0.0 for r in self.robots}
        prev = {}
        for row in self.rows:
            for i, r in enumerate(self.robots):
                x, y, th = row[6 + 4 * i:9 + 4 * i]
                qq = control_point(x, y, th)          # se mide el punto de control
                if r in prev:
                    dist[r] += math.dist(prev[r], qq)
                prev[r] = qq
        return {r: round(v, 3) for r, v in dist.items()}

    def _finish(self, reason):
        self.done = True
        self.stop_all()
        if not self.rows:
            self.get_logger().warn(f'Fin ({reason}) sin datos: no se guarda nada')
            return
        mass = float(np.sum(self.weights)) * self.cell_area
        rms = lambda h: round(math.sqrt(max(h, 0.0) / mass), 4)  # noqa: E731
        q, active, _, _ = self.last
        qa = [q[r] for r in active]
        C, M = core.ref_centroids(self.points, self.weights, qa, self.cell_area)
        _, A = core.ref_centroids(self.points, np.ones(len(self.points)), qa, self.cell_area)
        first, last = self.rows[0], self.rows[-1]
        # Adónde llegaría el algoritmo de Lloyd ideal desde las mismas posiciones de salida
        lq, lcosts = core.lloyd([self.q0[r] for r in self.robots], self.points, self.weights,
                                self.cell_area)
        lloyd_fail = None                   # ... y los que quedan, desde donde estaban al fallar
        if self.q_at_fail:
            rest = [self.q_at_fail[r] for r in self.robots if r != self.fail]
            lloyd_fail = round(core.lloyd(rest, self.points, self.weights,
                                          self.cell_area)[1][-1], 4)
        self.meta.update({
            'name': self.name, 'end_reason': reason, 'date': time.strftime('%Y-%m-%d %H:%M:%S'),
            'duration': last[0], 't_conv': self.t_conv, 't_fail': self.t_fail,
            't_reconv': self.t_reconv, 'final_error': last[3],
            'cost_initial': first[2], 'cost_final': last[2], 'cost_final_law': last[1],
            'rms_initial': rms(first[2]), 'rms_final': rms(last[2]),
            'cost_deployed': self.cost_deployed, 'cost_after_fail': self.cost_after_fail,
            'start_q': {r: [round(v, 4) for v in self.q0[r]] for r in self.robots},
            'q_at_fail': ({r: [round(v, 4) for v in p] for r, p in self.q_at_fail.items()}
                          if self.q_at_fail else None),
            'active': active, 'lost': self.lost,
            'final_q': {r: [round(v, 4) for v in q[r]] for r in self.robots},
            'centroids': {r: [round(float(v), 4) for v in C[i]] for i, r in enumerate(active)},
            'masses': {r: round(float(M[i]), 4) for i, r in enumerate(active)},
            'areas': {r: round(float(A[i]), 4) for i, r in enumerate(active)},
            'travelled': self._travelled(),
            'min_dist': round(self.min_dist, 3) if math.isfinite(self.min_dist) else None,
            'lloyd_cost': round(lcosts[-1], 4), 'lloyd_cost_fail': lloyd_fail,
            'lloyd_q': {r: [round(float(v), 4) for v in lq[i]]
                        for i, r in enumerate(self.robots)}})
        csv_path = os.path.join(self.out_dir, self.name + '.csv')
        header = ['t', 'coste', 'coste_ref', 'error', 'fase', 'activos']
        for r in self.robots:
            header += [f'{r}_x', f'{r}_y', f'{r}_theta', f'{r}_activo']
        with open(csv_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(header)
            w.writerows(self.rows)
        with open(os.path.join(self.out_dir, self.name + '.json'), 'w') as f:
            json.dump(self.meta, f, indent=2, ensure_ascii=False)

        m = self.meta
        tc = f'{self.t_conv:.1f} s' if self.t_conv is not None else 'NO'
        dep = m['cost_deployed'] if m['cost_deployed'] is not None else m['cost_final']
        lines = [f'\n==== FIN ({reason}) ====',
                 f'  salida {m["start"]}, densidad {m["density"]}, k={m["k"]}',
                 f'  despliegue: {tc}   coste H: {m["cost_initial"]:.2f} -> {dep:.2f} m⁴   '
                 f'(Lloyd ideal: {m["lloyd_cost"]:.2f})']
        if self.t_fail is not None:
            tr = f'{self.t_reconv:.1f} s' if self.t_reconv is not None else 'NO'
            lines.append(f'  avería de {self.fail} en t = {self.t_fail:.1f} s: H sube a '
                         f'{self.cost_after_fail:.2f} m⁴; recolocados en {tr}, H = '
                         f'{m["cost_final"]:.2f} m⁴   (Lloyd ideal: {lloyd_fail:.2f})')
        if getattr(self, 'name_warning', None):
            lines.append(f'  OJO: {self.name_warning}')
        lines.append(f'  distancia media de un punto a su robot: {m["rms_initial"]:.2f} -> '
                     f'{m["rms_final"]:.2f} m')
        for r in self.robots:
            if r in active:
                lines.append(f'  {r}: ({q[r][0]:+.2f}, {q[r][1]:+.2f})   celda de '
                             f'{m["areas"][r]:.2f} m²   recorrido {m["travelled"][r]:.2f} m')
            else:
                lines.append(f'  {r}: averiado en ({q[r][0]:+.2f}, {q[r][1]:+.2f})')
        if math.isfinite(last[1]) and abs(last[1] - last[2]) > 0.01 * max(last[2], 1e-9):
            lines.append(f'  OJO: vuestra coverage_cost da {last[1]:.3f} y la de referencia '
                         f'{last[2]:.3f}: revisadla con check_p3')
        elif not math.isfinite(last[1]):
            lines.append('  OJO: vuestra coverage_cost da error: revisadla con check_p3')
        lines.append(f'  datos: {csv_path}')
        lines.append(f'  gráficas: ros2 run robcoop_lab plot_p3 {csv_path}')
        self.get_logger().info('\n'.join(lines))


def main(args=None):
    rclpy.init(args=args)
    node = CoverageExperiment()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
        for _ in range(5):              # repetir la parada por si algún mensaje se pierde
            node.stop_all()
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        try:
            node.stop_all()
            if not node.done and node.rows:
                node._finish('interrumpido (Ctrl+C)')
        except Exception:
            pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
