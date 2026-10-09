"""Director y registro de un experimento de consenso (proporcionado).

Es un OBSERVADOR externo: no participa en el control, solo
  1. espera a que todos los robots tengan pose y publiquen su estado;
  2. activa el consenso (/consensus/enable = True);
  3. registra a 10 Hz la pose de cada robot y el error de consenso
        e(t) = max_i || ξ_i - media(ξ) ||
  4. (opcional) cuando converge, perturba un robot para probar la robustez;
  5. al converger (e < tol durante 'hold' s) o al agotar 'timeout', para todo y guarda
        <out_dir>/<nombre>.csv   y   <out_dir>/<nombre>.json  (resumen y métricas)
"""
import csv
import json
import os
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, Twist
from rclpy.node import Node
from std_msgs.msg import Bool, Float64MultiArray

from . import graphs
from .consensus_node import ENABLE_QOS
from .kinematics import control_point
from .robot_api import RobotInterface


def default_out_dir():
    repo = os.path.expanduser('~/ws/src/robcoop_lab')
    if os.path.isdir(repo):
        return os.path.join(repo, 'resultados', 'p1')
    return os.path.expanduser('~/ws/resultados/p1')


class ExperimentNode(Node):

    def __init__(self):
        super().__init__('consensus_experiment')
        p = self.declare_parameter
        p('robots', ['tb3_0', 'tb3_1', 'tb3_2', 'tb3_3'])
        p('offsets', [0.0])                 # dx0, dy0, dx1, dy1, ...
        p('topology', 'completo')
        p('formation', 'reunion')
        p('k', 0.1)
        p('mode', 'sim')
        p('name', '')
        p('out_dir', '')
        p('timeout', 120.0)
        p('tol', 0.05)
        p('hold', 3.0)
        p('disturb', '')                    # robot a perturbar ('' = ninguno)
        p('disturb_vel', [0.0, 0.15])       # m/s en 'map'
        p('disturb_duration', 3.0)
        g = lambda n: self.get_parameter(n).value  # noqa: E731

        self.robots = list(g('robots'))
        self.timeout = float(g('timeout'))
        n = len(self.robots)
        off = list(g('offsets'))
        self.delta = np.array(off, dtype=float).reshape(n, 2) if len(off) == 2 * n \
            else np.zeros((n, 2))
        self.meta = {'robots': self.robots, 'topology': g('topology'),
                     'formation': g('formation'), 'k': g('k'), 'mode': g('mode'),
                     'offsets': self.delta.tolist(), 'tol': g('tol'), 'hold': g('hold'),
                     'disturb': g('disturb'), 'disturb_vel': list(g('disturb_vel')),
                     'disturb_duration': g('disturb_duration')}
        A = graphs.adjacency(g('topology'), n)
        self.meta['adjacency'] = A.tolist()
        self.meta['lambda2'] = graphs.lambda2_reference(A)

        stamp = time.strftime('%Y%m%d_%H%M%S')
        default = f'{g("mode")}_{g("topology")}_{g("formation")}_k{g("k")}_{stamp}'
        self.name = g('name') or default
        self.out_dir = g('out_dir') or default_out_dir()
        os.makedirs(self.out_dir, exist_ok=True)

        self.ifaces = [RobotInterface(self, r, command=False) for r in self.robots]
        self.cmd_pubs = [self.create_publisher(Twist, f'/{r}/cmd_vel', 10) for r in self.robots]
        self.seen = set()
        for r in self.robots:
            self.create_subscription(PointStamped, f'/{r}/consensus/state',
                                     lambda m, r=r: self.seen.add(r), 10)
        self.enable_pub = self.create_publisher(Bool, '/consensus/enable', ENABLE_QOS)
        self.disturb_pub = None
        if g('disturb'):
            self.disturb_pub = self.create_publisher(
                Float64MultiArray, f'/{g("disturb")}/consensus/disturb', 10)

        self.phase = 'espera'
        self.t0 = None
        self.ready_since = None
        self.stable = 0.0
        self.t_conv = None            # convergencia inicial (s desde el inicio)
        self.t_disturb = None         # inicio de la perturbación
        self.t_reconv = None          # reconvergencia (s desde el FIN de la perturbación)
        self.rows = []
        self.done = False
        self.enable_pub.publish(Bool(data=False))
        self.create_timer(0.1, self._tick)
        self.get_logger().info(f'Experimento "{self.name}": {self.meta["topology"]}, '
                               f'{self.meta["formation"]}, k={self.meta["k"]}, '
                               f'λ2={self.meta["lambda2"]:.3f}. Esperando a los robots...')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _tick(self):
        if self.done:
            return
        poses = [i.pose() for i in self.ifaces]
        now = self._now()

        if self.phase == 'espera':
            missing = [r for r, p in zip(self.robots, poses) if p is None or r not in self.seen]
            if missing:
                self.ready_since = None
                self.get_logger().info(f'Esperando a: {", ".join(missing)}',
                                       throttle_duration_sec=3.0)
                return
            if self.ready_since is None:
                self.ready_since = now
            if now - self.ready_since < 1.0:
                return
            self.t0 = now
            self.phase = 'consenso'
            self.enable_pub.publish(Bool(data=True))
            self.get_logger().info('>>> Consenso ACTIVADO')

        t = now - self.t0
        if any(p is None for p in poses):
            self.get_logger().warn('Se ha perdido la TF de algún robot', throttle_duration_sec=2.0)
            if t > self.timeout:
                self._finish('tiempo agotado')
            return
        q = np.array([control_point(*p) for p in poses])
        xi = q - self.delta
        err = float(np.max(np.linalg.norm(xi - xi.mean(axis=0), axis=1)))
        row = [round(t, 3), round(err, 4), self.phase]
        for p in poses:
            row += [round(p[0], 4), round(p[1], 4), round(p[2], 4)]
        self.rows.append(row)

        tol, hold = self.meta['tol'], self.meta['hold']
        dur = self.meta['disturb_duration']

        if self.phase == 'perturbacion':
            if t >= self.t_disturb + dur:
                self.phase = 'recuperacion'
                self.stable = 0.0
            return

        self.stable = self.stable + 0.1 if err < tol else 0.0
        if self.stable >= hold:
            t_reached = round(t - self.stable + 0.1, 2)
            if self.phase == 'consenso':
                self.t_conv = t_reached
                self.get_logger().info(f'>>> Convergencia en {self.t_conv:.1f} s')
                if self.disturb_pub is not None:
                    self.phase = 'perturbacion'
                    self.t_disturb = t
                    vx, vy = self.meta['disturb_vel']
                    self.disturb_pub.publish(Float64MultiArray(data=[vx, vy, dur]))
                    self.get_logger().info(f'>>> Perturbando {self.meta["disturb"]}')
                    return
            else:
                self.t_reconv = round(t_reached - (self.t_disturb + dur), 2)
                self.get_logger().info(f'>>> Recuperada la formación {self.t_reconv:.1f} s '
                                       'después de la perturbación')
            self._finish('convergido')
        elif t > self.timeout:
            self._finish('tiempo agotado')

    def stop_all(self):
        self.enable_pub.publish(Bool(data=False))
        for pub in self.cmd_pubs:
            pub.publish(Twist())

    def _finish(self, reason):
        self.done = True
        self.stop_all()
        final_err = self.rows[-1][1] if self.rows else None
        self.meta.update({'name': self.name, 'end_reason': reason, 't_conv': self.t_conv,
                          't_disturb': self.t_disturb, 't_reconv': self.t_reconv,
                          'final_error': final_err,
                          'duration': self.rows[-1][0] if self.rows else 0.0,
                          'date': time.strftime('%Y-%m-%d %H:%M:%S')})
        csv_path = os.path.join(self.out_dir, self.name + '.csv')
        header = ['t', 'error', 'fase']
        for r in self.robots:
            header += [f'{r}_x', f'{r}_y', f'{r}_theta']
        with open(csv_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(header)
            w.writerows(self.rows)
        with open(os.path.join(self.out_dir, self.name + '.json'), 'w') as f:
            json.dump(self.meta, f, indent=2, ensure_ascii=False)
        tc = f'{self.t_conv:.1f} s' if self.t_conv is not None else 'NO'
        self.get_logger().info(
            f'\n==== FIN ({reason}) ====\n'
            f'  topología {self.meta["topology"]}  λ2={self.meta["lambda2"]:.3f}  '
            f'k={self.meta["k"]}\n'
            f'  convergencia: {tc}   error final: {final_err} m\n'
            + (f'  recuperación tras perturbar: {self.t_reconv} s\n'
               if self.t_disturb is not None else '') +
            f'  datos: {csv_path}\n'
            f'  gráficas: ros2 run robcoop_lab plot_p1 {csv_path}')


def main(args=None):
    rclpy.init(args=args)
    node = ExperimentNode()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
        # repetir la orden de parada unas veces por si algún mensaje se pierde
        for _ in range(5):
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
