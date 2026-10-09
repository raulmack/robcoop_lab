"""Del integrador simple al robot diferencial (proporcionado, no hay que modificarlo).

La ley de consenso trata cada robot como un punto que se mueve con la velocidad que
se le ordena (p_dot = u). Un TurtleBot3 no puede moverse de lado, así que se controla
un PUNTO DE CONTROL situado a una distancia d delante del eje de las ruedas:

    q = (x + d cos θ,  y + d sin θ)

Para ese punto existe una relación exacta entre (u_x, u_y) y las órdenes (v, ω):

    v =  cos θ · u_x + sin θ · u_y
    ω = (-sin θ · u_x + cos θ · u_y) / d

(linealización por realimentación). Por eso el consenso se hace sobre q, no sobre (x, y).
"""
import math

D_CONTROL = 0.10      # m, distancia del punto de control al eje de las ruedas
V_MAX = 0.15          # m/s, igual en simulación y en real
W_MAX = 1.5           # rad/s


def control_point(x, y, theta, d=D_CONTROL):
    """Punto de control q del robot."""
    return x + d * math.cos(theta), y + d * math.sin(theta)


def saturate(ux, uy, vmax=V_MAX):
    """Limita el módulo de la velocidad deseada conservando su dirección."""
    n = math.hypot(ux, uy)
    if n > vmax:
        return ux * vmax / n, uy * vmax / n
    return ux, uy


def integrator_to_unicycle(ux, uy, theta, d=D_CONTROL, w_max=W_MAX):
    """Convierte la velocidad deseada del punto de control en (v, ω)."""
    c, s = math.cos(theta), math.sin(theta)
    v = c * ux + s * uy
    w = (-s * ux + c * uy) / d
    w = max(-w_max, min(w_max, w))
    return v, w


def wrap(angle):
    """Ángulo en [-π, π)."""
    return (angle + math.pi) % (2 * math.pi) - math.pi
