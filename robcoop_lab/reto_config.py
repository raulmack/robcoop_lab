"""PRÁCTICA 4 — Reto final.  >>> ESTE ES EL FICHERO QUE TOCÁIS <<<

En el reto no hay ningún algoritmo nuevo que programar: la misión usa, una detrás de otra,
VUESTRAS tres leyes (consensus_law.py, auction_law.py y coverage_law.py). Aquí decidís cómo
las usa el equipo: cinco decisiones, cada una con su justificación.

Los valores que vienen puestos funcionan, pero son lentos a propósito. Cambiadlos con lo que
habéis medido en las prácticas 1, 2 y 3 y comprobad el efecto sin ROS, en segundos:

    ros2 run robcoop_lab kinsim_p4            # una misión completa con esta configuración
    ros2 run robcoop_lab check_p4             # las tres leyes + cuatro misiones de prueba

No hace falta compilar (cb) después de cambiar este fichero.
"""

# --- Fase 1 · Formación (consenso, Práctica 1) ----------------------------------------------
TOPOLOGIA = 'cadena'       # 'completo' | 'anillo' | 'cadena' | 'estrella'
FORMACION = 'linea'        # 'reunion' | 'linea' | 'poligono' | 'triangulo'
K_CONSENSO = 0.1           # ganancia de u_i = -k Σ (ξ_i - ξ_j)        (mayor que 0, hasta 1.0)

# --- Fase 2 · Inspección (subasta, Práctica 2) ----------------------------------------------
CRITERIO = 'suma'          # 'suma' (menos distancia en total) | 'max' (ruta más larga corta)

# --- Fases 3 y 4 · Cobertura y avería (Voronoi, Práctica 3) ---------------------------------
K_COBERTURA = 0.1          # ganancia de u_i = k (C_i - p_i)           (mayor que 0, hasta 3.0)

# --- Justificación (se evalúa) --------------------------------------------------------------
# Una o dos frases por decisión, con NÚMEROS de vuestros experimentos de las prácticas 1 a 3
# (λ2 y tiempos de convergencia, tiempo total con suma y con max, tiempo de despliegue con
# cada ganancia...) o de kinsim_p4. Ejemplo de forma, no de contenido:
#   'Con la topología X medimos t = ... s frente a ... s con Y, porque λ2 pasa de ... a ...'
POR_QUE = {
    'TOPOLOGIA': '',
    'FORMACION': '',
    'K_CONSENSO': '',
    'CRITERIO': '',
    'K_COBERTURA': '',
}
