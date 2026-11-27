# Práctica 4 — Reto final: una misión completa

| | |
|---|---|
| **Duración** | preparación en casa (solo simulación) + sesión 4 (ensayo y demostración con robots reales) |
| **Se programa** | nada nuevo: la misión usa vuestras `consensus_law.py`, `auction_law.py` y `coverage_law.py`. Solo se edita `robcoop_lab/reto_config.py` (cinco decisiones justificadas) |
| **Entrega** | demostración en vivo en la sesión 4 + etiqueta `p4` en GitHub, zip y vídeo (2 min como máximo) en el campus virtual, una semana después. **Sin informe** |
| **Peso** | 25 % de la nota de prácticas |

Esta práctica es distinta de las tres anteriores: no hay teoría ni algoritmo nuevo. Es una
**prueba**: el mismo equipo de robots tiene que hacer, seguido y sin que nadie lo recoloque, lo
que habéis programado en las prácticas 1, 2 y 3, con puntos de inspección que no conocéis de
antemano y con una avería que no sabéis a qué robot le va a tocar.

## Objetivos

1. Integrar las tres leyes (consenso, subasta y cobertura) en una misión multi-robot completa.
2. Decidir cómo se configura el equipo (grafo, formación, ganancias, criterio de puja) y
   justificarlo con lo medido en las prácticas anteriores.
3. Comprobar que el código es general: otros puntos, otra salida para la cobertura, cualquier
   robot averiado.
4. Ejecutar la misión con robots reales delante de la clase.

## Plan

| Cuándo | Paso | Resultado |
|---|---|---|
| S3 3:40–4:00 | Presentación del reto (profesor) | Reglas y marcador |
| Casa | 0. Traer el reto a vuestro repositorio | Paquete actualizado y compilado |
| Casa | 1. Primera misión sin ROS | Marcador y tiempo con la configuración inicial |
| Casa | 2. Decidir y justificar `reto_config.py` | `check_p4` en verde (6/6) |
| Casa | 3. Misión completa en Gazebo | `sim_reto`, figura y marcador |
| S4 0:00–0:10 | Reglas del día (profesor) | Puntos de inspección de la demostración |
| S4 0:10–1:30 | 4. Ensayo con robots reales: 5 min por pareja, al acabar su turno de la P3 | `real_ensayo` |
| S4 1:30–3:15 | 5. Demostración: dos intentos por pareja, cuenta el mejor | `real_reto_1`, `real_reto_2` |
| S4 3:15–4:00 | Puesta en común: simulación frente a real en todo el curso | |
| Casa | Vídeo y entrega | Etiqueta `p4` |

---

## La misión

Tres robots (`tb3_0`, `tb3_1`, `tb3_2`) salen de sus marcas de siempre. El **árbitro**
(`mission_node`) activa cada fase, mide y puntúa. Entre fase y fase los robots esperan 1.5 s
parados, y cada fase empieza donde acabó la anterior.

| Fase | Qué tiene que hacer el equipo | Se da por hecha cuando… | Límite | Puntos |
|---|---|---|---|---|
| 1 · Formación (P1) | Formar la figura que habéis elegido, por consenso | el error de consenso baja de 5 cm durante 3 s | 60 s | 20 |
| 2 · Inspección (P2) | Repartirse por subasta 6 puntos de inspección y visitarlos | todos los puntos están visitados (a 8 cm, 1 s parado en cada uno) | 150 s | 30 |
| 3 · Cobertura (P3) | Desplegarse para cubrir la zona de 3 × 3 m | cada robot está a menos de 2 cm de su centroide durante 3 s | 90 s | 20 |
| 4 · Avería (P3) | Un robot se para y deja de publicar: los demás se dan cuenta solos y se recolocan | los que quedan vuelven a estar a 2 cm de sus centroides durante 3 s | 90 s | 20 |
| Seguridad | Ningún contacto entre robots ni parada de emergencia | distancia entre centros siempre mayor que 21 cm | — | 10 |

**Marcador: 100 puntos.** Reglas:

* Si una fase agota su tiempo, la misión **sigue** con la siguiente. Una fase a medias puntúa en
  proporción a lo avanzado, hasta el 75 % de sus puntos: en la formación, lo que ha bajado el error;
  en la cobertura y la avería, lo que ha bajado el coste H hacia el del algoritmo de Lloyd ideal.
  La inspección puntúa por puntos visitados (5 cada uno con 6 puntos).
* Los 10 puntos de seguridad solo cuentan si el equipo suma al menos 45 de los otros 90: no
  moverse no es ser seguro.
* **Tiempo de misión:** suma de lo que tarda cada fase (sin las pausas). Solo vale si la misión
  está completa (las cuatro fases hechas) y sirve para ordenar a las parejas.
* **Avería:** con `averia:=auto` (por defecto) el árbitro avería **un robot al azar** 2 s después
  del despliegue. El robot averiado no avisa: simplemente deja de publicar su posición, como en
  `sim_fallo` de la P3.

Todo está en `robcoop_lab/mission_core.py` (límites, puntos y cálculo del marcador); el simulador
sin ROS y el árbitro de ROS usan las mismas reglas.

## Qué hay en el paquete

| Fichero | Qué es | ¿Se toca? |
|---|---|---|
| `robcoop_lab/reto_config.py` | **Vuestras cinco decisiones y su justificación** | **Sí** |
| `robcoop_lab/consensus_law.py`, `auction_law.py`, `coverage_law.py` | Vuestras leyes de las prácticas 1 a 3 | Solo si algo fallaba |
| `robcoop_lab/mission_core.py` | Reglas del reto y marcador, sin ROS | No |
| `robcoop_lab/agent_node.py` | Agente de un robot: sus tres nodos de las prácticas (consenso, subasta y cobertura) en un solo proceso | No |
| `robcoop_lab/mission_node.py` | Árbitro: activa las fases, registra y puntúa | No |
| `robcoop_lab/kinsim_p4.py` | Misión completa sin ROS, en segundos | No |
| `robcoop_lab/check_p4.py` | Comprobación: configuración, tres leyes y cuatro misiones | No |
| `robcoop_lab/plot_p4.py` | Figura de la misión, marcador y comparación | No |
| `robcoop_lab/fail_robot.py` (`averia`) | Avería simulada de un robot (la usa el profesor) | No |
| `launch/p4_sim.launch.py`, `p4_reto.launch.py`, `p4_real.launch.py` | Simulación, misión y robots reales | No |

El agente no añade nada nuevo: arranca los mismos `consensus_node`, `auction_node` y
`coverage_node` de las prácticas, con los parámetros que salen de `reto_config.py`. Los tres están
siempre vivos y publican su estado; solo mueve el robot el que el árbitro tiene activado.

---

## Paso 0 · Traer el reto a vuestro repositorio (10 min)

Como en la P2 y la P3, en la terminal de Ubuntu (WSL), *fuera* del contenedor:

```bash
cd ~/robcoop-docker/ws/src/robcoop_lab
git status -s                        # no debe salir nada: si sale, commit y push antes de seguir
git fetch plantilla
git checkout plantilla/main -- . ':!robcoop_lab/consensus_law.py' ':!robcoop_lab/formations.py' ':!robcoop_lab/graphs.py' ':!robcoop_lab/auction_law.py' ':!robcoop_lab/tasks.py' ':!robcoop_lab/coverage_law.py' ':!resultados'
git commit -m "P4: material de la plantilla" && git push
```

Compilar dentro del contenedor (hay ejecutables nuevos) y comprobar:

```bash
cd ~/robcoop-docker && ./robcoop.sh shell
cb
ros2 run robcoop_lab check_p4
```

`check_p4` debe dar **5/6**: la configuración inicial es válida, las tres leyes pasan y las cuatro
misiones se completan; solo falla la justificación, que aún no habéis escrito. Si falla alguna de
las leyes, arregladla antes de seguir (`check_p1`, `check_p2`, `check_p3` dicen qué función es).

> El prompt debe empezar por **`[SIM]`**. Si pone `[REAL]`, ejecutad `modo_sim`.

## Paso 1 · Primera misión, sin ROS (10 min)

```bash
ros2 run robcoop_lab kinsim_p4               # puntos A, avería de un robot al azar
ros2 run robcoop_lab kinsim_p4 --plot        # con las trayectorias de cada fase
```

Juega la misión entera en unos segundos con vuestras leyes y vuestro `reto_config.py`, y escribe
las rutas y el marcador. Con la configuración inicial sale algo así (aquí se averió `tb3_0`):

```
  fase            puntos   resultado
  1 Formación     20.0/20  32.9 s; error final 3.7 cm
  2 Inspección    30.0/30  6/6 puntos; 29.5 s
  3 Cobertura     20.0/20  49.3 s; H 9.38 -> 5.37 m⁴, Lloyd ideal 5.36
  4 Avería        20.0/20  65.4 s (tb3_0); H 10.83 -> 8.45 m⁴, Lloyd ideal 8.43
  Seguridad       10.0/10  distancia mínima entre robots 0.48 m
  MARCADOR: 100.0 / 100      tiempo de misión: 177.1 s
```

100 puntos, pero casi **tres minutos**: la configuración inicial funciona y es lenta a propósito.
Probad otros escenarios; vuestro código tiene que valer para todos:

```bash
ros2 run robcoop_lab kinsim_p4 --disposicion C --averia tb3_2
ros2 run robcoop_lab kinsim_p4 --disposicion aleatoria --semilla 7
```

## Paso 2 · Decidir y justificar (45 min)

Abrid `robcoop_lab/reto_config.py`. Hay cinco decisiones:

| Decisión | Opciones | De qué práctica sale la respuesta |
|---|---|---|
| `TOPOLOGIA` | `completo`, `anillo`, `cadena`, `estrella` | P1: λ2 y tiempo de convergencia |
| `FORMACION` | `reunion`, `linea`, `poligono`, `triangulo` | P1; y de dónde salen los robots hacia los puntos (P2) |
| `K_CONSENSO` | mayor que 0, hasta 1.0 | P1: efecto de la ganancia y saturación a 0.15 m/s |
| `CRITERIO` | `suma`, `max` | P2: distancia total frente a ruta más larga (tiempo) |
| `K_COBERTURA` | mayor que 0, hasta 3.0 | P3: `sim_k01`, `sim_uniforme`, `sim_k2` |

Cambiad un valor, guardad y volved a ejecutar `kinsim_p4` (no hace falta `cb`). Cuando tengáis
vuestra configuración, escribid en `POR_QUE` **una o dos frases por decisión, con números**: de
vuestros informes de las prácticas 1 a 3 o de `kinsim_p4`. Después:

```bash
ros2 run robcoop_lab check_p4
```

Debe dar **6/6**. La última parte juega cuatro misiones (puntos A, B, C y unos aleatorios,
averiando cada vez un robot distinto) y escribe el tiempo de cada fase y el tiempo medio.

Para pensar (no hay que entregarlo aparte, va en `POR_QUE`):

* ¿Cuánto baja el tiempo de la formación al triplicar la ganancia? ¿Y al volver a triplicarla?
  ¿Por qué deja de bajar?
* ¿La mejor configuración en simulación es la mejor para los robots reales? Pensad en lo que
  visteis con los robots en las prácticas 1 a 3 antes de subir las ganancias al máximo: en la
  demostración hay dos intentos y el ensayo dura 5 minutos.
* Una referencia: una misión bien configurada baja de **60 s** de media en `check_p4`.

## Paso 3 · Misión completa en Gazebo (30 min)

**Terminal 1** (3 robots en sus marcas, sin ventana de Gazebo; se abre RViz):

```bash
ros2 launch robcoop_lab p4_sim.launch.py
```

**Terminal 2** (la misión):

```bash
ros2 launch robcoop_lab p4_reto.launch.py nombre:=sim_reto
```

El árbitro espera a los tres robots, comprueba que están en sus marcas y va escribiendo cada
fase (`>>> FASE 1 · formación…`, `>>> Formados: …`). En RViz se ve lo que toca en cada momento:
adónde va cada robot en la formación, los puntos de inspección con el color de su robot y las
rutas, y las celdas de Voronoi con sus centroides; arriba, la fase, los puntos y el tiempo.
Termina solo, escribe el marcador y guarda `resultados/p4/sim_reto.csv` y `.json`. Figura y
marcador:

```bash
ros2 run robcoop_lab plot_p4 ~/ws/src/robcoop_lab/resultados/p4/sim_reto.csv
```

Guarda `sim_reto.png` (una arena por fase y la línea de tiempo de la misión) y
`sim_reto_marcador.md`. Los tiempos deben parecerse a los de `kinsim_p4` con los mismos puntos.

**Antes de cada misión, relanzad la simulación** (Ctrl+C en la terminal 1 y otra vez `p4_sim`):
la misión siempre sale de las marcas. Si no, el árbitro lo detecta y no la juega.

Repetidla al menos con otros puntos y otro robot averiado; es lo que pasará en la demostración:

```bash
ros2 launch robcoop_lab p4_reto.launch.py disposicion:=aleatoria semilla:=7 averia:=tb3_2 nombre:=sim_reto_b
```

Argumentos de `p4_reto` (el día de la demostración los fija el profesor):

| Argumento | Valores | Por defecto |
|---|---|---|
| `disposicion` | `A`, `B`, `C`, `aleatoria` (con `semilla:=n`) | `A` |
| `puntos` | `"x,y; x,y; …"` (sustituye a la disposición) | |
| `averia` | `auto` (un robot al azar), `tb3_i`, `profesor`, `ninguna` | `auto` |
| `densidad` | `uniforme`, `zona` | `uniforme` |
| `modo` | `sim`, `real` | `sim` |
| `nombre` | nombre de los ficheros de resultados | se genera |
| `robots` | `tb3_0,tb3_1,tb3_2` (o los cuatro) | los tres |

## Paso 4 · Ensayo con robots reales (sesión 4, 5 min al acabar vuestro turno de la P3)

Los robots ya están en la arena y vosotros en `modo_real`, porque venís del paso 6 de la P3.

1. Colocad `tb3_0`, `tb3_1` y `tb3_2` sobre sus marcas, mirando en la dirección de la flecha.
2. **Terminal 1**, anclar los robots y abrir RViz con la misión (Ctrl+C antes en `p3_real`):
   ```bash
   ros2 launch robcoop_lab p4_real.launch.py
   ```
3. **Terminal 3**, preparada para una parada de emergencia: `ros2 run robcoop_lab parar`
   (termina la misión; el árbitro guarda lo registrado hasta ese momento).
4. **Terminal 2**:
   ```bash
   ros2 launch robcoop_lab p4_reto.launch.py modo:=real nombre:=real_ensayo
   ```
5. Mirad el marcador. Si alguna fase no se completa o los robots corrigen con brusquedad, ahora es
   el momento de retocar `reto_config.py` (no hace falta `cb`); anotad el cambio en `POR_QUE`.
6. Al terminar la sesión: **`modo_sim`** y volved a abrir las terminales.

> **Seguridad.** Igual que siempre: 0.15 m/s, parada si hay algo a menos de 20 cm delante y una
> persona de la pareja junto a la arena. Si algo va mal, `parar`; si no basta, levantad el robot.

## Paso 5 · Demostración (sesión 4, delante de la clase)

1. Antes de la primera demostración, **todas las parejas** hacen commit de su `reto_config.py`:
   ```bash
   cd ~/robcoop-docker/ws/src/robcoop_lab && git add -A && git commit -m "P4: configuración de la demo" && git push
   ```
   A partir de ahí la configuración no se cambia (queda registrada en el JSON de cada misión).
2. El profesor anuncia los puntos de inspección, **los mismos para todas las parejas**
   (`disposicion:=aleatoria semilla:=<n>`). La avería es `auto`: un robot al azar.
3. Cada pareja coloca los robots en las marcas, lanza `p4_real` y juega **dos intentos**
   (entre uno y otro: robots a las marcas, Ctrl+C en `p4_real` y relanzarlo):
   ```bash
   ros2 launch robcoop_lab p4_reto.launch.py modo:=real disposicion:=aleatoria semilla:=<n> nombre:=real_reto_1
   ros2 launch robcoop_lab p4_reto.launch.py modo:=real disposicion:=aleatoria semilla:=<n> nombre:=real_reto_2
   ```
4. Cuenta el **mejor marcador** de los dos; a igualdad de puntos, el menor tiempo de misión.
5. Grabad el intento con el móvil (la arena entera) y, si podéis, la pantalla de RViz: es el
   material del vídeo.

Si el día de la demostración los robots o la red no funcionan, la demostración se hace en Gazebo
con el mismo comando (sin `modo:=real`) y los puntos del profesor.

Comparación simulación / real:

```bash
cd ~/ws/src/robcoop_lab/resultados/p4
ros2 run robcoop_lab plot_p4 sim_reto.csv real_reto_1.csv real_reto_2.csv --nombre sim_vs_real
```

---

## Entrega P4

**Plazo:** una semana después de la sesión 4. No hay informe.

Todo en la terminal de Ubuntu, **fuera del contenedor** (sin `[SIM]`/`[REAL]` en el prompt).

1. **Código y resultados en GitHub.** `reto_config.py` con su `POR_QUE` y la carpeta
   `resultados/p4` (CSV, JSON, PNG y marcadores `.md` de `sim_reto`, `real_reto_1`, `real_reto_2`
   y la comparación `sim_vs_real`):
   ```bash
   cd ~/robcoop-docker/ws/src/robcoop_lab
   git status -s                          # los resultados nuevos salen con ??
   git add -A && git commit -m "P4: entrega" && git push
   git tag p4 && git push origin p4
   git ls-remote --tags origin            # tiene que aparecer refs/tags/p4
   ```
2. **Vídeo** de 2 minutos como máximo (MP4, 100 MB como máximo, o un enlace): el mejor intento
   con los robots reales, con las cuatro fases identificadas (rótulos o voz) y, al final, 20 s en
   los que contáis qué decisión de `reto_config.py` ha pesado más y qué cambió de la simulación
   a los robots reales.
3. **Campus virtual.** Una entrega por pareja con el vídeo (o su enlace) y el zip del código
   etiquetado:
   ```bash
   git archive --format=zip -o ~/p4_<apellido1>_<apellido2>.zip p4
   python3 -m zipfile -l ~/p4_*.zip | grep -E "reto_config|resultados/p4"   # comprobad que están
   WIN=$(wslpath "$(cmd.exe /c 'echo %USERPROFILE%' 2>/dev/null | tr -d '\r')")   # vuestra carpeta de Windows
   cp ~/p4_*.zip "$WIN/Downloads/"                                   # para subirlo desde Windows
   ```
   Se evalúa lo que hay en la etiqueta `p4`.

### Evaluación (sobre 10)

| Criterio | Puntos | Cómo se mide |
|---|---|---|
| Demostración en vivo | 5 | Marcador del mejor de los dos intentos con robots reales: 5 × marcador / 100 |
| Tiempo de misión | 1 | Entre las parejas con la misión completa en real: 1 la más rápida, 0.7 la segunda, 0.4 la tercera |
| Preparación en simulación | 2 | `check_p4` 6/6 en la etiqueta `p4` (1) y `sim_reto` en Gazebo con 90 puntos o más en `resultados/p4` (1) |
| Decisiones justificadas | 1 | `POR_QUE` de `reto_config.py`: cada decisión con números propios |
| Vídeo | 1 | Dos minutos como máximo, fases identificadas y comentario final |

**Opcional (hasta +1 en la nota de prácticas):** avería durante la **inspección**. Si un robot
deja de publicar su latido (`/tb3_i/heartbeat`, 2 Hz) mientras recorre su ruta, sus puntos
pendientes se vuelven a subastar entre los demás. Hacedlo en un nodo vuestro (por ejemplo
`robcoop_lab/resubasta_node.py`) sin cambiar los proporcionados, y enseñadlo en el vídeo con una
ejecución en Gazebo.

## Si algo falla

| Síntoma | Causa probable y solución |
|---|---|
| `check_p4`: «reto_config.py: TOPOLOGIA = … no existe» (o lo mismo al lanzar `p4_reto`) | Valor mal escrito en `reto_config.py`: el mensaje da las opciones |
| `check_p4` falla en una ley que antes pasaba | Mirad con `check_p1`, `check_p2` o `check_p3` qué función es. Si habéis repetido el comando del paso 0 después de programar, puede haberse sobrescrito: `git log --oneline -- robcoop_lab/<fichero>` y `git checkout <commit> -- robcoop_lab/<fichero>` |
| «Los robots no están en sus marcas de salida» y la misión no se juega | Siguen donde acabó la misión anterior: Ctrl+C en la terminal 1, relanzad `p4_sim` y repetid. En real: colocadlos en las marcas y relanzad `p4_real` |
| Se queda en `Esperando a: tb3_…` | La simulación no está lanzada, falta un robot, mezcla de `[SIM]` y `[REAL]`, o queda otra misión abierta en otra terminal |
| Los robots no avanzan y sale «el LiDAR marca 0.12 m en todos los rayos» | Cerrad la simulación y relanzadla con `render:=software` |
| «FASE 1 NO completada» | Ganancia demasiado baja para el grafo elegido (mirad el error en la figura) o `consensus_law.py` no pasa `check_p1` |
| «FASE 2 NO completada (la subasta ha fallado)» | `choose_winner` no devuelve una puja válida: `check_p2` |
| «FASE 2 NO completada (tiempo agotado)» con puntos sin visitar | Un robot se ha quedado bloqueado por otro. Mirad la figura; probad otra formación (salen de ahí) u otro criterio |
| «FASE 3 NO completada» con H cerca del Lloyd ideal | Falta tiempo para afinar los 2 cm: subid `K_COBERTURA`. En real, si el robot oscila alrededor del centroide, bajadla |
| «FASE 4 NO completada» y un robot parado con «Filtro de seguridad: obstáculo delante» | El robot averiado le corta el paso hacia su centroide. Son puntos parciales; es raro, repetid el intento |
| La misión termina con «Nadie ha averiado un robot en 45 s» | Se lanzó con `averia:=profesor` y nadie ejecutó `ros2 run robcoop_lab averia tb3_i` |
| `Package 'robcoop_lab' not found` o no existe `check_p4` | Falta compilar tras el paso 0: `cb` |
| En real, los robots no acaban donde dice RViz | Robots mal colocados en sus marcas al lanzar `p4_real` |
| En real, un robot sigue moviéndose al cerrar | `ros2 run robcoop_lab parar` y avisad al profesor |
| El profesor publica un arreglo en la plantilla cuando ya habéis configurado el reto | Repetid el comando del paso 0 añadiendo `':!robcoop_lab/reto_config.py'` a la lista de excluidos; luego `cb` |
