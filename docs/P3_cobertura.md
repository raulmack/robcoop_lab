# Práctica 3 — Cobertura con Voronoi

| | |
|---|---|
| **Duración** | sesión 3 (simulación) + trabajo en casa (solo simulación) + turno con robots en la sesión 4 |
| **Se programa** | `robcoop_lab/coverage_law.py` (cuatro funciones cortas) |
| **Entrega** | una semana después de la sesión 4: etiqueta `p3` en GitHub + zip y PDF en el campus virtual |
| **Peso** | 25 % de la nota de prácticas |

## Objetivos

1. Repartir una zona entre los robots con el diagrama de Voronoi y medir lo bien cubierta que está
   con un coste de cobertura.
2. Implementar la ley de despliegue distribuida (cada robot va al centroide de su celda) y ver su
   relación con el algoritmo de Lloyd.
3. Analizar el efecto de la ganancia, de la posición de salida, de una zona de interés y de la
   avería de un robot.
4. Ejecutar el mismo código con robots reales y analizar las diferencias.

## Plan

| Cuándo | Paso | Resultado |
|---|---|---|
| S3 0:00–0:20 | Teoría (profesor) | Voronoi, centroide, coste de cobertura, Lloyd |
| S3 0:20–0:30 | 0. Traer la P3 a vuestro repositorio | Paquete actualizado y compilado |
| S3 0:30–1:15 | 1. Programar `coverage_law.py` | `check_p3` en verde |
| S3 1:15–1:45 | 2. Primer despliegue en Gazebo | `sim_uniforme`, primera gráfica |
| S3 1:45–3:40 | 3. Ganancia y salida · 4. Zona de interés y avería · 5. Análisis de Lloyd | Seis experimentos y dos tablas |
| S3 (turnos) | Robots reales de la **P2** (30 min por pareja) | Mientras, el resto sigue con la P3 |
| S3 3:40–4:00 | Presentación del reto final | |
| Casa | Terminar los pasos 3 a 5 (solo simulación) e informe de la P2 | |
| S4 (turnos) | 6. Robots reales de la P3 (20 min por pareja) | `real_uniforme`, `real_fallo` |

---

## Teoría mínima

Hay **N robots** y una zona **Q** que vigilar (aquí, un cuadrado de 3 × 3 m centrado en la arena).
Un punto q de la zona está tanto mejor atendido cuanto más cerca tiene un robot. Si cada punto lo
atiende su robot más cercano, la zona queda repartida en **celdas de Voronoi**:

```
V_i = { q de Q :  |q − p_i| <= |q − p_j|  para todo j }      (lo que le queda más cerca al robot i)
```

**Coste de cobertura.** Mide lo mal cubierta que está la zona: cada punto aporta su importancia
φ(q) por el cuadrado de la distancia a su robot más cercano. Cuanto **menor**, mejor.

```
H(p_1 … p_N) = Σ_i ∫_{V_i} φ(q) · |q − p_i|² dq
```

φ(q) es la **densidad**: vale 1 en todas partes si todos los puntos importan lo mismo (`uniforme`)
y más en una zona de interés (`zona`). Con φ = 1, H se mide en m⁴; para interpretarlo se da también
la distancia cuadrática media de un punto a su robot, `d = sqrt(H / masa total)`, en metros.

**Centroide y ley de control.** La masa y el centroide de una celda son

```
M_i = ∫_{V_i} φ(q) dq          C_i = (1 / M_i) · ∫_{V_i} φ(q) · q dq
```

y resulta que el gradiente del coste es `∂H/∂p_i = −2 M_i (C_i − p_i)`: moverse hacia el centroide
de la propia celda es bajar por H. La ley de despliegue (Cortés et al., 2004) es

```
u_i = k · (C_i − p_i)
```

Mientras los robots se mueven, las celdas cambian y los centroides también; el equipo se para
cuando cada robot está en el centroide de su celda (*configuración centroidal*). Es la versión
continua del **algoritmo de Lloyd** (calcular celdas → saltar al centroide → repetir). H nunca
sube, pero solo se garantiza un **mínimo local**: el resultado puede depender de la salida.

**Es distribuido.** Cada robot calcula *su* celda con *su* posición y las que le llegan de los
demás; nadie reparte la zona. Y es **tolerante a fallos**: si un robot deja de publicar, los demás
dejan de contar con él y sus celdas crecen hasta ocupar el hueco.

```
/tb3_i/coverage/state      robot i → todos       su posición (punto de control); hace de LATIDO
/tb3_i/coverage/centroid   robot i → RViz        centroide de su celda
/coverage/enable           experimento → todos   activar / parar la cobertura
/tb3_i/coverage/fail       experimento → robot i avería simulada: se para y deja de publicar
```

**En el ordenador: una rejilla.** La zona se divide en casillas de 5 cm (60 × 60 = 3600) y cada
casilla se representa por su centro. Las integrales pasan a ser sumas (ΔA = área de una casilla):

```
C_i = Σ φ(q) · q / Σ φ(q)   (casillas de V_i)          H = Σ_q φ(q) · min_i |q − p_i|² · ΔA
```

Como en la P1, el «robot» de las fórmulas es su **punto de control** (10 cm delante del eje), y la
velocidad deseada se satura a 0.15 m/s y se convierte en (v, ω) con la misma cinemática.

## Qué hay en el paquete

| Fichero | Qué hace | ¿Se toca? |
|---|---|---|
| `coverage_law.py` | `voronoi_mask`, `centroid`, `coverage_velocity`, `coverage_cost` | **Sí** |
| `coverage_node.py` | Nodo de un robot: publica su posición, calcula su celda y va al centroide | No |
| `coverage_experiment_node.py` | Activa, registra CSV + JSON, provoca la avería y dibuja en RViz | No |
| `coverage_core.py` | Rejilla, densidades, salidas, Lloyd ideal y funciones de referencia | No (mirad los parámetros) |
| `kinsim_p3.py`, `check_p3.py`, `compare_p3.py` | Simulador rápido, comprobación y análisis de Lloyd (sin ROS) | No |
| `plot_p3.py` | Gráficas y tablas a partir de los CSV/JSON | No |

Salidas (3 robots por defecto; el equipo se despliega igual con 2 o con 4):

```
marcas     las marcas del suelo de la P1 y la P2, repartidas por la arena
agrupados  los tres robots juntos en una esquina de la zona (solo en simulación)
```

Zona de interés (`densidad:=zona`): centrada en (0.6, −0.6), de unos 0.5 m de radio y 12 veces más
importante que el resto. Se puede cambiar con `centro:="x,y"`, `sigma:=` y `pico:=`. Con varias
zonas, sus centros separados por `;`: `centro:="0.6,-0.6;-0.8,0.8"` (todas con el mismo `sigma` y
`pico`). Para probarlo sin ROS: `ros2 run robcoop_lab kinsim_p3 --densidad zona --centro "0.6,-0.6;-0.8,0.8"`.

El despliegue se da por terminado cuando todos los robots están a menos de **2 cm** de su centroide
durante **3 s** seguidos. No hay evitación de obstáculos: cada robot va en línea recta hacia un
punto de su propia celda, que es convexa, así que no se cruza con los demás. El filtro de seguridad
de la P1 sigue activo.

---

## Paso 0 · Traer la P3 a vuestro repositorio (10 min)

Como en la P2, hay que traer los ficheros nuevos de la plantilla **sin tocar lo que habéis
programado ni vuestros resultados**. En la terminal de Ubuntu (WSL), *fuera* del contenedor:

```bash
cd ~/robcoop-docker/ws/src/robcoop_lab
git status -s                        # no debe salir nada: si sale, commit y push antes de seguir
git fetch plantilla
git checkout plantilla/main -- . ':!robcoop_lab/consensus_law.py' ':!robcoop_lab/formations.py' ':!robcoop_lab/graphs.py' ':!robcoop_lab/auction_law.py' ':!robcoop_lab/tasks.py' ':!resultados'
git commit -m "P3: material de la plantilla" && git push
```

Si `git fetch plantilla` dice que no existe ese remoto (por ejemplo, en un PC nuevo), añadidlo antes:
`git remote add plantilla https://github.com/raulmack/robcoop_lab.git`.

Compilar de nuevo dentro del contenedor (hay ejecutables nuevos) y comprobar:

```bash
cd ~/robcoop-docker && ./robcoop.sh shell
cb
ros2 run robcoop_lab check_p3        # debe FALLAR: aún no habéis programado nada
ros2 run robcoop_lab check_p2        # la P2 sigue igual (6/6)
```

> El prompt debe empezar por **`[SIM]`**. Si pone `[REAL]`, ejecutad `modo_sim`.

## Paso 1 · Programar `coverage_law.py` (45 min)

Abrid `ws/src/robcoop_lab/robcoop_lab/coverage_law.py` en VS Code y completad los cuatro `TODO`:

1. `voronoi_mask(points, p_i, p_others)`: qué casillas de la rejilla son del robot i.
2. `centroid(points, weights, mask)`: centroide de esas casillas, ponderado por la densidad
   (o `None` si la celda no tiene masa).
3. `coverage_velocity(p_i, c_i, k)`: la ley de control.
4. `coverage_cost(points, weights, robots, cell_area)`: el coste H del equipo.

Las cuatro se hacen con `numpy` sin recorrer las casillas una a una. Si no lo habéis usado mucho,
mirad antes las diapositivas «Python que vais a necesitar»: arrays (M, 2) y `axis=1`,
`np.linalg.norm`, máscaras booleanas (`d_i <= d_j`, `mask & otra`, `points[mask]`), `np.sum`,
`np.minimum`, `np.full(M, np.inf)` y devolver `None`. Las diapositivas «Las funciones, dibujadas»
muestran con una rejilla de 6 × 6 casillas qué hace cada función paso a paso y en qué orden las
llama el nodo de cada robot.

Comprobar, sin ROS:

```bash
cd ~/ws/src/robcoop_lab
python3 -m pytest test/test_coverage_law.py -v                    # 5 pruebas
ros2 run robcoop_lab check_p3                                     # 6/6 bloques correctos
ros2 run robcoop_lab kinsim_p3 --plot                             # despliegue en 2 segundos
ros2 run robcoop_lab kinsim_p3 --salida agrupados --densidad zona --plot
```

`check_p3` comprueba las funciones con casos hechos a mano y con la rejilla real, repite con ellas
el algoritmo de Lloyd ideal (deben salir los costes esperados y H no puede subir nunca) y simula
el despliegue con robots diferenciales, también con zona de interés y con una avería.

**Guardad el avance en la terminal de Ubuntu, fuera del contenedor** (sin `[SIM]` en el prompt).
Dentro del contenedor `git` no sabe quiénes sois y el commit falla con «Author identity unknown»:

```bash
cd ~/robcoop-docker/ws/src/robcoop_lab
git add -A && git commit -m "P3: coverage_law" && git push
```

## Paso 2 · Primer despliegue en Gazebo (30 min)

**Terminal 1** (3 robots en sus marcas, sin ventana de Gazebo; se abre RViz):

```bash
ros2 launch robcoop_lab p3_sim.launch.py
```

**Terminal 2** (experimento):

```bash
ros2 launch robcoop_lab p3_cobertura.launch.py nombre:=sim_uniforme
```

El nodo del experimento espera a los 3 robots, comprueba que están en una posición de salida,
activa la cobertura y **termina solo** cuando el equipo se ha desplegado (o a los 120 s). En RViz
se ve la zona repartida en celdas (cada una con el color de su robot), el centroide de cada celda
(esfera) y, arriba, la fase, el coste H y el error `e = max |C_i − p_i|`. Al final escribe el resumen y
guarda `resultados/p3/sim_uniforme.csv` y `.json`. Debe salir algo parecido a esto: desplegados en
unos 11 s y H de 9.45 a 5.37 m⁴ (el Lloyd ideal llega a 5.36). Gráficas:

```bash
ros2 run robcoop_lab plot_p3 ~/ws/src/robcoop_lab/resultados/p3/sim_uniforme.csv
```

Para ver las figuras desde Windows (y llevarlas al informe), abrid la carpeta de resultados en el
Explorador desde la terminal de Ubuntu, fuera del contenedor:
`explorer.exe "$(wslpath -w ~/robcoop-docker/ws/src/robcoop_lab/resultados/p3)"`.

Mirad lo que publica un robot mientras se repite el experimento: `ros2 topic echo
/tb3_0/coverage/state`.

**Grafo de nodos para el informe.** En una tercera terminal (dentro del contenedor), con la
simulación en marcha: `ros2 run rqt_graph rqt_graph --clear-config` (sin `--clear-config` puede
abrirse partido en dos ventanas que no responden). **Antes** de lanzar el experimento elegid
**Nodes/Topics (all)** y marcad **Hide: tf** (si no, las flechas de `/tf` tapan todo). Repetid el
experimento con `k:=0.1 nombre:=sim_k01`, que dura unos 50 s, y en cuanto salga «Cobertura
ACTIVADA» pulsad refrescar. Guardadlo con el icono *Save as image* como
`/home/ros/ws/src/robcoop_lab/resultados/p3/grafo.png` (no con el nombre que propone).

> **Para repetir desde la salida:** Ctrl+C en la terminal 1 y volver a lanzar. Al acabar un
> experimento los robots se quedan donde terminaron; el nodo del experimento comprueba que cada
> robot está a menos de 15 cm de su posición de salida y, si no, cancela sin guardar nada.

> **Si los robots no se mueven** y sale «el LiDAR marca 0.12 m en todos los rayos», relanzad la
> simulación con `ros2 launch robcoop_lab p3_sim.launch.py render:=software`, como en la P2.

## Paso 3 · Ganancia y posición de salida (en simulación; se puede terminar en casa)

**Antes de cada experimento, Ctrl+C en la terminal 1 y relanzad `p3_sim`.**

```bash
ros2 launch robcoop_lab p3_cobertura.launch.py k:=0.1 nombre:=sim_k01
ros2 launch robcoop_lab p3_cobertura.launch.py k:=2.0 nombre:=sim_k2
```

Para el tercero, los robots salen juntos de una esquina. En la terminal 1:
`ros2 launch robcoop_lab p3_sim.launch.py salida:=agrupados`, y en la terminal 2:

```bash
ros2 launch robcoop_lab p3_cobertura.launch.py nombre:=sim_agrupados
```

**Después de cada experimento, mirad en el resumen la línea `salida …` y el H inicial**: 9.45 m⁴
desde las marcas y 31.07 m⁴ desde `agrupados`. Si el nombre del experimento dice una salida y los
robots han salido de otra (lo típico: relanzar `p3_sim` con la flecha arriba y sin
`salida:=agrupados`), el experimento avisa con «OJO» al empezar y en el resumen: repetidlo.

Figura comparativa de las tres ganancias y tabla resumen:

```bash
cd ~/ws/src/robcoop_lab/resultados/p3
ros2 run robcoop_lab plot_p3 sim_k01.csv sim_uniforme.csv sim_k2.csv --nombre ganancias
```

Para el informe:

* ¿Cómo cambia el tiempo de despliegue con k? De k = 0.5 a k = 2 la ganancia se multiplica por 4:
  ¿por qué el tiempo no se divide entre 4? ¿Qué pasaría con una k mucho mayor en un robot real?
* Desde `agrupados`, ¿llega el equipo a las mismas posiciones que desde las marcas? ¿Y al mismo
  coste H? ¿Qué robot recorre más y por qué?

## Paso 4 · Zona de interés y avería de un robot (en simulación)

Con `p3_sim` relanzada en las **marcas** antes de cada uno:

```bash
ros2 launch robcoop_lab p3_cobertura.launch.py densidad:=zona nombre:=sim_zona
ros2 launch robcoop_lab p3_cobertura.launch.py fallo:=tb3_1 nombre:=sim_fallo
```

Con `densidad:=zona` aparece en RViz un círculo amarillo en la zona de interés. Con
`fallo:=tb3_1`, cuando el equipo ya se ha desplegado, `tb3_1` se para y deja de publicar su
posición (en RViz aparece `AVERIADO` en rojo encima de él): en la terminal se ve cómo los otros dos lo dan por caído («no responde: lo excluyo») y
se recolocan. `plot_p3` de cada uno escribe, además de la figura, la tabla de celdas finales (área
y masa de cada celda).

Para el informe:

* Con la zona de interés, ¿cómo cambian las posiciones finales, las áreas y las masas de las
  celdas respecto al caso uniforme? ¿Por qué tarda más en desplegarse?
* En la avería: ¿cuánto vale H justo antes, justo después y cuando los dos robots restantes se han
  recolocado? Comparadlo con el Lloyd ideal de 2 robots. ¿Cuánto tardan en detectar la avería y
  cómo lo hacen, si nadie les avisa?

## Paso 5 · ¿Siempre se llega a lo mismo? (sin ROS, en segundos)

```bash
ros2 run robcoop_lab compare_p3
ros2 run robcoop_lab compare_p3 --densidad zona --nombre comparacion_zona
```

Con vuestras funciones, repite el algoritmo de Lloyd ideal con 1 a 6 robots desde 20 salidas al
azar (y desde `marcas` y `agrupados`) y guarda tabla (`.md`), datos (`.csv`) y figura (`.png`) en
`resultados/p3`: la mejor configuración con cada número de robots, el coste frente al número de
robots y el coste final de cada salida.

* ¿Con cuántos robots hay salidas que acaban en una configuración peor que la mejor? ¿Cuánto peor?
  Fijaos en 2 robots desde `agrupados`: ¿por qué se quedan en una configuración peor? ¿Qué
  significa para un equipo real que Lloyd solo garantice un mínimo local?
* ¿Cuánto mejora la cobertura al añadir un robot? Mirad la columna `N · mejor H`: ¿cómo depende H
  del número de robots? ¿Compensa el quinto robot?

## Paso 6 · Robots reales (sesión 4, turno de 20 min)

En la arena están las marcas de salida de siempre; la zona a cubrir es el cuadrado de 3 × 3 m
centrado en la arena.

1. `modo_real` y volved a abrir las terminales (`exit` y `./robcoop.sh shell`).
2. Colocad `tb3_0`, `tb3_1` y `tb3_2` sobre sus marcas, mirando en la dirección de la flecha.
3. **Terminal 1**, anclar los robots y abrir RViz con la cobertura:
   ```bash
   ros2 launch robcoop_lab p3_real.launch.py
   ```
4. **Terminal 3**, preparada para una parada de emergencia: `ros2 run robcoop_lab parar`
   (para también la cobertura y el experimento guarda lo registrado).
5. **Terminal 2**, mismo código y parámetros que en simulación:
   ```bash
   ros2 launch robcoop_lab p3_cobertura.launch.py modo:=real nombre:=real_uniforme
   ```
6. Volved a colocar los robots en sus marcas, Ctrl+C en la terminal 1, relanzadla y repetid con la
   avería: `modo:=real fallo:=tb3_1 nombre:=real_fallo`.
7. Al terminar: **`modo_sim`** y volved a abrir las terminales.

Comparación simulación / real:
`ros2 run robcoop_lab plot_p3 sim_uniforme.csv real_uniforme.csv sim_fallo.csv real_fallo.csv --nombre sim_vs_real`.

> **Seguridad.** Igual que en la P1 y la P2: 0.15 m/s, parada si hay algo a menos de 20 cm delante
> y una persona de la pareja siempre junto a la arena. Si algo va mal, `parar`; si no basta,
> levantad el robot.

---

## Entrega P3

**Plazo:** una semana después de la sesión 4.

Todo en la terminal de Ubuntu, **fuera del contenedor** (sin `[SIM]`/`[REAL]` en el prompt).

1. **Código y resultados en GitHub.** `coverage_law.py` y la carpeta `resultados/p3` (CSV, JSON,
   PNG y tablas `.md` que uséis en el informe):
   ```bash
   cd ~/robcoop-docker/ws/src/robcoop_lab
   git status -s                          # los resultados nuevos salen con ??
   git add -A && git commit -m "P3: entrega" && git push
   git tag p3 && git push origin p3
   git ls-remote --tags origin            # tiene que aparecer refs/tags/p3
   ```
   Si cambiáis algo después de etiquetar: commit y push, y moved la etiqueta con
   `git tag -f p3 && git push -f origin p3`.
2. **Informe en PDF** (3 páginas como máximo) con la plantilla `docs/plantilla_informe_P3.docx`
   (desde Windows: `\\wsl$\<distro>\home\<usuario>\robcoop-docker\ws\src\robcoop_lab\docs`):
   * tabla de los cinco despliegues en Gazebo y el real, y tabla de la avería en Gazebo y en real
     (los números los escribe `plot_p3`);
   * tabla de `compare_p3` con 1 a 6 robots;
   * figuras: `sim_uniforme` y `sim_zona`, `sim_fallo` y `comparacion_lloyd`;
   * captura de `rqt_graph`;
   * respuestas razonadas a las preguntas de los pasos 3, 4 y 5 y análisis simulación / real.
3. **Campus virtual.** Una entrega por pareja con el PDF y el zip del código etiquetado:
   ```bash
   git archive --format=zip -o ~/p3_<apellido1>_<apellido2>.zip p3
   python3 -m zipfile -l ~/p3_*.zip | grep -E "coverage_law|resultados/p3"   # comprobad que están
   WIN=$(wslpath "$(cmd.exe /c 'echo %USERPROFILE%' 2>/dev/null | tr -d '\r')")   # vuestra carpeta de Windows
   cp ~/p3_*.zip "$WIN/Downloads/"                                   # para subirlo desde Windows
   ```
   Se evalúa lo que hay en la etiqueta `p3`: si los resultados no están en GitHub con la etiqueta,
   no cuentan.

### Evaluación (sobre 10)

| Criterio | Puntos | Cómo se mide |
|---|---|---|
| Funcionamiento en simulación | 3 | `check_p3` en la etiqueta `p3` y experimentos de los pasos 2–5 en `resultados/p3` |
| Funcionamiento en robots reales | 2 | Experimentos `real_uniforme` y `real_fallo` con el mismo código y parámetros |
| Calidad del código | 2 | Claro, comentado, con `numpy` y sin valores fijos; commits de los dos miembros |
| Informe: resultados y análisis | 3 | Tablas con números, efecto de k, salida, zona y avería razonados, sim/real |

**Opcional (hasta +1 en la nota de prácticas):** repetid los experimentos `uniforme` y `zona` con
4 robots (`p3_sim.launch.py num_robots:=4` y `robots:=tb3_0,tb3_1,tb3_2,tb3_3`) y con una zona
de interés vuestra (`centro`, `sigma`, `pico`), y analizad cómo se reparten áreas y masas. Probad
también **dos zonas de interés** con 3 y con 4 robots, por ejemplo
`densidad:=zona centro:="0.6,-0.6;-0.8,0.8"`: ¿cuántos robots acaban en cada zona y por qué? ¿Cambia
si salen de `agrupados`?

En el reto final de la sesión 4 ([P4](P4_reto.md)) la cobertura es la última fase: un robot se
averiará y los demás tendrán que recolocarse, como en `sim_fallo`.

## Si algo falla

| Síntoma | Causa probable y solución |
|---|---|
| Los robots no se mueven y sale «Sin centroide (¿centroid sin programar o celda vacía?)» | `centroid` devuelve `None`: falta programarla o `voronoi_mask` deja la celda vacía. Pasad `check_p3` |
| Los robots no avanzan y sale «el LiDAR marca 0.12 m en todos los rayos» | El LiDAR simulado no mide con la GPU de ese PC: cerrad la simulación y relanzadla con `render:=software` |
| «Los robots no están en una posición de salida» y el experimento acaba sin guardar | Siguen donde acabó el experimento anterior: Ctrl+C en la terminal 1, relanzad `p3_sim` y repetid. En real: colocadlos en las marcas y relanzad `p3_real` |
| `Package 'robcoop_lab' not found` o no existe `check_p3` | Falta compilar tras el paso 0: `cb` |
| `git commit` falla con «Author identity unknown» | Estáis dentro del contenedor (`[SIM]` en el prompt). Las órdenes `git` van en la terminal de Ubuntu, fuera: `exit` o abrid otra terminal |
| «OJO: el nombre … dice «agrupados», pero los robots salen de «marcas»» | Se lanzó `p3_sim` sin `salida:=agrupados` (o al revés). Ctrl+C, relanzad `p3_sim` con la salida correcta y repetid el experimento: se sobrescribe |
| `rqt_graph` se abre en dos ventanas, una vacía, y no responde | Recupera una configuración guardada que falla en WSL: cerradlo y lanzadlo con `ros2 run rqt_graph rqt_graph --clear-config`. Si aun así no va, captura de pantalla con Win + Mayús + S |
| Se queda en `Esperando a: tb3_…` | La simulación no está lanzada, falta un robot o mezcla de `[SIM]` y `[REAL]` |
| El resumen dice «OJO: vuestra coverage_cost da … y la de referencia …» | `coverage_cost` está mal (¿falta ΔA o el cuadrado?): pasad `check_p3` |
| El experimento acaba por `tiempo agotado` | El equipo no ha llegado a 2 cm de sus centroides: mirad el error en la gráfica. Con k muy baja hace falta más tiempo (`timeout:=200`) |
| Un robot se queda parado con «Filtro de seguridad: obstáculo delante» | Tiene algo a menos de 20 cm delante (otro robot o una persona en real). Retirad el obstáculo; en simulación, relanzad |
| En real, los robots no acaban donde dice RViz | Robots mal colocados en sus marcas al lanzar `p3_real` |
| En real, el robot sigue moviéndose al cerrar | `ros2 run robcoop_lab parar` y avisad al profesor |
| Vuestro repositorio no está en este PC | Como en la P1: `gh auth login`, `gh auth setup-git`, `git config --global user.name/user.email` y `git clone` de vuestro repo en `~/robcoop-docker/ws/src`; después, el paso 0 |
| `git push`: «Invalid username or token» / «Authentication failed» | `gh auth status` (si no estáis conectados, `gh auth login`) y después `gh auth setup-git`; repetid el comando |
| El profesor publica un arreglo en la plantilla cuando ya habéis programado | **No** repitáis el comando del paso 0 tal cual: sobrescribiría vuestra `coverage_law.py` con la vacía. Añadidla a la lista de excluidos: `':!robcoop_lab/coverage_law.py'`; luego `cb`. Si ya os ha pasado y no habéis hecho commit: `git checkout HEAD -- robcoop_lab/coverage_law.py` |
