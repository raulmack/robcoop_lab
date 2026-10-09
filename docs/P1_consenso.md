# Práctica 1 — Formación por consenso

| | |
|---|---|
| **Duración** | 1 sesión de 4 h, por parejas |
| **Se programa** | `robcoop_lab/consensus_law.py` (tres funciones cortas) |
| **Entrega** | una semana después de la sesión: etiqueta `p1` en GitHub + zip en el campus virtual |
| **Peso** | 30 % de la nota de prácticas |

## Objetivos

1. Modelar un equipo de robots como un grafo y calcular su laplaciano y su conectividad
   algebraica λ2.
2. Implementar una ley de consenso distribuida: cada robot solo usa la información de sus vecinos.
3. Pasar de la simulación a los robots reales con el mismo código y analizar las diferencias.

## Plan de la sesión

| Tiempo | Paso | Resultado |
|---|---|---|
| 0:00–0:20 | Teoría (profesor) | Grafo, laplaciano, λ2, ley de consenso |
| 0:20–0:40 | 1. Repositorio de la pareja | Repo creado, clonado y compilado |
| 0:40–1:10 | 2. Programar la ley | `check_p1` en verde |
| 1:10–1:35 | 3. Primer experimento en Gazebo | Reunión de 4 robots, primera gráfica |
| 1:35–2:30 | 4. Topologías | 4 experimentos y tabla λ2 / tiempo de convergencia |
| 2:30–3:45 | 5. Formaciones y perturbación (simulación) | Línea, polígono y recuperación tras un empujón |
| 2:30–3:45 | 6. Robots reales, **turnos de 20 min** | Mismo código en la arena |
| 3:45–4:00 | Cierre | Qué entregar y cómo |

Mientras una pareja está en la arena, las demás siguen con el paso 5 en simulación.

---

## Teoría mínima

Cada robot *i* se trata como un punto que se mueve con la velocidad que se le ordena
(integrador simple). Con su posición *q_i*, su offset en la formación *δ_i* y sus vecinos *N_i*:

```
ξ_i = q_i − δ_i                         variable de consenso
u_i = −k · Σ_{j ∈ N_i} (ξ_i − ξ_j)      ley de consenso
```

* Si el grafo es conexo, todas las ξ_i convergen a la media de sus valores iniciales: los
  robots acaban en la formación definida por los δ_i, centrada en ese punto.
* En forma matricial ξ̇ = −k L ξ, con L = D − A el **laplaciano** del grafo. Sus autovalores son
  0 = λ1 ≤ λ2 ≤ … y el error decae aproximadamente como e^(−k λ2 t). A λ2 se le llama
  **conectividad algebraica**: cuanto mayor, más rápido es el consenso.
* Un TurtleBot3 no puede moverse de lado. El paquete controla un *punto de control* situado
  10 cm por delante del eje de las ruedas y convierte (u_x, u_y) en (v, ω) de forma exacta.
  Por eso *q_i* es ese punto y no el centro del robot. La velocidad está limitada a 0.15 m/s.
* Con δ_i = 0 todos irían al mismo punto, y eso no es posible con robots reales. La formación
  `reunion` usa un círculo de 30 cm de radio.

## Qué hay en el paquete

| Fichero | Qué hace | ¿Se toca? |
|---|---|---|
| `consensus_law.py` | `laplacian`, `algebraic_connectivity`, `consensus_velocity` | **Sí** |
| `consensus_node.py` | Nodo de un robot: publica su ξ_i, escucha solo a sus vecinos, aplica la ley | No |
| `experiment_node.py` | Activa el consenso, mide el error, para y guarda CSV + JSON | No |
| `graphs.py`, `formations.py` | Topologías (completo, anillo, cadena, estrella) y offsets | Se pueden ampliar |
| `kinematics.py`, `safety.py`, `robot_api.py` | Punto de control, filtro de seguridad, interfaz con el robot | No |
| `kinsim.py`, `check_p1.py` | Simulador rápido sin ROS y comprobación (la que usa el profesor) | No |
| `plot_p1.py` | Gráficas y tabla a partir de los CSV | No |
| `anchor_node.py` | Robots reales: une la odometría de cada robot al marco `map` de la arena | No |

Topologías con 4 robots (`ros2 run robcoop_lab graph_info anillo` muestra A y los vecinos):

```
completo: todos con todos     anillo: 0-1-2-3-0     cadena: 0-1-2-3     estrella: 0 con 1, 2 y 3
```

---

## Paso 1 · Repositorio de la pareja (20 min)

**Un miembro de la pareja**, en GitHub:

1. Abrir `https://github.com/raulmack/robcoop_lab` → **Use this template** → *Create a new repository*.
2. Nombre `robcoop-<apellido1>-<apellido2>`, visibilidad **Private**.
3. *Settings → Collaborators* → añadir a la pareja y al profesor (`raulmack`).

**Los dos**, en la terminal de Ubuntu (WSL), *fuera* del contenedor. Solo la primera vez, para
poder hacer `git push`:

```bash
sudo apt install -y gh
gh auth login          # GitHub.com → HTTPS → Login with a web browser
gh auth setup-git
git config --global user.name  "Nombre Apellido"
git config --global user.email "usuario@users.noreply.github.com"   # GitHub → Settings → Emails
```

Clonar **con el nombre `robcoop_lab`** dentro del workspace:

```bash
cd ~/robcoop-docker/ws/src
git clone https://github.com/<usuario>/robcoop-<apellido1>-<apellido2>.git robcoop_lab
```

Compilar y comprobar dentro del contenedor:

```bash
cd ~/robcoop-docker && ./robcoop.sh shell
cb
ros2 run robcoop_lab check_p1        # debe FALLAR: aún no habéis programado nada
```

> El prompt debe empezar por **`[SIM]`** en verde. Si pone `[REAL]`, ejecutad `modo_sim`.

## Paso 2 · Programar la ley de consenso (30 min)

Abrid `ws/src/robcoop_lab/robcoop_lab/consensus_law.py` en VS Code y completad los tres `TODO`:

1. `laplacian(A)`: L = D − A.
2. `algebraic_connectivity(L)`: segundo autovalor más pequeño de L.
3. `consensus_velocity(xi_i, xi_neighbors, k)`: la ley de consenso.

Comprobar, sin ROS:

```bash
cd ~/ws/src/robcoop_lab
python3 -m pytest test -v                              # 4 pruebas
ros2 run robcoop_lab check_p1                          # 4/4 bloques correctos
ros2 run robcoop_lab kinsim --topologia anillo --plot  # ver el resultado en una ventana
```

Como el workspace se compila con `--symlink-install`, no hace falta volver a compilar tras editar
un `.py`. Guardad el avance:

```bash
git add -A && git commit -m "P1: ley de consenso" && git push    # desde WSL o desde VS Code
```

## Paso 3 · Primer experimento en Gazebo (25 min)

**Terminal 1** (simulación: 4 robots en las marcas de salida, sin ventana de Gazebo; se abre RViz):

```bash
ros2 launch robcoop_lab p1_sim.launch.py
```

**Terminal 2** (experimento):

```bash
ros2 launch robcoop_lab p1_consensus.launch.py topologia:=completo formacion:=reunion
```

El experimento espera a tener la pose de los 4 robots, activa el consenso, mide el error y
**termina solo** al converger (error < 5 cm durante 3 s) o a los 120 s. Al final escribe el
resumen y dónde ha guardado los datos (`resultados/p1/*.csv` y `*.json`). Para las gráficas:

```bash
ros2 run robcoop_lab plot_p1 ~/ws/src/robcoop_lab/resultados/p1/<fichero>.csv
```

Se genera un PNG junto al CSV (se abre en VS Code): trayectorias y error de consenso en escala
logarítmica.

> **Para repetir desde las marcas:** Ctrl+C en la terminal 1 y volver a lanzar (unos 15 s). Los
> robots no vuelven solos a su sitio.

## Paso 4 · Influencia de la topología (55 min)

Repetid el experimento con las cuatro topologías, relanzando la simulación antes de cada una, y
ponedles nombre para encontrarlas después:

```bash
ros2 launch robcoop_lab p1_consensus.launch.py topologia:=completo nombre:=topo_completo
ros2 launch robcoop_lab p1_consensus.launch.py topologia:=anillo   nombre:=topo_anillo
ros2 launch robcoop_lab p1_consensus.launch.py topologia:=estrella nombre:=topo_estrella
ros2 launch robcoop_lab p1_consensus.launch.py topologia:=cadena   nombre:=topo_cadena
```

Comparación de las cuatro (figura `topologias.png` y tabla en Markdown para el informe):

```bash
cd ~/ws/src/robcoop_lab/resultados/p1
ros2 run robcoop_lab plot_p1 topo_*.csv --nombre topologias
```

Para el informe:

* Calculad λ2 de cada topología **con vuestra función** y comparadlo con el tiempo de
  convergencia medido. ¿Se cumple t_conv ∝ 1/(k·λ2)?
* Medid la pendiente de log(error) en la parte recta de cada curva y comparadla con −k·λ2.
* ¿Por qué la estrella, con 3 enlaces, es más lenta que el anillo, con 4?
* ¿Por qué el principio de la curva del grafo completo no es una recta? (pista: 0.15 m/s).
* Opcional: repetid una topología con `k:=0.05` y `k:=0.2`.

## Paso 5 · Formaciones y perturbación (en simulación)

```bash
ros2 launch robcoop_lab p1_consensus.launch.py formacion:=linea    nombre:=form_linea
ros2 launch robcoop_lab p1_consensus.launch.py formacion:=poligono nombre:=form_poligono
ros2 launch robcoop_lab p1_consensus.launch.py formacion:=poligono perturbar:=tb3_1 nombre:=perturbacion
```

Con `perturbar:=tb3_1`, cuando la formación converge se empuja a `tb3_1` durante 3 s
(0.15 m/s hacia +y) y se mide cuánto tarda en recuperarse (`t_reconv` en el resumen). Repetidlo
con `topologia:=cadena`: ¿cambia la recuperación? ¿Se recoloca toda la formación o solo el
robot empujado? ¿Por qué?

## Paso 6 · Robots reales (turno de 20 min)

Antes de vuestro turno el profesor tiene los robots encendidos y lanzados. Cada robot lleva su
nombre (`tb3_0`…`tb3_3`) y cada uno tiene **su marca** en el suelo con una flecha.

1. Pasad a modo real y abrid las terminales de nuevo:
   ```bash
   modo_real        # el prompt pasa a [REAL] en rojo
   exit             # y volver a entrar:  ./robcoop.sh shell  (en cada terminal)
   ```
2. Colocad cada robot sobre su marca, mirando en la dirección de la flecha.
3. **Terminal 1**, anclar los robots al marco de la arena y abrir RViz:
   ```bash
   ros2 launch robcoop_lab p1_real.launch.py
   ```
   Comprobad en RViz que los 4 robots aparecen en sus marcas.
4. **Terminal 3**, preparada para una parada de emergencia (no la ejecutéis todavía):
   ```bash
   ros2 run robcoop_lab parar
   ```
5. **Terminal 2**, mismo código y mismos parámetros que en simulación:
   ```bash
   ros2 launch robcoop_lab p1_consensus.launch.py modo:=real topologia:=completo formacion:=reunion nombre:=real_completo_reunion
   ```
6. Volved a colocar los robots en sus marcas, Ctrl+C en la terminal 1, relanzadla y repetid con
   `formacion:=poligono nombre:=real_completo_poligono`. Si queda tiempo, `topologia:=anillo`.
7. Al terminar el turno: **`modo_sim`** y volved a abrir las terminales.

Para comparar simulación y realidad en la misma figura:

```bash
ros2 run robcoop_lab plot_p1 topo_completo.csv real_completo_reunion.csv --nombre sim_vs_real
```

> **Seguridad.** Velocidad limitada a 0.15 m/s y parada automática si el LiDAR ve algo a menos
> de 20 cm delante. Pero un TurtleBot3 **sigue moviéndose con la última orden** si su programa de
> control muere: si algo va mal, `ros2 run robcoop_lab parar` y, si no basta, levantad el robot.
> Una persona de la pareja está siempre junto a la arena.

Si falta un robot, trabajad con 3: `robots:=tb3_0,tb3_1,tb3_2` en los dos lanzamientos.

---

## Entrega P1

**Plazo:** una semana después de la sesión.

1. **Código y resultados en GitHub.** Subid `consensus_law.py` y la carpeta `resultados/p1`
   (CSV, JSON y PNG de los experimentos que uséis en el informe). Marcad la versión entregada:
   ```bash
   git add -A && git commit -m "P1: entrega" && git push
   git tag p1 && git push origin p1
   ```
2. **Informe en PDF** (3 páginas como máximo) con la plantilla `docs/plantilla_informe_P1.docx`.
   Debe incluir, como mínimo:
   * tabla de las cuatro topologías: λ2 calculado, k, t_conv y error final;
   * figura comparativa del error con las cuatro topologías;
   * una figura de formación y la de la perturbación, con t_reconv;
   * comparación simulación / real (figura o tabla con t_conv y error final);
   * respuestas razonadas a las preguntas de los pasos 4 y 5 y análisis de las diferencias
     entre simulación y realidad (odometría, zona muerta de los motores, retardos, etc.).
3. **Campus virtual.** Una sola entrega por pareja con el PDF y el zip del código etiquetado:
   ```bash
   cd ~/robcoop-docker/ws/src/robcoop_lab
   git archive --format=zip -o ~/p1_<apellido1>_<apellido2>.zip p1
   cp ~/p1_*.zip /mnt/c/Users/<usuario_windows>/Downloads/   # para subirlo desde Windows
   ```

### Evaluación (sobre 10)

| Criterio | Puntos | Cómo se mide |
|---|---|---|
| Funcionamiento en simulación | 3 | `check_p1` en la etiqueta `p1` y experimentos de los pasos 3–5 en `resultados/p1` |
| Funcionamiento en robots reales | 2 | Experimentos `real_*` con el mismo código y parámetros |
| Calidad del código | 2 | Claro, comentado, sin valores fijos; commits de los dos miembros |
| Informe: resultados y análisis | 3 | Figuras con números, λ2 relacionado con t_conv, comparación sim/real razonada |

## Si algo falla

| Síntoma | Causa probable y solución |
|---|---|
| `Package 'robcoop_lab' not found` | Falta compilar: `cb` |
| El experimento se queda en `Esperando a: tb3_…` | La simulación no está lanzada, o una terminal está en `[SIM]` y otra en `[REAL]` |
| Los robots no se mueven y `check_p1` falla | La ley devuelve ceros: completad `consensus_velocity` |
| Los robots se alejan en vez de juntarse | Signo cambiado en la ley |
| `Filtro de seguridad: obstáculo delante` | Un robot tiene otro a menos de 20 cm; es normal si se cruzan |
| En real, RViz no muestra los robots | `modo_real` no aplicado en esa terminal, o el profesor no ha lanzado el robot |
| En real, el robot sigue moviéndose al cerrar | `ros2 run robcoop_lab parar` y avisad al profesor |
