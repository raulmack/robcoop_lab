# Práctica 2 — Asignación de tareas por subasta

| | |
|---|---|
| **Duración** | sesión 2 (simulación) + trabajo en casa (solo simulación) + turno con robots en la sesión 3 |
| **Se programa** | `robcoop_lab/auction_law.py` (cuatro funciones cortas) |
| **Entrega** | una semana después de la sesión 3: etiqueta `p2` en GitHub + zip y PDF en el campus virtual |
| **Peso** | 25 % de la nota de prácticas |

## Objetivos

1. Diseñar un protocolo de mensajes entre robots para repartir tareas: anuncio, puja y adjudicación.
2. Implementar las pujas de una subasta secuencial y entender cómo el criterio de puja cambia el reparto.
3. Comparar la solución distribuida (subasta) con una centralizada (algoritmo húngaro) y con el
   óptimo, en distancia, tiempo y mensajes.
4. Ejecutar el mismo código con robots reales y analizar las diferencias.

## Plan

| Cuándo | Paso | Resultado |
|---|---|---|
| S2 0:00–0:20 | Teoría (profesor) | Subasta secuencial, criterios de puja, húngaro |
| S2 0:20–0:30 | 0. Traer la P2 a vuestro repositorio | Paquete actualizado y compilado |
| S2 0:30–1:15 | 1. Programar `auction_law.py` | `check_p2` en verde |
| S2 1:15–1:45 | 2. Primera subasta en Gazebo | Disposición A, primera gráfica |
| S2 1:45–3:40 | 3. Criterios y disposiciones · 4. Comparación | Experimentos A, B, C y tabla de métodos |
| S2 (turnos) | Robots reales de la **P1** (25 min por pareja) | Mientras, el resto sigue con la P2 |
| S2 3:40–4:00 | Puesta en común y qué se hará en la sesión 3 | |
| Casa | Terminar los pasos 3 y 4 (solo simulación) e informe de la P1 | |
| S3 (turnos) | 5. Robots reales de la P2 (30 min por pareja, mientras el resto empieza la P3) | `real_C_*` |

---

## Teoría mínima

Hay **R robots** y **T tareas** (puntos del suelo que hay que visitar). Cada robot recorrerá sus
tareas en orden, empezando desde donde está, sin volver al inicio. La longitud de su ruta es

```
L(ruta) = |inicio − p_1| + |p_1 − p_2| + … + |p_{n−1} − p_n|
```

**Subasta secuencial de un solo ítem (SSI).** Un subastador anuncia en cada ronda todas las
tareas pendientes; cada robot calcula, con *su* posición y *su* ruta, una puja por cada una; se
adjudica la puja más baja de todas (una tarea por ronda) y el ganador la inserta en su ruta. Se
repite hasta que no quedan tareas: T rondas y T·(R + 2) mensajes (anuncio, R pujas, adjudicación).

```
/auction/announce   subastador → todos   {"round": 3, "tasks": [{"id": "P2", "x": .., "y": ..}, ...]}
/auction/bid        robot → subastador   {"robot": "tb3_1", "bids": [{"task": "P2", "cost": 1.27}, ...]}
/auction/award      subastador → todos   {"robot": "tb3_1", "task": {"id": "P2", ...}, "cost": 1.27}
```

El subastador **coordina pero no decide**: no necesita saber dónde están los robots ni sus rutas;
solo compara números. Cada robot calcula su puja con la **mejor inserción**: prueba a meter la
tarea en cada hueco de su ruta y se queda con el que menos la alarga. Dos criterios de puja:

| Criterio | Puja del robot i por la tarea t | Qué busca |
|---|---|---|
| `suma` | lo que se alarga su ruta: ΔL = L(ruta + t) − L(ruta) | menor distancia **total** del equipo |
| `max` | longitud que tendría su ruta: L(ruta) + ΔL | que la ruta **más larga** sea corta (menor tiempo total) |

La SSI es *miope*: decide tarea a tarea y no deshace lo adjudicado, así que no siempre encuentra el
óptimo. Pero es distribuida, escala bien y se adapta si cambian las tareas.

**Algoritmo húngaro.** Resuelve de forma óptima el problema de dar **una** tarea a cada robot
minimizando la suma de costes (`scipy.optimize.linear_sum_assignment`). Es centralizado: alguien
tiene que conocer la matriz de costes completa. Para T > R se usa aquí **por rondas**: en cada
ronda una tarea por robot, con el coste medido desde donde termina su ruta.

**Óptimo.** Con 6 tareas y 3 robots se pueden probar todos los repartos y órdenes (fuerza bruta);
sirve de referencia, pero crece de forma exponencial.

## Qué hay en el paquete

| Fichero | Qué hace | ¿Se toca? |
|---|---|---|
| `auction_law.py` | `route_length`, `best_insertion`, `choose_winner`, `hungarian` | **Sí** |
| `auction_node.py` | Nodo de un robot: puja, guarda su ruta y la recorre | No |
| `auctioneer_node.py` | Subastador + registro: rondas, salida, CSV + JSON y marcadores para RViz | No |
| `auction_core.py` | Pujas con los dos criterios, subasta sin ROS, húngaro por rondas, óptimo | No |
| `navigation.py` | Ir a un punto (mismo punto de control que la P1) con evitación por LiDAR | No |
| `tasks.py` | Disposiciones de puntos A, B, C y aleatoria | Se puede ampliar |
| `kinsim_p2.py`, `check_p2.py`, `compare_p2.py` | Simulador rápido, comprobación y comparación de métodos (sin ROS) | No |
| `plot_p2.py` | Gráficas y tablas a partir de los CSV/JSON | No |

Disposiciones (6 puntos; en la arena real se marcan con cinta los de la **C**):

```
A  repartidos por toda la arena          B  agrupados cerca de tb3_0
C  dos filas que cruzan la arena          aleatoria  6 puntos al azar (semilla:=N)
```

Los robots salen de las mismas marcas que en la P1 (`tb3_0`, `tb3_1`, `tb3_2`).

**Navegación.** Cada robot lleva su punto de control a la tarea (tolerancia 8 cm), se para 1 s
(«inspección») y sigue. Si ve otro robot delante lo rodea: si está justo delante, siempre por la
derecha (dos robots que se cruzan de frente se esquivan sin hablar); si no, por el lado que acerca
al objetivo. Si deja de avanzar, prueba por el otro lado, y si tampoco, retrocede y cede el paso
según su número. El filtro de seguridad de la P1 sigue activo.

---

## Paso 0 · Traer la P2 a vuestro repositorio (10 min)

Vuestro repositorio se creó desde la plantilla antes de la P2. Para traer los ficheros nuevos **sin
tocar vuestra `consensus_law.py` ni vuestros resultados**, en la terminal de Ubuntu (WSL), *fuera*
del contenedor:

```bash
cd ~/robcoop-docker/ws/src/robcoop_lab
git remote add plantilla https://github.com/raulmack/robcoop_lab.git   # solo la primera vez
git fetch plantilla
git checkout plantilla/main -- . ':!robcoop_lab/consensus_law.py' ':!robcoop_lab/formations.py' ':!robcoop_lab/graphs.py' ':!resultados'
git commit -m "P2: material de la plantilla" && git push
```

Compilar de nuevo dentro del contenedor (hay ejecutables nuevos) y comprobar:

```bash
cd ~/robcoop-docker && ./robcoop.sh shell
cb
ros2 run robcoop_lab check_p2        # debe FALLAR: aún no habéis programado nada
ros2 run robcoop_lab check_p1        # la P1 sigue igual
```

> El prompt debe empezar por **`[SIM]`**. Si pone `[REAL]`, ejecutad `modo_sim`.

## Paso 1 · Programar `auction_law.py` (45 min)

Abrid `ws/src/robcoop_lab/robcoop_lab/auction_law.py` en VS Code y completad los cuatro `TODO`:

1. `route_length(start, route)`: longitud del camino.
2. `best_insertion(start, route, task)`: mejor hueco para la tarea y cuánto alarga la ruta.
3. `choose_winner(bids)`: puja más baja, con la regla de desempate del enunciado.
4. `hungarian(cost)`: asignación óptima de una tarea por robot con `linear_sum_assignment`.

Si no habéis usado mucho Python, mirad antes las tres diapositivas «Python que vais a necesitar»:
`math.dist`, cortes de listas (`route[:k] + [t] + route[k:]`), comparación de tuplas con `min`,
`math.inf` / `math.isfinite`, comprensión de listas, `numpy`, `zip` e `int`. Para `hungarian` hay que
añadir arriba `from scipy.optimize import linear_sum_assignment`: la plantilla no trae esa importación.

Comprobar, sin ROS:

```bash
cd ~/ws/src/robcoop_lab
python3 -m pytest test/test_auction_law.py -v                     # 5 pruebas
ros2 run robcoop_lab check_p2                                     # 6/6 bloques correctos
ros2 run robcoop_lab kinsim_p2 --disposicion B --criterio max --plot
```

`check_p2` comprueba las funciones, repite la subasta y el húngaro en las disposiciones A, B y C
(deben salir las asignaciones esperadas) y simula la ejecución. Guardad el avance:
`git add -A && git commit -m "P2: auction_law" && git push`.

## Paso 2 · Primera subasta en Gazebo (30 min)

**Terminal 1** (3 robots en sus marcas, sin ventana de Gazebo; se abre RViz con las tareas):

```bash
ros2 launch robcoop_lab p2_sim.launch.py
```

**Terminal 2** (experimento):

```bash
ros2 launch robcoop_lab p2_subasta.launch.py disposicion:=A criterio:=suma nombre:=sim_A_suma
```

El subastador espera a los 3 robots, hace las 6 rondas (en la terminal se ve quién gana cada
tarea y con qué puja), da la salida y **termina solo** cuando se han visitado los 6 puntos (o a los
300 s). En RViz las tareas cambian de gris al color del robot que las gana, se dibuja la ruta de
cada uno y las tareas hechas se atenúan. Al final escribe el resumen y guarda
`resultados/p2/sim_A_suma.csv` y `.json`. Gráficas:

```bash
ros2 run robcoop_lab plot_p2 ~/ws/src/robcoop_lab/resultados/p2/sim_A_suma.csv
```

Mirad los mensajes de una ronda en otra terminal mientras se repite el experimento:
`ros2 topic echo /auction/bid`. Con `ejecutar:=false` se hace solo la subasta (los robots no se
mueven), útil para ver las rondas sin esperar.

**Grafo de nodos para el informe.** En una tercera terminal (`./robcoop.sh shell`), con la
simulación en marcha, lanzad `ros2 run rqt_graph rqt_graph` y repetid un experimento
(C dura unos 20 s). Mientras los robots se mueven: desplegable **Nodes/Topics (all)**, botón de
refrescar y, si queda muy cargado, marcad **Hide: tf**. Guardadlo con el icono *Save as image* en
`~/ws/src/robcoop_lab/resultados/p2/grafo.png`. Si el experimento lo hacéis solo para la captura,
borrad después sus ficheros (`rm resultados/p2/<nombre>.*`).

> **Para repetir desde las marcas:** Ctrl+C en la terminal 1 y volver a lanzar. Al acabar un
> experimento los robots se quedan donde terminaron; si no relanzáis, el siguiente saldría de ahí y sus
> números no se podrían comparar con la referencia ni con los robots reales. Por eso el subastador
> comprueba que cada robot está a menos de 15 cm de su marca y, si no, cancela sin guardar nada.

> **Si los robots no se mueven** y en la terminal 2 sale «el LiDAR marca 0.12 m en todos los rayos», el LiDAR
> simulado no funciona con la GPU de vuestro PC (pasa con algunas tarjetas). Relanzad la simulación con
> `ros2 launch robcoop_lab p2_sim.launch.py render:=software`: Gazebo dibuja por software y el LiDAR mide bien.

## Paso 3 · Criterios y disposiciones (en simulación; se puede terminar en casa)

Seis experimentos. **Antes de cada uno, Ctrl+C en la terminal 1 y relanzad `p2_sim`** (los robots
tienen que salir de sus marcas; si no, el subastador cancela el experimento):

```bash
ros2 launch robcoop_lab p2_subasta.launch.py disposicion:=A criterio:=suma nombre:=sim_A_suma
ros2 launch robcoop_lab p2_subasta.launch.py disposicion:=A criterio:=max  nombre:=sim_A_max
ros2 launch robcoop_lab p2_subasta.launch.py disposicion:=B criterio:=suma nombre:=sim_B_suma
ros2 launch robcoop_lab p2_subasta.launch.py disposicion:=B criterio:=max  nombre:=sim_B_max
ros2 launch robcoop_lab p2_subasta.launch.py disposicion:=C criterio:=suma nombre:=sim_C_suma
ros2 launch robcoop_lab p2_subasta.launch.py disposicion:=C criterio:=max  nombre:=sim_C_max
```

Tabla resumen y figura comparativa:

```bash
cd ~/ws/src/robcoop_lab/resultados/p2
ros2 run robcoop_lab plot_p2 sim_*.csv --nombre criterios
```

Para el informe:

* ¿Por qué en B, con `suma`, un robot se queda con las 6 tareas? ¿Cuál de los dos criterios da
  menor tiempo total y por qué? ¿Y en A, por qué dan lo mismo?
* Comparad la distancia planificada (`planned_lengths` del JSON) con la recorrida y el tiempo total
  medido con el que daría la ruta más larga a 0.15 m/s. ¿De dónde sale la diferencia?

## Paso 4 · Comparación con el húngaro y el óptimo (sin ROS, en segundos)

```bash
ros2 run robcoop_lab compare_p2                                       # A, B y C
ros2 run robcoop_lab compare_p2 --disposiciones aleatoria --semilla 1 --nombre ale1
ros2 run robcoop_lab compare_p2 --disposiciones aleatoria --semilla 2 --nombre ale2
ros2 run robcoop_lab compare_p2 --disposiciones aleatoria --semilla 3 --nombre ale3
```

Cada llamada calcula, para la misma situación, subasta (`suma` y `max`), húngaro por rondas y
óptimo (`suma` y `max`), simula la ejecución de cada uno y guarda tabla (`.md`), datos (`.csv`) y
figura con las rutas (`.png`) en `resultados/p2`.

* ¿En qué disposiciones la subasta no llega al óptimo de su criterio? Buscad en el JSON (`rounds`)
  qué adjudicación temprana lo impide.
* Subasta frente a húngaro: calidad, mensajes (T·(R+2) frente a 2R) e información que necesita
  cada uno. ¿Qué pasa si falla el subastador? ¿Y el coordinador del húngaro?
* Opcional: cambiad `best_insertion` para que solo añada al final de la ruta y repetid `compare_p2`.
  ¿Cuánto empeora? (dejadlo como estaba antes de entregar).

## Paso 5 · Robots reales (sesión 3, turno de 30 min)

En la arena están las marcas de salida de la P1 y los 6 puntos de la disposición **C** con cinta.

1. `modo_real` y volved a abrir las terminales (`exit` y `./robcoop.sh shell`).
2. Colocad `tb3_0`, `tb3_1` y `tb3_2` sobre sus marcas, mirando en la dirección de la flecha.
3. **Terminal 1**, anclar los robots y abrir RViz con las tareas:
   ```bash
   ros2 launch robcoop_lab p2_real.launch.py
   ```
4. **Terminal 3**, preparada para una parada de emergencia: `ros2 run robcoop_lab parar`
   (para también la subasta y el subastador guarda lo registrado).
5. **Terminal 2**, mismo código y parámetros que en simulación:
   ```bash
   ros2 launch robcoop_lab p2_subasta.launch.py modo:=real disposicion:=C criterio:=suma nombre:=real_C_suma
   ```
6. Volved a colocar los robots en sus marcas, Ctrl+C en la terminal 1, relanzadla y repetid con
   `criterio:=max nombre:=real_C_max`.
7. Al terminar: **`modo_sim`** y volved a abrir las terminales.

Comparación simulación / real: `ros2 run robcoop_lab plot_p2 sim_C_suma.csv real_C_suma.csv sim_C_max.csv real_C_max.csv --nombre sim_vs_real`.

> **Seguridad.** Igual que en la P1: 0.15 m/s, parada si hay algo a menos de 20 cm delante y una
> persona de la pareja siempre junto a la arena. Si algo va mal, `parar`; si no basta, levantad el robot.

---

## Entrega P2

**Plazo:** una semana después de la sesión 3.

Todo en la terminal de Ubuntu, **fuera del contenedor** (sin `[SIM]`/`[REAL]` en el prompt).

1. **Código y resultados en GitHub.** `auction_law.py` y la carpeta `resultados/p2` (CSV, JSON, PNG
   y tablas `.md` que uséis en el informe):
   ```bash
   cd ~/robcoop-docker/ws/src/robcoop_lab
   git status -s                          # los resultados nuevos salen con ??
   git add -A && git commit -m "P2: entrega" && git push
   git tag p2 && git push origin p2
   git ls-remote --tags origin            # tiene que aparecer refs/tags/p2
   ```
   Si cambiáis algo después de etiquetar (por ejemplo, al añadir los experimentos reales de la
   sesión 3): commit y push, y moved la etiqueta con `git tag -f p2 && git push -f origin p2`.
2. **Informe en PDF** (3 páginas como máximo) con la plantilla `docs/plantilla_informe_P2.docx`
   (desde Windows: `\\wsl$\<distro>\home\<usuario>\robcoop-docker\ws\src\robcoop_lab\docs`):
   * tabla de métodos de `compare_p2` (A, B y C; y las tres aleatorias en una frase o tabla);
   * tabla de los seis experimentos en Gazebo y los dos reales (tiempo total y distancia recorrida);
   * figura de rutas de los métodos y figuras de B con los dos criterios;
   * respuestas razonadas a las preguntas de los pasos 3 y 4 y análisis simulación / real.
3. **Campus virtual.** Una entrega por pareja con el PDF y el zip del código etiquetado:
   ```bash
   git archive --format=zip -o ~/p2_<apellido1>_<apellido2>.zip p2
   python3 -m zipfile -l ~/p2_*.zip | grep -E "auction_law|resultados/p2"   # comprobad que están
   WIN=$(wslpath "$(cmd.exe /c 'echo %USERPROFILE%' 2>/dev/null | tr -d '\r')")   # vuestra carpeta de Windows
   cp ~/p2_*.zip "$WIN/Downloads/"                                   # para subirlo desde Windows
   ```
   Se evalúa lo que hay en la etiqueta `p2`: si los resultados no están en GitHub con la etiqueta, no
   cuentan.

### Evaluación (sobre 10)

| Criterio | Puntos | Cómo se mide |
|---|---|---|
| Funcionamiento en simulación | 3 | `check_p2` en la etiqueta `p2` y experimentos de los pasos 3–4 en `resultados/p2` |
| Funcionamiento en robots reales | 2 | Experimentos `real_C_*` con el mismo código y parámetros |
| Calidad del código | 2 | Claro, comentado, sin valores fijos; commits de los dos miembros |
| Informe: resultados y análisis | 3 | Tablas con números, criterios y métodos comparados con razones, sim/real |

**Opcional (hasta +1 en la nota de prácticas):** detectar la caída de un robot por su latido
(`/tb3_i/heartbeat`, 2 Hz) y volver a subastar sus tareas pendientes. El latido se usará en el reto
final de la sesión 4.

## Si algo falla

| Síntoma | Causa probable y solución |
|---|---|
| Los robots no avanzan y sale «el LiDAR marca 0.12 m en todos los rayos» | El LiDAR simulado no mide con la GPU de ese PC: cerrad la simulación y relanzadla con `render:=software` (más lenta, pero funciona) |
| «Los robots no están en sus marcas de salida» y el experimento acaba sin guardar | Siguen donde acabó el experimento anterior: Ctrl+C en la terminal 1, relanzad `p2_sim` y repetid el experimento. En real: colocadlos en las marcas y relanzad `p2_real` |
| `Package 'robcoop_lab' not found` o no existe `check_p2` | Falta compilar tras el paso 0: `cb` |
| `choose_winner no devuelve ninguna puja` y el experimento acaba | Falta programar `choose_winner` |
| Se queda en `Esperando a: tb3_…` | La simulación no está lanzada, falta un robot o mezcla de `[SIM]` y `[REAL]` |
| Un robot da vueltas cerca de otro parado | Lo está rodeando; si no lo consigue, prueba por el otro lado y después retrocede y cede el paso |
| El experimento acaba por `tiempo agotado` | Un robot no ha llegado a una tarea; mirad su trayectoria en la gráfica |
| En real, los robots no van a los puntos de cinta | Robots mal colocados en sus marcas al lanzar `p2_real` |
| En real, el robot sigue moviéndose al cerrar | `ros2 run robcoop_lab parar` y avisad al profesor |
| Vuestro repositorio no está en este PC (p. ej. hicisteis la P1 en otro) | Como en la P1: `gh auth login`, `gh auth setup-git`, `git config --global user.name/user.email` y `git clone` de vuestro repo en `~/robcoop-docker/ws/src`; después, el paso 0 |
| `git push` o `git ls-remote`: «Invalid username or token» / «Authentication failed» | Git no está usando el token de `gh`: `gh auth status` (si no estáis conectados, `gh auth login`) y después `gh auth setup-git`; repetid el comando |
| El profesor publica un arreglo en la plantilla cuando ya habéis programado | **No** repitáis el comando del paso 0 tal cual: sobrescribiría vuestra `auction_law.py` con la vacía. Añadid también vuestro fichero a la lista de excluidos: `git fetch plantilla` y `git checkout plantilla/main -- . ':!robcoop_lab/consensus_law.py' ':!robcoop_lab/formations.py' ':!robcoop_lab/graphs.py' ':!robcoop_lab/auction_law.py' ':!resultados'`; luego `cb`. Si ya os ha pasado y no habéis hecho commit: `git checkout HEAD -- robcoop_lab/auction_law.py`; si ya hicisteis commit, buscad el vuestro con `git log --oneline -- robcoop_lab/auction_law.py` y `git checkout <commit> -- robcoop_lab/auction_law.py` |
