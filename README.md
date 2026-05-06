[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-22041afd0340ce965d47ae6ef1cefeee28c7c493a6346c4f15d667ab976d596c.svg)](https://classroom.github.com/a/7bjYLsjm)

# Proyecto final — Robot repartidor (Optimus)

Aplicación ROS 2 Jazzy que cumple los tres requisitos del enunciado:
- **Navegación autónoma** entre 2 waypoints con **Nav2** (esquiva obstáculos con láser, sin VFF custom).
- **Interacción HRI** (TTS + STT + Extract) con `simple_hri` local.
- **Detección YOLO 3D** para confirmar receptor antes de entregar.

Funciona en **simulación** (Gazebo + aws_house) y en el **robot real** (Kobuki + Xtion/Astra/OAK).

## 📚 Índice de la documentación (en orden)

| Documento | Para qué | Tiempo lectura |
|---|---|---|
| **README.md** (este fichero) | Visión rápida, comandos básicos | 3 min |
| **[INSTALL.md](INSTALL.md)** | Setup detallado paso a paso | 10 min |
| **[MEDIR_WAYPOINTS.md](MEDIR_WAYPOINTS.md)** | Cómo medir los waypoints reales en el aula | 8 min |
| **[PARA_COMPAÑERO.md](PARA_COMPAÑERO.md)** | Guion express para compañero tras `git pull` | 5 min |
| **[DEMO_DIA_D.md](DEMO_DIA_D.md)** | Guion paso a paso del día de la demo (T-2h → T0 → plan B) | 10 min |
| **[DOCUMENTACION_PROYECTO_FINAL.md](DOCUMENTACION_PROYECTO_FINAL.md)** | Documentación completa estilo P5/P6 (modo niño, evolución, 13 bugs documentados, sim vs real, checklist 10/10) | 25 min |

> **¿Primera vez en este PC?** → `./setup.sh` o lee [INSTALL.md](INSTALL.md).
> **¿Vas a la clase?** → lee [MEDIR_WAYPOINTS.md](MEDIR_WAYPOINTS.md) **antes de salir**.

## Historia

> *"Hola, soy Optimus. Pon el paquete encima y dime adónde lo llevo. Puedo ir a la cocina o a la secretaría."*

El robot escucha al usuario, extrae el destino, anuncia "Perfecto, voy a la X", navega, espera ver al receptor (YOLO), entrega ("Te dejo el paquete..."), espera un margen para que el usuario lo recoja, y vuelve al origen.

## Estructura del repo

```
proyecto-final-optimus/
├── README.md                     # este fichero (visión + cómo lanzar)
├── INSTALL.md                    # guía paso a paso de instalación
├── setup.sh                      # script de setup automatizado (idempotente)
└── proyecto_final/               # paquete ROS 2
    ├── proyecto_final/
    │   └── delivery_node.py      # FSM principal (16 estados)
    ├── launch/
    │   ├── delivery.launch.py        # core: solo FSM + converter YOLO
    │   ├── delivery_sim.launch.py    # all-in-one para simulación
    │   ├── delivery_real.launch.py   # YOLO + delivery (real, asume Kobuki+Nav2 lanzados)
    │   └── nav2_clase.launch.py      # helper Nav2 con mapa de la clase
    ├── config/
    │   ├── waypoints.yaml            # waypoints mapa real (clase)
    │   └── waypoints_sim.yaml        # waypoints aws_house
    ├── maps/
    │   └── clase.yaml                # mapa generado en clase
    └── package.xml / setup.py / setup.cfg
```

## FSM (16 estados, flow simplificado tras bug yesno)

```
INIT
 └─► GREETING ──► WAIT_DEST_TTS ──► ASK_DEST ──► WAIT_DEST_LISTEN
                                                       │
                                          (resolve directo del STT)
                                          ┌────────────┴────────────┐
                                       no match                  match
                                          │                         │
                                  WAIT_DEST_EXTRACT  ────────► CONFIRM_DEST
                                                                    │
                                                  ("Perfecto, voy a la X")
                                                                    │
                                                              WAIT_CONFIRM_TTS
                                                                    │
                                                              NAV_TO_DEST
                                                          ┌─────────┴─────────┐
                                                       success            fail (5 retries)
                                                          │                   │
                                                  WAIT_RECEIVER             ABORT ──► DONE
                                                  (YOLO 'person', 15s)
                                                          │
                                                       DELIVER
                                                  ("Te dejo el paquete...")
                                                          │
                                                  WAIT_DELIVER_TTS
                                                  (margen pickup_timeout)
                                                          │
                                                       NAV_BACK
                                                          │
                                                  REPORT_DONE ──► DONE
```

> **Nota**: la confirmación yes/no se eliminó deliberadamente. El patrón
> robusto de `simple_hri` requiere `STT + YesNo(texto)`, lo que añade
> latencia y un punto de fallo (Whisper transcribe mal → yesno devuelve
> "no" → bucle). Usamos confirmación informativa: si el STT identificó
> el destino, el robot anuncia "Perfecto, voy a la X" y manda el goal.
> Si el STT no entiende, el resolver+Extract reintentan hasta `max_dest_retries=3`
> y luego cancelan el envío (`ABORT`). Ver `DOCUMENTACION_PROYECTO_FINAL.md` §6.12.

## Instalación rápida

```bash
cd ~/kobuki_ws/src/proyecto-final-optimus
./setup.sh
```

El script clona deps, crea venv, hace pip install, **aplica el fix obligatorio de protobuf** (sin él rclpy crashea con SIGSEGV silenciosamente), compila y verifica. Idempotente.

## Cómo lanzar

### A) Simulación (Gazebo) — `delivery_sim.launch.py`

Un solo comando levanta Gazebo + Nav2 + YOLO + delivery:
```bash
# Terminal 1 (opcional si quieres voz): HRI services
ros2 launch simple_hri local_simple_hri.launch.py

# Terminal 2: TODO en simulación
ros2 launch proyecto_final delivery_sim.launch.py
```

Args útiles:
- `skip_hri:=true` — sin micrófono (va directo al `forced_dest`).
- `skip_yolo:=true` — sin esperar receptor.
- `launch_gazebo:=false` `launch_nav2:=false` `launch_yolo:=false` — para iterar el FSM sin relevantar Gazebo.

### B) Robot real (Kobuki + Xtion/Astra) — 5 terminales

> Cada terminal: `source /opt/ros/jazzy/setup.bash && source ~/kobuki_ws/venv_asr/bin/activate && source ~/kobuki_ws/install/setup.bash`

1. **Driver Kobuki**: `ros2 launch kobuki kobuki.launch.py`
2. **Nav2 + mapa de la clase**: `ros2 launch proyecto_final nav2_clase.launch.py`
   → en RViz, **2D Pose Estimate** sobre la pose real del robot.
3. **HRI local**: `ros2 launch simple_hri local_simple_hri.launch.py`
4. **App** (lanza YOLO + converter + delivery_node):
   ```bash
   ros2 launch proyecto_final delivery_real.launch.py            # Xtion (default)
   ros2 launch proyecto_final delivery_real.launch.py camera:=astra
   ```

## Modos de bypass para depurar por trozos

| Flag | Default | Para qué sirve |
|---|---|---|
| `skip_hri` | `false` | Salta TODO el HRI (no TTS, no STT, no nada). Va directo a `forced_dest`. |
| `skip_yolo` | `false` | No espera receptor; entrega inmediato. |
| `forced_dest` | `dest1` | Cuando `skip_hri=true` o `mock_voice=true`, qué destino usar. |
| `mock_voice` | `false` | **TTSs reales pero usuario simulado** (sin necesidad de micrófono). El robot habla por altavoces; el "usuario" simulado siempre dice `forced_dest`. Útil para **demo en simulación sin micrófono**. |
| `launch_yolo` | `true` | (real/sim) lanza yolo_bringup; pon `false` si lo lanzas tú. |

Ejemplo — validar SOLO Nav2 en sim sin tocar nada más:
```bash
ros2 launch proyecto_final delivery_sim.launch.py \
  skip_hri:=true skip_yolo:=true launch_yolo:=false forced_dest:=dest1
```

**Ejemplo demo en simulación con voz audible (sin necesitar micrófono)**:
```bash
# Terminal 1: levantar HRI services (TTS + sound_play)
ros2 launch simple_hri local_simple_hri.launch.py

# Terminal 2: simulación completa con voz mock
ros2 launch proyecto_final delivery_sim.launch.py \
  mock_voice:=true skip_yolo:=true launch_yolo:=false forced_dest:=dest1
```
El robot saluda por altavoces, "el usuario" simulado responde el destino configurado en `forced_dest`. El robot confirma informativamente ("Perfecto, voy a la X") y la FSM ejecuta TODO el flujo (TTS audible + Nav2 navegando + vuelta a casa). Sin yes/no.

## Editar waypoints (sin rebuild)

Edita los YAMLs en [proyecto_final/config/](proyecto_final/config/). Gracias a `--symlink-install`, los cambios se aplican al relanzar el nodo, **sin** `colcon build`.

Para sacar coordenadas en RViz:
- Click en **2D Goal Pose** sobre el punto → `ros2 topic echo --once /goal_pose`.
- `yaw` desde quaternion `(z, w)`: `python3 -c "import math; print(2*math.atan2($Z, $W))"`.

## ⚠️ RMW (DDS) — consistencia entre terminales

`setup.sh` inyecta `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` en
`venv_asr/bin/activate` para evitar el SIGSEGV de FastDDS con
`/dev/shm` multi-usuario.

**TODAS las terminales** que lances tienen que usar la **misma**
implementación DDS, si no NO se ven entre ellas. Antes de lanzar
cualquier comando, en cada terminal:

```bash
source ~/kobuki_ws/venv_asr/bin/activate
echo $RMW_IMPLEMENTATION   # debe imprimir: rmw_cyclonedds_cpp
```

Si una terminal está vacía y otra dice `rmw_cyclonedds_cpp`, los nodos
no se comunican y verás "Servicio no disponible" para todo.

Si quieres FastDDS por defecto (no recomendado en lab compartido), en
**TODAS** las terminales: `unset RMW_IMPLEMENTATION` antes de cada lanzamiento.

## Troubleshooting

| Síntoma | Causa | Fix |
|---|---|---|
| `delivery_node` muere con `[ros2run]: Segmentation fault` y log vacío | protobuf 5/6 incompatible con rclpy | `pip uninstall -y google-cloud-texttospeech googleapis-common-protos proto-plus && pip install --force-reinstall 'protobuf<5'` |
| `sound_play` muere con SIGSEGV silencioso (TTS no se oye) | FastDDS no funciona con `/dev/shm` multi-usuario | `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` (ya está en `venv_asr/bin/activate` tras `setup.sh`) |
| `extract_service` no carga su modelo | `transformers 5.x` eliminó `text2text-generation` | `pip install --no-cache-dir 'transformers<5'` |
| `import whisper` falla con `module 'coverage.types' has no attribute 'Tracer'` | `numba` necesita coverage>=7.5 | `pip install --upgrade 'coverage>=7.5'` |
| `colcon build` falla en `audio_play` | gst-app-dev no disponible | `touch ~/kobuki_ws/src/ThirdParty/audio_common/audio_play/COLCON_IGNORE` |
| FSM se queda en `WAIT_DEST_TTS` | `simple_hri` no corre | Lánzalo o usa `skip_hri:=true` |
| Nav2 no acepta goals | robot no localizado | RViz → "2D Pose Estimate" |
| YOLO no publica `/yolo/detections_3d` | Topics de cámara mal | `ros2 topic list \| grep -i color` y ajusta `camera:=` o `image_topic:=` |

Más detalles en [INSTALL.md §10](INSTALL.md).
