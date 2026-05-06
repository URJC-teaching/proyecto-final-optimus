#!/usr/bin/env python3
import math
import time
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data, QoSProfile, QoSDurabilityPolicy

from geometry_msgs.msg import PoseWithCovarianceStamped
from vision_msgs.msg import Detection3DArray

from hri_client.hri_client import HRIClient
from navigation_client.navigation_client import NavigationClient


class State(Enum):
    INIT = auto()

    # Saludo + pregunta destino
    GREETING = auto()
    WAIT_DEST_TTS = auto()
    ASK_DEST = auto()
    WAIT_DEST_LISTEN = auto()
    WAIT_DEST_EXTRACT = auto()

    # Confirmacion informativa antes de navegar
    CONFIRM_DEST = auto()
    WAIT_CONFIRM_TTS = auto()

    # Navegacion al destino
    NAV_TO_DEST = auto()
    WAIT_RECEIVER = auto()

    # Entrega
    DELIVER = auto()
    WAIT_DELIVER_TTS = auto()

    # Vuelta
    NAV_BACK = auto()
    REPORT_DONE = auto()

    DONE = auto()
    ABORT = auto()


class DeliveryNode(Node):

    def __init__(self):
        super().__init__('delivery_node')

        # Parametros: waypoints (origen + 2 destinos)
        self.declare_parameter('home.x', 0.0)
        self.declare_parameter('home.y', 0.0)
        self.declare_parameter('home.yaw', 0.0)

        self.declare_parameter('dest1.name', 'cocina')
        self.declare_parameter('dest1.x', 1.0)
        self.declare_parameter('dest1.y', 0.0)
        self.declare_parameter('dest1.yaw', 0.0)
        self.declare_parameter('dest1.keywords', ['cocina'])

        self.declare_parameter('dest2.name', 'secretaria')
        self.declare_parameter('dest2.x', 0.0)
        self.declare_parameter('dest2.y', 1.0)
        self.declare_parameter('dest2.yaw', 0.0)
        self.declare_parameter('dest2.keywords', ['secretaria', 'secretaría', 'despacho'])

        # Parametros: comportamiento
        self.declare_parameter('extract_interest', 'destino')
        self.declare_parameter('detection_topic', '/detections_3d')
        self.declare_parameter('target_class', 'person')
        self.declare_parameter('receiver_timeout_sec', 15.0)
        self.declare_parameter('pickup_timeout_sec', 15.0)
        self.declare_parameter('max_dest_retries', 3)
        # simple_hri puede tardar hasta 60s en cargar los modelos
        self.declare_parameter('hri_wait_timeout_sec', 60.0)
        # Reintentos si Nav2 rechaza un goal (bt_navigator no ACTIVE al arrancar)
        self.declare_parameter('max_nav_retries', 5)
        self.declare_parameter('nav_retry_delay_sec', 2.0)

        # Flags de bypass para validar el sistema por trozos
        self.declare_parameter('skip_hri', False)
        self.declare_parameter('skip_yolo', False)
        self.declare_parameter('forced_dest', 'dest1')
        self.declare_parameter('mock_voice', False)
        self.declare_parameter('auto_initial_pose', False)

        self.home = self._read_pose('home')
        self.dest1 = self._read_dest('dest1')
        self.dest2 = self._read_dest('dest2')
        self.extract_interest = self.get_parameter('extract_interest').value
        self.detection_topic = self.get_parameter('detection_topic').value
        self.target_class = self.get_parameter('target_class').value
        self.receiver_timeout = self.get_parameter('receiver_timeout_sec').value
        self.pickup_timeout = self.get_parameter('pickup_timeout_sec').value
        self.max_dest_retries = self.get_parameter('max_dest_retries').value
        hri_wait_timeout = self.get_parameter('hri_wait_timeout_sec').value
        self.skip_hri = self.get_parameter('skip_hri').value
        self.skip_yolo = self.get_parameter('skip_yolo').value
        self.mock_voice = self.get_parameter('mock_voice').value
        self.auto_initial_pose = self.get_parameter('auto_initial_pose').value

        # skip_hri y mock_voice son incompatibles: skip_hri tiene prioridad
        if self.skip_hri and self.mock_voice:
            self.get_logger().warn(
                'skip_hri y mock_voice son incompatibles. Desactivo mock_voice.')
            self.mock_voice = False

        forced_dest_name = self.get_parameter('forced_dest').value
        if forced_dest_name not in ('dest1', 'dest2'):
            self.get_logger().error(
                f'forced_dest="{forced_dest_name}" no valido. Uso dest1.')
            forced_dest_name = 'dest1'
        self.forced_dest = self.dest2 if forced_dest_name == 'dest2' else self.dest1
        self.max_nav_retries = self.get_parameter('max_nav_retries').value
        self.nav_retry_delay = self.get_parameter('nav_retry_delay_sec').value

        # Subsistemas
        self.nav = NavigationClient(self)
        self.hri = HRIClient(self) if not self.skip_hri else None

        if not self.skip_hri:
            # wait_for_services itera 4 servicios en serie: repartimos el timeout
            per_service = max(1.0, hri_wait_timeout / 4.0)
            if not self.hri.wait_for_services(per_service):
                self.get_logger().warn(
                    f'HRI no disponible tras {hri_wait_timeout:.0f}s. '
                    'Lanza simple_hri en otra terminal.')
        else:
            self.get_logger().warn(
                f'skip_hri=true -> bypass HRI, voy a "{self.forced_dest["name"]}".')

        if not self.nav.wait_for_action_server(timeout_sec=hri_wait_timeout):
            self.get_logger().warn('Nav2 action server no disponible.')

        if self.skip_yolo:
            self.get_logger().warn('skip_yolo=true -> entrego sin esperar receptor.')
        if self.mock_voice:
            self.get_logger().warn(
                f'mock_voice=true -> usuario simulado dice "{self.forced_dest["name"]}".')

        # YOLO: solo nos suscribimos si vamos a usarlo
        self.last_detection_time = None
        self.detection_sub = None
        if not self.skip_yolo:
            self.detection_sub = self.create_subscription(
                Detection3DArray,
                self.detection_topic,
                self._detection_callback,
                qos_profile_sensor_data,
            )

        # /initialpose: QoS latched para que AMCL lo reciba al suscribir
        latched_qos = QoSProfile(
            depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', latched_qos)
        self._initial_pose_published = False

        # Estado FSM
        self.state = State.INIT
        self.target_dest = None
        self.dest_retry_count = 0
        self.state_entered_at = self.get_clock().now()
        self.nav_retry_count = 0
        self.last_nav_dest = None
        self._mission_finished = False

        # Ciclo de control a 10 Hz
        self.timer = self.create_timer(0.1, self._control_loop)

        self.get_logger().info('DeliveryNode listo. Waypoints cargados:')
        self.get_logger().info(
            f'  home       = ({self.home["x"]:+.2f}, {self.home["y"]:+.2f}, '
            f'yaw={self.home["yaw"]:+.2f})')
        self.get_logger().info(
            f'  dest1 "{self.dest1["name"]}" = '
            f'({self.dest1["x"]:+.2f}, {self.dest1["y"]:+.2f}, yaw={self.dest1["yaw"]:+.2f})  '
            f'kw={self.dest1["keywords"]}')
        self.get_logger().info(
            f'  dest2 "{self.dest2["name"]}" = '
            f'({self.dest2["x"]:+.2f}, {self.dest2["y"]:+.2f}, yaw={self.dest2["yaw"]:+.2f})  '
            f'kw={self.dest2["keywords"]}')

        # Sanity checks de waypoints
        all_zero = all(
            wp['x'] == 0.0 and wp['y'] == 0.0 and wp['yaw'] == 0.0
            for wp in (self.home, self.dest1, self.dest2)
        )
        if all_zero:
            self.get_logger().warn(
                'Todos los waypoints en (0,0,0). ¿Yaml no cargado?')

        def _dist(a, b):
            return ((a['x'] - b['x'])**2 + (a['y'] - b['y'])**2)**0.5
        if _dist(self.dest1, self.dest2) < 0.30:
            self.get_logger().warn(
                f'dest1 y dest2 a <30 cm. ¿Copy-paste?')
        if _dist(self.home, self.dest1) < 0.30:
            self.get_logger().warn(
                f'home y dest1 a <30 cm. El robot apenas se moverá.')
        if self.dest1['name'].lower() == self.dest2['name'].lower():
            self.get_logger().error(
                f'dest1 y dest2 tienen el mismo nombre "{self.dest1["name"]}".')

    # -- Helpers de parametros ----------------------------------------------

    def _read_pose(self, prefix):
        return {
            'x': self.get_parameter(f'{prefix}.x').value,
            'y': self.get_parameter(f'{prefix}.y').value,
            'yaw': self.get_parameter(f'{prefix}.yaw').value,
        }

    def _read_dest(self, prefix):
        d = self._read_pose(prefix)
        d['name'] = self.get_parameter(f'{prefix}.name').value
        # Lista vacia en yaml -> param sin tipo. Caemos al nombre.
        try:
            kw = self.get_parameter(f'{prefix}.keywords').value
        except Exception:
            kw = None
        if not kw:
            kw = [d['name']]
        d['keywords'] = [k.lower() for k in kw]
        return d

    # -- YOLO ---------------------------------------------------------------

    def _detection_callback(self, msg: Detection3DArray):
        for det in msg.detections:
            for hyp in det.results:
                if hyp.hypothesis.class_id.lower() == self.target_class.lower():
                    self.last_detection_time = self.get_clock().now()
                    return

    def _seen_target_recently(self, max_age_sec=1.0) -> bool:
        if self.last_detection_time is None:
            return False
        age = self.get_clock().now() - self.last_detection_time
        return age < Duration(seconds=max_age_sec)

    # -- Resolucion de destino por palabras clave ---------------------------

    def _resolve_dest(self, raw_text: str):
        if not raw_text:
            return None
        t = raw_text.lower()
        for dest in (self.dest1, self.dest2):
            for kw in dest['keywords']:
                if kw and kw in t:
                    return dest
        return None

    # -- FSM helpers --------------------------------------------------------

    def _go(self, new_state: State):
        if new_state != self.state:
            self.get_logger().info(f'STATE: {self.state.name} -> {new_state.name}')
            self.state = new_state
            self.state_entered_at = self.get_clock().now()

    def _time_in_state(self) -> float:
        return (self.get_clock().now() - self.state_entered_at).nanoseconds / 1e9

    def _publish_initial_pose(self):
        if self._initial_pose_published or not self.auto_initial_pose:
            return
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(self.home['x'])
        msg.pose.pose.position.y = float(self.home['y'])
        msg.pose.pose.position.z = 0.0
        # Quaternion desde yaw (rotacion en Z)
        yaw = float(self.home['yaw'])
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        # Covarianza estandar de RViz 2D Pose Estimate
        msg.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,  0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,  0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,  0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,  0.0, 0.0, 0.0685,
        ]
        self.initial_pose_pub.publish(msg)
        self._initial_pose_published = True
        self.get_logger().info(
            f'/initialpose publicado en home '
            f'({self.home["x"]:.2f}, {self.home["y"]:.2f}, yaw={self.home["yaw"]:.2f}).'
        )

    def _say(self, text: str):
        if self.hri is None:
            self.get_logger().info(f'[skip_hri] {text}')
            return
        self.hri.start_speaking(text)

    def _tts_done(self) -> bool:
        if self.hri is None:
            return True
        return self.hri.is_speaking_done()

    # -- Control loop -------------------------------------------------------

    def _control_loop(self):
        s = self.state

        if s == State.INIT:
            self._publish_initial_pose()
            if self.skip_hri:
                self.target_dest = self.forced_dest
                self._send_nav_goal(self.target_dest)
                self._go(State.NAV_TO_DEST)
            else:
                self._go(State.GREETING)
            return

        if s == State.GREETING:
            self._say(
                f'Hola, soy Optimus. ¿A la {self.dest1["name"]} o a la '
                f'{self.dest2["name"]}?'
            )
            self._go(State.WAIT_DEST_TTS)
            return

        if s == State.WAIT_DEST_TTS:
            if self._tts_done():
                self._go(State.ASK_DEST)
            return

        if s == State.ASK_DEST:
            if self.mock_voice:
                self.get_logger().info(
                    f'[mock_voice] usuario "dice": "{self.forced_dest["name"]}"')
                self.target_dest = self.forced_dest
                self._go(State.CONFIRM_DEST)
                return
            self.get_logger().info('Esperando destino del usuario...')
            self.hri.start_listen()
            self._go(State.WAIT_DEST_LISTEN)
            return

        if s == State.WAIT_DEST_LISTEN:
            if self.hri.is_listen_done():
                listened = (self.hri.get_listened_text() or
                            self.hri.get_last_listened_text() or '')
                self.get_logger().info(f'STT escuchó: "{listened}"')

                # Primer intento: keywords directas (rapido, sin LLM)
                resolved = self._resolve_dest(listened)
                if resolved is not None:
                    self.dest_retry_count = 0
                    self.target_dest = resolved
                    self._go(State.CONFIRM_DEST)
                    return

                # Fallback: pasar texto al Extract (LLM)
                if listened.strip():
                    self.get_logger().info('Pattern matching falló, lanzo Extract.')
                    self.hri.start_extract(self.extract_interest, listened)
                    self._go(State.WAIT_DEST_EXTRACT)
                else:
                    self._handle_dest_failure('')
            return

        if s == State.WAIT_DEST_EXTRACT:
            if self.hri.is_extract_done():
                extracted = self.hri.get_extracted_info()
                self.get_logger().info(f'Extract devolvió: "{extracted}"')
                resolved = self._resolve_dest(extracted)
                if resolved is not None:
                    self.dest_retry_count = 0
                    self.target_dest = resolved
                    self._go(State.CONFIRM_DEST)
                else:
                    self._handle_dest_failure(extracted)
            return

        if s == State.CONFIRM_DEST:
            self._say(f'Perfecto, voy a la {self.target_dest["name"]}.')
            self._go(State.WAIT_CONFIRM_TTS)
            return

        if s == State.WAIT_CONFIRM_TTS:
            if self._tts_done():
                self._send_nav_goal(self.target_dest)
                self._go(State.NAV_TO_DEST)
            return

        if s == State.NAV_TO_DEST:
            if self.nav.is_goal_done():
                if self.nav.was_goal_successful():
                    self.get_logger().info('Destino alcanzado.')
                    self.last_detection_time = None
                    self._go(State.WAIT_RECEIVER)
                elif self._retry_nav_if_possible():
                    pass
                else:
                    self._say('No he podido llegar al destino. Cancelo el envío.')
                    self._go(State.ABORT)
            return

        if s == State.WAIT_RECEIVER:
            if self.skip_yolo:
                self.get_logger().info('[skip_yolo] entrego sin esperar receptor.')
                self._go(State.DELIVER)
            elif self._seen_target_recently():
                self.get_logger().info(f'{self.target_class} detectado en destino.')
                self._go(State.DELIVER)
            elif self._time_in_state() > self.receiver_timeout:
                self.get_logger().warn(
                    f'No vi a nadie en {self.receiver_timeout:.0f}s, entrego igualmente.')
                self._go(State.DELIVER)
            return

        if s == State.DELIVER:
            self._say(
                f'He llegado a la {self.target_dest["name"]}. Te dejo el paquete, '
                'vuelvo en un momento.'
            )
            self._go(State.WAIT_DELIVER_TTS)
            return

        if s == State.WAIT_DELIVER_TTS:
            if self._tts_done():
                # Margen para que el usuario coja el paquete
                if not self.skip_hri and self._time_in_state() < self.pickup_timeout:
                    return
                self._send_nav_goal(self.home)
                self._go(State.NAV_BACK)
            return

        if s == State.NAV_BACK:
            if self.nav.is_goal_done():
                if self.nav.was_goal_successful():
                    self._say('He vuelto. Listo para otro envío.')
                    self._go(State.REPORT_DONE)
                elif self._retry_nav_if_possible():
                    pass
                else:
                    self._say('No he podido volver al origen.')
                    self._go(State.REPORT_DONE)
            return

        if s == State.REPORT_DONE:
            if self._tts_done():
                self._go(State.DONE)
            return

        if s == State.ABORT:
            if self._tts_done():
                self._go(State.DONE)
            return

        if s == State.DONE:
            if not self._mission_finished:
                self.get_logger().info('Mision finalizada.')
                self.timer.cancel()
                self._mission_finished = True
            return

    # -- Manejo de fallos en deteccion de destino ---------------------------

    def _handle_dest_failure(self, raw_text: str):
        self.dest_retry_count += 1
        if self.dest_retry_count >= self.max_dest_retries:
            self._say('No te he entendido. Cancelo el envío.')
            self._go(State.ABORT)
            return
        msg = (
            f'No te he entendido. Por favor, dime si quieres ir a la '
            f'{self.dest1["name"]} o a la {self.dest2["name"]}.'
        )
        self._say(msg)
        self._go(State.WAIT_DEST_TTS)

    # -- Envio de goal Nav2 -------------------------------------------------

    def _send_nav_goal(self, dest):
        pose = self.nav.create_pose_stamped(dest['x'], dest['y'], dest['yaw'])
        self.get_logger().info(
            f'Navegando a "{dest.get("name", "home")}" '
            f'({dest["x"]:.2f}, {dest["y"]:.2f}, yaw={dest["yaw"]:.2f}).'
        )
        self.nav.send_goal(pose)
        self.last_nav_dest = dest
        self.nav_retry_count = 0

    def _retry_nav_if_possible(self) -> bool:
        if self.last_nav_dest is None:
            return False
        if self.nav_retry_count >= self.max_nav_retries:
            return False
        self.nav_retry_count += 1
        self.get_logger().warn(
            f'Goal fallido. Reintento {self.nav_retry_count}/{self.max_nav_retries} '
            f'tras {self.nav_retry_delay:.1f}s.'
        )
        time.sleep(self.nav_retry_delay)
        pose = self.nav.create_pose_stamped(
            self.last_nav_dest['x'], self.last_nav_dest['y'], self.last_nav_dest['yaw']
        )
        self.nav.send_goal(pose)
        return True


def main(args=None):
    rclpy.init(args=args)
    node = DeliveryNode()
    try:
        while rclpy.ok() and not node._mission_finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        try:
            if node.nav.is_goal_active():
                node.get_logger().info('Ctrl+C: cancelando goal Nav2.')
                node.nav.cancel_goal()
        except Exception:
            pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
