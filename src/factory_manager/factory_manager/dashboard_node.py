"""ROS-backed HTTP dashboard for the simulated factory."""

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import threading
import time
import webbrowser

from factory_manager.constants import (
    DESTINATION_BY_CLASS,
    FACTORY_LABELS,
    ITEM_CLASS,
)
from factory_manager.dashboard_page import DASHBOARD_HTML
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Int32, String
from vision_msgs.msg import Detection3DArray


class DashboardNode(Node):
    """Collect factory topics and expose a dependency-free web dashboard."""

    def __init__(self):
        super().__init__('factory_dashboard')
        self.declare_parameter('port', 8080)
        self.declare_parameter('auto_open', True)
        self.port = int(self.get_parameter('port').value)
        self.auto_open = bool(self.get_parameter('auto_open').value)
        self.lock = threading.Lock()
        self.started_at = time.monotonic()
        self.current_class = ''
        self.last_factory_state = ''
        self.last_vision_status = ''
        self.data = {
            'factory_state': 'IDLE',
            'arm_state': 'INITIALIZING',
            'conveyor_state': 'STOPPED',
            'andon_state': '대기',
            'andon_level': 'idle',
            'vision_status': '카메라 대기',
            'vision_class': '',
            'vision_confidence': 0.0,
            'item_count': 0,
            'target_count': 3,
            'success_count': 0,
            'failure_count': 0,
            'class_counts': {'A': 0, 'B': 0, 'C': 0},
            'box_states': {
                'A': '적재 대기',
                'B': '적재 대기',
                'C': '적재 대기',
            },
            'dock_states': {
                'A': '도크 A 대기',
                'B': '도크 B 대기',
                'C': '도크 C 대기',
            },
            'jobs': [],
            'events': [],
            'arrival_order': [],
            'cycle_number': 1,
        }
        with self.lock:
            self._append_event_locked('info', '관제 대시보드 시작')

        self.create_subscription(
            String, '/factory/state', self._factory_callback, 10
        )
        self.create_subscription(
            String, '/arm/state', self._arm_callback, 10
        )
        self.create_subscription(
            String, '/conveyor/state', self._conveyor_callback, 10
        )
        self.create_subscription(
            String, '/vision/status', self._vision_status_callback, 10
        )
        self.create_subscription(
            Detection3DArray,
            '/vision/detections',
            self._detections_callback,
            10,
        )
        self.create_subscription(
            Int32, '/box/item_count', self._count_callback, 10
        )
        self.create_subscription(
            Bool, '/arm/task_complete', self._complete_callback, 10
        )
        self.create_subscription(
            String, '/amr/state', self._amr_callback, 10
        )
        self.create_subscription(
            String, '/factory/event', self._event_callback, 10
        )
        manifest_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            String,
            '/item_spawner/manifest',
            self._manifest_callback,
            manifest_qos,
        )

        self.server = self._start_server()
        if self.auto_open and self.server is not None:
            self.create_timer(2.0, self._open_browser_once)

    def _set(self, key, value):
        with self.lock:
            self.data[key] = value

    def _elapsed_label(self):
        elapsed = int(time.monotonic() - self.started_at)
        minutes, seconds = divmod(elapsed, 60)
        return f'{minutes:02d}:{seconds:02d}'

    def _append_event_locked(self, level, text):
        events = self.data['events']
        if events and events[0]['text'] == text and events[0]['level'] == level:
            return
        events.insert(0, {
            'time': self._elapsed_label(),
            'level': level,
            'text': text,
        })
        del events[12:]

    def _set_andon_locked(self, level, state):
        self.data['andon_level'] = level
        self.data['andon_state'] = state

    def _set_box_locked(self, item_class, state):
        if item_class in self.data['box_states']:
            self.data['box_states'][item_class] = state

    def _set_dock_locked(self, item_class, state):
        if item_class in self.data['dock_states']:
            self.data['dock_states'][item_class] = state

    def _update_job_locked(self, item_class, state):
        for job in self.data['jobs']:
            if job['class'] == item_class:
                job['state'] = state
                return
        if item_class in DESTINATION_BY_CLASS:
            self.data['jobs'].append({
                'work_id': f'수동-{item_class}',
                'class': item_class,
                'item': item_class,
                'box': f'BOX {item_class}',
                'destination': DESTINATION_BY_CLASS.get(
                    item_class, '출하 도크'
                ),
                'state': state,
            })

    def _factory_callback(self, message):
        with self.lock:
            self.data['factory_state'] = message.data
            if message.data != self.last_factory_state:
                label = FACTORY_LABELS.get(message.data, message.data)
                self._append_event_locked('info', f'공정 상태: {label}')
                self.last_factory_state = message.data

            if message.data in ('CONVEYOR_RUNNING', 'ITEM_READY', 'PICKING'):
                self._set_andon_locked('running', '생산 중')
            elif message.data in ('BOX_READY', 'AMR_MOVING'):
                self._set_andon_locked('running', '출하 운송 중')
            elif message.data == 'DELIVERED':
                self._set_andon_locked('done', '배송 완료')
            elif message.data == 'DELIVERY_FAILED':
                self._set_andon_locked('error', '배송 실패')
            elif message.data == 'RESETTING':
                self._set_andon_locked('warning', '다음 주기 준비')
            elif message.data == 'IDLE':
                self._set_andon_locked('idle', '대기')

            if message.data == 'BOX_READY':
                for item_class in self.data['box_states']:
                    self.data['box_states'][item_class] = '적재 완료'

    def _arm_callback(self, message):
        self._set('arm_state', message.data)
        parts = message.data.split(':', 1)
        if len(parts) == 2 and parts[1] in ITEM_CLASS:
            self.current_class = ITEM_CLASS[parts[1]]

    def _conveyor_callback(self, message):
        self._set('conveyor_state', message.data)

    def _vision_status_callback(self, message):
        with self.lock:
            self.data['vision_status'] = message.data
            if message.data.startswith('DETECTED:'):
                return
            if message.data != self.last_vision_status:
                self.last_vision_status = message.data
                self._append_event_locked('info', f'비전: {message.data}')

    def _detections_callback(self, message):
        candidates = [
            detection for detection in message.detections
            if detection.results
        ]
        if not candidates:
            return
        detection = max(
            candidates, key=lambda item: item.bbox.center.position.x
        )
        hypothesis = detection.results[0].hypothesis
        with self.lock:
            self.data['vision_class'] = hypothesis.class_id
            self.data['vision_confidence'] = float(hypothesis.score)

    def _count_callback(self, message):
        self._set('item_count', message.data)

    def _complete_callback(self, message):
        if not message.data:
            return
        with self.lock:
            self.data['success_count'] += 1
            if self.current_class:
                self.data['class_counts'][self.current_class] += 1
                self.data['box_states'][self.current_class] = '적재 완료'
                self._update_job_locked(self.current_class, '흡착 적재 완료')
                self._append_event_locked(
                    'success',
                    f'BOX {self.current_class} 적재 완료',
                )

    def _amr_callback(self, message):
        with self.lock:
            if message.data == 'BOXES_MOVING':
                for item_class in self.data['box_states']:
                    self.data['box_states'][item_class] = '운송 중'
                self._set_andon_locked('running', '출하 운송 중')
                self._append_event_locked('info', '모바일 박스 출하 운송 시작')
            elif message.data.startswith('NAV2_STARTING:'):
                item_class = message.data.split(':', 1)[1]
                self._set_box_locked(item_class, 'Nav2 시작')
                self._set_andon_locked('running', 'Nav2 준비')
                self._update_job_locked(item_class, 'Nav2 시작')
                self._append_event_locked(
                    'info',
                    f'BOX {item_class} Nav2 운송 시작',
                )
            elif message.data.startswith('ARRIVED:'):
                item_class = message.data.split(':', 1)[1]
                self._set_box_locked(item_class, '구역 도착')
                self._set_dock_locked(item_class, '문 열림 · 램프 ON')
                self._update_job_locked(item_class, '배송 완료')
                self._append_event_locked(
                    'success',
                    f'BOX {item_class} '
                    f'{DESTINATION_BY_CLASS.get(item_class, "출하 도크")} 도착',
                )
            elif message.data.startswith('NAV2_PLANNING:'):
                item_class = message.data.split(':', 1)[1]
                self._set_box_locked(item_class, '경로 계획 중')
                self._update_job_locked(item_class, '경로 계획 중')
                self._append_event_locked(
                    'info',
                    f'BOX {item_class} 목적지 경로 계획',
                )
            elif message.data.startswith('NAVIGATING:'):
                item_class = message.data.split(':', 1)[1]
                self._set_box_locked(item_class, 'Nav2 주행 중')
                self._set_dock_locked(item_class, '도착 대기')
                self._update_job_locked(item_class, 'Nav2 주행 중')
                self._set_andon_locked('running', 'Nav2 주행 중')
            elif message.data.startswith('WAYPOINT:'):
                parts = message.data.split(':', 2)
                if len(parts) < 3:
                    return
                _, item_class, progress = parts
                self._set_box_locked(item_class, f'경유점 {progress}')
                self._update_job_locked(item_class, f'경유점 {progress}')
            elif message.data.startswith('RETRY:'):
                parts = message.data.split(':', 3)
                if len(parts) < 4:
                    return
                _, item_class, attempt, reason = parts
                self._set_box_locked(item_class, '실패 감지·재시도')
                self._update_job_locked(item_class, f'재시도 {attempt}')
                self._set_andon_locked('warning', '장애물/실패 감지')
                self._append_event_locked(
                    'warning',
                    f'BOX {item_class} 재시도 {attempt}: {reason}',
                )
            elif message.data.startswith('FAILED:'):
                parts = message.data.split(':', 2)
                if len(parts) < 3:
                    return
                _, item_class, reason = parts
                self._set_box_locked(item_class, '배송 실패')
                self._set_dock_locked(item_class, '도착 실패')
                self._update_job_locked(item_class, f'배송 실패: {reason}')
                self._set_andon_locked('error', '배송 실패')
                self.data['failure_count'] += 1
                self._append_event_locked(
                    'error',
                    f'BOX {item_class} 배송 실패: {reason}',
                )
            elif message.data == 'BOXES_RETURNING':
                for item_class in self.data['box_states']:
                    self.data['box_states'][item_class] = '복귀 중'
                    self._update_job_locked(item_class, '복귀 중')
                self._set_andon_locked('running', '복귀 중')
            elif message.data.startswith('HOME:'):
                item_class = message.data.split(':', 1)[1]
                self._set_box_locked(item_class, '적재 위치 복귀')
                self._update_job_locked(item_class, '적재 위치 복귀')
            elif message.data == 'RETURNED':
                for item_class in self.data['box_states']:
                    self.data['box_states'][item_class] = '적재 대기'
                    self.data['dock_states'][item_class] = (
                        f'도크 {item_class} 대기'
                    )
                    self._update_job_locked(item_class, '주기 완료')
                self._set_andon_locked('idle', '다음 주기 대기')
                self._append_event_locked('info', '박스 복귀 완료 · 다음 주기 대기')
            elif message.data == 'DELIVERED':
                for item_class in self.data['box_states']:
                    self.data['box_states'][item_class] = '구역 도착'
                    self._update_job_locked(item_class, '배송 완료')
                self._set_andon_locked('done', '모든 도크 배송 완료')
                self._append_event_locked('success', 'A/B/C 도크 배송 완료')

    def _event_callback(self, message):
        with self.lock:
            level = 'info'
            text = message.data
            if ':' in message.data:
                maybe_level, rest = message.data.split(':', 1)
                if maybe_level in {'info', 'success', 'warning', 'error'}:
                    level = maybe_level
                    text = rest
            if level == 'warning':
                self._set_andon_locked('warning', '주의 이벤트')
            elif level == 'error':
                self._set_andon_locked('error', '오류 이벤트')
            self._append_event_locked(level, text)

    def _manifest_callback(self, message):
        try:
            manifest = json.loads(message.data)
        except json.JSONDecodeError:
            self.get_logger().warning('Invalid item manifest JSON')
            return
        with self.lock:
            self.data['arrival_order'] = manifest.get('arrival_order', [])
            self.data['cycle_number'] = manifest.get('cycle', 1)
            self.data['item_count'] = 0
            self.data['success_count'] = 0
            self.data['failure_count'] = 0
            self.data['class_counts'] = {'A': 0, 'B': 0, 'C': 0}
            self.data['box_states'] = {
                'A': '적재 대기',
                'B': '적재 대기',
                'C': '적재 대기',
            }
            self.data['dock_states'] = {
                'A': '도크 A 대기',
                'B': '도크 B 대기',
                'C': '도크 C 대기',
            }
            self.data['jobs'] = [
                {
                    'work_id': item.get('work_id', f"cycle-{index}"),
                    'class': item.get('item_class', ''),
                    'item': item.get('model', ''),
                    'box': f"BOX {item.get('item_class', '-')}",
                    'destination': DESTINATION_BY_CLASS.get(
                        item.get('item_class', ''), '출하 도크'
                    ),
                    'state': '투입 대기',
                }
                for index, item in enumerate(manifest.get('items', []), start=1)
            ]
            self._set_andon_locked('idle', '신규 배치 대기')
            order = ' → '.join(self.data['arrival_order'])
            self._append_event_locked(
                'info',
                f"{self.data['cycle_number']}주기 배치 준비: {order}",
            )

    def snapshot(self):
        with self.lock:
            output = json.loads(json.dumps(self.data))
        output['elapsed_seconds'] = int(time.monotonic() - self.started_at)
        return output

    def _start_server(self):
        dashboard = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                if self.path.startswith('/api/status'):
                    body = json.dumps(
                        dashboard.snapshot(), ensure_ascii=False
                    ).encode()
                    content_type = 'application/json; charset=utf-8'
                elif self.path == '/' or self.path.startswith('/?'):
                    body = DASHBOARD_HTML.encode()
                    content_type = 'text/html; charset=utf-8'
                else:
                    self.send_error(404)
                    return
                self.send_response(200)
                self.send_header('Content-Type', content_type)
                self.send_header('Cache-Control', 'no-store')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def log_message(self, _format, *_args):
                return

        try:
            server = ThreadingHTTPServer(('127.0.0.1', self.port), Handler)
        except OSError as error:
            self.get_logger().error(
                f'Could not start dashboard on port {self.port}: {error}'
            )
            return None
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        self.get_logger().info(
            f'Factory dashboard: http://127.0.0.1:{self.port}'
        )
        return server

    def _open_browser_once(self):
        if not self.auto_open:
            return
        self.auto_open = False
        webbrowser.open(f'http://127.0.0.1:{self.port}')

    def destroy_node(self):
        if self.server is not None:
            self.server.shutdown()
            self.server.server_close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DashboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
