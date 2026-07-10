#!/usr/bin/env python3
"""
lego22_server
--------------------
PyQt5 + pyqtgraph 로 만들어졌던 LEGO CAD 빌더를 브라우저(Three.js)에서 쓸 수 있도록
정적 파일을 서빙하고, "출력하기" 버튼을 눌렀을 때 지금까지 쌓은 블록들의 중심 좌표를
/centers 토픽으로 한 번 발행해주는 ROS2 노드.

실행:
    ros2 run lego22 lego22

실행 후 브라우저에서 http://localhost:8080 (또는 지정한 포트) 접속.

파라미터:
    port  (int)    기본 8080
    host  (string) 기본 '0.0.0.0'  (같은 네트워크의 다른 기기에서도 접속 가능)

/centers 토픽 (std_msgs/String):
    웹 UI에서 "출력하기" 버튼을 누르면, 현재 배치된 모든 블록에 대해
        [블록타입 번호, [중심x, 중심y, 중심z]]
    형태의 항목을 모은 리스트를 JSON 문자열로 만들어 /centers 에 한 번 발행한다.

    블록타입 번호: 1x1=0, 1x2=1, 2x2=2, 2x3=3

    중심 좌표 계산 (앵커 좌표 (x, y, z), 크기 sx x sy 기준):
        cx = x + (sx - 1) / 2
        cy = y + (sy - 1) / 2
        cz = z
    예) (0,0,0)에 1x1 블록 -> 중심 (0, 0, 0)
        (0,0,0)~(1,1,0) 4칸을 차지하는 2x2 블록 -> 중심 (0.5, 0.5, 0)

    예시 발행 데이터: "[[0, [0.0, 0.0, 0.0]], [2, [0.5, 0.5, 0.0]]]"
"""
import os
import base64
import re
import json
import threading
import http.server
import socketserver

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:  # ament 환경이 아닌 경우를 위한 안전장치
    get_package_share_directory = None


# =============================================================
# ★ 저장/불러오기 폴더 경로 설정 ★
# 여기 경로만 원하는 곳으로 바꾸면 됩니다. (바꾼 뒤 colcon build 다시 실행)
# 저장할 때마다 이 폴더 안에 map1.json, map2.json, ... 순서로 파일이 생깁니다.
# '~' 는 홈 디렉토리로 자동 변환됩니다.
# =============================================================
SAV_PATH = '~/lego_map_saves'


def resolve_static_dir():
    """설치된 share 디렉토리의 static/ 을 우선 사용하고,
    (colcon build 전 로컬 실행 등) 못 찾으면 소스 트리의 static/ 을 사용한다."""
    if get_package_share_directory is not None:
        try:
            share_dir = get_package_share_directory('lego22')
            candidate = os.path.join(share_dir, 'static')
            if os.path.isdir(candidate):
                return candidate
        except Exception:
            pass

    here = os.path.dirname(os.path.abspath(__file__))
    candidate = os.path.join(here, 'static')
    if os.path.isdir(candidate):
        return candidate

    raise RuntimeError(
        "static 디렉토리를 찾을 수 없습니다. colcon build 후 다시 시도해 주세요."
    )


def make_handler(static_dir, publish_centers_fn, save_map_fn, load_map_fn, list_maps_fn, thumb_fn, delete_map_fn, progress_fn, logger):
    class Handler(http.server.SimpleHTTPRequestHandler):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, directory=static_dir, **kwargs)

        def log_message(self, fmt, *args):
            logger.debug("%s - %s" % (self.address_string(), fmt % args))

        def _send_json(self, obj, status=200):
            body = json.dumps(obj, ensure_ascii=False).encode('utf-8')
            self.send_response(status)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(body)

        def do_OPTIONS(self):
            self.send_response(204)
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
            self.send_header('Access-Control-Allow-Headers', 'Content-Type')
            self.end_headers()

        def do_GET(self):
            if self.path.startswith('/api/progress'):
                # 로봇의 /current_block_status 진행 상황 조회 (?after=<seq> 이후 것만)
                after = 0
                if '?' in self.path:
                    for part in self.path.split('?', 1)[1].split('&'):
                        if part.startswith('after='):
                            try:
                                after = int(part.split('=', 1)[1])
                            except ValueError:
                                pass
                try:
                    data = progress_fn(after)
                    self._send_json({'ok': True, **data})
                except Exception as e:
                    self._send_json({'ok': False, 'error': str(e)}, status=500)
                return

            if self.path == '/api/maps':
                # 저장 폴더 안의 맵 파일 목록 반환
                try:
                    self._send_json({'ok': True, 'maps': list_maps_fn()})
                except Exception as e:
                    self._send_json({'ok': False, 'error': str(e)}, status=500)
                return

            if self.path.startswith('/api/thumb'):
                # 저장된 맵의 썸네일 이미지(png) 서빙
                try:
                    from urllib.parse import urlparse, parse_qs
                    qs = parse_qs(urlparse(self.path).query)
                    name = qs.get('name', [None])[0]
                    if not name:
                        raise ValueError('name 이 필요합니다.')
                    data = thumb_fn(name)
                    self.send_response(200)
                    self.send_header('Content-Type', 'image/png')
                    self.send_header('Content-Length', str(len(data)))
                    self.send_header('Cache-Control', 'no-cache')
                    self.end_headers()
                    self.wfile.write(data)
                except FileNotFoundError:
                    self.send_response(404)
                    self.end_headers()
                except Exception:
                    self.send_response(500)
                    self.end_headers()
                return

            if self.path.startswith('/api/load'):
                try:
                    from urllib.parse import urlparse, parse_qs
                    qs = parse_qs(urlparse(self.path).query)
                    name = qs.get('name', [None])[0]
                    if not name:
                        raise ValueError('불러올 파일 이름(name)이 필요합니다.')
                    blocks = load_map_fn(name)
                    self._send_json({'ok': True, 'name': name, 'blocks': blocks})
                except FileNotFoundError:
                    self._send_json({'ok': False, 'error': '해당 파일이 없습니다.'}, status=404)
                except Exception as e:
                    self._send_json({'ok': False, 'error': str(e)}, status=500)
                return

            # 그 외 경로는 정적 파일 서빙 (index.html, app.js, style.css ...)
            super().do_GET()

        def do_POST(self):
            if self.path == '/api/publish_centers':
                try:
                    length = int(self.headers.get('Content-Length', 0))
                    raw = self.rfile.read(length) if length > 0 else b'[]'
                    centers = json.loads(raw.decode('utf-8'))
                    publish_centers_fn(centers)
                    self._send_json({'ok': True, 'count': len(centers)})
                except Exception as e:
                    self._send_json({'ok': False, 'error': str(e)}, status=500)
                return

            if self.path == '/api/save':
                try:
                    length = int(self.headers.get('Content-Length', 0))
                    raw = self.rfile.read(length) if length > 0 else b'[]'
                    payload = json.loads(raw.decode('utf-8'))
                    # {blocks, thumbnail, name, overwrite} 또는 그냥 [...] 둘 다 허용
                    if isinstance(payload, dict):
                        blocks = payload.get('blocks', [])
                        thumbnail = payload.get('thumbnail')
                        name = payload.get('name')
                        overwrite = bool(payload.get('overwrite'))
                    else:
                        blocks, thumbnail, name, overwrite = payload, None, None, False
                    if not isinstance(blocks, list):
                        raise ValueError('blocks 는 리스트여야 합니다.')
                    path = save_map_fn(blocks, thumbnail, name, overwrite)
                    self._send_json({'ok': True, 'count': len(blocks), 'path': path})
                except FileExistsError as e:
                    self._send_json({'ok': False, 'error': 'exists', 'name': str(e)}, status=409)
                except Exception as e:
                    self._send_json({'ok': False, 'error': str(e)}, status=500)
                return

            if self.path == '/api/delete':
                try:
                    length = int(self.headers.get('Content-Length', 0))
                    raw = self.rfile.read(length) if length > 0 else b'{}'
                    payload = json.loads(raw.decode('utf-8'))
                    name = payload.get('name')
                    if not name:
                        raise ValueError('삭제할 파일 이름(name)이 필요합니다.')
                    delete_map_fn(name)
                    self._send_json({'ok': True, 'name': name})
                except FileNotFoundError:
                    self._send_json({'ok': False, 'error': '해당 파일이 없습니다.'}, status=404)
                except Exception as e:
                    self._send_json({'ok': False, 'error': str(e)}, status=500)
                return

            self.send_response(404)
            self.end_headers()

    return Handler


class ThreadingHTTPServer(socketserver.ThreadingMixIn, http.server.HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


def bind_server_with_retry(host, start_port, handler_cls, logger, max_tries=10):
    """start_port 가 이미 사용 중이면 start_port+1, +2 ... 순으로 최대 max_tries번 시도한다.
    바인딩에 성공한 (httpd, 실제로 물린 port) 를 반환하고, 전부 실패하면 RuntimeError를 낸다."""
    last_err = None
    for offset in range(max_tries):
        port = start_port + offset
        try:
            httpd = ThreadingHTTPServer((host, port), handler_cls)
            return httpd, port
        except OSError as e:
            last_err = e
            if getattr(e, 'errno', None) == 98:  # Address already in use
                if offset < max_tries - 1:
                    logger.warn(f"포트 {port} 이(가) 이미 사용 중입니다. 다음 포트로 재시도합니다...")
                continue
            raise

    if max_tries == 1:
        raise RuntimeError(
            f"포트 {start_port} 이(가) 이미 사용 중이라 서버를 시작할 수 없습니다 "
            f"(마지막 오류: {last_err}). "
            f"해당 포트를 쓰는 프로세스를 종료한 뒤 다시 실행해 주세요. "
            f"(확인: 'lsof -i :{start_port}' 또는 'fuser -k {start_port}/tcp')"
        )
    raise RuntimeError(
        f"{start_port}번부터 {start_port + max_tries - 1}번까지 포트를 모두 사용할 수 없습니다 "
        f"(마지막 오류: {last_err}). "
        f"'ros2 run lego22 lego22 --ros-args -p port:=<다른_포트번호>' 로 "
        f"직접 포트를 지정해서 실행해 보세요."
    )


class LegoCadWebServerNode(Node):
    def __init__(self):
        super().__init__('lego22_server')

        self.declare_parameter('port', 8080)
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('sav_path', SAV_PATH)

        requested_port = self.get_parameter('port').value
        self.host = self.get_parameter('host').value
        self.sav_path = os.path.expanduser(self.get_parameter('sav_path').value)

        self.static_dir = resolve_static_dir()

        # /centers 발행자 (std_msgs/String, JSON 페이로드)
        self.centers_pub = self.create_publisher(String, 'centers', 10)

        # /current_block_status 구독자 (로봇이 지금 조립 중인 블록) -> 웹 폴링용 버퍼
        self._progress_lock = threading.Lock()
        self._progress_items = []  # [{'seq': n, 'block': [...]}, ...] 최근 500개 유지
        self._progress_seq = 0
        self.block_status_sub = self.create_subscription(
            String, '/current_block_status', self.on_block_status, 10)

        handler_cls = make_handler(
            self.static_dir, self.publish_centers,
            self.save_map, self.load_map, self.list_maps,
            self.get_thumb, self.delete_map, self.get_progress, self.get_logger()
        )
        # 8080이 사용 중이면 8081, 8082 ... 순으로 다음 포트를 자동으로 시도한다
        self.httpd, self.port = bind_server_with_retry(
            self.host, requested_port, handler_cls, self.get_logger(), max_tries=10
        )

        self.get_logger().info(f"정적 파일 경로: {self.static_dir}")
        if self.port != requested_port:
            self.get_logger().warn(
                f"요청한 포트 {requested_port} 대신 {self.port} 번 포트를 사용합니다."
            )
        self.get_logger().info(
            f"LEGO CAD 웹 서버 시작됨 -> http://localhost:{self.port}  "
            f"(같은 네트워크에서는 http://<이_PC_IP>:{self.port})"
        )
        self.get_logger().info("웹 UI의 '출력하기' 버튼을 누르면 /centers 토픽으로 한 번 발행됩니다.")
        self.get_logger().info(f"저장/불러오기 폴더 (sav_path): {self.sav_path}")

    # ---------- 저장/불러오기 ----------
    _MAP_RE = re.compile(r'^map(\d+)\.json$')

    def _safe_map_path(self, name):
        """파일 이름 검증 후 저장 폴더 안의 절대 경로 반환 (경로 탈출 방지)."""
        if name != os.path.basename(name) or not name.endswith('.json'):
            raise ValueError(f'잘못된 파일 이름: {name}')
        return os.path.join(self.sav_path, name)

    def _next_map_name(self):
        """폴더 안의 map<번호>.json 중 가장 큰 번호 + 1 로 새 이름을 만든다."""
        max_n = 0
        if os.path.isdir(self.sav_path):
            for fn in os.listdir(self.sav_path):
                m = self._MAP_RE.match(fn)
                if m:
                    max_n = max(max_n, int(m.group(1)))
        return f'map{max_n + 1}.json'

    def _normalize_name(self, name):
        """사용자가 입력한 맵 이름을 검증하고 파일 이름(.json)으로 정리한다."""
        name = name.strip()
        if not name:
            return None
        if name.startswith('.'):
            raise ValueError('이름은 . 으로 시작할 수 없습니다.')
        if not name.endswith('.json'):
            name += '.json'
        self._safe_map_path(name)  # 경로 탈출 등 검증 (실패 시 ValueError)
        return name

    def save_map(self, blocks, thumbnail_data_url=None, name=None, overwrite=False):
        """blocks 리스트를 sav_path 폴더에 저장한다.
        name 이 주어지면 그 이름(.json)으로, 없으면 map1, map2 ... 자동 번호로 저장한다.
        같은 이름이 이미 있으면 overwrite=True 일 때만 덮어쓴다 (아니면 FileExistsError).
        thumbnail_data_url 이 주어지면 같은 이름의 .png 썸네일도 함께 저장한다."""
        os.makedirs(self.sav_path, exist_ok=True)

        if name:
            fname = self._normalize_name(name)
        else:
            fname = None
        if not fname:
            fname = self._next_map_name()

        path = os.path.join(self.sav_path, fname)
        png_path = path[:-len('.json')] + '.png'

        if os.path.exists(path) and not overwrite:
            raise FileExistsError(fname)

        tmp_path = path + '.tmp'
        with open(tmp_path, 'w', encoding='utf-8') as f:
            json.dump({'version': 1, 'blocks': blocks}, f, ensure_ascii=False, indent=2)
        os.replace(tmp_path, path)  # 원자적 교체

        # 덮어쓰기 시 옛 썸네일이 남지 않도록 먼저 제거
        if os.path.isfile(png_path):
            os.remove(png_path)
        if thumbnail_data_url:
            try:
                b64 = thumbnail_data_url.split(',', 1)[1]
                with open(png_path, 'wb') as f:
                    f.write(base64.b64decode(b64))
            except Exception as e:
                self.get_logger().warn(f"썸네일 저장 실패 (맵은 정상 저장됨): {e}")

        self.get_logger().info(f"맵 저장됨: 블록 {len(blocks)}개 -> {path}")
        return path

    def get_thumb(self, name):
        """맵 이름(mapN.json)에 대응하는 썸네일 png 바이트를 반환한다."""
        json_path = self._safe_map_path(name)
        png_path = json_path[:-len('.json')] + '.png'
        with open(png_path, 'rb') as f:
            return f.read()

    def delete_map(self, name):
        """맵 파일(json)과 썸네일(png)을 함께 삭제한다."""
        json_path = self._safe_map_path(name)
        if not os.path.isfile(json_path):
            raise FileNotFoundError(json_path)
        os.remove(json_path)
        png_path = json_path[:-len('.json')] + '.png'
        if os.path.isfile(png_path):
            os.remove(png_path)
        self.get_logger().info(f"맵 삭제됨: {json_path}")

    def load_map(self, name):
        """sav_path 폴더 안의 지정된 파일에서 blocks 리스트를 읽는다."""
        path = self._safe_map_path(name)
        with open(path, 'r', encoding='utf-8') as f:
            data = json.load(f)
        blocks = data.get('blocks', []) if isinstance(data, dict) else data
        if not isinstance(blocks, list):
            raise ValueError('저장 파일 형식이 올바르지 않습니다.')
        self.get_logger().info(f"맵 불러옴: 블록 {len(blocks)}개 <- {path}")
        return blocks

    def list_maps(self):
        """저장 폴더 안의 .json 맵 파일 목록을 [{name, blocks, mtime}, ...] 로 반환한다."""
        if not os.path.isdir(self.sav_path):
            return []
        items = []
        for fn in sorted(os.listdir(self.sav_path)):
            if not fn.endswith('.json'):
                continue
            path = os.path.join(self.sav_path, fn)
            entry = {
                'name': fn,
                'blocks': None,
                'mtime': int(os.path.getmtime(path)),
                'thumb': os.path.isfile(path[:-len('.json')] + '.png'),
            }
            try:
                with open(path, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                b = data.get('blocks', []) if isinstance(data, dict) else data
                if isinstance(b, list):
                    entry['blocks'] = len(b)
            except Exception:
                pass  # 깨진 파일이어도 목록에는 표시
            items.append(entry)
        # map 번호 순 정렬 (map2 < map10), 그 외 이름은 뒤에 사전순
        def sort_key(e):
            m = self._MAP_RE.match(e['name'])
            return (0, int(m.group(1))) if m else (1, e['name'])
        items.sort(key=sort_key)
        return items

    def on_block_status(self, msg):
        """로봇이 발행하는 /current_block_status ([타입, [gx, gy, gz]] JSON) 수신."""
        try:
            block = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f"/current_block_status 파싱 실패: {e}")
            return
        with self._progress_lock:
            self._progress_seq += 1
            self._progress_items.append({'seq': self._progress_seq, 'block': block})
            if len(self._progress_items) > 500:
                self._progress_items = self._progress_items[-500:]
        self.get_logger().info(f"조립 진행 수신 #{self._progress_seq}: {msg.data}")

    def get_progress(self, after=0):
        """seq가 after보다 큰 진행 항목들과 최신 seq를 반환 (웹 폴링용)."""
        with self._progress_lock:
            items = [it for it in self._progress_items if it['seq'] > after]
            return {'latest': self._progress_seq, 'items': items}

    def publish_centers(self, centers):
        """centers: [[type_index, [cx, cy, cz]], ...] 형태의 리스트.
        std_msgs/String 에 JSON으로 인코딩해서 /centers 로 한 번 발행한다."""
        msg = String()
        msg.data = json.dumps(centers, ensure_ascii=False)
        self.centers_pub.publish(msg)
        self.get_logger().info(f"/centers 발행: 블록 {len(centers)}개 -> {msg.data}")

    def serve_forever(self):
        self.httpd.serve_forever()

    def shutdown(self):
        self.httpd.shutdown()
        self.httpd.server_close()


def main(args=None):
    rclpy.init(args=args)

    try:
        node = LegoCadWebServerNode()
    except RuntimeError as e:
        print(f"[lego22] 시작 실패: {e}", file=__import__('sys').stderr)
        rclpy.shutdown()
        return 1

    server_thread = threading.Thread(target=node.serve_forever, daemon=True)
    server_thread.start()

    # 브라우저 자동 오픈 (localhost:<실제 바인딩된 포트>)
    url = f'http://localhost:{node.port}'
    opened = False
    try:
        import webbrowser
        opened = bool(webbrowser.open(url))
    except Exception:
        opened = False
    if not opened:
        # webbrowser 가 실패하는 환경(WSL, xdg 미설정 등)을 위한 폴백
        import shutil
        import subprocess
        for cmd in (['xdg-open', url], ['wslview', url],
                    ['cmd.exe', '/c', 'start', '', url], ['open', url]):
            if shutil.which(cmd[0]):
                try:
                    subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                    opened = True
                    break
                except Exception:
                    continue
    if opened:
        node.get_logger().info(f"브라우저를 자동으로 열었습니다 -> {url}")
    else:
        node.get_logger().warn(f"브라우저 자동 오픈에 실패했습니다. 직접 접속해 주세요 -> {url}")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
