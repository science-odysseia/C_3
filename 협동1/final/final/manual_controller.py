#!/usr/bin/env python3
"""Web-controlled version of the interrupt keyboard jog program."""
import threading
import time

import DSR_ROBOT2 as dsr
from pymodbus.client import ModbusTcpClient


# jog_multi 속도는 로봇 정격 TCP 조그 속도에 대한 백분율이다.
# speedl 스트림을 반복 갱신하지 않고 전용 연속 조그 명령을 사용한다.
JOG_SPEED_PERCENT = 10.0
HOME_VEL = 30.0
HOME_ACC = 30.0

COMPUTE_BOX_IP = "192.168.1.1"
COMPUTE_BOX_PORT = 502
RG2_DEVICE_ID = 65
OPEN_WIDTH = 50.0
CLOSE_WIDTH = 0.0
GRIPPER_FORCE = 40.0


class InterruptManualController:
    """TCP jog, RG2 and home controls exposed to the web API."""

    def __init__(self, logger):
        self.logger = logger
        self.lock = threading.Lock()
        self.command_lock = threading.Lock()
        self.stop_event = threading.Event()
        self.jog_running = threading.Event()
        self.active_jog = None
        self.heartbeat_at = 0.0
        self.rg2_client = ModbusTcpClient(
            COMPUTE_BOX_IP, port=COMPUTE_BOX_PORT)
        self.rg2_connected = False
        self.ui = {
            'active': False,
            'ready': False,
            'busy': False,
            'moving': False,
            'status': '출력 중단 대기 중',
            'tcp_position': None,
            'solution': None,
            'gripper_connected': False,
        }

    def snapshot(self):
        with self.lock:
            data = dict(self.ui)
            if self.ui['tcp_position'] is not None:
                data['tcp_position'] = list(self.ui['tcp_position'])
            return data

    def _update(self, **values):
        with self.lock:
            self.ui.update(values)

    def activate(self):
        # emstop이 Soft Stop 완료를 확인하기 전에는 DSR 명령을 보내지 않는다.
        self.stop_event.set()
        self.jog_running.clear()
        with self.command_lock:
            self.active_jog = None
        self._update(
            active=True, ready=False, busy=False, moving=False,
            status='인위적 정지 완료를 확인하는 중입니다...')

    def deactivate(self):
        self.stop_jog()
        self._update(
            active=False, ready=False, busy=False, moving=False,
            status='출력 중단 대기 중')

    def set_ready(self, ready):
        ready = bool(ready)
        if not self.snapshot()['active']:
            return
        self._update(
            ready=ready,
            status=(
                '수동 조작 준비 - 방향 버튼을 누르는 동안 이동합니다.'
                if ready else '인위적 정지 완료를 확인하는 중입니다...'
            ),
        )

    def _require_active(self):
        state = self.snapshot()
        if not state['active']:
            raise RuntimeError('중단 수동 조작 모드가 활성화되지 않았습니다.')
        if not state['ready']:
            raise RuntimeError('로봇의 인위적 정지 완료를 기다리는 중입니다.')

    @staticmethod
    def _velocity_for_key(key):
        vectors = {
            'i': [0, 1, 0, 0, 0, 0],
            'k': [0, -1, 0, 0, 0, 0],
            'j': [-1, 0, 0, 0, 0, 0],
            'l': [1, 0, 0, 0, 0, 0],
            'w': [0, 0, 1, 0, 0, 0],
            's': [0, 0, -1, 0, 0, 0],
        }
        if key not in vectors:
            raise ValueError('조그 키는 i/k/j/l/w/s 중 하나여야 합니다.')
        return vectors[key]

    def _send_stop(self):
        try:
            result = dsr.jog_multi(
                [0, 0, 0, 0, 0, 0],
                ref=dsr.DR_BASE,
                speed=0.0,
            )
            if result not in (0, None):
                self.logger.warn(f'수동 조그 정지 실패: return={result}')
        except Exception as error:
            self.logger.warn(f'수동 조그 정지 오류: {error}')

    def _jog_worker(self, key):
        velocity = self._velocity_for_key(key)
        labels = {
            'i': 'Y+', 'k': 'Y−', 'j': 'X−',
            'l': 'X+', 'w': 'Z+', 's': 'Z−',
        }
        self._update(moving=True, status=f'{labels[key]} 연속 이동 중')
        try:
            result = dsr.jog_multi(
                velocity,
                ref=dsr.DR_BASE,
                speed=JOG_SPEED_PERCENT,
            )
            if result not in (0, None):
                raise RuntimeError(f'jog_multi return={result}')

            # 조그 명령은 한 번 시작하면 정지 명령까지 연속 이동한다.
            # 이 스레드는 브라우저 누름 상태만 감시하고 재전송하지 않는다.
            while not self.stop_event.is_set():
                with self.command_lock:
                    heartbeat_age = time.monotonic() - self.heartbeat_at
                    current = self.active_jog
                if current != key or heartbeat_age > 0.35:
                    break
                time.sleep(0.02)
        except Exception as error:
            self.logger.error(f'수동 jog_multi 오류: {error}')
            self._update(status=f'수동 조그 오류: {error}')
        finally:
            self._send_stop()
            with self.command_lock:
                self.active_jog = None
            self.jog_running.clear()
            self.stop_event.clear()
            state = self.snapshot()
            if state['active'] and state['ready']:
                self._update(
                    moving=False,
                    status='수동 조작 준비 - 방향 버튼을 누르는 동안 이동합니다.')

    def start_jog(self, key):
        self._require_active()
        key = str(key).lower()
        self._velocity_for_key(key)
        with self.command_lock:
            if self.jog_running.is_set():
                if self.active_jog != key:
                    return False
                self.heartbeat_at = time.monotonic()
                return True
            if self.snapshot()['busy']:
                return False
            self.active_jog = key
            self.heartbeat_at = time.monotonic()
            self.jog_running.set()
            self.stop_event.clear()
        threading.Thread(target=self._jog_worker, args=(key,), daemon=True).start()
        return True

    def stop_jog(self):
        self.stop_event.set()
        self._update(moving=False)
        return True

    def _rg2_write_registers(self, address, values):
        last_error = None
        for keyword in ('device_id', 'slave', 'unit'):
            try:
                return self.rg2_client.write_registers(
                    address, values, **{keyword: RG2_DEVICE_ID})
            except TypeError as error:
                last_error = error
        raise last_error

    def _rg2_move(self, width_mm, force_n):
        if not self.rg2_connected:
            self.rg2_connected = bool(self.rg2_client.connect())
            self._update(gripper_connected=self.rg2_connected)
        if not self.rg2_connected:
            raise RuntimeError('RG2 Modbus 연결 실패')
        try:
            result = self._rg2_write_registers(
                0, [int(round(force_n * 10)), int(round(width_mm * 10)), 1])
            if result.isError():
                raise RuntimeError(f'RG2 Modbus 응답 오류: {result}')
        except Exception:
            self.rg2_connected = False
            self._update(gripper_connected=False)
            raise

    def gripper(self, command):
        self._require_active()
        if command not in ('open', 'close'):
            raise ValueError('그리퍼 명령은 open 또는 close여야 합니다.')
        if self.snapshot()['busy'] or self.jog_running.is_set():
            return False
        self._update(busy=True, status='그리퍼 명령 처리 중...')

        def worker():
            try:
                width = OPEN_WIDTH if command == 'open' else CLOSE_WIDTH
                self._rg2_move(width, GRIPPER_FORCE)
                label = '열기 (50 mm)' if command == 'open' else '닫기 (0 mm)'
                self._update(status=f'그리퍼 {label} 명령 완료')
            except Exception as error:
                self.logger.error(f'RG2 명령 실패: {error}')
                self._update(status=f'RG2 명령 실패: {error}')
            finally:
                self._update(busy=False)

        threading.Thread(target=worker, daemon=True).start()
        return True

    def read_position(self):
        self._require_active()
        if self.snapshot()['busy'] or self.jog_running.is_set():
            return False
        self._update(busy=True, status='현재 TCP 위치 조회 중...')

        def worker():
            try:
                pos, solution = dsr.get_current_posx()
                values = [float(pos[index]) for index in range(6)]
                self._update(
                    tcp_position=values, solution=int(solution),
                    status='현재 TCP 위치를 갱신했습니다.')
            except Exception as error:
                self.logger.error(f'TCP 위치 조회 실패: {error}')
                self._update(status=f'TCP 위치 조회 실패: {error}')
            finally:
                self._update(busy=False)

        threading.Thread(target=worker, daemon=True).start()
        return True

    def move_home(self):
        self._require_active()
        if self.snapshot()['busy'] or self.jog_running.is_set():
            return False
        self._update(busy=True, status='Home Joint 이동 중...')

        def worker():
            try:
                result = dsr.movej(
                    dsr.posj(0, 0, 90, 0, 90, 0),
                    vel=HOME_VEL, acc=HOME_ACC)
                self._update(status=f'Home Joint 이동 완료 (return: {result})')
            except Exception as error:
                self.logger.error(f'Home Joint 이동 오류: {error}')
                self._update(status=f'Home Joint 이동 오류: {error}')
            finally:
                self._update(busy=False)

        threading.Thread(target=worker, daemon=True).start()
        return True

    def shutdown(self):
        self.deactivate()
        try:
            self.rg2_client.close()
        except Exception:
            pass
