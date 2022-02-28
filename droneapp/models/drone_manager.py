import logging
import contextlib
import os
import socket
import subprocess
import threading
import time
from pathlib import Path

import cv2 as cv
import numpy as np

from droneapp.models.base import Singleton

logger = logging.getLogger(__name__)

DEFAULT_DISTANCE = 0.30
DEFAULT_SPEED = 10
DEFAULT_DEGREE = 10

FRAME_X = int(960/3)
FRAME_Y = int(720/3)
FRAME_AREA = FRAME_X * FRAME_Y

FRAME_SIZE = FRAME_AREA * 3
FRAME_CENTER_X = FRAME_X / 2
FRAME_CENTER_Y = FRAME_Y / 2

CMD_FFMPEG = (f'ffmpeg -hwaccel auto -hwaccel_device opencl -i pipe:0 '
              f'-pix_fmt bgr24 -s {FRAME_X}x{FRAME_Y} -f rawvideo pipe:1')

FACE_DETECT_XML_FILE = './droneapp/models/haarcascade_frontalface_default.xml'

SNAPSHOT_IMAGE_FOLDER = './droneapp/static/img/snapshots/'


class ErrorNoFaceDetectXMLFile(Exception):
    """Error: There is no face detect xml file"""


class ErrorNoImageDir(Exception):
    """Error: There is no image dir"""


class DroneManager(metaclass=Singleton):
    """参考にしたもの　


    url: https://dl-cdn.ryzerobotics.com/downloads/Tello/Tello%20SDK%202.0%20User%20Guide.pdf
    github : https://github.com/dji-sdk/Tello-Python


    """

    def __init__(self, host_ip='192.168.10.2', host_port=8889, drone_ip='192.168.10.1',
                 drone_port=8889, speed=DEFAULT_SPEED):

        self.host_ip = host_ip
        self.host_port = host_port
        self.drone_ip = drone_ip
        self.drone_port = drone_port
        self.drone_address = (self.drone_ip, self.drone_port)

        # ドローンのスピード調整
        self.speed = speed

        # socket初期化処理
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((self.host_ip, self.host_port))

        self.response = None
        self.stop_event = threading.Event()
        self._response_thread_from_drone = threading.Thread(
            target=self.receive_response_from_drone,
            args=(self.stop_event, )
        )
        # ここでドローンからのレスポンスを受け取るスレッドスタート
        self._response_thread_from_drone.start()

        self.patrol_event = None
        self.is_patrol = False
        self._patrol_semaphore = threading.Semaphore(1)
        self._thread_patrol = None

        self.proc = subprocess.Popen(
            CMD_FFMPEG.split(' '),
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE
        )
        self.proc_stdin = self.proc.stdin
        self.proc_stdout = self.proc.stdout

        self.video_port = 11111

        self._receive_video_thread = threading.Thread(
            target=self.receive_video,
            args=(self.stop_event, self.proc_stdin, self.host_ip, self.video_port, )
        )
        self._receive_video_thread.start()

        # xmlファイルがあるか調べる
        if not Path(FACE_DETECT_XML_FILE).exists():
            raise ErrorNoFaceDetectXMLFile
        self.face_cascade = cv.CascadeClassifier(FACE_DETECT_XML_FILE)
        self._is_enable_face_detect = False

        # ドローンが撮影したスナップショット格納ディレクトリあるかどうか調べる
        if not Path(SNAPSHOT_IMAGE_FOLDER).exists():
            raise ErrorNoImageDir
        self.is_snapshot = False

        self._command_semaphore = threading.Semaphore(1)
        self._command_thread = None

        # ドローン飛ばす前にまずこのコマンドを送信しないといけない
        self.send_command('command')
        self.send_command('streamon')

        self.set_speed(self.speed)

    def receive_response_from_drone(self, stop_event):
        while not stop_event.is_set():
            try:
                self.response, ip = self.socket.recvfrom(3000)
                logger.info({'action': 'receive_response', 'response': self.response})
            except socket.error as ex:
                logger.error({'action': 'receive_response', 'ex': ex})
                break

    def __del__(self):
        self.stop()

    def stop(self):
        self.stop_event.set()

        retry = 0
        while self._response_thread_from_drone.is_alive():
            time.sleep(0.3)
            if retry > 100:
                break
            retry += 1

        self.socket.close()

        import signal
        os.kill(self.proc.pid, signal.CTRL_C_EVENT)

    def send_command(self, command, blocking=True):
        self._command_thread = threading.Thread(
            target=self._send_command,
            args=(command, blocking),
        )
        self._command_thread.start()

    def _send_command(self, command, blocking=True):
        is_acquire = self._command_semaphore.acquire(blocking=blocking)
        if is_acquire:
            with contextlib.ExitStack() as stack:
                stack.callback(self._command_semaphore.release)
                logger.info({'action': '_send_command', 'command': command})
                self.socket.sendto(command.encode('utf-8'), self.drone_address)

                retry = 0
                while self.response is None:
                    time.sleep(0.3)
                    if retry > 3:
                        break
                    retry += 1

                if self.response is None:
                    response = None
                else:
                    response = self.response.decode('utf-8')
                self.response = None

                return response

        else:
            logger.warning({'action': '_send_command', 'command': command, 'status': 'is_not_acquired'})

    def set_speed(self, speed=DEFAULT_SPEED):
        return self.send_command(f'speed {speed}')

    def clockwise(self, degree=DEFAULT_DEGREE):
        return self.send_command(f'cw {degree}')

    def counter_clockwise(self, degree=DEFAULT_DEGREE):
        return self.send_command(f'ccw {degree}')

    def flip_front(self):
        self.send_command('flip f')

    def flip_back(self):
        self.send_command('flip b')

    def flip_left(self):
        self.send_command('flip l')

    def flip_right(self):
        self.send_command('flip r')

    def takeoff(self):
        self.send_command('takeoff')

    def land(self):
        self.send_command('land')

    def move(self, move_direction, distance):
        """

        このmoveメソッドをベースに
        下記、up, down, left, right, forward, backメソッドを実装

        """

        distance = int(round(distance * 100))
        return self.send_command(f'{move_direction} {distance}')

    def up(self, distance=DEFAULT_DISTANCE):
        return self.move('up', distance)

    def down(self, distance=DEFAULT_DISTANCE):
        return self.move('down', distance)

    def right(self, distance=DEFAULT_DISTANCE):
        return self.move('right', distance)

    def left(self, distance=DEFAULT_DISTANCE):
        return self.move('left', distance)

    def forward(self, distance=DEFAULT_DISTANCE):
        return self.move('forward', distance)

    def back(self, distance=DEFAULT_DISTANCE):
        return self.move('back', distance)

    def patrol(self):
        if not self.is_patrol:
            self._thread_patrol = threading.Thread(
                target=self._patrol,
                args=(self._patrol_semaphore, self.patrol_event)
            )
            self._thread_patrol.start()
            self.is_patrol = True
        else:
            logger.info({'action': 'patrol', 'status': 'is_patrolling_now'})

    def _patrol(self, patrol_semaphore, patrol_event):
        is_acquire = patrol_semaphore.acquire(blocking=False)
        if is_acquire:
            logger.info({'action': '_patrol', 'status': 'acquired'})

            with contextlib.ExitStack as stack:
                stack.callback(patrol_semaphore.release)

                """statusの数字によってドローンの動きを変えていく"""
                change_status = 0
                while not patrol_event.is_set():
                    change_status += 1
                    if change_status == 1:
                        self.up()
                    if change_status == 2:
                        self.forward()
                    if change_status == 3:
                        self.back()
                    if change_status == 4:
                        self.flip_left()
                    if change_status == 5:
                        self.up()
                    if change_status == 6:
                        change_status = 0

                    time.sleep(10)

        else:
            logger.warning({'action': '_patrol', 'status': 'not_acquire'})

    def stop_patrol(self):
        if self.is_patrol:
            self.patrol_event.set()

            retry = 0
            while self._thread_patrol.is_Alive():
                time.sleep(0.3)
                if retry > 100:
                    break
                retry += 1

            self.is_patrol = False

    def receive_video(self, stop_event, pipe_in, host_ip, video_port):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock_video:
            sock_video.settimeout(.5)
            sock_video.bind((host_ip, video_port))
            data = bytearray(2048)

            while not stop_event.is_set():
                try:
                    size, _ = sock_video.recvfrom_into(data)
                except socket.timeout as ex:
                    logger.warning({'action': 'receive_video', 'ex': ex})
                    continue
                except socket.error as ex:
                    logger.error({'action': 'receive_video', 'ex': ex})
                    break

                try:
                    pipe_in.write(data[:size])
                    pipe_in.flush()
                except Exception as ex:
                    logger.error({'action': 'receive_video', 'ex': ex})
                    break

    def video_binary_generator(self):
        while True:
            try:
                frame = self.proc_stdout.read(FRAME_SIZE)
            except Exception as ex:
                logger.error({'action': 'video_binary_generator', 'ex': ex})
                continue

            if not frame:
                continue

            frame = np.fromstring(frame, np.uint8).reshape(FRAME_Y, FRAME_X, 3)
            yield frame

    def enable_face_detect(self):
        self._is_enable_face_detect = True

    def disable_face_detect(self):
        self._is_enable_face_detect = False

    def video_jpeg_generator(self):
        for frame in self.video_binary_generator():
            if self._is_enable_face_detect:
                if self.is_patrol:
                    self.stop_patrol()

                gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
                faces = self.face_cascade.detectMultiScale(gray, 1.3, 6)

                for (x, y, width, height) in faces:
                    cv.rectangle(frame, (x,y), (x+width, y+height), (255, 0, 0), 2)

                    face_center_x = x + (width / 2)
                    face_center_y = y + (height / 2)
                    diff_x = FRAME_CENTER_X - face_center_x
                    diff_y = FRAME_CENTER_Y - face_center_y
                    face_area = width * height
                    percent_face = face_area / FRAME_AREA

                    drone_x, drone_y, drone_z, speed = 0, 0, 0, self.speed
                    if diff_x < -30:
                        drone_y = -30
                    if diff_x > 30:
                        drone_y = 30
                    if diff_y < -15:
                        drone_z = -30
                    if diff_y > 15:
                        drone_z = 30
                    if percent_face > 0.30:
                        drone_x = -30
                    if percent_face < 0.02:
                        drone_x = 30
                    self.send_command(f'go {drone_x} {drone_y} {drone_z} {speed}',
                                      blocking=False)
                    break

            _, jpeg = cv.imencode('.jpg', frame)
            jpeg_binary = jpeg.tobytes()

            # 下記snapshotメソッド呼ばれた時のjpeg_binaryをファイルに書き込む
            if self.is_snapshot:
                backup_file = time.strftime("%Y%m%d-%H%M%S") + '.jpg'
                snapshot_file = 'snapshot.jpg'
                for filename in (backup_file, snapshot_file):
                    file_path = Path(SNAPSHOT_IMAGE_FOLDER) / filename

                    with open(file_path, 'wb') as f:
                        f.write(jpeg_binary)
                self.is_snapshot = False

            yield jpeg_binary

    def snapshot(self):
        self.is_snapshot = True

        retry = 0
        while retry < 10:
            if not self.is_patrol:
                return True
            time.sleep(0.5)
            retry += 1
        return False
