import socket
import time

class RobotiqGripper:
    def __init__(self):
        self.client = None

    def connect(self, ip, port):
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client.connect((ip, port))

    def send_command(self, command):
        if self.client:
            self.client.sendall(command.encode())
            time.sleep(0.1)
            return self.client.recv(1024).decode()
        return None

    def activate(self):
        return self.send_command("ACTIVATE")

    def move_and_wait_for_pos(self, pos, speed, force):
        command = f"SET POS {pos} SPEED {speed} FORCE {force}"
        return self.send_command(command)

    def get_current_position(self):
        return self.send_command("GET POS")

    def is_open(self):
        return self.send_command("IS OPEN")

    def is_closed(self):
        return self.send_command("IS CLOSED")

    def close(self):
        if self.client:
            self.client.close()
            self.client = None
