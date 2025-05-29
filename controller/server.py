import lib.logging as logging
import threading
import socket
import time


class ControlServer(threading.Thread):
    NO_CONTROLLER_TIMEOUT = 0.5  # Seconds before assuming controller is lost

    def __init__(self, listen_address, command_state_ref):
        super().__init__(daemon=True)
        self.listen_address = listen_address
        self.log = logging.getLogger(self.__class__.__name__)
        self.command_state = command_state_ref
        self.server_socket = None
        self.running = True
        self.last_message_time = time.time()
        self.controller_active = False

    def run(self):
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.server_socket.setsockopt(
            socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        try:
            self.server_socket.bind(self.listen_address)
            self.log.info(
                f"Listening for full state on {self.listen_address[0]}:{self.listen_address[1]}")
            # Shorter timeout for more responsive checks
            self.server_socket.settimeout(0.1)

            while self.running:
                try:
                    data, addr = self.server_socket.recvfrom(
                        1024)  # Buffer size
                    state_str = data.decode().strip()
                    # self.log.debug(f"Received state: '{state_str}' from {addr}") # For debugging

                    parts = state_str.split(':')
                    if len(parts) == 3:
                        throttle_state, pitch_state, shoot_request = parts
                        self.command_state['throttle'] = (
                            throttle_state == '1')
                        self.command_state['pitch'] = int(pitch_state)
                        if shoot_request == '1':
                            self.command_state['shoot_request'] = True
                        # Note: shoot_request is consumed by game logic and set to False there.
                        # Controller should send '1' only for frames it wants to shoot.

                        self.last_message_time = time.time()
                        if not self.controller_active:
                            self.log.info("Controller connected/active.")
                            self.controller_active = True
                    else:
                        if self.running:  # Avoid print if stopping
                            self.log.warning(
                                f"Received malformed state: {state_str}")

                except socket.timeout:
                    # Check if controller has timed out
                    if self.controller_active and (time.time() - self.last_message_time > self.NO_CONTROLLER_TIMEOUT):
                        self.log.warning(
                            f"Controller timed out (no messages for {self.NO_CONTROLLER_TIMEOUT}s). Resetting actions.")
                        self._reset_actions_to_neutral()
                        self.controller_active = False  # Mark as inactive until new message
                    continue  # Loop back to check self.running flag and try recvfrom again
                except OSError as e:
                    if self.running:
                        self.log.error(f"OSError: {e}")
                    break
                except ValueError as e:  # For int() conversion errors
                    if self.running:
                        self.log.error(
                            f"Error parsing state string '{state_str}': {e}")
                except Exception as e:
                    if self.running:
                        self.log.error(
                            f"Error processing command: {e}")

        except Exception as e:
            self.log.error(f"Could not start server: {e}")
        finally:
            if self.server_socket:
                self.server_socket.close()
            self._reset_actions_to_neutral()  # Ensure actions are neutral when server stops
            self.log.info("Stopped.")

    def _reset_actions_to_neutral(self):
        """Resets ship actions to a neutral state."""
        self.command_state['throttle'] = False
        self.command_state['pitch'] = 0
        self.command_state['shoot_request'] = False

    def stop(self):
        self.log.info("Attempting to stop...")
        self.running = False
        if self.server_socket:
            # Closing the socket will make recvfrom in the run loop raise an error or return,
            # allowing the loop to terminate if it's blocked there.
            self.server_socket.close()
