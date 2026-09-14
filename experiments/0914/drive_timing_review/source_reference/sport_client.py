"""Tiny client for the existing Jazzy-to-Foxy Unitree Sport Unix socket."""

from __future__ import annotations

import itertools
import math
import socket
import struct
import time
import zlib
from dataclasses import dataclass


_MAGIC = b"RNC1"
_VERSION = 3
_REQUEST = 1
_RESPONSE = 2
_PING = 1
_MOVE = 2
_STOP = 3
_BODY_LOOK = 4
_BODY_RETURN = 5
_BODY_STATUS = 6
_HEADER = struct.Struct("!4sBBBBQII")
_MOVE_PAYLOAD = struct.Struct("!qQfff")
_RESPONSE_PAYLOAD = struct.Struct("!qi")
_BODY_REQUEST_PAYLOAD = struct.Struct("!qQb")
_BODY_STATUS_PAYLOAD = struct.Struct("!qiBQqf64s")


@dataclass(frozen=True)
class BodyStatus:
    state: int  # 0 idle, 1 pitching, 2 holding, 3 returning, 4 complete, 5 failed
    look_id: int
    frame_stamp_ns: int
    relative_pitch_deg: float
    error: str


class SportClientError(RuntimeError):
    pass


def _wire_velocity(value: float) -> float:
    """Encode float32 speed without increasing its requested magnitude.

    Round-to-nearest turns 0.6 into 0.60000002384 on this protocol. The
    receiver correctly rejects that against its exact 0.6 safety limit.
    Use the adjacent float32 toward zero when ordinary rounding overshoots;
    keep the receiver's limits and rejection behavior unchanged.
    """
    value = float(value)
    if not math.isfinite(value):
        raise ValueError("velocity must be finite")
    try:
        packed = struct.pack("!f", value)
    except (OverflowError, struct.error) as error:
        raise ValueError("velocity is outside the wire format") from error
    encoded = struct.unpack("!f", packed)[0]
    if abs(encoded) > abs(value):
        bits = struct.unpack("!I", packed)[0]
        encoded = struct.unpack("!f", struct.pack("!I", bits - 1))[0]
    return encoded


class SportClient:
    """One socket connection per command, matching the Foxy sidecar."""

    def __init__(self, socket_path: str, timeout_s: float = 0.5) -> None:
        if not socket_path:
            raise ValueError("socket_path must not be empty")
        if not math.isfinite(timeout_s) or timeout_s <= 0.0:
            raise ValueError("timeout_s must be positive")
        self.socket_path = socket_path
        self.timeout_s = float(timeout_s)
        self._session_id = 0
        seed = time.monotonic_ns() & 0x3FFFFFFFFFFFFFFF
        self._request_ids = itertools.count(max(1, seed))

    def ping(self) -> int:
        # A failed readiness check must not leave the previous motion session
        # usable. Stop remains available without a session.
        self._session_id = 0
        session_id, result = self._call(_PING)
        if result != 0:
            raise SportClientError("Sport ping failed with code %d" % result)
        # An idle executor may predate a standalone body-look probe. Re-enable
        # explicitly hands commands back without restarting that executor.
        # Keep old/duplicate requests rejected by the server; do not reseed on
        # ordinary motion calls or reset the server's accepted request ID.
        self._request_ids = itertools.count(max(
            next(self._request_ids), time.monotonic_ns() & 0x3FFFFFFFFFFFFFFF))
        self._session_id = session_id
        return session_id

    def move(self, linear_x_mps: float, angular_z_radps: float) -> None:
        values = (float(linear_x_mps), float(angular_z_radps))
        if not all(math.isfinite(value) for value in values):
            raise ValueError("velocity must be finite")
        if not self._session_id:
            raise SportClientError("Sport Move requires an explicit readiness ping")
        session_id, result = self._call(
            _MOVE,
            linear_x_mps=values[0],
            angular_z_radps=values[1],
        )
        if session_id != self._session_id:
            self._session_id = 0
            raise SportClientError("Sport server restarted; motion must be re-enabled")
        if result != 0:
            raise SportClientError("Sport Move failed with code %d" % result)

    def stop(self) -> None:
        _session_id, result = self._call(_STOP)
        if result != 0:
            raise SportClientError("Sport StopMove failed with code %d" % result)

    def body_look(self, action):
        if action not in ('look_up', 'look_down'):
            raise ValueError('invalid body look direction')
        self._body_call(_BODY_LOOK, direction=1 if action == 'look_down' else -1)

    def body_return(self):
        self._body_call(_BODY_RETURN)

    def body_status(self):
        return self._body_call(_BODY_STATUS)

    def _body_call(self, operation, direction=0):
        if not self._session_id:
            raise SportClientError('Body look requires an explicit readiness ping')
        response = self._call(operation, look_direction=direction)
        session_id, result = response[:2]
        if session_id != self._session_id:
            self._session_id = 0
            raise SportClientError('Sport server restarted; motion must be re-enabled')
        if result != 0:
            raise SportClientError('Body command rejected with code %d' % result)
        if operation == _BODY_STATUS:
            state, look_id, frame_stamp, pitch, error = response[2:]
            if state not in range(6) or not math.isfinite(pitch):
                raise SportClientError('invalid body status')
            return BodyStatus(state, look_id, frame_stamp, pitch, error.rstrip(b'\0').decode('utf-8', errors='replace'))

    def _call(
        self,
        operation: int,
        *,
        linear_x_mps: float = 0.0,
        angular_z_radps: float = 0.0,
        look_direction: int = 0,
    ) -> tuple[int, int]:
        request_id = next(self._request_ids) & 0x7FFFFFFFFFFFFFFF
        if request_id == 0:
            request_id = 1
        if operation == _MOVE:
            payload = _MOVE_PAYLOAD.pack(
                self._session_id,
                time.monotonic_ns() + int(min(self.timeout_s, 0.2) * 1e9),
                _wire_velocity(linear_x_mps),
                0.0,
                _wire_velocity(angular_z_radps),
            )
        elif operation in (_BODY_LOOK, _BODY_RETURN, _BODY_STATUS):
            payload = _BODY_REQUEST_PAYLOAD.pack(self._session_id,
                time.monotonic_ns()+int(min(self.timeout_s, .2)*1e9), look_direction)
        else:
            payload = b""
        response_format = _BODY_STATUS_PAYLOAD if operation == _BODY_STATUS else _RESPONSE_PAYLOAD
        header = _HEADER.pack(
            _MAGIC,
            _VERSION,
            _REQUEST,
            operation,
            0,
            request_id,
            len(payload),
            zlib.crc32(payload) & 0xFFFFFFFF,
        )
        try:
            with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
                connection.settimeout(self.timeout_s)
                connection.connect(self.socket_path)
                connection.sendall(header + payload)
                response_header = self._receive_exact(connection, _HEADER.size)
                (
                    magic,
                    version,
                    kind,
                    response_operation,
                    flags,
                    response_id,
                    payload_size,
                    payload_crc,
                ) = _HEADER.unpack(response_header)
                if (
                    magic != _MAGIC
                    or version != _VERSION
                    or kind != _RESPONSE
                    or response_operation != operation
                    or flags != 0
                    or response_id != request_id
                    or payload_size != response_format.size
                ):
                    raise SportClientError("invalid Sport response header")
                response_payload = self._receive_exact(connection, payload_size)
        except (OSError, TimeoutError) as error:
            raise SportClientError("Sport command socket unavailable: %s" % error) from error
        if zlib.crc32(response_payload) & 0xFFFFFFFF != payload_crc:
            raise SportClientError("invalid Sport response payload")
        response = response_format.unpack(response_payload)
        session_id, result = response[:2]
        if session_id <= 0:
            raise SportClientError("invalid Sport response session")
        return response

    @staticmethod
    def _receive_exact(connection: socket.socket, size: int) -> bytes:
        chunks = bytearray()
        while len(chunks) < size:
            chunk = connection.recv(size - len(chunks))
            if not chunk:
                raise SportClientError("Sport command socket closed early")
            chunks.extend(chunk)
        return bytes(chunks)
