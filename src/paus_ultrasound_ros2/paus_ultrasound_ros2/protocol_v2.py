# SPDX-License-Identifier: Apache-2.0
"""Bounded USFR v2 framing, shared by Windows sender and Linux receiver."""

from dataclasses import dataclass
import json
import math
import socket
import struct
import time


PREFIX = struct.Struct('<4sIQ')
MAGIC = b'USFR'
VERSION = 2
MAX_SEQUENCE = 2**53 - 1  # Exact in MATLAB's JSON double representation.


class ProtocolError(ValueError):
    """Malformed or unsupported wire data; close the connection."""


@dataclass(frozen=True)
class Limits:
    header_bytes: int = 16384
    payload_bytes: int = 16 * 1024 * 1024
    dimension: int = 4096

    def __post_init__(self):
        for value in (self.header_bytes, self.payload_bytes, self.dimension):
            if type(value) is not int or value <= 0:
                raise ValueError('protocol limits must be positive integers')


@dataclass(frozen=True)
class Packet:
    header: dict
    payload: bytes


def number(value, name, low=0.0, high=2147483647.0):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ProtocolError(f'{name}: expected a number')
    # Range first: converting a maliciously large JSON integer to float can overflow.
    if not low <= value <= high or not math.isfinite(value):
        raise ProtocolError(f'{name}: outside [{low}, {high}]')
    return float(value)


def integer(value, name, low=0, high=MAX_SEQUENCE):
    result = number(value, name, low, high)
    if result != int(result):
        raise ProtocolError(f'{name}: expected an integer')
    return int(result)


def _text(header, name, choices=None, allow_empty=False):
    value = header.get(name)
    if not isinstance(value, str) or len(value) > 256 or ('\x00' in value):
        raise ProtocolError(f'{name}: invalid string')
    try:
        value.encode('utf-8', errors='strict')
    except UnicodeError as exc:
        raise ProtocolError(f'{name}: invalid Unicode') from exc
    if not value and not allow_empty:
        raise ProtocolError(f'{name}: empty string')
    if choices is not None and value not in choices:
        raise ProtocolError(f'{name}: unsupported value {value!r}')
    return value


def _vector(header, name, count, low, high):
    value = header.get(name)
    if not isinstance(value, list) or len(value) != count:
        raise ProtocolError(f'{name}: expected {count} values')
    return [number(v, name, low, high) for v in value]


def validate_header(header, payload_length, limits=Limits()):
    if not isinstance(header, dict):
        raise ProtocolError('header must be an object')
    if integer(header.get('version'), 'version') != VERSION:
        raise ProtocolError('unsupported protocol version (v2 required)')
    kind = _text(header, 'type', {'frame', 'ack'})
    _text(header, 'session_id')
    integer(header.get('sequence_id'), 'sequence_id')
    if kind == 'ack':
        if payload_length != 0 or header.get('status') != 'accepted':
            raise ProtocolError('invalid ACK')
        return
    height = integer(header.get('height'), 'height', 1, limits.dimension)
    width = integer(header.get('width'), 'width', 1, limits.dimension)
    encoding = _text(header, 'encoding', {'mono8', 'mono16'})
    if height * width * (1 if encoding == 'mono8' else 2) != payload_length:
        raise ProtocolError('payload length does not match image dimensions')
    _text(header, 'storage_order', {'row_major', 'column_major'})
    _text(header, 'byte_order', {'little'})
    _text(header, 'modality', {'bmode'})
    _text(header, 'source')
    _text(header, 'timestamp_kind', {'display_callback', 'acquisition', 'synthetic'})
    _text(header, 'intensity_mapping')
    for name in ('source_time_s', 'send_time_s'):
        number(header.get(name), name)
    number(header.get('sender_queue_age_s'), 'sender_queue_age_s', 0, 60)
    integer(header.get('sender_dropped'), 'sender_dropped')
    if type(header.get('geometry_valid')) is not bool:
        raise ProtocolError('geometry_valid must be boolean')
    _text(header, 'geometry_version', allow_empty=not header['geometry_valid'])
    spacing = _vector(header, 'pixel_spacing_m', 2, 0, 1)
    _vector(header, 'origin_m', 3, -10, 10)
    column = _vector(header, 'column_direction', 3, -1, 1)
    row = _vector(header, 'row_direction', 3, -1, 1)
    if header['geometry_valid']:
        if min(spacing) <= 0:
            raise ProtocolError('valid geometry needs positive pixel spacing')
        if any(abs(sum(v*v for v in axis) - 1) > 1e-5 for axis in (row, column)):
            raise ProtocolError('geometry directions must be unit length')
        if abs(sum(a*b for a, b in zip(row, column))) > 1e-5:
            raise ProtocolError('geometry directions must be orthogonal')
    if 'roi_confirmed' in header and type(header['roi_confirmed']) is not bool:
        raise ProtocolError('roi_confirmed must be boolean')
    for name in ('profile_revision', 'imaging_context'):
        if name in header:
            _text(header, name, allow_empty=True)
    if 'capture_size' in header:
        values = _vector(header, 'capture_size', 2, 1, 32768)
        if any(value != int(value) for value in values):
            raise ProtocolError('capture_size must contain integers')
    if 'roi_pixels' in header:
        values = _vector(header, 'roi_pixels', 4, 0, 32768)
        if any(value != int(value) for value in values):
            raise ProtocolError('roi_pixels must contain integers')
        x, y, roi_width, roi_height = values
        if roi_width != width or roi_height != height:
            raise ProtocolError('roi_pixels dimensions must match image dimensions')
        if 'capture_size' in header:
            capture_width, capture_height = header['capture_size']
            if x + roi_width > capture_width or y + roi_height > capture_height:
                raise ProtocolError('roi_pixels exceeds capture_size')
    if 'display_scale_mm_per_pixel' in header:
        scale = header['display_scale_mm_per_pixel']
        if not isinstance(scale, dict) or set(scale) != {'horizontal', 'vertical'}:
            raise ProtocolError('display_scale_mm_per_pixel requires horizontal and vertical axes')
        for axis, value in scale.items():
            if value is not None and number(value, f'display_scale.{axis}', 0, 1000) <= 0:
                raise ProtocolError('display scale must be positive or null')


def _reject_constant(value):
    raise ProtocolError(f'non-finite JSON constant: {value}')


def _finite_float(value):
    result = float(value)
    if not math.isfinite(result):
        raise ProtocolError('JSON number exceeds finite float range')
    return result


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ProtocolError(f'duplicate JSON key: {key}')
        result[key] = value
    return result


def encode_packet(header, payload=b'', limits=Limits()):
    payload = bytes(payload)
    if len(payload) > limits.payload_bytes:
        raise ProtocolError('payload exceeds limit')
    validate_header(header, len(payload), limits)
    raw = json.dumps(header, allow_nan=False, separators=(',', ':')).encode('utf-8')
    if len(raw) > limits.header_bytes:
        raise ProtocolError('header exceeds limit')
    return PREFIX.pack(MAGIC, len(raw), len(payload)) + raw + payload


def _read_exact(sock, length, deadline, stop_event=None):
    """Keep partial bytes on socket timeout; a total deadline ends the packet."""
    data = bytearray()
    while len(data) < length:
        if stop_event is not None and stop_event.is_set():
            raise EOFError('stopping')
        if time.monotonic() >= deadline:
            raise TimeoutError('packet deadline exceeded')
        try:
            chunk = sock.recv(min(length - len(data), 65536))
        except socket.timeout:
            continue
        if not chunk:
            raise EOFError('peer closed during packet')
        data.extend(chunk)
    return bytes(data)


def read_packet(sock, timeout_s=1.0, limits=Limits(), stop_event=None):
    """Socket must have a short timeout; all reads share one monotonic deadline."""
    deadline = time.monotonic() + timeout_s
    magic, header_len, payload_len = PREFIX.unpack(
        _read_exact(sock, PREFIX.size, deadline, stop_event))
    if magic != MAGIC:
        raise ProtocolError('invalid magic')
    if not 0 < header_len <= limits.header_bytes:
        raise ProtocolError('invalid header length')
    if payload_len > limits.payload_bytes:
        raise ProtocolError('payload exceeds limit')
    raw = _read_exact(sock, header_len, deadline, stop_event)
    try:
        header = json.loads(raw.decode('utf-8'), parse_constant=_reject_constant, parse_float=_finite_float,
                            object_pairs_hook=_unique_object)
    except (UnicodeError, ValueError, RecursionError) as exc:
        raise ProtocolError(f'invalid JSON: {exc}') from exc
    # Validate BEFORE allocating/reading the pixel payload.
    validate_header(header, payload_len, limits)
    return Packet(header, _read_exact(sock, payload_len, deadline, stop_event))


def acknowledgement(header):
    return encode_packet({
        'version': VERSION, 'type': 'ack', 'status': 'accepted',
        'session_id': header['session_id'], 'sequence_id': header['sequence_id'],
    })


def frame_header(sequence_id, source_time_s, height, width, **overrides):
    """Uncalibrated geometry by default. No per-frame intensity normalization."""
    header = dict(
        version=VERSION, type='frame', session_id='unset', sequence_id=sequence_id,
        height=height, width=width, encoding='mono8', storage_order='row_major',
        byte_order='little', modality='bmode', source='synthetic',
        timestamp_kind='synthetic', source_time_s=source_time_s,
        send_time_s=source_time_s, sender_queue_age_s=0.0, sender_dropped=0,
        geometry_valid=False, geometry_version='', pixel_spacing_m=[0.0, 0.0],
        origin_m=[0.0, 0.0, 0.0], column_direction=[1.0, 0.0, 0.0],
        row_direction=[0.0, 1.0, 0.0], intensity_mapping='identity',
    )
    header.update(overrides)
    return header
