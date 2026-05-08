from __future__ import annotations

# 导入 json，用于序列化桥接头。
import json
# 导入 socket，用于从 TCP 连接中读取数据。
import socket
# 导入 struct，用于打包和解包固定长度头部。
import struct
import zlib
from typing import Any


# 定义 4 字节无符号整数的网络字节序格式。
HEADER_LENGTH_FORMAT = "!I"
# 定义最大 payload 大小，避免异常数据无限占内存。
MAX_PAYLOAD_SIZE = 64 * 1024 * 1024


# 打包桥接层消息。
def pack_frame_packet(header: dict[str, Any], payload: bytes) -> bytes:
    # 复制一份头信息，避免修改调用方对象。
    header_copy = dict(header)
    # 写入当前 payload 的字节数。
    header_copy["payload_size"] = len(payload)
    # 将头信息序列化为 UTF-8 JSON。
    header_bytes = json.dumps(header_copy, ensure_ascii=False).encode("utf-8")
    # 先打包头长度，再拼接头和二进制 payload。
    return struct.pack(HEADER_LENGTH_FORMAT, len(header_bytes)) + header_bytes + payload


# 从 TCP 流中精确读取指定字节数。
def recv_exact(sock: socket.socket, size: int) -> bytes:
    # 初始化缓冲区。
    chunks: list[bytes] = []
    # 记录还需读取的字节数。
    remaining = size
    # 循环读取直到满足长度要求。
    while remaining > 0:
        chunk = sock.recv(remaining)
        # 若连接断开，则抛出错误。
        if not chunk:
            raise ConnectionError("Socket closed while receiving data.")
        chunks.append(chunk)
        remaining -= len(chunk)
    # 返回完整字节串。
    return b"".join(chunks)


# 从 TCP 连接中读取一条完整桥接消息。
def recv_frame_packet(sock: socket.socket) -> tuple[dict[str, Any], bytes]:
    # 先读取固定长度的头长度字段。
    header_length_bytes = recv_exact(sock, struct.calcsize(HEADER_LENGTH_FORMAT))
    # 解包得到头部 JSON 长度。
    header_length = struct.unpack(HEADER_LENGTH_FORMAT, header_length_bytes)[0]
    # 再读取 JSON 头。
    header_bytes = recv_exact(sock, header_length)
    # 解析头信息。
    header = json.loads(header_bytes.decode("utf-8"))
    # 获取 payload 长度。
    payload_size = int(header.get("payload_size", 0))
    # 对 payload 长度做简单安全检查。
    if payload_size < 0 or payload_size > MAX_PAYLOAD_SIZE:
        raise ValueError(f"Invalid payload size: {payload_size}")
    # 再读取 payload。
    payload = recv_exact(sock, payload_size)
    # 返回头和负载。
    return header, payload


# 压缩桥接层负载。
def compress_payload(payload: bytes, level: int = 6) -> bytes:
    return zlib.compress(payload, int(level))


# 解压桥接层负载。
def decompress_payload(payload: bytes) -> bytes:
    return zlib.decompress(payload)
