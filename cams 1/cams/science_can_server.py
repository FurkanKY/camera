#!/usr/bin/env python3

import argparse
import json
import math
import threading
import time
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Dict, Optional

import can


SENSOR_IDS = (
    "o2",
    "co2",
    "co",
    "nh3",
    "press",
    "hum",
    "atemp",
    "stemp",
    "smois",
    "ec",
    "salinity",
)


class SensorStore:
    def __init__(self, points: int = 25, sample_period: float = 30.0):
        self._lock = threading.Lock()

        # Grafiğe kaydedilmiş 30 saniyelik örnekler.
        # Sınırsız grafik geçmişi. Her 30 saniyelik örnek saklanır.
        self._data: Dict[str, list] = {
            sensor_id: []
            for sensor_id in SENSOR_IDS
        }

        # CAN'den en son alınan anlık değerler.
        self._latest: Dict[str, Optional[float]] = {
            sensor_id: None
            for sensor_id in SENSOR_IDS
        }

        self._last_update: Dict[str, Optional[float]] = {
            sensor_id: None
            for sensor_id in SENSOR_IDS
        }

        self._received_frames = 0
        self._saved_samples = 0
        self._sample_period = float(sample_period)
        self._last_sample_time = time.monotonic()

    def update(self, values: Dict[str, float]) -> None:
        """
        Her CAN frame'inde yalnızca son değerleri günceller.
        Grafik geçmişine burada veri eklenmez.
        """
        now = time.time()

        with self._lock:
            updated = False

            for sensor_id, value in values.items():
                if sensor_id not in self._latest:
                    continue

                numeric_value = float(value)

                if not math.isfinite(numeric_value):
                    continue

                self._latest[sensor_id] = round(numeric_value, 3)
                self._last_update[sensor_id] = now
                updated = True

            if updated:
                self._received_frames += 1

    def save_sample_if_due(self) -> Optional[Dict[str, float]]:
        """
        Her sample_period saniyede bir, CAN'den alınmış en son değerleri
        grafik geçmişine kaydeder.
        """
        now_monotonic = time.monotonic()

        with self._lock:
            elapsed = now_monotonic - self._last_sample_time

            if elapsed < self._sample_period:
                return None

            # Periyodu kaydırmadan ilerlet.
            periods = max(1, int(elapsed // self._sample_period))
            self._last_sample_time += periods * self._sample_period

            saved = {}

            for sensor_id, value in self._latest.items():
                if value is None:
                    continue

                self._data[sensor_id].append(value)
                saved[sensor_id] = value

            if saved:
                self._saved_samples += 1
                return saved

            return None

    def sensor_snapshot(self) -> dict:
        with self._lock:
            return {
                sensor_id: list(values)
                for sensor_id, values in self._data.items()
            }

    def status_snapshot(self) -> dict:
        with self._lock:
            return {
                "latest": dict(self._latest),
                "last_update": dict(self._last_update),
                "received_frames": self._received_frames,
                "saved_samples": self._saved_samples,
                "sample_period_seconds": self._sample_period,
                "server_time": time.time(),
            }


def unsigned_u16_be(data, index: int) -> int:
    if len(data) < index + 2:
        raise ValueError(
            f"Packet too short: len={len(data)}, index={index}"
        )

    return (int(data[index]) << 8) | int(data[index + 1])


def signed_i16_be(data, index: int) -> int:
    raw = unsigned_u16_be(data, index)
    return raw - 0x10000 if raw & 0x8000 else raw


def decode_sensor_packet(data) -> Optional[Dict[str, float]]:
    if len(data) < 7:
        return None

    header = int(data[0])

    if header == 0x30:
        return {
            "co2": float(signed_i16_be(data, 1)),
            "nh3": float(unsigned_u16_be(data, 3)),
            "o2": signed_i16_be(data, 5) / 10.0,
        }

    if header == 0x31:
        return {
            "atemp": signed_i16_be(data, 1) / 10.0,
            "press": signed_i16_be(data, 3) / 10.0,
            "hum": signed_i16_be(data, 5) / 10.0,
        }

    if header == 0x32:
        return {
            "co": signed_i16_be(data, 1) / 10.0,
            "stemp": signed_i16_be(data, 3) / 10.0,
            "smois": signed_i16_be(data, 5) / 10.0,
        }

    if header == 0x33:
        return {
            "ec": float(unsigned_u16_be(data, 1)),
            "salinity": float(unsigned_u16_be(data, 3)),
        }

    return None


class GraphSampler(threading.Thread):
    def __init__(self, store: SensorStore):
        super().__init__(daemon=True)
        self.store = store
        self.stop_event = threading.Event()

    def run(self) -> None:
        while not self.stop_event.is_set():
            saved = self.store.save_sample_if_due()

            if saved:
                formatted = " | ".join(
                    f"{key}={value}"
                    for key, value in saved.items()
                )
                print(f"[GRAPH SAMPLE 30s] {formatted}")

            self.stop_event.wait(0.1)

    def stop(self) -> None:
        self.stop_event.set()


class CanReader(threading.Thread):
    def __init__(
        self,
        store: SensorStore,
        channel: str,
        bitrate: int,
        can_id: int,
        print_raw: bool,
        print_decoded: bool,
    ):
        super().__init__(daemon=True)

        self.store = store
        self.channel = channel
        self.bitrate = bitrate
        self.can_id = can_id
        self.print_raw = print_raw
        self.print_decoded = print_decoded

        self.stop_event = threading.Event()
        self.bus = None

    def run(self) -> None:
        try:
            self.bus = can.interface.Bus(
                channel=self.channel,
                interface="socketcan",
            )

            self.bus.set_filters([
                {
                    "can_id": self.can_id,
                    "can_mask": 0x7FF,
                    "extended": False,
                }
            ])

            print(
                f"[CAN] Listening on {self.channel}, "
                f"configured bitrate={self.bitrate}, "
                f"CAN ID=0x{self.can_id:03X}"
            )

        except Exception as exc:
            print(f"[CAN ERROR] Bus could not be opened: {exc}")
            return

        while not self.stop_event.is_set():
            try:
                msg = self.bus.recv(timeout=0.5)

                if msg is None:
                    continue

                if msg.is_extended_id:
                    continue

                if msg.arbitration_id != self.can_id:
                    continue

                data = bytes(msg.data)

                if self.print_raw:
                    hex_data = " ".join(f"{byte:02X}" for byte in data)
                    print(
                        f"[CAN RAW] "
                        f"ID=0x{msg.arbitration_id:03X} "
                        f"DLC={msg.dlc} "
                        f"DATA={hex_data}"
                    )

                values = decode_sensor_packet(data)

                if values is None:
                    header_text = (
                        f"0x{data[0]:02X}"
                        if len(data) > 0
                        else "none"
                    )
                    print(
                        f"[CAN IGNORE] Unknown or short packet, "
                        f"header={header_text}, len={len(data)}"
                    )
                    continue

                self.store.update(values)

                if self.print_decoded:
                    formatted = " | ".join(
                        f"{key}={value}"
                        for key, value in values.items()
                    )
                    print(
                        f"[CAN DECODED 0x{data[0]:02X}] {formatted}"
                    )

            except can.CanError as exc:
                print(f"[CAN ERROR] {exc}")
                time.sleep(0.2)

            except Exception as exc:
                print(f"[DECODE ERROR] {exc}")

    def stop(self) -> None:
        self.stop_event.set()

        if self.bus is not None:
            try:
                self.bus.shutdown()
            except Exception:
                pass


class ScienceRequestHandler(SimpleHTTPRequestHandler):
    store: SensorStore = None
    html_file = "science_interface_original_can.html"

    def do_GET(self):
        request_path = self.path.split("?", 1)[0]

        if request_path == "/api/sensors":
            # HTML ile uyumlu doğrudan sensör sözlüğü.
            response_data = self.store.sensor_snapshot()

            payload = json.dumps(
                response_data,
                ensure_ascii=False,
                separators=(",", ":"),
            ).encode("utf-8")

            self.send_response(200)
            self.send_header(
                "Content-Type",
                "application/json; charset=utf-8",
            )
            self.send_header(
                "Cache-Control",
                "no-store, no-cache, must-revalidate",
            )
            self.send_header(
                "Content-Length",
                str(len(payload)),
            )
            self.end_headers()
            self.wfile.write(payload)
            return

        if request_path == "/api/status":
            response_data = self.store.status_snapshot()

            payload = json.dumps(
                response_data,
                ensure_ascii=False,
                separators=(",", ":"),
            ).encode("utf-8")

            self.send_response(200)
            self.send_header(
                "Content-Type",
                "application/json; charset=utf-8",
            )
            self.send_header(
                "Cache-Control",
                "no-store, no-cache, must-revalidate",
            )
            self.send_header(
                "Content-Length",
                str(len(payload)),
            )
            self.end_headers()
            self.wfile.write(payload)
            return

        if request_path in ("/", ""):
            self.path = "/" + self.html_file

        super().do_GET()

    def log_message(self, fmt, *args):
        # 500 ms API sorguları terminali doldurmasın.
        if not self.path.startswith("/api/"):
            super().log_message(fmt, *args)


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Read atmospheric sensors from SocketCAN "
            "and serve the science interface."
        )
    )

    parser.add_argument(
        "--channel",
        default="can2",
    )
    parser.add_argument(
        "--bitrate",
        type=int,
        default=1_000_000,
    )
    parser.add_argument(
        "--can-id",
        type=lambda value: int(value, 0),
        default=0x000,
    )
    parser.add_argument(
        "--host",
        default="0.0.0.0",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=8000,
    )
    parser.add_argument(
        "--points",
        type=int,
        default=25,
    )
    parser.add_argument(
        "--sample-period",
        type=float,
        default=30.0,
        help="Seconds between graph samples. Default: 30 seconds.",
    )
    parser.add_argument(
        "--html-file",
        default="science_interface_original_can.html",
    )
    parser.add_argument(
        "--directory",
        default=str(Path(__file__).resolve().parent),
    )
    parser.add_argument(
        "--no-raw-log",
        action="store_true",
        help="Do not print raw CAN frames.",
    )
    parser.add_argument(
        "--no-decoded-log",
        action="store_true",
        help="Do not print decoded sensor values.",
    )

    return parser.parse_args()


def main():
    args = parse_args()

    directory = Path(args.directory).resolve()
    ScienceRequestHandler.html_file = args.html_file

    html_path = directory / ScienceRequestHandler.html_file

    if not html_path.exists():
        raise FileNotFoundError(
            f"Interface file not found: {html_path}"
        )

    store = SensorStore(
        points=args.points,
        sample_period=args.sample_period,
    )
    ScienceRequestHandler.store = store

    def handler(*handler_args, **handler_kwargs):
        return ScienceRequestHandler(
            *handler_args,
            directory=str(directory),
            **handler_kwargs,
        )

    reader = CanReader(
        store=store,
        channel=args.channel,
        bitrate=args.bitrate,
        can_id=args.can_id,
        print_raw=not args.no_raw_log,
        print_decoded=not args.no_decoded_log,
    )
    reader.start()

    sampler = GraphSampler(store)
    sampler.start()

    server = ThreadingHTTPServer(
        (args.host, args.port),
        handler,
    )

    print(f"[WEB] Local:   http://127.0.0.1:{args.port}")
    print(f"[WEB] Network: http://ROVER_IP:{args.port}")
    print(f"[WEB] Data:    http://127.0.0.1:{args.port}/api/sensors")
    print(f"[WEB] Status:  http://127.0.0.1:{args.port}/api/status")

    try:
        server.serve_forever()

    except KeyboardInterrupt:
        print("\nStopping...")

    finally:
        sampler.stop()
        reader.stop()
        server.server_close()


if __name__ == "__main__":
    main()