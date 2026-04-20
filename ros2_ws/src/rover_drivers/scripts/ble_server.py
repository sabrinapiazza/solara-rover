import asyncio
import json
import signal
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from bless import (
    BlessServer,
    GATTAttributePermissions,
    GATTCharacteristicProperties,
)
from bleak.uuids import normalize_uuid_str

SERVICE_UUID = "12345678-1234-5678-1234-56789abcdef0"
COMMAND_CHAR_UUID = "12345678-1234-5678-1234-56789abcdef1"
STATUS_CHAR_UUID = "12345678-1234-5678-1234-56789abcdef2"


class BLEBridgeNode(Node):
    def __init__(self):
        super().__init__('ble_bridge')

        self._server = None
        self._loop = None

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.get_logger().info('BLE bridge node started')

    def cmd_vel_callback(self, msg):
        if self._server is None or self._loop is None:
            return
        payload = json.dumps({
            'linear_x': msg.linear.x,
            'angular_z': msg.angular.z,
        }).encode()
        asyncio.run_coroutine_threadsafe(
            self._async_push_status(bytearray(payload)),
            self._loop,
        )

    async def _async_push_status(self, data):
        if self._server is None:
            return
        char = self._server.get_characteristic(STATUS_CHAR_UUID)
        if char is None:
            return
        char.value = data
        self._server.update_value(SERVICE_UUID, STATUS_CHAR_UUID)

    def on_read(self, characteristic, request):
        if characteristic.value is None:
            return bytearray(b'{}')
        return bytearray(characteristic.value)

    def on_write(self, characteristic, value, request):
        norm = normalize_uuid_str(str(characteristic.uuid))
        if norm != normalize_uuid_str(COMMAND_CHAR_UUID):
            return
        characteristic.value = bytearray(value)
        self.get_logger().info(f'BLE command write: {bytes(value)!r}')


async def _run_ble_server(node):
    loop = asyncio.get_running_loop()
    initial_status = bytearray(
        json.dumps({'linear_x': 0.0, 'angular_z': 0.0}).encode()
    )
    gatt = {
        SERVICE_UUID: {
            COMMAND_CHAR_UUID: {
                'Properties': GATTCharacteristicProperties.write,
                'Permissions': GATTAttributePermissions.writeable,
                'Value': None,
            },
            STATUS_CHAR_UUID: {
                'Properties': (
                    GATTCharacteristicProperties.read
                    | GATTCharacteristicProperties.notify
                ),
                'Permissions': GATTAttributePermissions.readable,
                'Value': initial_status,
            },
        },
    }
    server = BlessServer(
        name='SOLARA Rover',
        loop=loop,
        on_read=node.on_read,
        on_write=node.on_write,
    )
    try:
        await server.add_gatt(gatt)
        await server.start()
    except Exception as e:
        msg = str(e)
        node.get_logger().error(f'BLE server startup failed: {msg}')
        if 'Could not locate bluetooth adapter' in msg:
            node.get_logger().error(
                'Bluetooth adapter not found: ensure a BLE adapter is present, '
                'powered on, and managed by BlueZ (e.g. hci0).'
            )
        raise
    node._server = server
    node._loop = loop
    stop = asyncio.Event()

    def request_stop():
        stop.set()

    try:
        loop.add_signal_handler(signal.SIGINT, request_stop)
        loop.add_signal_handler(signal.SIGTERM, request_stop)
    except NotImplementedError:
        pass

    node.get_logger().info('BLE GATT server advertising as SOLARA Rover')
    try:
        await stop.wait()
    finally:
        try:
            loop.remove_signal_handler(signal.SIGINT)
        except (NotImplementedError, OSError, ValueError):
            pass
        try:
            loop.remove_signal_handler(signal.SIGTERM)
        except (NotImplementedError, OSError, ValueError):
            pass
        try:
            await server.stop()
        except Exception as e:
            node.get_logger().error(f'BLE server stop error: {e}')
        node._server = None
        node._loop = None


def main(args=None):
    rclpy.init(args=args)
    node = BLEBridgeNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    try:
        loop.run_until_complete(_run_ble_server(node))
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted')
    finally:
        loop.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

# ─── Flutter ble_constants.dart values ───────────────────────────────────────
# kRoverServiceUUID         = "12345678-1234-5678-1234-56789abcdef0"
# kCommandCharacteristicUUID = "12345678-1234-5678-1234-56789abcdef1"
# kStatusCharacteristicUUID  = "12345678-1234-5678-1234-56789abcdef2"
# kUseMockBLE               = false
# Device name filter        = "SOLARA Rover"
# ─────────────────────────────────────────────────────────────────────────────
