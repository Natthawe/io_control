#!/usr/bin/env python3
import socket, struct
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Int32, Float32, Int32MultiArray, Bool
from gmr_msgs.msg import ControllerStatus

def _parse_reliability(s: str) -> ReliabilityPolicy:
    return ReliabilityPolicy.RELIABLE if (s or "").lower() == "reliable" else ReliabilityPolicy.BEST_EFFORT

def _parse_durability(s: str) -> DurabilityPolicy:
    return DurabilityPolicy.TRANSIENT_LOCAL if (s or "").lower() == "transient_local" else DurabilityPolicy.VOLATILE

class IOUDPNode(Node):
    def __init__(self):
        super().__init__('io_control')

        # ===== Params =====
        self.declare_parameter('udp_ip', '10.1.100.200')
        self.declare_parameter('udp_port', 8001)
        self.declare_parameter('udp_listen_port', 8002)

        self.declare_parameter('topic_stop', '/stopRobotX')
        self.declare_parameter('topic_status', '/gmr_controllers/status')
        self.declare_parameter('topic_distance', '/closest_object_distance')
        self.declare_parameter('topic_inputs', '/io/inputs')
        self.declare_parameter('topic_bypass', '/io/bypass')       # Int32: 3 / -3

        # ----- outputs -----
        self.declare_parameter('topic_outputs', '/io/outputs')            # Int32MultiArray [O1..O8]
        self.declare_parameter('topic_outputs_mask', '/io/outputs_mask')  # Int32 (mask)

        self.declare_parameter('distance_threshold', 1.0)
        self.declare_parameter('alert_value', -1)
        self.declare_parameter('clear_value', 0)
        self.declare_parameter('distance_send_clear_on_edge_up', True)

        self.declare_parameter('suppress_alert_when_emergency_active', True)
        self.declare_parameter('rearm_delay_sec', 2.0)

        self.declare_parameter('qos_depth', 10)
        self.declare_parameter('qos_reliability', 'reliable')
        self.declare_parameter('qos_durability', 'volatile')

        self.declare_parameter('dedupe_same_value', True)
        self.declare_parameter('require_status_before_distance', True)

        # Debug state topics
        self.declare_parameter('publish_debug_bools', True)

        # ===== Read params =====
        self.udp_ip            = self.get_parameter('udp_ip').get_parameter_value().string_value
        self.udp_port          = int(self.get_parameter('udp_port').get_parameter_value().integer_value)
        self.udp_listen_port   = int(self.get_parameter('udp_listen_port').get_parameter_value().integer_value)

        self.topic_stop        = self.get_parameter('topic_stop').get_parameter_value().string_value
        self.topic_status      = self.get_parameter('topic_status').get_parameter_value().string_value
        self.topic_distance    = self.get_parameter('topic_distance').get_parameter_value().string_value
        self.topic_inputs      = self.get_parameter('topic_inputs').get_parameter_value().string_value
        self.topic_bypass      = self.get_parameter('topic_bypass').get_parameter_value().string_value

        # ----- outputs -----
        self.topic_outputs_arr = self.get_parameter('topic_outputs').get_parameter_value().string_value
        self.topic_outputs_mask= self.get_parameter('topic_outputs_mask').get_parameter_value().string_value

        self.distance_threshold = float(self.get_parameter('distance_threshold').get_parameter_value().double_value)
        self.alert_value        = int(self.get_parameter('alert_value').get_parameter_value().integer_value)
        self.clear_value        = int(self.get_parameter('clear_value').get_parameter_value().integer_value)
        self.distance_send_clear_on_edge_up = bool(self.get_parameter('distance_send_clear_on_edge_up').get_parameter_value().bool_value)

        self.suppress_alert_when_emergency_active = bool(self.get_parameter('suppress_alert_when_emergency_active').get_parameter_value().bool_value)
        self.rearm_delay_sec = float(self.get_parameter('rearm_delay_sec').get_parameter_value().double_value)

        qos_depth       = int(self.get_parameter('qos_depth').get_parameter_value().integer_value)
        qos_reliability = _parse_reliability(self.get_parameter('qos_reliability').get_parameter_value().string_value)
        qos_durability  = _parse_durability(self.get_parameter('qos_durability').get_parameter_value().string_value)
        self.qos = QoSProfile(depth=qos_depth, reliability=qos_reliability, durability=qos_durability)

        self.dedupe_same_value = bool(self.get_parameter('dedupe_same_value').get_parameter_value().bool_value)
        self.require_status_before_distance = bool(self.get_parameter('require_status_before_distance').get_parameter_value().bool_value)
        self.publish_debug_bools = bool(self.get_parameter('publish_debug_bools').get_parameter_value().bool_value)

        # ===== UDP sockets =====
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx_sock.setblocking(False)
        self.rx_sock.bind(('0.0.0.0', self.udp_listen_port))

        # ===== Subscriptions =====
        self.create_subscription(Int32,   self.topic_stop,     self.stop_callback,     self.qos)
        self.create_subscription(ControllerStatus, self.topic_status,   self.status_callback,   self.qos)
        self.create_subscription(Float32, self.topic_distance, self.distance_callback, self.qos)
        self.create_subscription(Int32,   self.topic_bypass,   self.bypass_callback_int32, 10)

        # ===== Publishers =====
        self.pub_inputs       = self.create_publisher(Int32MultiArray, self.topic_inputs, self.qos)
        self.pub_outputs_arr  = self.create_publisher(Int32MultiArray, self.topic_outputs_arr, self.qos)
        self.pub_outputs_mask = self.create_publisher(Int32,           self.topic_outputs_mask, self.qos)

        if self.publish_debug_bools:
            self.pub_em     = self.create_publisher(Bool, '/io/emergency_active', 10)
            self.pub_bp     = self.create_publisher(Bool, '/io/bumper_active', 10)
            # self.pub_ack    = self.create_publisher(Bool, '/io/ack_needed', 10)
            # self.pub_armed  = self.create_publisher(Bool, '/io/ack_armed', 10)
        else:
            self.pub_em = self.pub_bp = self.pub_ack = self.pub_armed = None

        # ===== States =====
        self._last_emergency_val = None
        self._emergency_state = None
        self._status_initialized = False
        self._last_sent_val = None
        self._distance_under_threshold = False
        self._delay_timer = None

        self._last_inputs_tuple = None
        self._last_mask = 0
        self._last_outputs_mask = None
        self._last_outputs_tuple = None

        self._ack_needed = False
        self._ack_armed  = False

        self._rx_timer = self.create_timer(0.01, self._poll_udp_rx)

        self.get_logger().info(
            f'Node ready. TX->{self.udp_ip}:{self.udp_port}, RX<-:{self.udp_listen_port} | '
            f'th={self.distance_threshold}, alert={self.alert_value}, clear={self.clear_value}'
        )

    # ===== UDP send helper =====
    def _send_udp_int32(self, val: int):
        ival = int(val)
        if self.dedupe_same_value and self._last_sent_val == ival:
            return
        try:
            payload = struct.pack('<i', ival)
            self.sock.sendto(payload, (self.udp_ip, self.udp_port))
            self._last_sent_val = ival
            self.get_logger().info(f'Sent Int32 {ival} -> {self.udp_ip}:{self.udp_port}')
        except Exception as e:
            self.get_logger().error(f'UDP send failed: {e}')
        if ival == 1:
            self._cancel_delay_timer()

    # ===== Helpers: bit mask -> array [O1..O8] =====
    @staticmethod
    def _mask_to_outputs_array(mask: int):
        mask &= 0xFF
        # bit0=O1 ... bit7=O8  (ตรงกับฝั่ง ESP32)
        return [ (mask >> i) & 1 for i in range(8) ]  # [O1..O8]

    # ===== Publishers helpers =====
    def _publish_inputs_array(self, mask: int):
        mask &= 0xFF
        em = bool(mask & (1 << 0))
        bp = bool(mask & (1 << 1))

        arr = [ int(em), int(bp), (mask>>2)&1, (mask>>3)&1 ]
        t = tuple(arr)
        if self._last_inputs_tuple != t:
            msg = Int32MultiArray(); msg.data = arr
            self.pub_inputs.publish(msg)
            self._last_inputs_tuple = t
            self.get_logger().info(f'/io/inputs: {arr} (mask=0b{mask:08b})')

        # mirror ACK state
        last_em = bool(self._last_mask & 1)
        last_bp = bool(self._last_mask & 2)

        if bp and not last_bp:
            self._ack_needed = True
            self._ack_armed  = False
            self.get_logger().info('ACK required (bumper triggered)')

        if self._ack_needed:
            if em and not last_em:
                self._ack_armed = True
                self.get_logger().info('ACK armed (EMER pressed)')
            if (not em) and last_em and self._ack_armed:
                self._ack_needed = False
                self.get_logger().info('ACK complete (EMER released)')

        self._last_mask = mask

        if self.publish_debug_bools:
            try:
                self.pub_em.publish(Bool(data=em))
                self.pub_bp.publish(Bool(data=bp))
                self.pub_ack.publish(Bool(data=self._ack_needed))
                self.pub_armed.publish(Bool(data=self._ack_armed))
            except Exception:
                pass

    def _publish_outputs(self, mask: int):
        # ----- Int32 (mask) -----
        mask &= 0xFF
        if self._last_outputs_mask != mask:
            msg_m = Int32(); msg_m.data = int(mask)
            self.pub_outputs_mask.publish(msg_m)
            self._last_outputs_mask = mask
            self.get_logger().info(f'/io/outputs_mask: {mask} (0b{mask:08b})')

        # ----- Int32MultiArray [O1..O8] -----
        arr = self._mask_to_outputs_array(mask)  # [O1..O8]
        t = tuple(arr)
        if self._last_outputs_tuple != t:
            msg_a = Int32MultiArray(); msg_a.data = arr
            self.pub_outputs_arr.publish(msg_a)
            self._last_outputs_tuple = t
            self.get_logger().info(f'/io/outputs: {arr}')

    # ===== UDP RX =====
    def _poll_udp_rx(self):
        while True:
            try:
                data, addr = self.rx_sock.recvfrom(128)
            except BlockingIOError:
                break
            except Exception as e:
                self.get_logger().warn(f'UDP RX error: {e}')
                break

            if len(data) == 4:
                # บอร์ดส่ง binary int32 เฉพาะกรณี IN (เวอร์ชันนี้ OUT ส่งเป็น ASCII)
                (val,) = struct.unpack('<i', data)
                self._publish_inputs_array(val)
                continue

            try:
                s = data.decode(errors='ignore').strip()
            except Exception:
                continue

            s_up = s.upper()
            if s_up.startswith('IN'):
                # "IN:<mask>"
                s2 = s.split(':',1)[1].strip() if ':' in s else s
                try: self._publish_inputs_array(int(s2, 0))
                except Exception: pass
                continue

            if s_up.startswith('OUT'):
                # "OUT:<mask>"
                s2 = s.split(':',1)[1].strip() if ':' in s else s
                try: self._publish_outputs(int(s2, 0))             # NEW/CHANGED
                except Exception: pass
                continue

            # รองรับ "<mask>" โดดๆ สำหรับ IN
            try:
                self._publish_inputs_array(int(s, 0))
            except Exception:
                pass

    # ===== Misc helpers =====
    def _emergency_active(self) -> bool:
        return bool(self._emergency_state) is True

    def _cancel_delay_timer(self):
        if self._delay_timer is not None:
            try: self._delay_timer.cancel()
            except Exception: pass
            self._delay_timer = None

    def _schedule_delayed_alert(self):
        self._cancel_delay_timer()
        self._delay_timer = self.create_timer(self.rearm_delay_sec, self._delayed_alert_cb)

    def _delayed_alert_cb(self):
        self._cancel_delay_timer()
        if self._distance_under_threshold and not self._emergency_active():
            self._send_udp_int32(self.alert_value)

    # ===== Callbacks =====
    def stop_callback(self, msg: Int32):
        ival = int(msg.data)
        if ival == 1:
            self._emergency_state = True; self._status_initialized = True
        elif ival == 0:
            self._emergency_state = False; self._status_initialized = True
        self._send_udp_int32(ival)
        if self.publish_debug_bools and self.pub_em:
            try: self.pub_em.publish(Bool(data=(ival == 1)))
            except Exception: pass
        if ival == 0 and self._distance_under_threshold:
            self._schedule_delayed_alert()

    def status_callback(self, msg: ControllerStatus):
        val = 1 if msg.emergency_status else 0
        self._emergency_state = (val == 1); self._status_initialized = True
        if not hasattr(self, '_last_emergency_val'):
            self._last_emergency_val = None
        if self._last_emergency_val is None:
            self._last_emergency_val = val
            if self.publish_debug_bools and self.pub_em:
                try: self.pub_em.publish(Bool(data=(val == 1)))
                except Exception: pass
            return
        if val != self._last_emergency_val:
            self._last_emergency_val = val
            self._send_udp_int32(val)
            if self.publish_debug_bools and self.pub_em:
                try: self.pub_em.publish(Bool(data=(val == 1)))
                except Exception: pass
            if val == 0 and self._distance_under_threshold:
                self._schedule_delayed_alert()

    def distance_callback(self, msg: Float32):
        if self.require_status_before_distance and not self._status_initialized:
            self._distance_under_threshold = (msg.data < self.distance_threshold)
            return
        under = (msg.data < self.distance_threshold)
        if under and not self._distance_under_threshold:
            self._distance_under_threshold = True
            if self.suppress_alert_when_emergency_active and self._emergency_active():
                pass
            else:
                self._send_udp_int32(self.alert_value)  # -> -1
            return
        if (not under) and self._distance_under_threshold:
            self._distance_under_threshold = False
            self._cancel_delay_timer()
            if self.distance_send_clear_on_edge_up and not self._emergency_active():
                self._send_udp_int32(self.clear_value)  # -> 0
            return

    # ===== BYPASS: Int32 (expect 3 or -3) =====
    def bypass_callback_int32(self, msg: Int32):
        v = int(msg.data)
        if v == 3 or v == -3:
            self._send_udp_int32(v)
        else:
            # ป้องกันค่าหลุด: เตือนและไม่ส่ง
            self.get_logger().warn(f'/io/bypass expects 3 or -3, got {v}; ignored.')

def main(args=None):
    rclpy.init(args=args)
    node = IOUDPNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
