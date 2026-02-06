#!/usr/bin/env python3
import socket
import re

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, UInt8
from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker

# Espera: found,cx,cy,shape  (ej: 1,23,40,2)
CSV_RE = re.compile(r'^\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)(?:\s*,\s*(\d+))?\s*$')

# Tamaño del frame del código Nootropic
W_FRAME = 128.0  # x: 0..127
H_FRAME = 96.0   # y: 0..95

class UdpReceiver(Node):
    def __init__(self):
        super().__init__('udp_receiver')

        # Parámetros
        self.declare_parameter('listen_ip', '0.0.0.0')
        self.declare_parameter('port', 5005)
        self.declare_parameter('frame_id', 'camera')

        listen_ip = self.get_parameter('listen_ip').value
        port = int(self.get_parameter('port').value)
        self.frame_id = self.get_parameter('frame_id').value

        # Publishers
        self.pub_found  = self.create_publisher(Bool, 'vision/found', 10)
        self.pub_point  = self.create_publisher(PointStamped, 'vision/point_px', 10)
        self.pub_shape  = self.create_publisher(UInt8, 'vision/shape', 10)
        self.pub_marker = self.create_publisher(Marker, 'vision/marker', 10)
        self.pub_frame  = self.create_publisher(Marker, 'vision/frame', 1)

        # Socket UDP
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((listen_ip, port))
        self.sock.setblocking(False)

        self.get_logger().info(f'Escuchando UDP en {listen_ip}:{port}')
        self.get_logger().info('Formato: found,cx,cy,shape  (ej: 1,23,40,2)')
        self.get_logger().info('Publicando: /vision/found  /vision/point_px  /vision/shape  /vision/marker  /vision/frame')

        # Para publicar el marco solo una vez
        self.frame_sent = False

        # Revisar el socket 100 veces por segundo
        self.timer = self.create_timer(0.01, self.tick)

    def publish_frame_once(self, now):
        """Publica un rectángulo 128x96 como marco de referencia."""
        frame = Marker()
        frame.header.frame_id = self.frame_id
        frame.header.stamp = now
        frame.ns = "vision"
        frame.id = 1
        frame.type = Marker.LINE_STRIP
        frame.action = Marker.ADD

        # Grosor de la línea
        frame.scale.x = 1.0

        # Color (blanco)
        frame.color.a = 1.0
        frame.color.r = 1.0
        frame.color.g = 1.0
        frame.color.b = 1.0

        # El marco estará en el MISMO sistema que usamos para RViz:
        # x = cx
        # y = (H-1) - cy   (Y invertida para que arriba en imagen sea arriba en RViz)
        pts = []
        for (xx, yy) in [(0, 0), (127, 0), (127, 95), (0, 95), (0, 0)]:
            p = Point()
            p.x = float(xx)
            p.y = float(yy)
            p.z = 0.0
            pts.append(p)

        frame.points = pts
        self.pub_frame.publish(frame)
        self.frame_sent = True

    def tick(self):
        while True:
            try:
                data, addr = self.sock.recvfrom(256)
            except BlockingIOError:
                return
            except Exception as e:
                self.get_logger().warn(f'Error UDP: {e}')
                return

            line = data.decode('utf-8', errors='ignore').strip()
            m = CSV_RE.match(line)
            if not m:
                self.get_logger().warn(f'Paquete no válido desde {addr}: "{line}"')
                continue

            found = int(m.group(1))
            cx = int(m.group(2))
            cy = int(m.group(3))
            shape = int(m.group(4)) if m.group(4) is not None else 0

            # Log
            self.get_logger().info(f'Recibido {addr} -> found={found} cx={cx} cy={cy} shape={shape}')

            # Publicar found + shape
            self.pub_found.publish(Bool(data=(found == 1)))
            self.pub_shape.publish(UInt8(data=shape))

            now = self.get_clock().now().to_msg()

            # Convertir coordenadas para RViz (invertir Y)
            x_rviz = float(cx)
            y_rviz = (H_FRAME - 1.0) - float(cy)   # 95 - cy

            # 1) PointStamped
            ps = PointStamped()
            ps.header.frame_id = self.frame_id
            ps.header.stamp = now
            ps.point.x = x_rviz
            ps.point.y = y_rviz
            ps.point.z = 0.0
            self.pub_point.publish(ps)

            # 2) Marker (esfera)
            mk = Marker()
            mk.header.frame_id = self.frame_id
            mk.header.stamp = now
            mk.ns = "vision"
            mk.id = 0
            mk.type = Marker.SPHERE
            mk.action = Marker.ADD

            mk.pose.position.x = x_rviz
            mk.pose.position.y = y_rviz
            mk.pose.position.z = 0.0
            mk.pose.orientation.w = 1.0

            # Tamaño esfera (ajusta aquí)
            mk.scale.x = 2.0
            mk.scale.y = 2.0
            mk.scale.z = 2.0

            # Color (verde)
            mk.color.a = 1.0
            mk.color.r = 0.0
            mk.color.g = 1.0
            mk.color.b = 0.0

            self.pub_marker.publish(mk)

            # 3) Marco 128x96 (solo una vez, cuando llega el primer paquete)
            if not self.frame_sent:
                self.publish_frame_once(now)

def main():
    rclpy.init()
    node = UdpReceiver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()