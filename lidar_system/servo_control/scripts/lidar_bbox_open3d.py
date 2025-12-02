#!/usr/bin/env python3
# lidar_bbox_open3d.py – ROS 2 Humble
#
# • subscribes  /lidar_points  (PointCloud2, in base_link)
# • publishes   /lidar_bboxes  (MarkerArray -- view in RViz “MarkerArray” display)

import rclpy, numpy as np, open3d as o3d
from rclpy.node             import Node
from sensor_msgs.msg        import PointCloud2
from visualization_msgs.msg import MarkerArray, Marker
from collections            import deque

class BBoxO3D(Node):
    def __init__(self):
        super().__init__("bbox_open3d")
        self.sub  = self.create_subscription(PointCloud2,
                                             "/lidar_points",
                                             self.on_cloud,   10)
        self.pub  = self.create_publisher(MarkerArray,
                                          "/lidar_bboxes",    1)

        # --- rolling buffer (N most-recent frames) ------------
        self.buffer      = deque(maxlen=30)   # 5 × @10 Hz ≈ 0.5 s
        self.frame_id    = "base_link"
        self.cluster_eps = 0.1              # metres
        self.min_pts     = 5

    # -------- util: fast xyz extraction -----------------------
    @staticmethod
    def pc2_to_xyz(msg: PointCloud2) -> np.ndarray:
        step  = msg.point_step                # bytes per point (16)
        xyz_floats = np.frombuffer(msg.data,  dtype=np.float32)
        xyz_floats = xyz_floats.reshape(-1, step//4)[:, :3]
        return xyz_floats                     # (N,3) float32

    # -------- main callback -----------------------------------
    def on_cloud(self, msg: PointCloud2) -> None:
        xyz = self.pc2_to_xyz(msg)
        #if xyz.shape[0] < 30:
         #   return                            # ignore nearly-empty slices

        self.buffer.append(xyz)               # add newest slice
        pts = np.vstack(self.buffer)          # stacked cloud (≈0.5 s)

        pc          = o3d.geometry.PointCloud()
        pc.points   = o3d.utility.Vector3dVector(pts.astype(np.float64))
        labels      = np.array(
            pc.cluster_dbscan(eps=self.cluster_eps,
                              min_points=self.min_pts,
                              print_progress=False)
        )
        n_clusters  = labels.max() + 1
        marray      = MarkerArray()
        stamp       = self.get_clock().now().to_msg()

        for cid in range(n_clusters):
            part = pts[labels == cid]
            if part.size == 0:
                continue

            min_xyz, max_xyz = part.min(axis=0), part.max(axis=0)
            centre           = (min_xyz + max_xyz) / 2.0
            size             =  (max_xyz - min_xyz)

            mk = Marker()
            mk.header.frame_id = self.frame_id
            mk.header.stamp    = stamp
            mk.ns, mk.id       = "bbox_o3d", int(cid)
            mk.type            = Marker.CUBE
            mk.action          = Marker.ADD

            mk.pose.position.x, mk.pose.position.y, mk.pose.position.z = \
                float(centre[0]), float(centre[1]), float(centre[2])
            mk.pose.orientation.w = 1.0

            mk.scale.x, mk.scale.y, mk.scale.z = \
               float(size[0]),  float(size[1]),  float(size[2])
            #mk.text = f"{centre}"
            mag = mk.scale.x**2 + mk.scale.y**2 + mk.scale.z**2
            pos_mag = mk.pose.position.x**2 + mk.pose.position.y**2 + mk.pose.position.z**2
            #print(f"Magnitude = {mag}")
            #print(f"Size = {size}")
            # ---- simple “human-height” heuristic -------------
            #if 1.0 < size[1] < 2.2 and mag < 3:           # ~1.4–2.2 m tall
            if 0.2 < size[2] < 3.0 and pos_mag > 1.0:
                print("CENTER")
                print(centre)
                print("SIZE")
                print(size)
                print("min and max xyz")
                print(min_xyz)
                print(max_xyz)
                mk.color.r, mk.color.g, mk.color.b, mk.color.a = 1.0, 0.0, 0.0,0.6   # red
            #mk.color.a = 0.35

            marray.markers.append(mk)

        self.pub.publish(marray)

# --------------------------- main ----------------------------------------
def main():
    rclpy.init()
    node = BBoxO3D()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
