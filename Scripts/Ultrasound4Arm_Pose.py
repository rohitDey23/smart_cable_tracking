import sys
import serial
import open3d as o3d
import numpy as np
import serial.tools.list_ports, serial
import serial.rs485
import time


class linkage_construction:
    def __init__(self):
        self.link_1_length = 28.89
        self.link_2_length = 26.57
        self.link_3_length = 26.04
        self.link_4_length = 25.72

        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window()

        self.add_linkages_to_visualizer()

    def add_linkages_to_visualizer(self):
        self.link1 = self.create_links(length=self.link_1_length, color=[0.1, 0.9, 0.1])
        self.link2 = self.create_links(length=self.link_2_length, color=[0.9, 0.1, 0.1])
        self.link3 = self.create_links(length=self.link_3_length, color=[0.5, 0.2, 0.1])
        self.link4 = self.create_links(length=self.link_4_length, color=[0.1, 0.1, 0.5])

        self.initial_transforms()

        self.frame0 = self.create_frames(origin=None)
        self.frame1 = self.create_frames(origin=self.create_translation_mat(self.link_1_length))
        self.frame2 = self.create_frames(origin=self.create_translation_mat(self.link_2_length + self.link_1_length))
        self.frame3 = self.create_frames(origin=self.create_translation_mat(self.link_3_length + self.link_2_length + self.link_1_length))
        self.frame4 = self.create_frames(origin=self.create_translation_mat(self.link_4_length + self.link_3_length + self.link_2_length + self.link_1_length))

        self.vis.add_geometry(self.link1)
        self.vis.add_geometry(self.link2)
        self.vis.add_geometry(self.link3)
        self.vis.add_geometry(self.link4)

        self.vis.add_geometry(self.frame0)
        self.vis.add_geometry(self.frame1)
        self.vis.add_geometry(self.frame2)
        self.vis.add_geometry(self.frame3)
        self.vis.add_geometry(self.frame4)

    def initial_transforms(self):
        rot_mat = o3d.geometry.get_rotation_matrix_from_axis_angle([0, np.pi/2, 0])
        transformation_mat = np.identity(4)
        transformation_mat[:3, :3] = rot_mat

        transformation_mat[:3,  3] = np.array([self.link_1_length/2, 0, 0])
        self.link1.transform(transformation_mat)
        #
        transformation_mat[:3, 3] = np.array([(self.link_2_length/2), 0, 0])
        self.link2.transform(transformation_mat)
        #
        transformation_mat[:3, 3] = np.array([(self.link_3_length/2), 0, 0])
        self.link3.transform(transformation_mat)
        #
        transformation_mat[:3, 3] = np.array([(self.link_4_length/2), 0, 0])
        self.link4.transform(transformation_mat)

    def create_homogeneous_matrix(self, quat_val_1, quat_val_2, quat_val_3, quat_val_4):
        quat1 = quat_val_1[:4]
        quat2 = quat_val_2[:4]
        quat3 = quat_val_3[:4]
        quat4 = quat_val_4[:4]

        cali1 = quat_val_1[4]
        cali2 = quat_val_2[4]
        cali3 = quat_val_3[4]
        cali4 = quat_val_4[4]

        quat1 = [quat1[3], quat1[0], quat1[1], quat1[2]]
        quat2 = [quat2[3], quat2[0], quat2[1], quat2[2]]
        quat3 = [quat3[3], quat3[0], quat3[1], quat3[2]]
        quat4 = [quat4[3], quat4[0], quat4[1], quat4[2]]

        R01 = o3d.geometry.get_rotation_matrix_from_quaternion(quat1)
        R02 = o3d.geometry.get_rotation_matrix_from_quaternion(quat2)
        R03 = o3d.geometry.get_rotation_matrix_from_quaternion(quat3)
        R04 = o3d.geometry.get_rotation_matrix_from_quaternion(quat4)

        T01 = np.array(self.create_translation_mat(0))
        T12 = np.array(self.create_translation_mat(self.link_1_length))
        T23 = np.array(self.create_translation_mat(self.link_2_length))
        T34 = np.array(self.create_translation_mat(self.link_3_length))

        T01 = T01*0
        T02 = (R01 @ T12) + T01
        T03 = (R02 @ T23) + T02
        T04 = (R03 @ T34) + T03

        H01 = np.identity(4)
        H02 = np.identity(4)
        H03 = np.identity(4)
        H04 = np.identity(4)

        H01[:3, :3] = R01
        H01[:3, 3] = T01

        H02[:3, :3] = R02
        H02[:3, 3] = T02

        H03[:3, :3] = R03
        H03[:3, 3] = T03

        H04[:3, :3] = R04
        H04[:3, 3] = T04

        return H01, H02, H03, H04, np.array([cali1,cali2,cali3, cali4])

    def transform_links(self, H01, H02, H03, H04):
        self.link1.transform(H01)
        self.link2.transform(H02)
        self.link3.transform(H03)
        self.link4.transform(H04)

        self.frame1.transform(H01)
        self.frame2.transform(H02)
        self.frame3.transform(H03)
        self.frame4.transform(H04)

    def reset_transform(self, H01, H02, H03, H04):
        self.link1.transform(self.invert_matrix(H01))
        self.link2.transform(self.invert_matrix(H02))
        self.link3.transform(self.invert_matrix(H03))
        self.link4.transform(self.invert_matrix(H04))

        self.frame1.transform(self.invert_matrix(H01))
        self.frame2.transform(self.invert_matrix(H02))
        self.frame3.transform(self.invert_matrix(H03))
        self.frame4.transform(self.invert_matrix(H04))

    def invert_matrix(self, mat):
        R_trans = np.transpose(mat[:3, :3])
        T_trans = -1 * R_trans @ mat[:3, 3]

        H = np.identity(4)
        H[:3, :3] = R_trans
        H[:3, 3] = T_trans

        return H

    def update_geometries(self):
        self.vis.update_geometry(self.link1)
        self.vis.update_geometry(self.link2)
        self.vis.update_geometry(self.link3)
        self.vis.update_geometry(self.link4)

        self.vis.update_geometry(self.frame1)
        self.vis.update_geometry(self.frame2)
        self.vis.update_geometry(self.frame3)
        self.vis.update_geometry(self.frame4)

        self.vis.poll_events()
        self.vis.update_renderer()

    @staticmethod
    def create_links(radius=0.6, length=2.0, color=[0.1, 0.9, 0.1]):
        mesh_cylinder = o3d.geometry.TriangleMesh.create_cylinder(radius=radius, height=length)
        mesh_cylinder.compute_vertex_normals()
        mesh_cylinder.paint_uniform_color(color)

        return mesh_cylinder

    @staticmethod
    def create_frames(size=10, origin=None):
        origin = None
        if origin is None:
            origin = [0, 0, 0]

        mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=size, origin=origin)

        return mesh_frame

    @staticmethod
    def create_translation_mat(length):
        return [length, 0, 0]


class Read_Data:

    def __init__(self, port='COM11', baud=115200):
        try:
            self.conn = serial.rs485.RS485(port=port, baudrate=baud, timeout=1, write_timeout=1)
            self.conn.rs485_mode = serial.rs485.RS485Settings(rts_level_for_rx=True, rts_level_for_tx=False)
            time.sleep(1)

        except serial.SerialException as var:
            print(f"Error Occurred in Establishing Connection:{var}")
            self.conn = None
        else:
            print("Working")

    def read_data(self):
        try:
            data_r = self.conn.readline()
            data1 = data_r.decode('utf-8')
            if data1 != '':
                pose_list = list(map(lambda x: float(x), data1.split(",")))
            else:
                pose_list = None

            # print(f"GOT SOMETHING: {data1}")

        except Exception as e:
            pose_list = None
            print(f"Read Level Error: {e}")

        return pose_list


def main():
    o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Debug)
    link_chain = linkage_construction()
    read_data = Read_Data(port='COM11', baud=115200)

    while read_data.conn.isOpen():

        pose_list = read_data.read_data()
        if pose_list is not None:
            link1_quat = pose_list[0:5]
            link2_quat = pose_list[5:10]
            link3_quat = pose_list[10:15]
            link4_quat = pose_list[15:20]

            H01, H02, H03, H04, system_stat = link_chain.create_homogeneous_matrix(link1_quat, link2_quat, link3_quat, link4_quat)

            if np.all(system_stat > 0):
                link_chain.link4.paint_uniform_color([0.1, 0.9, 0.1])
                print(f"Calibrated {system_stat}")
            else:
                print(f"Needs Calibration {system_stat}")
                link_chain.link4.paint_uniform_color([0.9, 0.1, 0.9])

            link_chain.transform_links(H01, H02, H03, H04)
            link_chain.update_geometries()
            link_chain.reset_transform(H01, H02, H03, H04)

    link_chain.vis.destroy_window()
    o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Info)


if __name__ == '__main__':
    main()
