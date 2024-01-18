
import sys
import cv2
import transforms3d # pip3 install transforms3d==0.4.1
import math
import numpy


class Camera_config():
    def __init__(self, intrinsic_yaml_file, extrinsic_yaml_file, camera_name= "camera_name"):

        self.camera_name = camera_name
        intrinsic_yaml = cv2.FileStorage(intrinsic_yaml_file, cv2.FILE_STORAGE_READ)
        extrinsic_yaml = cv2.FileStorage(extrinsic_yaml_file, cv2.FILE_STORAGE_READ)

        # https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
        # https://www.geeksforgeeks.org/calibratecamera-opencv-in-python/
        self.in_K = intrinsic_yaml.getNode("K") # 9 items: Camera matrix
        self.in_D = intrinsic_yaml.getNode("D") # 5 items: Distortion coefficients
        self.in_xi = intrinsic_yaml.getNode("xi") # 0
        in_image_shape = intrinsic_yaml.getNode("image_shape") # 3 items

        self.ex_rot_mat = extrinsic_yaml.getNode("rot_mat") # 9 items: rotation
        self.ex_t_vec = extrinsic_yaml.getNode("t_vec") # 3 items:
        self.ex_camera_matrix_p = extrinsic_yaml.getNode("camera_matrix_p") # 12 items:
        self.ex_position = extrinsic_yaml.getNode("position") # 3 items: xyz position

        # http://sdformat.org/tutorials?tut=specify_pose
        # The elements x y z specify the position vector (in meters), and the elements roll pitch yaw are Euler angles (in radians) that specify the orientation, which can be computed by an extrinsic X-Y-Z rotation

        self.x_ = self.ex_position.mat()[0][0]
        self.y_ = self.ex_position.mat()[1][0]
        self.z_ = self.ex_position.mat()[2][0]

        invR = numpy.linalg.inv(self.ex_rot_mat.mat())
        neg_invR = numpy.negative(invR)
        neg_invR_t = numpy.dot(neg_invR, self.ex_t_vec.mat())
        self.x = neg_invR_t[0][0]
        self.y = neg_invR_t[1][0]
        self.z = neg_invR_t[2][0]

        rpy = transforms3d.euler.mat2euler(mat=self.ex_rot_mat.mat(), axes='sxyz')  # other possible _AXES2TUPLE : https://github.com/matthew-brett/transforms3d/blob/main/transforms3d/euler.py#L148

        self.r =  rpy[0]
        # THIS IS A HACK, our guess is that we didn't use the right value for _AXES2TUPLE! We tried few transformation but couldn't find a "p" that is multiplier of math.pi/2
        # We even tried another online convertor https://www.andre-gaschler.com/rotationconverter/
        self.p = math.pi/2 - rpy[1] 
        self.w =  rpy[2]

        self.width= int(in_image_shape.mat()[0][0]/10)
        self.height= int(in_image_shape.mat()[1][0]/10)

        # 84 degree to 1.46607657 radians
        # http://sdformat.org/spec?ver=1.11&elem=sensor#camera_distortion

        self.k1 = self.in_D.mat()[0][0]
        self.k2 = self.in_D.mat()[1][0]
        self.k3 = self.in_D.mat()[2][0]
        self.p1 = self.in_D.mat()[3][0]
        self.p2 = self.in_D.mat()[4][0]

    def print_xacro(self):
        print(f"""<xacro:camera number="{self.camera_name}" x="{self.x}" y="{self.y}" z="{self.z}" r="{self.r}" p="{self.p}" w="{self.w}" width="{self.width}" height="{self.height}" k1="{self.k1}" k2="{self.k2}" k3="{self.k3}" p1="{self.p1}" p2="{self.p2}" horizental_fov="1.46607657"  />""")

if __name__ == "__main__":

    if len(sys.argv) != 4 :
        print("first argument: [opencv_intrinsic.yaml] [opencv_extrinsic.yaml] [camera_name]")

    intrinsic_yaml_file = sys.argv[1]
    extrinsic_yaml_file = sys.argv[2]
    camera_name= sys.argv[3]

    conf = Camera_config(intrinsic_yaml_file, extrinsic_yaml_file, camera_name )
    conf.print_xacro()

