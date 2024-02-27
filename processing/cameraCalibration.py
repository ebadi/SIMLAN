
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
        self.in_K = intrinsic_yaml.getNode("K").mat() # 9 items: Camera matrix
        self.in_D = intrinsic_yaml.getNode("D").mat() # 5 items: Distortion coefficients
        self.in_xi = intrinsic_yaml.getNode("xi").mat() # 0
        in_image_shape = intrinsic_yaml.getNode("image_shape").mat() # 3 items

        self.ex_rot_mat = extrinsic_yaml.getNode("rot_mat").mat() # 9 items: rotation
        self.ex_t_vec = extrinsic_yaml.getNode("t_vec").mat() # 3 items:
        self.ex_camera_matrix_p = extrinsic_yaml.getNode("camera_matrix_p").mat() # 12 items:
        self.ex_position = extrinsic_yaml.getNode("position").mat() # 3 items: xyz position

        # http://sdformat.org/tutorials?tut=specify_pose
        # The elements x y z specify the position vector (in meters), and the elements roll pitch yaw are Euler angles (in radians) that specify the orientation, which can be computed by an extrinsic X-Y-Z rotation

        self.x_ = self.ex_position[0][0]
        self.y_ = self.ex_position[1][0]
        self.z_ = self.ex_position[2][0]

        invR = numpy.linalg.inv(self.ex_rot_mat)
        neg_invR = numpy.negative(invR)
        neg_invR_t = numpy.dot(neg_invR, self.ex_t_vec)
        self.x = neg_invR_t[0][0]
        self.y = neg_invR_t[1][0]
        self.z = neg_invR_t[2][0]

        rpy = transforms3d.euler.mat2euler(mat=self.ex_rot_mat, axes='sxyz')  # other possible _AXES2TUPLE : https://github.com/matthew-brett/transforms3d/blob/main/transforms3d/euler.py#L148

        self.r =  rpy[0]
        # THIS IS A HACK, our guess is that we didn't use the right value for _AXES2TUPLE! We tried few transformation but couldn't find a "p" that is multiplier of math.pi/2
        # We even tried another online convertor https://www.andre-gaschler.com/rotationconverter/
        self.p = math.pi/2 - rpy[1] 
        self.w =  rpy[2]

        self.width= int(in_image_shape[0][0])
        self.height= int(in_image_shape[1][0])

        # 84 degree to 1.46607657 radians
        # http://sdformat.org/spec?ver=1.11&elem=sensor#camera_distortion

        self.k1 = self.in_D[0][0]
        self.k2 = self.in_D[1][0]
        self.k3 = self.in_D[2][0]
        self.p1 = self.in_D[3][0]
        self.p2 = self.in_D[4][0]

    def xacro_str(self):
        return (f"""<xacro:camera number="{self.camera_name}" x="{self.x}" y="{self.y}" z="{self.z}" r="{self.r}" p="{self.p}" w="{self.w}" width="{self.width}" height="{self.height}" k1="{self.k1}" k2="{self.k2}" k3="{self.k3}" p1="{self.p1}" p2="{self.p2}" horizental_fov="1.46607657"  />\n""")

    def voxel_str(self):
        # https://github.com/DLu/tf_transformations/tree/main#context
        # The new API has more consistent naming, but even then, it is not a one-to-one translation. for example, tf.transformations.quaternion_from_euler could be replaced with transforms3d.euler.euler2quat, but tf returns the quaternion with the ordering x, y, z, w and transforms3d returns w, x, y, z.
        qw, qx, qy, qz = transforms3d.euler.euler2quat(self.r, self.p, self.w)
        # Intrinsic parameters are specific to a camera. They include information like focal length (fx,fy) and optical centers (cx,cy) (is it the same as principle point?)
        # cameramatrix=|fx 0 cx|
        #              |0 fy cy|
        #              |0  0  0⎥
        # https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
        px = self.in_K[0][2]
        py = self.in_K[1][2]
        fx = self.in_K[0][0]
        fy = self.in_K[1][1]
        #print("K=\n{:10.2f} {:10.2f} {:10.2f}\n{:10.2f} {:10.2f} {:10.2f}\n{:10.2f} {:10.2f} {:10.2f}".format(self.in_K[0][0],self.in_K[0][1],self.in_K[0][2],self.in_K[1][0],self.in_K[1][1],self.in_K[1][2],self.in_K[2][0],self.in_K[2][1],self.in_K[2][2])) 
        #print("W= ",  self.width/2, "H=", self.height/2 )
        return (f"""{self.camera_name} {self.x} {self.y} {self.z} {qx} {qy} {qz} {qw} {int(self.width)} {int(self.height)} {int(self.width/2)} {int(self.width/2)} {50} {50}""")



if __name__ == "__main__":

    if len(sys.argv) != 5 :
        print("first argument: [opencv_intrinsic.yaml] [opencv_extrinsic.yaml] [camera_name] [xacro/voxel]")

    intrinsic_yaml_file = sys.argv[1]
    extrinsic_yaml_file = sys.argv[2]
    camera_name= sys.argv[3]

    conf = Camera_config(intrinsic_yaml_file, extrinsic_yaml_file, camera_name )
    if sys.argv[4] == "xacro":
        print(conf.xacro_str())
    else:
        print(conf.voxel_str())

