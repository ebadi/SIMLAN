### multi-view 3D reconstruction

voxels : 3d pixels

issue with one plane projection:
- shadows 
- object with thin/small connection with floor (human foot, table legs)


Shape From Silhouette(SFS)
- The visual hull cannot capture concavities not visible in the silhouettes
- Volumetric Approach: Define the scene's bounding box and discretize it. - Evaluate for each voxel: "am I in the object ?"
- Octree speed up
- Space Carving
 - Initialize a volume with a superset of the true scene
 - Repeat until convergence :
	- Project a surface voxel into all images in which it is visible.
	- Remove if not photoconsistent.

### References:

- https://en.wikipedia.org/wiki/Visual_hull
- https://campar.in.tum.de/twiki/pub/Chair/TeachingWs10Cv2/Multi-View3D-Reconstruction.pdf
- https://github.com/bluestyle97/awesome-3d-reconstruction-papers#multi-view-1
- https://medium.com/@satya15july_11937/3d-image-reconstruction-from-multi-view-stereo-782e6912435b
- https://github.com/unclearness/vacancy.git


### build
To build and test voxelcarver with default iamges:
```
./build_voxelcarver.sh
```

Make sure that simulation is running first (`./build_world.sh` and `./spawn_static_agent.sh`):

```
./ros2_cameras.sh
```

f3d can be used to open `ply` files that is generated as the result in `data` directory:

```bash
sudo apt install f3d openctm-tools
f3d file.ply
ctmviewer file.ply
```



### Camera intrinsic file format

The `cameraCalibration.py` script generates `vacancy/data/cameras.txt` that is used by `VoxelCarving.cc`.

```
00000 -39.163902 -48.510956 -718.163391 0.000000 0.000000 0.000000 1.000000 320 240 159.3 127.65 258.65 258.25
```

- FIELD[0]: Camera ID
- FIELD[1]: Pos X
- FIELD[2]: Pos Y
- FIELD[3]: Pos Z
- FIELD[4]: rot X
- FIELD[5]: rot Y
- FIELD[6]: rot Z
- FIELD[7]: rot W
- FIELD[8]: width, 320
- FIELD[9]: height, 240
- FIELD[10]: principal_point_x, 159.3f
- FIELD[11]: principal_point_y, 127.65f
- FIELD[12]: focal_length_x, 258.65f
- FIELD[13]: focal_length_y, 258.25f

# voxel_curver command line argument parameters 
- argv[0]: executable name
- argv[1]: bb_offset, 20.0f
- argv[2]: bb_min_x, -250.000000f
- argv[3]: bb_min_y, -344.586151f
- argv[4]: bb_min_z -129.982697f
- argv[5]: bb_max_x, 250.000000f
- argv[6]: bb_max_y, 150.542343f
- argv[7]: bb_max_z, 257.329224f
- argv[8]: resolution  10.0f = 10mm 
- argv[9]: data directory
- argv[10]: timestamp directory inside data directory
