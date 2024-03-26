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

