/*
 * Copyright (C) 2019, unclearness
 * All rights reserved.
 */

/*
Based on https://github.com/unclearness/vacancy/blob/master/VoxelCarving.cc
Modification by Hamid Ebadi, 2024
*/
#include <stdio.h>
#include <fstream>
#include "vacancy/voxel_carver.h"
#include <stdlib.h>
#include <iostream>

using namespace std;

vector<string> Split(const string& input, char delimiter) {
  istringstream stream(input);
  string field;
  vector<string> result;
  while (getline(stream, field, delimiter)) {
    result.push_back(field);
  }
  return result;
}

int main(int argc, char* argv[]) {
  (void)argc;
  (void)argv;


  string line;
  if (argc <  10){
    vacancy::LOGE("wrong cmd line argument: %d items found \n", argc) ;
    return false;
  }

  string data_dir(argv[9]);
  string timestamp(argv[10]);
  ifstream ifs(data_dir  + "cameras.txt");
  Eigen::Affine3d pose;
  Eigen::Translation3d t;
  Eigen::Quaterniond q;

  vacancy::VoxelCarver carver;
  vacancy::VoxelCarverOption option;

  // =================== [Non-camera settings from ARGV] ===================
  // voxel resolution in mm
  option.resolution = atof(argv[8]);

  // exact mesh bounding box computed in advance
  option.bb_min = Eigen::Vector3f(atof(argv[2]), atof(argv[3]), atof(argv[4]));
  option.bb_max = Eigen::Vector3f(atof(argv[5]), atof(argv[6]), atof(argv[7]));

  // add offset to the bounding box to keep boundary clean
  float bb_offset = atof(argv[1]);

  option.bb_min[0] -= bb_offset;
  option.bb_min[1] -= bb_offset;
  option.bb_min[2] -= bb_offset;

  option.bb_max[0] += bb_offset;
  option.bb_max[1] += bb_offset;
  option.bb_max[2] += bb_offset;

  carver.set_option(option);
  carver.Init();

  while (getline(ifs, line)) {
      vector<string> splited = Split(line, ' ');
      if (splited.size() != 14) {
        vacancy::LOGE("wrong format: %d items found in the camera config file\n", splited.size() );
        return false;
      }

    // =================== [camera settings from config file] ===================
    string camera_id_str = splited[0];
    t.x() = atof(splited[1].c_str());
    t.y() = atof(splited[2].c_str());
    t.z() = atof(splited[3].c_str());

    q.x() = atof(splited[4].c_str());
    q.y() = atof(splited[5].c_str());
    q.z() = atof(splited[6].c_str());
    q.w() = atof(splited[7].c_str());
    pose = t * q;
    // image size and intrinsic parameters
    int width  = atoi(splited[8].c_str());
    int height = atoi(splited[9].c_str());
    Eigen::Vector2f principal_point(atof(splited[10].c_str()), atof(splited[11].c_str()));
    Eigen::Vector2f focal_length(atof(splited[12].c_str()), atof(splited[13].c_str()));

    // =================== [End of input parsing] ===================
    shared_ptr<vacancy::Camera> camera =
        make_shared<vacancy::PinholeCamera>(width, height,
                                                Eigen::Affine3d::Identity(),
                                                principal_point, focal_length);

    cout << "camera:"            << endl << camera_id_str << endl;
    cout << "width, height:"     << endl << width << " " << height << endl ;
    cout << "option.bb_min:"     << endl << option.bb_min << endl ;
    cout << "option.bb_max:"     << endl << option.bb_max << endl ;
    cout << "principal_point:"   << endl << principal_point << endl;
    cout << "focal_length:"      << endl << focal_length << endl;
    cout << "option.resolution:" << endl << option.resolution << endl;
    cout << "bb_offset:"         << endl << bb_offset << endl;

    camera->set_c2w(pose);

    vacancy::Image1b silhouette;
    silhouette.Load(data_dir + timestamp +"/mask_" + camera_id_str + ".png");

    vacancy::Image1f sdf;
    // Carve() is the main process to update voxels. Corresponds to the fusion
    // step in KinectFusion
    carver.Carve(*camera, silhouette, &sdf);
    // save SDF visualization
/*    
    vacancy::Image3b vis_sdf;
    vacancy::SignedDistance2Color(sdf, &vis_sdf, -1.0f, 1.0f);
    vis_sdf.WritePng(data_dir + timestamp + "/sdf_" + camera_id_str + ".png");
*/
    vacancy::Mesh mesh;
    // voxel extraction
    // slow for algorithm itself and saving to disk
    carver.ExtractVoxel(&mesh);
    mesh.WritePly(data_dir + timestamp + "/voxel_" + camera_id_str + ".ply");
/*
    // marching cubes
    // smoother and faster
    carver.ExtractIsoSurface(&mesh, 0.0);
    mesh.WritePly(data_dir + timestamp + "/surface_" + camera_id_str + ".ply");

    // No linear interpolation for marching cubes, angular surface
    carver.ExtractIsoSurface(&mesh, 0.0, false);
    mesh.WritePly(data_dir + timestamp + "/surface_nointerp_" + camera_id_str + ".ply");
*/    
  }

  return 0;
}