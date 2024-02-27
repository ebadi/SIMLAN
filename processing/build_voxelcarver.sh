# git clone https://github.com/unclearness/vacancy.git --recursive
cd vacancy
rm data/*.ply
rm data/sdf*

mkdir build; cd build
cmake .. 
make
cd ../../

## process uses too much memory, CPU and get killed, reduce resolution parameter 
./vacancy/bin/voxel_curver 20.0 -250.000000 -344.586151 -129.982697 250.000000 150.542343 257.329224 10.0 ./vacancy/data/ /
