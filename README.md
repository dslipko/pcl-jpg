# pcl-jpg
Read and process points cloud 3d, export in jpg. Requires `pcl` library. 

PCL-JPG exports 3D view of [.bin] files in [.jpg] images. 

## Install: 
```
mkdir build
cd build
cmake ..
make
```

## Usage: 
```
./pcl-jpg cloud_filename.[bin]
```

Then press `s` key to save the image in current folder. Close window to exit.

## Image examples:

![Image example 1](https://github.com/dslipko/pcl-jpg/blob/092e42afa551148f3d51e25fcb9947ed79dbb927/images/Image2022-03-24%2012:10:35.jpg)

![Image example 2](https://github.com/dslipko/pcl-jpg/blob/4247de9f5bca8363592aa34d5b62e129631a1f9b/images/Image2022-03-24%2012:10:53.jpg)

## Solved tasks:

- [x] open `car.bin`
- [x] export view as `.jpg`
- [x] find center of the cloud
- [x] transform coordinates 
- [x] generate 2 different views
- [x] export results in  `.jpg`
- [x] tidying 
- [x] write docs and `readme.md`

## Input files serialization
```
std::ofstream binary("filename.bin", std::ios::out | std::ios::binary | std::ios::trunc);

for (const auto &point : points){
   binary.write(reinterpret_cast<const char *>(&point.x), sizeof(float));
   binary.write(reinterpret_cast<const char *>(&point.y), sizeof(float));
   binary.write(reinterpret_cast<const char *>(&point.z), sizeof(float));
}
```