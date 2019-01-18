# Meshes for the Pepper 1.7 model

Those meshes are used to display the PepperVirtual model.

### Conversion from dae to obj
This part can be used to convert the .dae to .obj. You won't need it to launch the PepperVirtual model, all the conversions have been made. To perform a conversion, the following actions have to be taken (tested on Linux):
* Install assimp on your computer
* Clone and compile [bullet3](https://github.com/bulletphysics/bullet3)
* Open the desired .dae file
* Change __<up_axis>Z_UP</up_axis>__ by __<up_axis>Y_UP</up_axis>__
* Look in the .dae file the number of used textures, let n be that number
* Create a .obj and a .mtl via the following command
```bash
assimp export your_file.dae your_file.obj
mv your_file.obj.mtl your_file.mtl
```

* Open your .obj file and change __mtllib your_file.obj.mtl__ by __mtllib your_file.mtl__
* If n == 1, you're done, be sure to modify your URDF accordingly, to use the .obj file as a visual mesh
* If n > 1, type the following command from this mesh folder:
```bash
# Please note that the name App_obj2sdf_gmake_x64_release can vary depending on
# the tools you used to compile bullet3 and your software environment
/path/to/bullet_code/bullet3/bin/App_obj2sdf_gmake_x64_release --fileName="your_file.obj"
```

* You can then remove your_file.obj and the newsdf.sdf files, and rename the partX.obj files relatively to the link and the texture represented.
* Be sure to update your URDF to take those new visual obj into account, you'll have to create additional links to display the extra textures (see pepper.urdf)
