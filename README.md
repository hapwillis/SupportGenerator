# SupportGenerator
This program is pre-alpha.  It generates branching support structures intended for 3d printing, and is optimized for program speed and print speed.  Rendering is donw through openGL, GLAD, and GLM, window control is through GLFW, and model importing is done with AssImp.  This is a visual studio solution, but there's a makefile version over in [nanoGUI-Testing](https://github.com/hapwillis/nanoGUI-Testing).

SupportGenerator is quite fast; placing new supports takes ~1 millisecond.  The algorithm creates a descent surface from the mesh itself rather than eg voxels, so pathfinding is much faster.  Hipster David takes ~15 seconds to generate supports for 200 points, compared to ~45 seconds in Meshmixer.

![Hipster David with overhangs shaded red](https://raw.githubusercontent.com/hapwillis/SupportGenerator/master/SupportGenerator/resources/supportPoints.bmp)
Mitchell's algorithm is used to spread support points evenly around the model.  Points are added based on their distance from other nodes.  This concentrates supports according to surface area.  Z-distance can be over- or under-emphasized to increase/decrease the amount of support added to highly overhanging faces.  Bridging faces are ignored.  

Points are added using an octree.  This makes it easier to locate nearby points.  

![Hipster David's navigation mesh](https://raw.githubusercontent.com/hapwillis/SupportGenerator/master/SupportGenerator/resources/navmesh.bmp)
The large majority (>90%) of computing time is spent generating the navigation mesh.  Every edge in the model is loaded into a priority heap by edge length.  The shortest edges are pruned by combining the vertices connecting them.  This proceeds until ~99.99% of the vertices are discarded.  I tried a few methods to ensure that the vertices were always gift-wrapping the model, but it's a waste of time; it's faster to just adjust the navigation mesh around the model afterwards.

![Supports drawn onto Hipster David](https://raw.githubusercontent.com/hapwillis/SupportGenerator/master/SupportGenerator/resources/paths.bmp)
Pathfinding is done using A*, but Djikstra is more efficient for print speed.  The supports are created in ascending order from lowest to highest, and the path along an existing support is always zero.  This encourages paths to join existing paths, saving print material and time.  It's a greedy algorithm and could be improved.

Pathfinding is assisted with an octree, which is created in a separate thread.  The octree holds all overhanging faces in the navigation mesh, represented by AABB.  This makes self-intersection tests much faster.



TODO list:
- [x] File importing and processing
- [x] Camera controls
- [x] Overhang shader
- [x] Automatic support placement
- [ ] Support cantilevered faces at their edges
- [x] Octree functions (intersection, point proximity)
- [x] Mesh simplification
- [ ] Mesh regularization (verifying distance from original mesh)
- [x] Pathfinding
- [x] Path unions
- [ ] Path regularizing and relaxation
- [ ] Geometry generation
- [ ] STL exporting
- [ ] Model adjustment
- [ ] Support add/delete
- [ ] GUI (qt, most likely)
