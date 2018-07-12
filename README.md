# SupportGenerator
This program is pre-alpha.  It generates branching support structures intended for 3d printing, and is optimized for program speed and print speed.  Rendering is donw through openGL, GLAD, and GLM, window control is through GLFW, and model importing is done with AssImp.

SupportGenerator is quite fast; placing new supports takes ~1 millisecond.  The algorithm creates a descent surface from the mesh itself rather than eg voxels, so pathfinding is much faster.  







TODO list:
- [x] File importing and processing
- [x] Camera controls
- [x] Overhang shader
- [x] Automatic support placement
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
