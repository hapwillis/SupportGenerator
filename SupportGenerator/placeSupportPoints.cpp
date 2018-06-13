#include "Model.h"

void InitializeSupportPoints(Model model) 
{
	model.octree = Octree(model.meshes, model.vertices, model.faces);
}

void PlaceSupportPoints(Model model)
{
	InitializeSupportPoints(model);
}