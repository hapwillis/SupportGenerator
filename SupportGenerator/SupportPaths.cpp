#include "SupportPaths.h"

PFNode::PFNode()
{
}

PFNode::~PFNode()
{
}

PathFindingGraph::PathFindingGraph()
{
}

PathFindingGraph::~PathFindingGraph()
{
}

Path::Path()
{

}

Path::~Path()
{

}

void Path::Geometry()
{
}

void Path::Draw()
{
}

int Path::getCost()
{
	return 0;
}

void Path::intersection()
{
}

void Path::cylinder()
{
}

SupportPaths::SupportPaths(Graph *g, std::vector<glm::vec3> p, float max) :
	navGraph(g), points(p), maxOverhang(max)
{

}


SupportPaths::~SupportPaths()
{

}

void SupportPaths::FindPaths()
{

}

void SupportPaths::Relax(int degree)
{

}

void SupportPaths::Geometry(int faces, float tipD)
{

}

void SupportPaths::Draw()
{

}

void SupportPaths::DeleteSupport()
{

}

void SupportPaths::findStartNode(glm::vec3 point)
{

}

