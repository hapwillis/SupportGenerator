#include "SupportPaths.h"

Cylinder::Cylinder(int numSides, float supportWidth) : faces(numSides), width(supportWidth)
{

}

void Cylinder::updateGeometry(glm::vec3 startP, glm::vec3 endP)
{
	start = startP;
	end = endP;
	glm::vec3 sinVec = glm::cross(glm::normalize(end - start), cosVec);

	vertices.clear();
	indices.clear();

	genVerts(sinVec, start);
	genVerts(sinVec, end);

	genIndices();
}

void Cylinder::genVerts(glm::vec3 sinVec, glm::vec3 center)
{
	float frac = pi2 / faces;
	for (int face = 0; face < faces; face++) {
		float theta = frac * face;
		glm::vec3 pos = center;
		pos += std::sin(theta) * sinVec + std::cos(theta) * cosVec;
	
		vertices.push_back(pos);
	}
}

void Cylinder::genIndices()
{
	for (int a = 0; a < faces - 1; a++) {
		int b = a + 1;
		int c = a + faces;
		int d = b + faces;

		indices.push_back(a);
		indices.push_back(b);
		indices.push_back(d);

		indices.push_back(a);
		indices.push_back(d);
		indices.push_back(c);
	}
}

PFNode::PFNode(Node *aNode) : node(aNode)
{

}

PFNode::~PFNode()
{
}

Path::Path(std::vector<PFNode> *nodes, float overhang, glm::mat4 model) 
	: maxOverhang(overhang), nav(nodes), transform(model)
{

}

Path::~Path()
{
	delete(path[0]);
	delete(path[1]);
}

void Path::addNode(Node *node)
{
	PFNode *pathNode = new PFNode(node);
	path.push_back(pathNode);
	pathNode->costToEnd = getCost(pathNode);

	closed.insert(node->ID);
	for (int c : node->connections) {
		PFNode *pNode = &(*nav)[c];
		pNode->costToEnd = getCost(pNode);
		open.push(pNode);
		seen.insert(c);
	}
}

void Path::aStar(bool pathFound)
{
	while (!open.empty()) {
		PFNode *current = open.top();
		glm::vec3 currentPos = current->node->vertex.Position;
		if (currentPos.x <= 0)
			retracePath(current);
		closed.insert(current->node->ID);
		open.pop();
		PFNode *pNode = &(*nav)[current->node->connections[0]];
		float dist = glm::distance(currentPos, pNode->node->vertex.Position);
		float least = pNode->costToEnd + dist;

		for (int c : current->node->connections) {
			if (closed.count(c) == 0) {
				pNode = &(*nav)[c];
				dist = glm::distance(currentPos, pNode->node->vertex.Position);
				float toStart = current->costToStart + dist;
				float temp = pNode->costToEnd + dist;
				if (temp < least)
					least = temp;

				if (seen.count(c) == 0 || toStart < pNode->costToStart) { //only add old nodes
					if (pNode->costToEnd >= 0.0) { //filter invalid nodes
						pNode->costToStart = toStart;
						pNode->parent = current;
						pNode->pathCost = toStart + pNode->costToEnd;
						open.push(pNode);
					}
				}
			}
		}
		current->costToEnd = least;
	}

	if (pathFound) {
		delete(path[0]);
		delete(path[1]);
		path.clear();
	} else {
		restartAStar(); // No direct path found- check if any nodes can see bed
	}
}

void Path::Geometry()
{
	std::vector<Vertex> vertices;
	std::vector<int> indices;
	int vOffset = 0;
	int iOffset = 0;
	vertices.reserve(faces * path.size() * 2);
	indices.reserve(faces * path.size() * 6);
	Cylinder c = Cylinder(faces, width);

	for (int i = 1; i < path.size() - 1; i++) {
		c.updateGeometry(path[i]->node->vertex.Position, path[i + 1]->node->vertex.Position);
		glm::vec3 norm = glm::vec3(0.0f, 0.0f, 0.0f);
		
		for (glm::vec3 pos : c.vertices) {
			vertices.emplace_back(pos, norm, true);
		}
		vOffset = vertices.size();

		for (int i : c.indices) {
			indices.push_back(i);
		}
		iOffset = indices.size();
	}
}

void Path::Draw(DefaultShader shader)
{
	if (pathGeometry) {
		pathGeometry->Draw(shader);
	} else {
		Geometry();
	}
}

float Path::getCost(PFNode *node)
{
	if (node->node->connections.size() == 0) {
		return -1;
	}

	return (transform*(glm::vec4(node->node->vertex.Position, 1.0f))).x;
}

void Path::restartAStar()
{
	closed.clear();
	seen.clear();
	open = std::priority_queue<PFNode*>();
	open.push(path.back());
	seen.insert(path.back()->node->ID);
	for (int i = 0; i < path.size() - 1; i++) {
		closed.insert(path[i]->node->ID);
	}
	aStar(true);
}

void Path::retracePath(PFNode* end)
{
	std::vector<PFNode*> backwards;
	backwards.push_back(end);
	while (backwards.back()->parent) {
		backwards.push_back(backwards.back()->parent);
	}

	while (backwards.size() > 0) {
		path.push_back(backwards.back());
		backwards.pop_back();
	}

	closed.clear();
	seen.clear();
	open = std::priority_queue<PFNode*>();
}

SupportPaths::SupportPaths(Graph *model, Graph *nav, std::vector<glm::vec3> p, float max, float offset) :
	modelGraph(model), navGraph(nav), points(p), maxOverhang(max)
{
	for (Node *n : navGraph->nodes) {
		nodes.push_back(PFNode(n));
		nodes.back().costToEnd = (transform*(glm::vec4(n->vertex.Position, 1.0f))).x;
	}

	paths.reserve(points.size());
	navGraph->scale(offset);

	// TODO: thread these
	modelGraph->buildOctree();
	navGraph->buildOctree();
}

SupportPaths::~SupportPaths()
{

}

void SupportPaths::FindPaths()
{
	for (glm::vec3 &point : points) {
		Path *path = findStartNode(point);
		paths.push_back(path);
		path->aStar(false);
	}

	regroupPaths();
}

void SupportPaths::Relax(float degree)
{
	//degree is the fraction of nodes to remove- 1 means totally straight

}

void SupportPaths::Geometry(int faces, float tipD)
{

}

void SupportPaths::Draw(DefaultShader shader)
{
	for (Path *path : paths) {
		path->Draw(shader);
	}
}

void SupportPaths::DeleteSupport()
{

}

Path* SupportPaths::findStartNode(glm::vec3 point)
{
	Face *face = modelGraph->octree->getNearestFace(point);
	Path *path = new Path(&nodes, maxOverhang, transform);
	Node *startNode = new Node(-1, point, face->normal, true);
	path->addNode(startNode);

	glm::vec3 pointTwo = navGraph->octree->rayCast(point, face->normal);
	Node *nodeTwo = new Node(-1, pointTwo, face->normal, true);
	path->addNode(nodeTwo);

	int index = navGraph->octree->getNearestNodeIndex(pointTwo);
	path->addNode(navGraph->nodes[index]);

	return path;
}

void SupportPaths::regroupPaths()
{

}

void SupportPaths::intersectionGeometry()
{

}
