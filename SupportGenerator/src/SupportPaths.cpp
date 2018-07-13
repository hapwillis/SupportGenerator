#include "SupportPaths.h"


Cylinder::Cylinder(int numSides, float supportWidth) : faces(numSides), width(supportWidth)
{

}

// 
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

// 
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

// 
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
	delete pathGeometry;
	delete(path[0]);
	delete(path[1]);
}

// 
void Path::addNode(Node *node)
{
	PFNode *pathNode = new PFNode(node);
	if (path.size() > 0)
		closed.insert(path.back()->node->ID);
	path.push_back(pathNode);
	pathNode->costToEnd = getCost(pathNode);
}

// 
void Path::aStar(bool pathFound)
{
	glm::vec3 origin = path[1]->node->vertex.Position;

	while (!open.empty()) {
		PFNode *current = open.top();
		glm::vec3 currentPos = current->node->vertex.Position;
		if (currentPos.z <= 0.000000001f) {
			retracePath(current);
			return;
		}
		closed.insert(current->node->ID);
		open.pop();
		if (current->node->connections.size() > 0) {
			PFNode *pNode = &(*nav)[current->node->connections[0]];
			float dist = glm::distance(currentPos, pNode->node->vertex.Position);
			float least = pNode->costToEnd + dist;

			for (int c : current->node->connections) { 
				if (closed.count(c) == 0) {
					glm::vec3 cVect = glm::normalize((*nav)[c].node->vertex.Position - origin);
					float angle = std::acos(glm::dot(cVect, glm::vec3(0.0f, 0.0f, -1.0f)));

					if (angle < maxOverhang) {
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
								pNode->pathCost = pNode->costToStart + pNode->costToEnd;
								open.push(pNode);
							}
						}
					} else {
						closed.insert(c);
					}
				}
			}
			current->costToEnd = least; 
			current->pathCost = current->costToStart + current->costToEnd;
		} else {
			std::cout << "Error: unconnected node." << std::endl;
		}
	}

	if (pathFound) {
		delete(path[0]);
		delete(path[1]);
		path.clear();
	} else {
		restartAStar(); // No direct path found- check if any nodes can see bed
	}
}

// 
void Path::Geometry()
{
	if (path.size() < 1)
		return;
	std::vector<Vertex> vertices;
	std::vector<unsigned int> indices;
	vertices.reserve(faces * path.size() * 2);
	indices.reserve(faces * path.size() * 6);
	Cylinder c = Cylinder(faces, 0.5f); //set the characteristics of the cylinder generator

	// TODO: move this logic into Cylinder; have it add new vertices and indices each time it's given a new point.
	for (int i = 1; i < (path.size() - 1); i++) {
		c.updateGeometry(path[i]->node->vertex.Position, path[i + 1]->node->vertex.Position);
		glm::vec3 norm = glm::vec3(0.0f, 0.0f, 0.0f);
		
		int vOffset = vertices.size();
		for (glm::vec3 pos : c.vertices) {
			vertices.emplace_back(pos, norm, true);
			//vertices.push_back(Vertex(pos, norm, true));
		}
		
		for (int i : c.indices) {
			indices.push_back(i + vOffset);
		}
	}

	pathGeometry = new Mesh(vertices, indices);

	for (PFNode *n : path) {
		renderpoints.push_back(n->node->vertex);
	}

	pathStep = 1;
	glEnable(GL_PROGRAM_POINT_SIZE);
	glPointSize(10.0f);

	glGenVertexArrays(1, &VAO);
	glGenBuffers(1, &VBO);

	glBindVertexArray(VAO);
	// load data into vertex buffers
	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	// A great thing about structs is that their memory layout is sequential for all its items.
	// The effect is that we can simply pass a pointer to the struct and it translates perfectly to a glm::vec3/2 array which
	// again translates to 3/2 floats which translates to a byte array.
	glBufferData(GL_ARRAY_BUFFER, renderpoints.size() * sizeof(Vertex), &renderpoints[0], GL_STATIC_DRAW);

	// set the vertex attribute pointers
	// vertex Positions
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)0);
	// vertex normals
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, Normal));
	glEnableVertexAttribArray(2);
	glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, Wireframe));
	glBindVertexArray(0);
}

// 
void Path::Draw(DefaultShader shader)
{
	if (pathGeometry) {
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		shader.setVec4("color", glm::vec4(1.0f, 1.0f, 0.0f, 1.0f));
		pathGeometry->Draw(shader);
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

		glPolygonMode(GL_FRONT_AND_BACK, GL_POINTS);
		glBindVertexArray(VAO);
		shader.setVec4("color", glm::vec4(0.0f, 0.0f, 1.0f, 1.0f));
		//glDrawArrays(GL_POINTS, 0, pathStep);
		glBindVertexArray(0);
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

		pathStep++;
		if (pathStep == renderpoints.size())
			pathStep = 1;
		
	} else {
		Geometry();
	}
}

// 
float Path::getCost(PFNode *node)
{
	if (node->node->connections.size() == 0) {
		return -1;
	}

	return (transform*(glm::vec4(node->node->vertex.Position, 1.0f))).z;
}

// 
void Path::restartAStar()
{
	closed.clear();
	seen.clear();
	open = std::priority_queue<PFNode*, std::vector<PFNode*>, PFNodeComparator>();
	open.push(path.back());
	seen.insert(path.back()->node->ID);
	for (int i = 0; i < path.size() - 1; i++) {
		closed.insert(path[i]->node->ID);
	}
	aStar(true);
}

// 
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
	open = std::priority_queue<PFNode*, std::vector<PFNode*>, PFNodeComparator>();
}


SupportPaths::SupportPaths(Graph *model, Graph *nav, std::vector<std::tuple<glm::vec3, Face*>> p, float max, float offset) :
	modelGraph(model), navGraph(nav), maxOverhang(max)
{
	//std::cout << "Checking validity of navGraph..." << std::endl;
	//navGraph->test();
	
	for (auto tuple : p) {
		points.push_back(std::get<0>(tuple));
	}

	for (Node *n : navGraph->nodes) {
		nodes.push_back(PFNode(n));
		nodes.back().costToEnd = (transform*(glm::vec4(n->vertex.Position, 1.0f))).z;
	}

	paths.reserve(points.size()); 
}


SupportPaths::~SupportPaths()
{

}

// 
void SupportPaths::FindPaths()
{
	int failedPaths = 0;

	for (glm::vec3 &point : points) {
		Path *path = findStartNode(point);
		paths.push_back(path);
		if (path->path.size() > 0) {
			path->open.push(path->path.back());
			path->aStar(false);
		} else {
			failedPaths++;
		}

		// works best when points are sorted from lowest to highest
		// Also works best with Djikstra's instead of A*; A* doesn't search out other supports.
		if (OVERLAP_PATHS) {  
			for (PFNode* node : path->path) {
				node->costToEnd = 0.0f;
			}
		}
	}
	std::cout << "Failed paths: " << failedPaths << std::endl;

	regroupPaths();
}

// 
void SupportPaths::Geometry(int faces, float tipD)
{
	// TODO: Geometry()
}

// 
void SupportPaths::Draw(DefaultShader shader)
{
	// TODO: completed paths should have their own geometry- render that 
	// if it exists, otherwise render the wireframes.
	for (Path *path : paths) {
		path->Draw(shader);
	}
}

// 
void SupportPaths::DeleteSupport()
{
	// TODO: DeleteSupport
}

// 
Path* SupportPaths::findStartNode(glm::vec3 point)
{
	// TODO: can just take face from ConnectionPoints.pointFaces
	Face *face = modelGraph->getOctree()->getNearestFace(point);
	Path *path = new Path(&nodes, maxOverhang, transform);
	Node *startNode = new Node(-1, point, face->normal, true);
	path->addNode(startNode);

	// TODO: rayCast is broken.  Probably because the octree is unscaled.
	//glm::vec3 pointTwo = point + 2.0f * face->normal;
	glm::vec3 pointTwo = navGraph->getOctree()->rayCast(point, face->normal);
	Node *nodeTwo = new Node(-1, pointTwo, face->normal, true);
	path->addNode(nodeTwo);

	int index = navGraph->getOctree()->getNearestNodeIndex(pointTwo);
	path->addNode(navGraph->nodes[index]);

	//TEST:
	/*float dist = glm::distance(pointTwo, navGraph->nodes[index]->vertex.Position);
	float shortest = 10000.0f;
	int newindex = 0;
	int mindex = 0;
	for (Node *n : navGraph->nodes) {
		if (glm::distance(pointTwo, n->vertex.Position) < shortest) {
			shortest = glm::distance(pointTwo, n->vertex.Position);
			mindex = newindex;
		}
		newindex++;
	}

	if (mindex != index)
		std::cout << "Failed to find actual nearest index!" << std::endl;
	*/

	return path;
}

// 
void SupportPaths::regroupPaths()
{
	// TODO: regroupPaths
}

// 
void SupportPaths::descendPaths() //rename to RegularizePaths
{
	// TODO: path relaxation of anything under maxAngle
}

// 
void SupportPaths::Relax(float degree)
{
	// TODO: Path Relaxation
	// degree is the fraction of nodes to remove- 1 means totally straight
	// Optimal relaxation is an NP-complete problem

	// Step 1 (if degree < 1.0f): Greedy algorithm
	//	While (notStuck && dist < degree * minDist)
	//		if (comboHeap.empty) :
	//			for (node : path) : add smallest combos to heap
	//		else :
	//			combine(heap.pop())


}

// 
void SupportPaths::intersectionGeometry()
{
	// TODO: intersectionGeometry
	//use: CGAL 4.12 - Polygon Mesh Processing


	// given a vector<cylinder>, project from each line segment to all the faces in the other cylinders
	// move far vertices back to the point of collision, split the collided face
}

// 
void SupportPaths::tip()
{
	// TODO: tip
}

// 
void SupportPaths::base()
{
	// TODO: base
}

// 
void SupportPaths::cylinderSegment()
{
	// TODO: cylinderSegment
}
