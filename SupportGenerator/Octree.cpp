#include "Octree.h"

Face::Face(glm::vec3 e1, glm::vec3 e2, glm::vec3 v) : edgeOne(e1), edgeTwo(e2), vertex(v)
{

}

bool Face::MollerTrumbore(glm::vec3 rayOrigin, glm::vec3 rayEnd)
{
	const float EPSILON = 0.0000001;
	glm::vec3 h, s, q;
	float a, f, u, v;
	glm::vec3 rayVector = glm::normalize(rayEnd - rayOrigin);
	h = glm::cross(rayVector, edgeTwo);
	a = glm::dot(edgeOne, h);
	if (a > -EPSILON && a < EPSILON)
		return false;
	f = 1 / a;
	s = rayOrigin - vertex;
	u = f * (glm::dot(s, h));
	if (u < 0.0 || u > 1.0)
		return false;
	q = glm::cross(s, edgeOne);
	v = f * glm::dot(rayVector, q);
	if (v < 0.0 || u + v > 1.0)
		return false;
	// At this stage we can compute t to find out where the intersection point is on the line.
	float t = f * glm::dot(edgeTwo, q);
	if (t > EPSILON) // ray intersection
	{
		//glm::vec3 ourIntersectionPoint = rayOrigin + rayVector * t;
		return true;
	}
	else // This means that there is a line intersection but not a ray intersection.
		return false;
}

FaceCell::FaceCell(glm::vec3 c, float r) : center(c), range(r)
{

}

FaceCell::~FaceCell() 
{
	for (Face *f : faces) {
		delete f;
	}
	for (FaceCell *c : children) {
		delete c;
	}
}

bool FaceCell::intersects(glm::vec3 rayOrigin, glm::vec3 rayEnd, float doubleSize)
{
	// First, test all faces in this FaceCell
	for (Face *f : faces) {
		if (f->MollerTrumbore(rayOrigin, rayEnd))
			return true;
	}

	//if smaller than a child, test if it fits into a single octant
	if (doubleSize < range) {
		glm::vec3 p1 = glm::greaterThanEqual(rayOrigin, center);
		glm::vec3 p2 = glm::greaterThanEqual(rayEnd, center);

		if (glm::all(glm::equal(p1, p2))) {
			FaceCell *c = children[findQuadrant(rayOrigin)];
			if (c) {
				return c->intersects(rayOrigin, rayEnd, doubleSize);
			} else {
				return false;
			}
		}
	} else { //just test all faces in the region
		for (FaceCell *c : children) {
			if (c->intersects(rayOrigin, rayEnd, doubleSize))
				return true;
		}
	}
}

int FaceCell::findQuadrant(glm::vec3 p)
{
	int i = 0;
	if (p.x >= center.x) { i = i + 1; }
	if (p.y >= center.y) { i = i + 2; }
	if (p.z >= center.z) { i = i + 4; }
	return i;
}

void FaceCell::add(Face *f, glm::vec3 faceCenter, float doubleSize)
{
	if (doubleSize > range) {
		faces.push_back(f);
		return;
	}
	int child = findQuadrant(faceCenter);
	// TODO: Fix this
	if (!children[child])
		children[child] = populateChild(child);
	children[child]->add(f, faceCenter, doubleSize);
}

FaceCell* FaceCell::populateChild(int i)
{
	float o = range / 4.0;
	glm::vec3 c(center);

	switch (i) {
		case 0: c += glm::vec3(-o, -o, -o);
		case 1: c += glm::vec3(o, -o, -o);
		case 2: c += glm::vec3(-o, o, -o);
		case 3: c += glm::vec3(o, o, -o);

		case 4: c += glm::vec3(-o, -o, o);
		case 5: c += glm::vec3(o, -o, o);
		case 6: c += glm::vec3(-o, o, o);
		case 7: c += glm::vec3(o, o, o);

	}
	return new FaceCell(c, range / 2.0);
}

std::vector<int> Cell::getPoints(glm::vec3 point, float radius, float minD, std::vector<Vertex> &vertices)
{
	if (range > radius * 4) {
		return children[findOctant(point)]->getPoints(point, radius, minD, vertices);
	} else {
		if (filled) {
			// optionally you could eliminate children here, but that probably just slows it down.
			std::vector<int> verts;
			for (Cell *c : children) {
				std::vector<int> v = c->getPoints(point, radius, minD, vertices);
				verts.insert(verts.end(), v.begin(), v.end());
			}

			return verts;
		} else {
			std::vector<int> returnList;

			for (int index : points) {
				float d = glm::distance(vertices[index].Position, point);
				if (d <  radius && d > minD)
					returnList.push_back(index);
			}

			return returnList;
		}
	}
}

int Cell::findOctant(glm::vec3 p)
{
	int i = 0;
	if (p.x >= center.x) { i = i + 1; }
	if (p.y >= center.y) { i = i + 2; }
	if (p.z >= center.z) { i = i + 4; }
	return i;
}

void Cell::add(glm::vec3 p, int pointIndex, std::vector<Vertex> &vertices)
{
	if (!filled) {
		if (points.size() < max) {
			points.push_back(pointIndex);
		} else {
			points.push_back(pointIndex);
			filled = true;
			split(vertices);
		}
	} else {
		Cell *cell = children[findOctant(p)];
		while (cell->filled) {
			cell = cell->children[cell->findOctant(p)];
		}
		
		cell->add(p, pointIndex, vertices);
	}
}

void Cell::split(std::vector<Vertex> &vertices)
{
	children.reserve(8);
	PopulateChildren(children, center, range / 2.0);

	for (int p : points) {
		glm::vec3 pos = vertices[p].Position;
		children[findOctant(pos)]->add(pos, p, vertices);
	}

	points.clear();
}

bool Cell::pointsNotEqual(glm::vec3 p, glm::vec3 q)
{
	//return !glm::all(glm::equal(q, p));
	bool a = false;
	bool b = false;
	bool c = false;
	if (std::abs(p.x - q.x) == 0.0) {
		a = true;
	}
	if (std::abs(p.y - q.y) == 0.0) {
		b = true;
	}
	if (std::abs(p.z - q.z) == 0.0) {
		c = true;
	}
	
	return !(a && b && c);
}

void Cell::PopulateChildren(std::vector<Cell*> &children, glm::vec3 center, float r)
{
	// TODO: lazy initialization
	float offset = r / 2.0;
	glm::vec3 c = center;
	c -= glm::vec3(offset, offset, offset);
	children.push_back(new Cell(c, r));
	c.x += r;
	children.push_back(new Cell(c, r));
	c.x -= r;
	c.y += r;
	children.push_back(new Cell(c, r));
	c.x += r;
	children.push_back(new Cell(c, r));

	c.x -= r;
	c.y -= r;
	c.z += r;
	children.push_back(new Cell(c, r));
	c.x += r;
	children.push_back(new Cell(c, r));
	c.x -= r;
	c.y += r;
	children.push_back(new Cell(c, r));
	c.x += r;
	children.push_back(new Cell(c, r));
}

Cell Cell::find(glm::vec3 p)
{
	int child = findOctant(p);
	if (children[child]) {
		if (children[child]->filled) {
			if (children[child]->children.size() < 1) {
				return *children[child];
			}
			return children[child]->find(p);
		}
	}

	return *(new Cell(center, range));
}

Cell::Cell(glm::vec3 c, float r) : center(c), range(r), filled(false)
{

}

Cell::~Cell()
{
	for (Cell *c : children) {
		delete c;
	}
}

Octree::Octree(std::vector<Mesh> &meshes)
{
	std::cout << "Constructing Octree." << std::endl;
	float time = glfwGetTime();
	//0.131 seconds to remove vertices
	removeDuplicateVertices(meshes);
	std::cout << "Time to remove verts: " << glfwGetTime() - time << std::endl;
	time = glfwGetTime();
	//0.045 seconds to remove faces
	removeDuplicateFaces();
	std::cout << "Time to remove faces: " << glfwGetTime() - time << std::endl;
	time = glfwGetTime();
	getRange();
	std::cout << "Time to get range: " << glfwGetTime() - time << std::endl;
	root = new Cell(glm::vec3(0.0, 0.0, 0.0), range);
	faceRoot = new FaceCell(glm::vec3(0.0, 0.0, 0.0), range);

	time = glfwGetTime();
	int vIndex = 0, fIndex = 0, offset = 0;
	//0.0284 seconds to build vertex octree
	for (int i = 0; i < vertices.size(); i++) {
		addVertex(i);
	}
	std::cout << "Time to insert vertices: " << glfwGetTime() - time << std::endl;
	time = glfwGetTime();
	for (int i = 0; i < faces.size(); i += 3) {
		//addFace(i);
	}
	std::cout << "Time to insert faces: " << glfwGetTime() - time << std::endl;
}


Octree::~Octree()
{
	delete root;
	delete faceRoot;
}

void Octree::removeDuplicateVertices(std::vector<Mesh> &meshes)
{
	
	int translateSize = 0, indicesSize = 0;
	for (Mesh mesh : meshes) { // Low priority: slow
		translateSize += mesh.vertices.size();
		indicesSize += mesh.indices.size();
	}

	std::vector<int> translate;
	translate = constructUniqueVertices(translateSize, meshes);

	//translate faces into new indices
	faces.reserve(indicesSize);
	int offset = 0;
	for (int i = 0; i < meshes.size(); i++) {
		for (int j : meshes[i].indices) {
			faces.push_back(translate[j + offset]); // Low priority: slow
		}
		offset += meshes[i].vertices.size();
	}
}

std::vector<int> Octree::constructUniqueVertices(int size, std::vector<Mesh> &meshes)
{
	std::vector<int> translate;
	translate.reserve(size);
	std::unordered_multimap<float, int> vertexMap;
	int vIndex = 0;

	for (int i = 0; i < meshes.size(); i++) {
		Mesh *mesh = &meshes[i];
		for (Vertex v : mesh->vertices) {
			bool notFound = true;
			float key = v.Position.x;
			auto range = vertexMap.equal_range(key); 

			//check vertexMap for key, find any equal vertices
			for (auto r = range.first; r != range.second; r++) { 
				int index = r->second;
				bool equal = glm::all(glm::equal(v.Position, vertices[index].Position));
				// if found, set translate[tIndex] = value
				if (equal) { 
					translate.push_back(index);
					notFound = false;
					break;
				}
			}
			//if not found, insert this point and set translate[tIndex] = vIndex
			if (notFound) {
				translate.push_back(vIndex);
				vertexMap.insert(std::pair<float, int>(key, vIndex));
				vertices.push_back(v);
				vIndex++;
			}
		}
	}

	return translate;
}

void Octree::removeDuplicateFaces()
{
	// Faster to replace faces than delete individual faces.
	std::vector<unsigned int> newFaces; 
	std::unordered_multimap<int, int> faceMap;

	int fIndex = 0;
	for (int i = 0; i < faces.size(); i += 3) {
		bool notFound = true;
		int key = faces[i];
		auto range = faceMap.equal_range(key); 

		//check faceMap for key, then find any equal faces
		for (auto r = range.first; r != range.second; r++) {
			int index = r->second;

			if ((faces[i + 1] == faces[index]) && (faces[i + 2] == faces[index + 1])) {
				notFound = false;
				break;
			}
		}

		//if not found, insert this point and set translate[tIndex] = vIndex
		if (notFound) {
			faceMap.insert(std::pair<int, int>(key, fIndex + 1)); 
			newFaces.push_back(key); 
			newFaces.push_back(faces[i + 1]);
			newFaces.push_back(faces[i + 2]);
			fIndex += 3;
		}
	}

	faces = newFaces;
}

void Octree::getRange()
{
	float deltaX = 0.0, deltaY = 0.0, deltaZ = 0.0;
	for (Vertex v : vertices) {
		glm::vec3 *vert = &(v.Position);

		int t = std::abs(vert->x);
		if (t > deltaX)
			deltaX = t;
		t = std::abs(vert->y);
		if (t > deltaY)
			deltaY = t;
		t = std::abs(vert->z);
		if (t > deltaZ)
			deltaZ = t;
	}

	range = 2.0 * std::max({ deltaX, deltaY, deltaZ });
}

void Octree::oldGetRange()
{
	float maxX = 0, minX = 0;
	float maxY = 0, minY = 0;
	float maxZ = 0, minZ = 0;

	for (Vertex v : vertices) {
		glm::vec3 *vert = &(v.Position);

		if (vert->x > maxX)
			maxX = vert->x;
		if (vert->x < minX)
			minX = vert->x;

		if (vert->y > maxY)
			maxY = vert->y;
		if (vert->y < minY)
			minY = vert->y;

		if (vert->z > maxZ)
			maxZ = vert->z;
		if (vert->z < minZ)
			minZ = vert->z;
		
	}

	range = std::min({ (maxX - minX), (maxY - minY), (maxZ - minZ) });
}

void Octree::addVertex(int index)
{
	root->add(vertices[index].Position, index, vertices);
}

void Octree::addFace(int index) {
	glm::vec3 vertex0, vertex1, vertex2, edge1, edge2, center;

	vertex0 = vertices[faces[index]].Position;
	vertex1 = vertices[faces[index + 1]].Position;
	vertex2 = vertices[faces[index + 2]].Position;

	edge1 = vertex1 - vertex0;
	edge2 = vertex2 - vertex0;

	// make AABB
	float doubleSize, minX, maxX, minY, maxY, minZ, maxZ;
	minX = std::min({ vertex0.x, vertex1.x, vertex2.x });
	maxX = std::max({ vertex0.x, vertex1.x, vertex2.x });
	minY = std::min({ vertex0.y, vertex1.y, vertex2.y });
	maxY = std::max({ vertex0.y, vertex1.y, vertex2.y });
	minZ = std::min({ vertex0.z, vertex1.z, vertex2.z });
	maxZ = std::max({ vertex0.z, vertex1.z, vertex2.z });
	doubleSize = 2.0 * std::max({ maxX - minX, maxY - minY, maxZ - minZ, });
	center.x = (maxX - minX) / 2.0;
	center.y = (maxY - minY) / 2.0;
	center.z = (maxZ - minZ) / 2.0;

	faceRoot->add(new Face(edge1, edge2, vertex0), center, doubleSize);
}

glm::vec3 Octree::getNearest(glm::vec3 p)
{
	Cell leaf = root->find(p);
	float r = leaf.range; 
	glm::vec3 *p1 = NULL;
	glm::vec3 *p2 = NULL;
	float d1 = -1;

	if (leaf.points.size() > 0 && leaf.children.size() < 1) {
		for (int index : leaf.points) {
			glm::vec3 t = vertices[index].Position;
			if (glm::all(glm::equal(t, p))) {
				p1 = &t;
				d1 = glm::distance(t, p);
			}
		}
	}

	while (!p2) {
		r *= 2.0;
		std::vector<int> nearby = root->getPoints(p, r, 0.0, vertices);
		for (int i : nearby) {
			p2 = &vertices[i].Position;
			float d2 = glm::distance(*p2, p);
			if (!p1 || d2 < d1) {
				p1 = p2;
				d1 = d2;
			}
		}
	}

	return *p1;
}

std::vector<int> Octree::findInRadius(glm::vec3 point, float radius, float minD)
{
	return root->getPoints(point, radius, minD, vertices);
}

bool Octree::intersects(glm::vec3 rayOrigin, glm::vec3 rayEnd)
{
	float doubleSize = (2.0 * glm::distance(rayOrigin, rayEnd));
	return faceRoot->intersects(rayOrigin, rayEnd, doubleSize);
}

Node::Node(int index, glm::vec3 pos, glm::vec3 norm) : 
	ID(index), position(pos), normal(norm)
{

}

void Node::addConnection(int index)
{
	connections.push_back(index);
}

Graph::Graph(Model model, float dist) : displacement(dist)
{
	modelRef = &model;
	octree = new Octree(model.meshes);
	nodes.reserve(octree->vertices.size());

	int i = 0;
	for (Vertex &v : octree->vertices) {
		glm::vec3 pos = v.Position;
		glm::vec3 norm = v.Normal;
		pos += (norm * dist);
		nodes.push_back(new Node(i, pos, norm));
		i++;
	}

	// Populate every node with its connections:
	std::vector<unsigned int> faces = octree->faces;
	std::vector<std::unordered_set<int>> connections;
	connections.assign(nodes.size(), std::unordered_set<int>());
	int size = faces.size();
	for (int i = 0; i < size; i += 3) {
		int a = faces[i];
		int b = faces[i + 1];
		int c = faces[i + 2];

		connections[a].insert(b);
		connections[a].insert(c);

		connections[b].insert(a);
		connections[b].insert(c);

		connections[c].insert(a);
		connections[c].insert(b);
	}

	for (int n = 0; n < nodes.size(); n++) {
		for (int i : connections[n]) {
			nodes[n]->addConnection(i);
		}
	}
}

Graph::Graph(std::vector<Node*> newNodes, float dist) : 
	nodes(newNodes), displacement(dist)
{
	octree = NULL; // TODO: build octree
	modelRef = NULL;
}

Graph::~Graph()
{
	delete(octree);
}

int Graph::CombineNodes(int n1, int n2)
{
	// TODO: fix this- vertex position isn't great
	glm::vec3 pos = (nodes[n1]->position + nodes[n2]->position);
	pos *= 0.5;
	glm::vec3 norm = (nodes[n1]->normal + nodes[n2]->normal);
	norm *= 0.5;
	float distance;

	glm::intersectRayPlane(pos, norm, nodes[n1]->position, nodes[n1]->normal, distance);
	if (distance > 0.0) {
		pos += norm * distance;
	}
	
	int index = nodes.size();
	Node *node = new Node(index, pos, norm);
	nodes.push_back(node);

	std::unordered_set<int> connections;
	for (int i : nodes[n1]->connections) {
		if (nodes[i] && i != n2) {
			connections.insert(i);
		}
	}
	for (int i : nodes[n2]->connections) {
		if (nodes[i] && i != n1) {
			connections.insert(i);
		}
	}
	for (int i : connections) {
		node->connections.push_back(i);

		// rebuild all connections from attached nodes
		Node *c = nodes[i];
		for (int j = 0; c->connections.size(); j++) {
			int cIndex = c->connections[j];
			if (cIndex == n1 || cIndex == n2) {
				c->connections[j] = index;
				break; // break early
			}
		}
	}

	delete(nodes[n1]);
	nodes[n1] = NULL;
	delete(nodes[n2]);
	nodes[n2] = NULL;
	return index;
}

Graph* Graph::ReduceFootprint()
{
	std::vector<Node*> newNodes;
	std::vector<int> translate; // on the theory that this is faster than a hashtable
	translate.reserve(nodes.size());
	int index = 0;

	for (Node *n : nodes) {
		if (n) {
			newNodes.push_back(n);
			translate.push_back(index);
			index++;
		} else {
			translate.push_back(-1);
		}
	}

	cleanConnections(newNodes, translate);
	return new Graph(newNodes, displacement);
}

void Graph::cleanConnections(std::vector<Node*> &nodeList, std::vector<int> &translate) 
{
	std::vector<std::unordered_set<int>> connections;
	connections.assign(nodeList.size(), std::unordered_set<int>());
	for (int n = 0; n < nodeList.size(); n++) {
		for (int c : nodeList[n]->connections) {
			if (nodes[c]) {
				connections[n].insert(c);
				connections[translate[c]].insert(n);
			}
		}
	}

	for (int n = 0; n < nodeList.size(); n++) {
		Node *node = new Node(n, nodeList[n]->position, nodeList[n]->normal);

		for (int i : connections[n]) {
			if (nodes[i])
				node->addConnection(translate[i]);
		}

		nodeList[n] = node;
	}
}
