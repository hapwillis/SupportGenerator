#include "Octree.h"

void ordered_set::insert(int i) {
	if (set.count(i) == 0) {
		set.insert(i);
		vector.push_back(i);
	}
}

int ordered_set::size()
{
	return vector.size();
}

int ordered_set::popFromSet()
{
	for (int t : vector) {
		if (set.count(t) == 1) {
			set.erase(t);
			return t;
		}
	}
	return -1;
}

int ordered_set::pop()
{
	int t = vector[0];
	set.erase(t);
	return t;
}

Face::Face(glm::vec3 e1, glm::vec3 e2, glm::vec3 v) : edgeOne(e1), edgeTwo(e2), vertex(v)
{
	normal = glm::cross(glm::normalize(e1), glm::normalize(e2));
}

Face::Face(int v1, int v2, int v3, std::vector<Vertex> &vertices) : v1(v1), v2(v2), v3(v3)
{
	edgeOne = vertices[v2].Position - vertices[v1].Position;
	edgeTwo = vertices[v3].Position - vertices[v1].Position;
	vertex = vertices[v1].Position;

	normal = glm::cross(glm::normalize(edgeOne), glm::normalize(edgeTwo));
}

void Face::update(const std::vector<Node*> &nodes)
{
	edgeOne = nodes[v2]->position - nodes[v1]->position;
	edgeTwo = nodes[v3]->position - nodes[v1]->position;
	vertex = nodes[v1]->position;

	normal = glm::cross(glm::normalize(edgeOne), glm::normalize(edgeTwo));
}

bool Face::MollerTrumbore(glm::vec3 rayOrigin, glm::vec3 rayEnd)
{
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
	//0.0284 seconds to build vertex octree
	for (int i = 0; i < vertices.size(); i++) {
		addVertex(i);
	}
	std::cout << "Time to insert vertices: " << glfwGetTime() - time << std::endl;

	time = glfwGetTime();
	for (int i = 0; i < faces.size(); i += 3) {
		//TODO: add faces
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
				// if found, set translate[tIndex] = value
				if (pointsEqual(v.Position, vertices[index].Position)) {
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

bool Octree::pointsEqual(glm::vec3 p, glm::vec3 q)
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

	return (a && b && c);
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

			if ((faces[i + 1] == newFaces[index]) && (faces[i + 2] == newFaces[index + 1])) {
				notFound = false;
				break;
			}
		}

		//if not found, insert this face
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

void Node::addFace(int face)
{
	faces.insert(face);
}

void Node::updateNormal(glm::vec3 n) {
	normal = n;
}

Graph::Graph(Model model)
{
	modelRef = &model;
	octree = new Octree(model.meshes);
	nodes.reserve(octree->vertices.size());

	int i = 0;
	for (Vertex &v : octree->vertices) {
		nodes.push_back(new Node(i, v.Position, v.Normal));
		i++;
	}

	// Populate every node with its faces and connections:
	std::vector<ordered_set> connections;
	populateFaces();
	populateConnections(connections);

	float time = glfwGetTime();
	//orderConnections(connections);
	addConnections(connections);
	std::cout << "Time to order connections: " << glfwGetTime() - time << std::endl;

	std::vector<glm::vec3> normals;
	for (Node *n : nodes) {
		normals.push_back(n->normal);
	}

	time = glfwGetTime();
	//recalculateNormals();
	//recalculateNormalsFromFaces();
	std::cout << "Time to recalculate normals: " << glfwGetTime() - time << std::endl;

	int invalidNormals = 0;
	for (Node *n : nodes) {
		if (glm::length(n->normal) > 1.0) {
			invalidNormals++;
		}
		else {
			//if (glm::angle(normals[]))
		}
	}
	std::cout << "Unusual normals: " << invalidNormals << std::endl;
}

Graph::Graph(std::vector<Node*> newNodes, std::vector<Face*> faces) 
	: nodes(newNodes), faceVector(faces)
{
	octree = NULL; // TODO: build octree
	modelRef = NULL;
}

Graph::~Graph()
{
	delete(octree);
	for (Face *f : faceVector) {
		//delete(f);
	}
}

void Graph::populateConnections(std::vector<ordered_set> &connections) 
{
	std::vector<unsigned int> faces = octree->faces;
	connections.assign(nodes.size(), ordered_set());
	int size = faces.size();
	for (int i = 0; i < size; i += 3) {
		int a = faces[i];
		int b = faces[i + 1];
		int c = faces[i + 2];

		connections[a].insert(b);
		connections[a].insert(c);

		// this preserves winding order in the first two connections-
		// required to rebuild normals.
		connections[b].insert(c);
		connections[b].insert(a);

		connections[c].insert(b);
		connections[c].insert(a);
	}
}

void Graph::populateFaces() 
{
	std::vector<unsigned int> faces = octree->faces;
	faceVector.reserve(std::ceil(faces.size() / 3.0));

	for (int i = 0; i < faces.size(); i += 3) {
		faceVector.push_back(new Face(faces[i], faces[i + 1], faces[i + 2], octree->vertices));
	}

	std::cout << "number of faces: " << faceVector.size() << std::endl;
	std::cout << "number of vertices: " << nodes.size() << std::endl;

	for (int face = 0; face < faceVector.size(); face++) {
		nodes[faceVector[face]->v1]->addFace(face);
		nodes[faceVector[face]->v2]->addFace(face);
		nodes[faceVector[face]->v3]->addFace(face);
	}
}

void Graph::addConnections(std::vector<ordered_set> connections)
{
	for (int n = 0; n < nodes.size(); n++) {
		for (int i : connections[n].vector) {
			if (n != i)
				nodes[n]->addConnection(i);
		}
	}
}

void Graph::orderConnections(std::vector<ordered_set> connections)
{
	int brokenVerts = 0;

	for (int n = 0; n < nodes.size(); n++) {
		std::vector<int> nodeConnections;
		nodeConnections.reserve(connections[n].size());
		// we know the first two connections should be correctly ordered:
		nodeConnections.push_back(connections[n].popFromSet());
		nodeConnections.push_back(connections[n].popFromSet());
		bool brokenVert = false;

		while (!connections[n].set.empty()) {
			int last = nodeConnections.back();
			for (int i : connections[nodeConnections.back()].vector) {
				if (connections[n].set.count(i) == 1) {
					nodeConnections.push_back(i);
					connections[n].set.erase(i);
					break;
				}
			}
			if (last == nodeConnections.back()) {
				brokenVerts++;
				brokenVert = true;
				break;
			}
				
		}

		if (brokenVert) {
			for (int i : connections[n].vector) {
				nodes[n]->addConnection(i);
			}
		}
		else {
			for (int i : nodeConnections) {
				nodes[n]->addConnection(i);
			}
		}
	}

	std::cout << "Unable to rebuild " << brokenVerts << " connections." << std::endl;
}

void Graph::recalculateNormals()
{
	// recalculate face-weighted normals
	for (int n = 0; n < nodes.size(); n++) {
		Node *node = nodes[n];
		glm::vec3 p = node->position;
		int last = node->connections.size() - 1;
		glm::vec3 e1 = nodes[node->connections[last]]->position - p;
		glm::vec3 e2 = nodes[node->connections[0]]->position - p;
		glm::vec3 faceNormal = glm::normalize(glm::cross(e1, e2));

		for (int i = 0; i < node->connections.size() - 1; i++) {
			e1 = nodes[node->connections[i]]->position - p;
			e2 = nodes[node->connections[i + 1]]->position - p;
			faceNormal += glm::normalize(glm::cross(e1, e2));
		}

		faceNormal = glm::normalize(faceNormal);
		if (glm::dot(nodes[n]->normal, faceNormal) > 0) {
			nodes[n]->normal = faceNormal;
		} else {
			nodes[n]->normal = -faceNormal;
		}
	}
}

void Graph::recalculateNormalsFromFaces()
{
	// this generates invalid normals, which are then not rendered.  Reason unknown.
	for (int n = 0; n < nodes.size(); n++) {
		//glm::vec3 newNormal = glm::vec3(0.0f, 0.0f, 0.0f);
		std::vector<int> tvect;
		for (int face : nodes[n]->faces) {
			tvect.push_back(face);
			//newNormal += faceVector[face]->normal;
		}
		glm::vec3 tNormal = faceVector[tvect[0]]->normal;
		for (int i = 1; i < tvect.size(); i++) {
			tNormal = tNormal + glm::normalize(faceVector[tvect[i]]->normal);
		}
		glm::vec3 newNormal = glm::normalize(tNormal);

		nodes[n]->updateNormal(newNormal);
	}
}

void Graph::recalculateNormalsFaceWeight()
{
	// recalculate face-weighted normals
	for (int n = 0; n < nodes.size(); n++) {
		Node *node = nodes[n];
		glm::vec3 p = node->position;
		int last = node->connections.size() - 1;
		glm::vec3 e1 = nodes[node->connections[last]]->position - p;
		glm::vec3 e2 = nodes[node->connections[0]]->position - p;
		glm::vec3 faceNormal = glm::cross(e1, e2);

		for (int i = 0; i < node->connections.size() - 1; i++) {
			e1 = nodes[node->connections[i]]->position - p;
			e2 = nodes[node->connections[i + 1]]->position - p;
			faceNormal += glm::cross(e1, e2);
		}

		faceNormal = glm::normalize(faceNormal);
		if (glm::dot(nodes[n]->normal, faceNormal) > 0) {
			nodes[n]->normal = faceNormal;
		}
		else {
			nodes[n]->normal = -faceNormal;
		}
	}
}

void Graph::scale(float displacement)
{
	for (Node *n : nodes) {
		// TODO: 
		//call relocateVert() with offset, newPos, newNormal, and list of faces

		// float dist = 0;
		//for each associated face
			// find intersect of face plane and vertex normal
			// dist = std::max(dist, intersection);
		glm::vec3 pos = n->position;
		glm::vec3 norm = n->normal;
		n->position = pos + (norm * displacement);
	}
}

int Graph::CombineNodes(int n1, int n2)
{
	//change to relocateVert
	Node *node = nodeFromAverage(nodes[n1], nodes[n2]);
	nodes.push_back(node);

	CombineConnections(n1, n2, node);
	CombineFaces(n1, n2, node);

	delete(nodes[n1]);
	nodes[n1] = NULL;
	delete(nodes[n2]);
	nodes[n2] = NULL;
	return node->ID;
}

void Graph::CombineConnections(int n1, int n2, Node *node) 
{
	int index = node->ID;
	ordered_set connections;
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
	for (int i : connections.vector) {
		node->connections.push_back(i);

		// rebuild all connections from attached nodes
		Node *c = nodes[i];
		for (int j = 0; c->connections.size(); j++) {
			int cIndex = c->connections[j];
			if (cIndex == n1 || cIndex == n2) {
				c->connections[j] = index;
				break;
			}
		}
	}
}

void Graph::CombineFaces(int n1, int n2, Node *node) 
{
	// delete faces that belong to both nodes
	int index = node->ID;
	for (int i : nodes[n1]->faces) {
		if (faceVector[i]) {
			node->faces.insert(i);
		}
	}
	for (int i : nodes[n2]->faces) {
		if (faceVector[i]) {
			if (node->faces.count(i) == 1) {
				node->faces.erase(i);
				delete(faceVector[i]);
				faceVector[i] = NULL;
			} else {
				node->faces.insert(i);
			}
		}
	}

	// update all other faces to new values
	for (int i : node->faces) {
		int v1 = faceVector[i]->v1;
		int v2 = faceVector[i]->v2;
		int v3 = faceVector[i]->v3;

		if (v1 == n1 || v1 == n2)
			faceVector[i]->v1 = index;
		if (v2 == n1 || v2 == n2)
			faceVector[i]->v2 = index;
		if (v3 == n1 || v3 == n2)
			faceVector[i]->v3 = index;

		faceVector[i]->update(nodes);
	}
}

Node* Graph::nodeFromAverage(Node* n1, Node* n2)
{
	glm::vec3 norm = glm::normalize(n1->normal + n2->normal);
	glm::vec3 pos = (n1->position + n2->position) * 0.5f;

	return new Node(nodes.size(), pos, norm);
}

Node* Graph::nodeFromPreservedEdges(Node* n1, Node* n2)
{
	glm::vec3 norm = glm::normalize(n1->normal + n2->normal);
	float dot1 = glm::dot(n1->normal, norm);
	float dot2 = glm::dot(n2->normal, norm);

	if (dot1 > dot2)
		return new Node(nodes.size(), n1->position, norm);

	return new Node(nodes.size(), n2->position, norm);
}

Node* Graph::nodeFromIntercept(Node* n1, Node* n2)
{
	// fix this- vertex position isn't great
	glm::vec3 pos = (n1->position + n2->position);
	pos *= 0.5;
	glm::vec3 norm = glm::normalize(n1->normal + n2->normal);
	float distance;

	glm::intersectRayPlane(pos, norm, n1->position, n1->normal, distance);
	if (distance > 0.0) {
		pos += norm * distance;
	}

	return new Node(nodes.size(), pos, norm);
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

	std::vector<Face*> newFaces = cleanFaces(newNodes, translate);
	cleanConnections(newNodes, translate);
	return new Graph(newNodes, newFaces);
}

void Graph::cleanConnections(std::vector<Node*> &nodeList, std::vector<int> &translate) 
{
	std::vector<ordered_set> connections;
	connections.assign(nodeList.size(), ordered_set());
	for (int n = 0; n < nodeList.size(); n++) {
		for (int c : nodeList[n]->connections) {
			if (nodes[c]) {
				connections[n].insert(translate[c]);
				connections[translate[c]].insert(n);
			}
		}
	}

	for (int n = 0; n < nodeList.size(); n++) {
		Node *node = new Node(n, nodeList[n]->position, nodeList[n]->normal);
		for (int f : nodeList[n]->faces) {
			node->faces.insert(f);
		}

		for (int i : connections[n].vector) {
			if (nodeList[i])
				node->addConnection(i);
		}

		nodeList[n] = node;
	}
}

std::vector<Face*> Graph::cleanFaces(std::vector<Node*> &nodeList, std::vector<int> &translate)
{
	std::vector<Face*> faceList;
	std::vector<int> faceTran;
	faceTran.reserve(faceList.size());

	int index = 0;
	for (Face *f : faceVector) {
		if (f) {
			faceTran.push_back(index);
			f->v1 = translate[f->v1];
			f->v2 = translate[f->v2];
			f->v3 = translate[f->v3];
			faceList.push_back(f);
			index++;
		} else {
			faceTran.push_back(-1);
		}
	}

	int invalid = 0;
	for (Node* node : nodeList) {
		// Push the faces
		std::unordered_set<int> newFaces;
		for (int f: node->faces) {
			int tran = faceTran[f];
			if (tran != -1) 
				newFaces.insert(tran);
		}
		node->faces = std::unordered_set<int>();

		for (int f : newFaces) {
			node->faces.insert(f);
		}
	}

	return faceList;
}

bool Graph::verifyFacesFromConnections(int node)
{
	if (nodes[node]) {
		std::unordered_set<int> connectionsFromFaces;
		std::unordered_set<int> uniqueConnections;

		for (int face : nodes[node]->faces) {
			if (face != -1 && faceVector[face]) {
				int facecon = faceVector[face]->v1;
				if (nodes[facecon])
					connectionsFromFaces.insert(facecon);
				facecon = faceVector[face]->v2;
				if (nodes[facecon])
					connectionsFromFaces.insert(facecon);
				facecon = faceVector[face]->v3;
				if (nodes[facecon])
					connectionsFromFaces.insert(facecon);
			}
		}
		connectionsFromFaces.erase(nodes[node]->ID);

		for (int c : nodes[node]->connections) {
			if(nodes[c])
				uniqueConnections.insert(c);
		}

		for (int c : uniqueConnections) {
			if (connectionsFromFaces.count(c) == 1 || c == node) {
				connectionsFromFaces.erase(c);
			}
			else {
				std::vector<Node*> nodeConnections;
				std::vector<Face*> faceConnections;
				std::vector<Node*> relatedNodes;

				for (int nc: nodes[node]->connections) {
					nodeConnections.push_back(nodes[nc]);
					//relatedNodes.push_back(nodes[nc]);
				}
				for (int nf : nodes[node]->faces) {
					faceConnections.push_back(faceVector[nf]);
					relatedNodes.push_back(nodes[faceVector[nf]->v1]);
					relatedNodes.push_back(nodes[faceVector[nf]->v2]);
					relatedNodes.push_back(nodes[faceVector[nf]->v3]);
				}

				if (nodes[c])

					return false;
			}
		}

		return connectionsFromFaces.size() == 0;
	}
	
	return true;
}