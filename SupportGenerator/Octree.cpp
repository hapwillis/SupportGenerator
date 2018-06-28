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

Node::Node(int index, Vertex v) :
	ID(index), vertex(v)
{

}

Node::Node(int index, glm::vec3 pos, glm::vec3 norm, bool wireframe) :
	ID(index)
{
	vertex = Vertex(pos, norm, wireframe);
}

void Node::addConnection(int index)
{
	connections.push_back(index);
}

void Node::addFace(int face)
{
	faces.insert(face);
}

Face::Face(glm::vec3 e1, glm::vec3 e2, glm::vec3 v) : edgeOne(e1), edgeTwo(e2), vertex(v)
{
	normal = glm::cross(glm::normalize(e1), glm::normalize(e2));
}

Face::Face(int v1, int v2, int v3, const std::vector<Node*> &nodes) : v1(v1), v2(v2), v3(v3)
{
	edgeOne = nodes[v2]->vertex.Position - nodes[v1]->vertex.Position;
	edgeTwo = nodes[v3]->vertex.Position - nodes[v1]->vertex.Position;
	vertex = nodes[v1]->vertex.Position;

	normal = glm::cross(glm::normalize(edgeOne), glm::normalize(edgeTwo));
}

void Face::update(const std::vector<Node*> &nodes)
{
	edgeOne = nodes[v2]->vertex.Position - nodes[v1]->vertex.Position;
	edgeTwo = nodes[v3]->vertex.Position - nodes[v1]->vertex.Position;
	vertex = nodes[v1]->vertex.Position;

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

std::vector<int> Octant::getPoints(glm::vec3 point, float radius, float minD, std::vector<Vertex*> &vertices)
{
	if (range > radius * 4) {
		return children[findOctant(point)]->getPoints(point, radius, minD, vertices);
	} else {
		if (filled) {
			// optionally you could eliminate children here, but that probably just slows it down.
			std::vector<int> verts;
			for (Octant *c : children) {
				std::vector<int> v = c->getPoints(point, radius, minD, vertices);
				verts.insert(verts.end(), v.begin(), v.end());
			}

			return verts;
		} else {
			std::vector<int> returnList;

			for (int index : points) {
				float d = glm::distance(vertices[index]->Position, point);
				if (d <  radius && d > minD)
					returnList.push_back(index);
			}

			return returnList;
		}
	}
}

int Octant::findOctant(glm::vec3 p)
{
	int i = 0;
	if (p.x >= center.x) { i = i + 1; }
	if (p.y >= center.y) { i = i + 2; }
	if (p.z >= center.z) { i = i + 4; }
	return i;
}

void Octant::add(glm::vec3 p, int pointIndex, std::vector<Vertex*> &vertices)
{
	// TODO: return early for identical verts
	// build on top of root if position > range
	if (!filled) {
		if (points.size() < maxChildren) {
			points.push_back(pointIndex);
		} else {
			points.push_back(pointIndex);
			filled = true;
			split(vertices);
		}
	} else {
		Octant *cell = children[findOctant(p)];
		while (cell->filled) {
			cell = cell->children[cell->findOctant(p)];
		}
		
		cell->add(p, pointIndex, vertices);
	}
}

void Octant::split(std::vector<Vertex*> &vertices)
{
	children.reserve(8);
	PopulateChildren(children, center, range / 2.0);

	for (int p : points) {
		glm::vec3 pos = vertices[p]->Position;
		children[findOctant(pos)]->add(pos, p, vertices);
	}

	points.clear();
}

bool Octant::pointsNotEqual(glm::vec3 p, glm::vec3 q)
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

void Octant::PopulateChildren(std::vector<Octant*> &children, glm::vec3 center, float r)
{
	// TODO: lazy initialization
	float offset = r / 2.0;
	glm::vec3 c = center;
	c -= glm::vec3(offset, offset, offset);
	children.push_back(new Octant(c, r));
	c.x += r;
	children.push_back(new Octant(c, r));
	c.x -= r;
	c.y += r;
	children.push_back(new Octant(c, r));
	c.x += r;
	children.push_back(new Octant(c, r));

	c.x -= r;
	c.y -= r;
	c.z += r;
	children.push_back(new Octant(c, r));
	c.x += r;
	children.push_back(new Octant(c, r));
	c.x -= r;
	c.y += r;
	children.push_back(new Octant(c, r));
	c.x += r;
	children.push_back(new Octant(c, r));
}

Octant Octant::find(glm::vec3 p)
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

	return *(new Octant(center, range));
}

Octant::Octant(glm::vec3 c, float r) : center(c), range(r), filled(false)
{

}

Octant::~Octant()
{
	for (Octant *c : children) {
		delete c;
	}
}

Octree::Octree(const std::vector<Node*> *nodes, std::vector<Face*> *aFaces, float range) :
	Range(range)
{
	root = new Octant(glm::vec3(0.0, 0.0, 0.0), range);
	faceRoot = new FaceCell(glm::vec3(0.0, 0.0, 0.0), range);
	vertices.reserve(nodes->size());
	for (Node *n : *nodes) {
		vertices.push_back(&n->vertex);
	}

	float time = glfwGetTime();
	//0.0284 seconds to build vertex octree
	for (int i = 0; i < vertices.size(); i++) {
		addVertex(i);
	}
	std::cout << "Time to insert vertices: " << glfwGetTime() - time << std::endl;

	time = glfwGetTime();
	for (int i = 0; i < (*faces).size(); i += 3) {
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

void Octree::addVertex(int index)
{
	// TODO: return early for identical verts
	// build on top of root if position > range
	root->add(vertices[index]->Position, index, vertices);
}

void Octree::addFace(int index) {
	glm::vec3 vertex0, vertex1, vertex2, edge1, edge2, center;

	vertex0 = vertices[(*faces)[index]]->Position;
	vertex1 = vertices[(*faces)[index + 1]]->Position;
	vertex2 = vertices[(*faces)[index + 2]]->Position;

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

glm::vec3 Octree::getNearestPoint(glm::vec3 p)
{
	Octant leaf = root->find(p);
	float r = leaf.range; 
	glm::vec3 *p1 = NULL;
	glm::vec3 *p2 = NULL;
	float d1 = -1;

	if (leaf.points.size() > 0 && leaf.children.size() < 1) {
		for (int index : leaf.points) {
			glm::vec3 t = vertices[index]->Position;
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
			p2 = &vertices[i]->Position;
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

