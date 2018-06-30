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

Face::Face(int v1, int v2, int v3, const std::vector<Node*> &nodes) : index1(v1), index2(v2), index3(v3)
{
	edge21 = nodes[index2]->vertex.Position - nodes[index1]->vertex.Position;
	edge31 = nodes[index3]->vertex.Position - nodes[index1]->vertex.Position;
	edge32 = nodes[index3]->vertex.Position - nodes[index2]->vertex.Position;
	vertex1 = nodes[index1]->vertex.Position;
	vertex2 = nodes[index2]->vertex.Position;
	vertex3 = nodes[index3]->vertex.Position;

	normal = glm::normalize(glm::cross(edge21, edge31));
	edge21Normal = glm::normalize(glm::cross(edge21, normal));
	edge32Normal = glm::normalize(glm::cross(edge32, normal));
	edge13Normal = glm::normalize(glm::cross(-edge31, normal));
}

void Face::update(const std::vector<Node*> &nodes)
{
	edge21 = nodes[index2]->vertex.Position - nodes[index1]->vertex.Position;
	edge31 = nodes[index3]->vertex.Position - nodes[index1]->vertex.Position;
	edge32 = nodes[index3]->vertex.Position - nodes[index2]->vertex.Position;
	vertex1 = nodes[index1]->vertex.Position;
	vertex2 = nodes[index2]->vertex.Position;
	vertex3 = nodes[index3]->vertex.Position;

	normal = glm::normalize(glm::cross(edge21, edge31));
	edge21Normal = glm::normalize(glm::cross(edge21, normal));
	edge32Normal = glm::normalize(glm::cross(edge32, normal));
	edge13Normal = glm::normalize(glm::cross(-edge31, normal));
}

bool Face::MollerTrumbore(glm::vec3 rayOrigin, glm::vec3 rayEnd, glm::vec3 **intersectionPtr)
{
	glm::vec3 h, s, q;
	float a, f, u, v;
	// optimization potential: provide direction as an arg instead of implicitly
	glm::vec3 rayVector = glm::normalize(rayEnd - rayOrigin);
	h = glm::cross(rayVector, edge31);
	a = glm::dot(edge21, h);
	if (a > -EPSILON && a < EPSILON)
		return false;
	f = 1 / a;
	s = rayOrigin - vertex1;
	u = f * (glm::dot(s, h));
	if (u < 0.0 || u > 1.0)
		return false;
	q = glm::cross(s, edge21);
	v = f * glm::dot(rayVector, q);
	if (v < 0.0 || u + v > 1.0)
		return false;
	// At this stage we can compute t to find out where the intersection point is on the line.
	float t = f * glm::dot(edge31, q);
	if (t > EPSILON) // ray intersection
	{
		glm::vec3 inter = rayOrigin + rayVector * t;
		*intersectionPtr = new glm::vec3(inter);
		return true;
	}
	else // This means that there is a line intersection but not a ray intersection.
		return false;
}

float Face::faceDistanceSquared(glm::vec3 point)
{
	glm::vec3 vecP1 = point - vertex1;
	glm::vec3 vecP2 = point - vertex2;
	glm::vec3 vecP3 = point - vertex3;

	//first test if point is inside the prism of the normal-projected face
	bool insidePrism =	(glm::dot(edge21Normal, vecP1) > 0.0f) &&
						(glm::dot(edge32Normal, vecP2) > 0.0f) &&
						(glm::dot(edge13Normal, vecP3) > 0.0f);
	if (insidePrism) {
		// if yes, return distance to face normal
		float d = glm::dot(normal, vecP1);
		return d * d;
	} 

	// if no, return min (distance from edge)
	glm::vec3 proj21 =	edge21 * std::clamp(glm::dot(edge21, vecP1) /
						glm::dot(edge21, edge21), 0.0f, 1.0f) - vecP1;
	glm::vec3 proj32 =	edge32 * std::clamp(glm::dot(edge32, vecP2) /
						glm::dot(edge32, edge32), 0.0f, 1.0f) - vecP2;
	glm::vec3 proj13 =	-edge31 * std::clamp(glm::dot(-edge31, vecP3) /
						glm::dot(edge31, edge31), 0.0f, 1.0f) - vecP3;

	return std::min({ glm::dot(proj21, proj21),  glm::dot(proj32, proj32),  glm::dot (proj13, proj13)});
}

FaceCell::FaceCell(glm::vec3 c, float r) : center(c), range(r)
{
	FaceCell *nullpoint = { 0 };
	children.assign(8, nullpoint);
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

bool FaceCell::intersects(glm::vec3 rayOrigin, glm::vec3 rayEnd, float doubleSize, glm::vec3 **intersectionPtr)
{
	// First, test all faces in this FaceCell
	for (Face *f : faces) {
		if (f && f->MollerTrumbore(rayOrigin, rayEnd, intersectionPtr))
			return true;
	}

	//if smaller than a child, test if it fits into a single octant
	if (doubleSize < range) {
		glm::vec3 p1 = glm::greaterThanEqual(rayOrigin, center);
		glm::vec3 p2 = glm::greaterThanEqual(rayEnd, center);

		if (glm::all(glm::equal(p1, p2))) {
			FaceCell *c = children[findOctant(rayOrigin)];
			if (c) {
				return c->intersects(rayOrigin, rayEnd, doubleSize, intersectionPtr);
			} else {
				return false;
			}
		}
	} else { //just test all faces in the region
		for (FaceCell *c : children) {
			if (c && c->intersects(rayOrigin, rayEnd, doubleSize, intersectionPtr))
				return true;
		}
	}

	return false;
}

float FaceCell::closestFace(glm::vec3 point, Face** resultSquared)
{
	Face *best = NULL;
	float bestDist = -1.0f;


	FaceCell *bestChild = children[findOctant(point)];
	if (bestChild)
		bestDist = bestChild->closestFace(point, &best);

	for (Face *face : faces) {
		float dist = face->faceDistanceSquared(point);
		if (bestDist < 0.0f || (dist < bestDist && dist > 0.0f)) {
			bestDist = dist;
			best = face;
		}
	}

	// search other octants
	if (bestDist < 0.0f) {
		for (FaceCell *octant : children) {
			if (octant && octant != bestChild) {
				Face *face = NULL;
				float dist = octant->closestFace(point, &face);

				if (bestDist < 0.0f || (dist < bestDist && dist > 0.0f)) {
					bestDist = dist;
					best = face;
				}
			}
		}
	}

	*resultSquared = best;
	return bestDist;
}

int FaceCell::findOctant(glm::vec3 p)
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
	int child = findOctant(faceCenter);
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
			break;
		case 1: c += glm::vec3(o, -o, -o);
			break;
		case 2: c += glm::vec3(-o, o, -o);
			break;
		case 3: c += glm::vec3(o, o, -o);
			break;

		case 4: c += glm::vec3(-o, -o, o);
			break;
		case 5: c += glm::vec3(o, -o, o);
			break;
		case 6: c += glm::vec3(-o, o, o);
			break;
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

bool Octant::add(glm::vec3 p, int pointIndex, std::vector<Vertex*> &vertices)
{
	if (!filled) {
		for (int i : points) {
			if (i == pointIndex || pointsNotEqual(vertices[i]->Position, p))
				return false;
		}

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
		
		return cell->add(p, pointIndex, vertices);
	}

	return true;
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
	
	return !((a && b) && c);
}

void Octant::PopulateChildren(std::vector<Octant*> &children, glm::vec3 center, float r)
{
	// can save time and space by converting this to lazy initialization (like FaceCell)
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
	if (children.size() > 0) {
		int child = findOctant(p);
		if (children[child]) {
			if (children[child]->filled) {
				if (children[child]->children.size() < 1) {
					return *children[child];
				}
				return children[child]->find(p);
			}
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
	faces(aFaces), Range(range)
{
	root = new Octant(glm::vec3(0.0, 0.0, 0.0), range);
	faceRoot = new FaceCell(glm::vec3(0.0, 0.0, 0.0), range);
	vertices.reserve(nodes->size());
	for (Node *n : *nodes) {
		vertices.push_back(&n->vertex);
	}

	//float time = glfwGetTime();
	//0.0284 seconds to build vertex octree
	for (int i = 0; i < vertices.size(); i++) {
		addVertex(i);
	}
	//std::cout << "Time to insert vertices: " << glfwGetTime() - time << std::endl;

	//time = glfwGetTime();
	for (Face *face : *aFaces) {
		addFace(face);
	}
	//std::cout << "Time to insert faces: " << glfwGetTime() - time << std::endl;
}

Octree::Octree()
{
	Range = 20.0f;
	root = new Octant(glm::vec3(0.0, 0.0, 0.0), Range);
	faceRoot = new FaceCell(glm::vec3(0.0, 0.0, 0.0), Range);
	vertices.reserve(100);
}

Octree::~Octree()
{
	for (int i : destroyList) {
		delete vertices[i];
	}
	delete root;
	delete faceRoot;
}

void Octree::addPoint(glm::vec3 p)
{
	int index = vertices.size();
	//vertices.emplace_back(p, glm::vec3(0.0, 0.0, 0.0), true);
	vertices.push_back(new Vertex(p, glm::vec3(0.0, 0.0, 0.0), true));
	addVertex(index);

	//For now this is less complicated than worrying about smart pointers.
	destroyList.push_back(index);
}

void Octree::addVertex(int index)
{
	// TODO: block from adding nodes without connections, pass nodes/vertices instead of indices
	glm::vec3 pos = vertices[index]->Position;
	float newRange = 2.0 * std::max({ pos.x, pos.y, pos.z });
	while (root->range < newRange) {
		glm::vec3 center = root->center;
		float oldRange = root->range;
		Octant *child = root;
		root = new Octant(center, oldRange * 2.0);
		root->PopulateChildren(root->children, center, oldRange);

		for (Octant *o : root->children) {
			o->PopulateChildren(o->children, center, oldRange / 2);
		}

		for (Octant *o : child->children) {
			Octant *toFill = root->children[root->findOctant(o->center)];
			Octant *toReplace = toFill->children[toFill->findOctant(o->center)];

			delete toReplace;
			toReplace = o;
		}

		Range *= 2.0;
	}

	root->add(pos, index, vertices);
}

void Octree::addFace(Face *face) {
	glm::vec3 vertex0, vertex1, vertex2, center;

	vertex0 = face->vertex1; 
	vertex1 = face->vertex2;
	vertex2 = face->vertex3;

	// make AABB
	float doubleSize, minX, maxX, minY, maxY, minZ, maxZ;
	minX = std::min({ vertex0.x, vertex1.x, vertex2.x });
	maxX = std::max({ vertex0.x, vertex1.x, vertex2.x });
	minY = std::min({ vertex0.y, vertex1.y, vertex2.y });
	maxY = std::max({ vertex0.y, vertex1.y, vertex2.y });
	minZ = std::min({ vertex0.z, vertex1.z, vertex2.z });
	maxZ = std::max({ vertex0.z, vertex1.z, vertex2.z });
	doubleSize = 2.0 * std::max({ maxX - minX, maxY - minY, maxZ - minZ, });
	center.x = (maxX + minX) / 2.0f;
	center.y = (maxY + minY) / 2.0f;
	center.z = (maxZ + minZ) / 2.0f;

	while (doubleSize > faceRoot->range) {
		enlargeFaceRoot();
	}

	if (doubleSize > face->EPSILON) {
		faceRoot->add(face, center, doubleSize);
	} else {
		//std::cout << "Invalid faces detected!" << std::endl;
	}
}

void Octree::enlargeFaceRoot()
{
	glm::vec3 center = faceRoot->center;
	float oldRange = faceRoot->range;
	FaceCell *child = faceRoot;
	faceRoot = new FaceCell(center, oldRange * 2.0);

	for (int i = 0; i < 8; i++) {
		faceRoot->children[i] = faceRoot->populateChild(i);
	}

	for (int i = 0; i < 8; i++) {
		FaceCell *o = child->children[i];
		if (o) {
			FaceCell *toFill = faceRoot->children[i];
			toFill->children[toFill->findOctant(o->center)] = o;
		}
	}

	for (Face *face : child->faces) {
		addFace(face);
	}
}

glm::vec3 Octree::getNearestPoint(glm::vec3 p)
{
	return vertices[getNearestNodeIndex(p)]->Position;
}

int Octree::getNearestNodeIndex(glm::vec3 p)
{
	Octant leaf = root->find(p);
	float r = leaf.range;
	glm::vec3 *p1 = NULL;
	glm::vec3 *p2 = NULL;
	float d1 = -1;
	int nearestIndex;

	if (leaf.points.size() > 0 && leaf.children.size() < 1) {
		for (int index : leaf.points) {
			glm::vec3 t = vertices[index]->Position;
			if (glm::all(glm::equal(t, p))) {
				p1 = &t;
				d1 = glm::distance(t, p);
				nearestIndex = index;
			}
		}
	}

	while (!p2) {
		r *= 2.0;
		std::vector<int> nearby = root->getPoints(p, r, 0.0, vertices);
		for (int index : nearby) {
			p2 = &vertices[index]->Position;
			float d2 = glm::distance(*p2, p);
			if (!p1 || d2 < d1) {
				p1 = p2;
				d1 = d2;
				nearestIndex = index;
			}
		}
	}

	return nearestIndex;
}

Face * Octree::getNearestFace(glm::vec3 p)
{
	// TODO: *majorly* fucked
	Face *result = NULL;
	float distanceSquared = faceRoot->closestFace(p, &result);

	// Not required, but provides a check on faceCell being changed:
	if (distanceSquared < 0.0f) 
		return NULL;

	return result;
}

std::vector<int> Octree::findInRadius(glm::vec3 point, float radius, float minD)
{
	return root->getPoints(point, radius, minD, vertices);
}

bool Octree::intersects(glm::vec3 rayOrigin, glm::vec3 rayEnd)
{
	glm::vec3 *n = NULL;
	float doubleSize = (2.0 * glm::distance(rayOrigin, rayEnd));
	return faceRoot->intersects(rayOrigin, rayEnd, doubleSize, &n);
}

glm::vec3 Octree::rayCast(glm::vec3 rayOrigin, glm::vec3 direction)
{
	glm::vec3 rayEnd = (faceRoot->range * 2.0f) * direction + rayOrigin;

	glm::vec3 *intersection = NULL;
	if (faceRoot->intersects(rayOrigin, rayEnd, faceRoot->range, &intersection))
		return *intersection;

	return glm::vec3(0.0f, 0.0f, 0.0f);
}

