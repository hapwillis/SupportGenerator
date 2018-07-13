#include "Octree.h"

// Inserts an index, unless that index is already in the set.
void ordered_set::insert(int i) {
	if (set.count(i) == 0) {
		set.insert(i);
		vector.push_back(i);
	}
}

// Returns the number of all (including non-unique) contained objects.
int ordered_set::size()
{
	return vector.size();
}

// Removes items from the set in insertion order.  The vector is preserved.
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

// RRemoves items in insertion order.
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

// Adds a face to this Node, ignoring duplicates.
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

// Calculates an approximate center and Axis Aligned Bounding Box.
void Face::size(float & doubleSize, glm::vec3 & center)
{
	// make AABB
	float minX, maxX, minY, maxY, minZ, maxZ;
	minX = std::min({ vertex1.x, vertex2.x, vertex3.x });
	maxX = std::max({ vertex1.x, vertex2.x, vertex3.x });
	minY = std::min({ vertex1.y, vertex2.y, vertex3.y });
	maxY = std::max({ vertex1.y, vertex2.y, vertex3.y });
	minZ = std::min({ vertex1.z, vertex2.z, vertex3.z });
	maxZ = std::max({ vertex1.z, vertex2.z, vertex3.z });

	doubleSize = 2.0 * std::max({ maxX - minX, maxY - minY, maxZ - minZ, });
	center.x = (maxX + minX) / 2.0f;
	center.y = (maxY + minY) / 2.0f;
	center.z = (maxZ + minZ) / 2.0f;
}

// Recalculates the edges, vertices and normal of this face.  
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

// Uses the Moller Trumbore algorithm to calculate intersections.
// This is fast for Faces with uninitialized edges.
std::tuple<bool, glm::vec3> Face::MollerTrumbore(glm::vec3 rayOrigin, glm::vec3 rayEnd)
{
	// TODO: this is all overkill; the edges are already found when this is used.
	glm::vec3 h, s, q;
	float a, f, u, v;
	// optimization potential: provide direction as an arg instead of implicitly
	glm::vec3 rayVector = glm::normalize(rayEnd - rayOrigin);
	h = glm::cross(rayVector, edge31);
	a = glm::dot(edge21, h);
	if (a > -EPSILON && a < EPSILON)
		return std::make_tuple(false, glm::vec3(0.0f, 0.0f, 0.0f));
	f = 1 / a;
	s = rayOrigin - vertex1;
	u = f * (glm::dot(s, h));
	if (u < 0.0 || u > 1.0)
		return std::make_tuple(false, glm::vec3(0.0f, 0.0f, 0.0f));
	q = glm::cross(s, edge21);
	v = f * glm::dot(rayVector, q);
	if (v < 0.0 || u + v > 1.0)
		return std::make_tuple(false, glm::vec3(0.0f, 0.0f, 0.0f));
	// At this stage we can compute t to find out where the intersection point is on the line.
	float t = f * glm::dot(edge31, q);
	if (t > EPSILON) // ray intersection
	{
		return std::make_tuple(true, (rayOrigin + rayVector * t));
	}
	else // This means that there is a line intersection but not a ray intersection.
		return std::make_tuple(false, glm::vec3(0.0f, 0.0f, 0.0f));
}

// Returns the distance*distance of a point to the closest point on this face.
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
	glm::vec3 proj21 =	edge21 * clamp(glm::dot(edge21, vecP1) / glm::dot(edge21, edge21)) - vecP1;
	glm::vec3 proj32 =	edge32 * clamp(glm::dot(edge32, vecP2) / glm::dot(edge32, edge32)) - vecP2;
	glm::vec3 proj13 =	-edge31 * clamp(glm::dot(-edge31, vecP3) / glm::dot(edge31, edge31)) - vecP3;

	return std::min({ glm::dot(proj21, proj21),  glm::dot(proj32, proj32),  glm::dot (proj13, proj13)});
}

// Clamps n between 0.0f amd 1.0f.
float Face::clamp(float n)
{
	return std::max(0.0f, std::min(n, 1.0f));
}

// Returns the points within the specified volume around a point.
std::vector<int> Octant::getPoints(glm::vec3 point, float radius, float minD, std::vector<glm::vec3> &vertices)
{
	if (range > radius * 4) {
		return children[findOctant(point)]->getPoints(point, radius, minD, vertices);
	} else {
		if (filled) {
			// optionally you could eliminate children here, but that probably just slows it down.
			std::vector<int> verts;
			for (Octant *c : children) {
				if (c) {
					std::vector<int> v = c->getPoints(point, radius, minD, vertices);
					verts.insert(verts.end(), v.begin(), v.end());
				}
			}

			return verts;
		} else {
			std::vector<int> returnList;

			for (int index : points) {
				float d = glm::distance(vertices[index], point);
				if (d <  radius && d > minD)
					returnList.push_back(index);
			}

			return returnList;
		}
	}
}

// Returns the octant of a point relative to the center of this octant.
int Octant::findOctant(glm::vec3 p)
{
	int i = 0;
	if (p.x >= center.x) { i = i + 1; }
	if (p.y >= center.y) { i = i + 2; }
	if (p.z >= center.z) { i = i + 4; }
	return i;
}

// Overloaded function to insert a point or face to the octree.
bool Octant::add(glm::vec3 p, int pointIndex, std::vector<glm::vec3> &vertices)
{
	if (!filled) {
		for (int i : points) {
			if (i == pointIndex || !pointsNotEqual(vertices[i], p))
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
		int child = findOctant(p);
		if (!children[child])
			children[child] = populateChild(child);

		Octant *cell = children[child];
		while (cell->filled) { // if the cell is filled, it must have children
			child = cell->findOctant(p);
			if (cell->children[child]) {
				cell = cell->children[child];
			} else {
				return cell->add(p, pointIndex, vertices);
			}
		}
		
		return cell->add(p, pointIndex, vertices);
	}

	return true;
}

// Overloaded function to insert a point or face to the octree.
void Octant::add(Face *f, glm::vec3 faceCenter, float doubleSize)
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

// Moves the points from this octant into child octants.
void Octant::split(std::vector<glm::vec3> &vertices)
{
	for (int p : points) {
		glm::vec3 pos = vertices[p];
		int child = findOctant(pos);
		if (!children[child])
			children[child] = populateChild(child);
		children[child]->add(pos, p, vertices);
	}

	points.clear();
}

// Checks if two points are distinct (accounting for floating point errors).
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

// creates a new octant to assign as a child.
Octant* Octant::populateChild(int i)
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
	return new Octant(c, range / 2.0);
}

// Finds the lowest node that may contain this point.
Octant* Octant::findLeaf(glm::vec3 p) 
{
	if (filled) {
		int child = findOctant(p);

		if (children[child]) {
			if (children[child]->filled) {
				return children[child]->findLeaf(p);
			}

			return children[child];
		}
	}

	Octant *t = new Octant(center, range);
	t->children = children;
	t->points = points;
	t->faces = faces;

	return t; 
}


Octant::Octant(glm::vec3 c, float r) : center(c), range(r), filled(false)
{
	Octant *nullpoint = { 0 };
	children.assign(8, nullpoint);
}


Octant::~Octant()
{
	for (Octant *c : children) {
		delete c;
	}
}

// 
std::tuple<bool, glm::vec3, float> Octant::intersects(glm::vec3 rayOrigin, glm::vec3 rayEnd, float doubleSize)
{
	float minDist = range * 2;
	bool found = false;
	std::tuple<bool, glm::vec3, float> intersection;
	intersection = std::make_tuple(found, glm::vec3(0.0f, 0.0f, 0.0f), 0.0f);

	// First, test all faces in this FaceCell
	for (Face *f : faces) { 
		if (f) {
			bool hit; glm::vec3 inter;
			std::tie(hit, inter) = f->MollerTrumbore(rayOrigin, rayEnd);

			if (hit) {
				float dist = glm::distance(rayOrigin, inter);
				if (dist < minDist) {
					found = true;
					minDist = dist;
					intersection = std::make_tuple(hit, inter, dist);
				}
			}
		}
	}

	//if smaller than a child, test if it fits into a single octant 
	//TODO: optimize this, it forces every face to be checked on a raycast.  if findOctant(origin) == findOctant(end)
	if (doubleSize < range) {
		glm::vec3 p1 = glm::greaterThanEqual(rayOrigin, center);
		glm::vec3 p2 = glm::greaterThanEqual(rayEnd, center);
		if (glm::all(glm::equal(p1, p2))) {
		
			Octant *c = children[findOctant(rayOrigin)];
			if (c) {
				auto result = c->intersects(rayOrigin, rayEnd, doubleSize);
				if (std::get<0>(result) && std::get<2>(result) < minDist) {
					found = true;
					minDist = std::get<2>(result);
					intersection = result;
				}
			}
		}
	} else { //just test all faces in the region
		for (Octant *c : children) {
			if (c) {
				auto result = c->intersects(rayOrigin, rayEnd, doubleSize);
				if (std::get<0>(result) && std::get<2>(result) < minDist) {
					found = true;
					minDist = std::get<2>(result);
					intersection = result;
				} 
			}
		}
	}
	
	return intersection;
}

// Returns the closest face to a point.
// NB: Do not use this function directly.  The return is the square of the actual distance.
float Octant::closestFace(glm::vec3 point, Face** resultSquared)
{
	Face *best = NULL;
	float bestDist = -1.0f;

	Octant *bestChild = children[findOctant(point)];
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
		for (Octant *octant : children) {
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

void Octree::operator()()
{
}


Octree::Octree(const std::vector<Node*> *nodes, std::vector<Face*> *aFaces, float range) :
	faces(aFaces), Range(range)
{
	root = new Octant(glm::vec3(0.0, 0.0, 0.0), range);
	vertices.reserve(nodes->size());
	for (Node *n : *nodes) {
		vertices.push_back(n->vertex.Position);
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
	vertices.reserve(100);
}


Octree::~Octree()
{
	delete root;
}

// Fills the octree with the given inputs.
void Octree::update(const std::vector<Node*>* nodes, std::vector<Face*>* aFaces, float range)
{
	faces = aFaces; 
	Range = range;
	vertices.reserve(nodes->size());
	
	for (Node *n : *nodes) {
		vertices.push_back(n->vertex.Position);
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

// Inserts a point to the octree, enlarging it if necessary, and creating a new placeholder Vertex.
void Octree::addPoint(glm::vec3 p)
{
	int index = vertices.size();
	vertices.emplace_back(p.x, p.y, p.z);
	//vertices.push_back(p);
	addVertex(index);

	//For now this is less complicated than worrying about smart pointers.
	destroyList.push_back(index);
}

// Inserts a vertex to the octree, enlarging it if necessary. 
void Octree::addVertex(int index)
{
	glm::vec3 pos = vertices[index];
	float newRange = 2.0 * std::max({ pos.x, pos.y, pos.z });

	while (root->range < newRange) {
		enlargeRoot();
	}

	if (!(isnan(pos.x) || isnan(pos.y) || isnan(pos.z)))
		root->add(pos, index, vertices);
}

// Inserts a face to the octree, enlarging it if necessary. 
void Octree::addFace(Face *face) {
	float doubleSize;
	glm::vec3 center;
	face->size(doubleSize, center);

	while (doubleSize > root->range) {
		enlargeRoot();
	}

	if (doubleSize > face->EPSILON) {
		root->add(face, center, doubleSize);
	} else {
		//std::cout << "Invalid faces detected!" << std::endl;
	}
}

// Subordinates the root octant to a new, larger octant.
void Octree::enlargeRoot()
{
	glm::vec3 center = root->center;
	float oldRange = root->range;
	Octant *child = root;
	root = new Octant(center, oldRange * 2.0);

	for (int i = 0; i < 8; i++) {
		root->children[i] = root->populateChild(i);
	}

	for (int i = 0; i < 8; i++) {
		Octant *o = child->children[i];
		if (o) {
			Octant *toFill = root->children[i];
			toFill->children[toFill->findOctant(o->center)] = o;
		}
	}

	for (Face *face : child->faces) {
		addFace(face);
	}

	for (int point : child->points) {
		root->points.push_back(point);
	}
}

// Returns the position of the closest node to p.
glm::vec3 Octree::getNearestPoint(glm::vec3 p)
{
	int index = getNearestNodeIndex(p);
	glm::vec3 t = vertices[index];
	return t;
}

// Returns the index of the closest node to p.
int Octree::getNearestNodeIndex(glm::vec3 p)
{
	Octant* leaf = root->findLeaf(p);
	float r = leaf->range;
	glm::vec3 *p1 = NULL;
	glm::vec3 *p2 = NULL;
	float d1 = -1;
	int nearestIndex;

	if (leaf->points.size() > 0 && !leaf->filled) {
		for (int index : leaf->points) {
			glm::vec3 t = vertices[index];
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
			p2 = &vertices[index];
			float d2 = glm::distance(*p2, p);
			if (!p1 || d2 < d1) {
				p1 = p2;
				d1 = d2;
				nearestIndex = index;
			}
		}
	}

	// TEST:
	/*float dist = glm::distance(p, vertices[nearestIndex]);
	float shortest = root->range;
	int index = 0;
	int mindex = 0;
	for (glm::vec3 v : vertices) {
		if (glm::distance(v, p) < shortest) {
			shortest = glm::distance(v, p);
			mindex = index;
		}
		index++;
	}

	if (mindex != nearestIndex)
		std::cout << "Failed to find actual nearest index!" << std::endl;
	*/	

	return nearestIndex;
}

// Returns the a pointer to the closest face to a point.
Face * Octree::getNearestFace(glm::vec3 p)
{
	Face *result = NULL;
	float distanceSquared = root->closestFace(p, &result);

	// Not required, but provides a check on faceCell being changed:
	if (distanceSquared < 0.0f) 
		return NULL;

	return result;
}

// Finds all the points within a radius and over a minimum distance.
// Returns the indices in a vector.
std::vector<int> Octree::findInRadius(glm::vec3 point, float radius, float minD)
{
	return root->getPoints(point, radius, minD, vertices);
}

// Returns the closest point of intersection of a line segment with any face.
bool Octree::intersects(glm::vec3 rayOrigin, glm::vec3 rayEnd)
{
	float doubleSize = (2.0 * glm::distance(rayOrigin, rayEnd));
	auto intersection = root->intersects(rayOrigin, rayEnd, doubleSize);

	return std::get<0>(intersection); 
}

// Returns the closest point of intersection of a ray with any face.
glm::vec3 Octree::rayCast(glm::vec3 rayOrigin, glm::vec3 direction)
{
	glm::vec3 rayEnd = (root->range * 2.0f) * direction + rayOrigin;

	auto intersection = root->intersects(rayOrigin, rayEnd, root->range);

	return std::get<1>(intersection);
}

