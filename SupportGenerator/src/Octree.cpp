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

Face::Face(int v1, int v2, int v3, const vector<shared_ptr<Vertex>> &vertices) 
  : index1(v1), index2(v2), index3(v3), 
  vertex1(vertices[index1]), vertex2(vertices[index2]), vertex3(vertices[index3])
{
	edge21 = vertex2->Position - vertex1->Position;
	edge31 = vertex3->Position - vertex1->Position;
	edge32 = vertex3->Position - vertex2->Position;

	normal = glm::normalize(glm::cross(edge21, edge31));
	edge21Normal = glm::normalize(glm::cross(edge21, normal));
	edge32Normal = glm::normalize(glm::cross(edge32, normal));
	edge13Normal = glm::normalize(glm::cross(-edge31, normal));
}

Face::Face(const Face & that) {
}

Face::Face(Face && that) {
}

Face & Face::operator=(const Face & that) {
  // TODO: insert return statement here
}

Face & Face::operator=(Face && that) {
  // TODO: insert return statement here
}

// Calculates an approximate center_ and Axis Aligned Bounding Box.
void Face::size(float & doubleSize, vec3 & center_)
{
	// make AABB
	float minX, maxX, minY, maxY, minZ, maxZ;
	minX = std::min({ vertex1->Position.x, vertex2->Position.x, vertex3->Position.x });
	maxX = std::max({ vertex1->Position.x, vertex2->Position.x, vertex3->Position.x });
	minY = std::min({ vertex1->Position.y, vertex2->Position.y, vertex3->Position.y });
	maxY = std::max({ vertex1->Position.y, vertex2->Position.y, vertex3->Position.y });
	minZ = std::min({ vertex1->Position.z, vertex2->Position.z, vertex3->Position.z });
	maxZ = std::max({ vertex1->Position.z, vertex2->Position.z, vertex3->Position.z });

	doubleSize = 2.0 * std::max({ maxX - minX, maxY - minY, maxZ - minZ, });
	center_.x = (maxX + minX) / 2.0f;
	center_.y = (maxY + minY) / 2.0f;
	center_.z = (maxZ + minZ) / 2.0f;
}

// Recalculates the edges, vertices and normal of this face.  
void Face::update(const vector<shared_ptr<Vertex>> &vertices)
{
	vertex1 = vertices[index1];
	vertex2 = vertices[index2];
	vertex3 = vertices[index3];
  edge21 = vertex2->Position - vertex1->Position;
  edge31 = vertex3->Position - vertex1->Position;
  edge32 = vertex3->Position - vertex2->Position;

	normal = glm::normalize(glm::cross(edge21, edge31));
	edge21Normal = glm::normalize(glm::cross(edge21, normal));
	edge32Normal = glm::normalize(glm::cross(edge32, normal));
	edge13Normal = glm::normalize(glm::cross(-edge31, normal));
}

// Uses the Moller Trumbore algorithm to calculate intersections.
// This is fast for faces_ with uninitialized edges.
tuple<bool, vec3> Face::MollerTrumbore(vec3 rayOrigin, vec3 rayEnd)
{
	// TODO: this is all overkill; the edges are already found when this is used.
	vec3 h, s, q;
	float a, f, u, v;
	// optimization potential: provide direction as an arg instead of implicitly
	vec3 rayVector = glm::normalize(rayEnd - rayOrigin);
	h = glm::cross(rayVector, edge31);
	a = glm::dot(edge21, h);
	if (a > -EPSILON && a < EPSILON)
		return std::make_tuple(false, vec3(0.0f, 0.0f, 0.0f));
	f = 1 / a;
	s = rayOrigin - vertex1->Position;
	u = f * (glm::dot(s, h));
	if (u < 0.0 || u > 1.0)
		return std::make_tuple(false, vec3(0.0f, 0.0f, 0.0f));
	q = glm::cross(s, edge21);
	v = f * glm::dot(rayVector, q);
	if (v < 0.0 || u + v > 1.0)
		return std::make_tuple(false, vec3(0.0f, 0.0f, 0.0f));
	// At this stage we can compute t to find out where the intersection point is on the line.
	float t = f * glm::dot(edge31, q);
	if (t > EPSILON) // ray intersection
	{
		return std::make_tuple(true, (rayOrigin + rayVector * t));
	}
	else // This means that there is a line intersection but not a ray intersection.
		return std::make_tuple(false, vec3(0.0f, 0.0f, 0.0f));
}

// Returns the distance*distance of a point to the closest point on this face.
float Face::faceDistanceSquared(vec3 point)
{
	vec3 vecP1 = point - vertex1->Position;
	vec3 vecP2 = point - vertex2->Position;
	vec3 vecP3 = point - vertex3->Position;

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
	vec3 proj21 =	edge21 * clamp(glm::dot(edge21, vecP1) / glm::dot(edge21, edge21)) - vecP1;
	vec3 proj32 =	edge32 * clamp(glm::dot(edge32, vecP2) / glm::dot(edge32, edge32)) - vecP2;
	vec3 proj13 =	-edge31 * clamp(glm::dot(-edge31, vecP3) / glm::dot(edge31, edge31)) - vecP3;

	return std::min({ glm::dot(proj21, proj21),  glm::dot(proj32, proj32),  glm::dot (proj13, proj13)});
}

// Clamps n between 0.0f amd 1.0f.
float Face::clamp(float n)
{
	return std::max(0.0f, std::min(n, 1.0f));
}

// Returns the points_ within the specified volume around a point.
vector<shared_ptr<Vertex>> Octant::getPoints(vec3 point, float radius, float minD, vector<shared_ptr<Vertex>> &vertices_) {
	if (range_ > radius * 4) {
		return children_[findOctant(point)]->getPoints(point, radius, minD, vertices_);
	} else {
		if (filled_) {
			// optionally you could eliminate children_ here, but that probably just slows it down.
			vector<shared_ptr<Vertex>> verts;
			for (unique_ptr<Octant>& c : children_) {
				if (c) {
					vector<shared_ptr<Vertex>> v = c->getPoints(point, radius, minD, vertices_);
					verts.insert(verts.end(), v.begin(), v.end());
				}
			}

			return verts;
		} else {
			vector<shared_ptr<Vertex>> returnList;

			for (shared_ptr<Vertex> v : points_) {
				float d = glm::distance(v->Position, point);
				if (d <  radius && d > minD)
					returnList.push_back(v);
			}

			return returnList;
		}
	}
}

// Returns the octant of a point relative to the center_ of this octant.
int Octant::findOctant(vec3 p)
{
	int i = 0;
	if (p.x >= center_.x) { i = i + 1; }
	if (p.y >= center_.y) { i = i + 2; }
	if (p.z >= center_.z) { i = i + 4; }
	return i;
}

// Overloaded function to insert a point or face to the octree.
bool Octant::add(shared_ptr<Vertex> vertex)
{
	if (!filled_) {
		for (shared_ptr<Vertex> v : points_) {
			if (!pointsNotEqual(v->Position, vertex->Position))
				return false;
		}

		if (points_.size() < maxChildren) {
			points_.push_back(vertex);
		} else {
			points_.push_back(vertex);
			filled_ = true;
			split();
		}
	} else {
		int child = findOctant(vertex->Position);
    if (!children_[child]) {
      children_[child] = populateChild(child);
    }

    return children_[child]->add(vertex);
	}

	return true;
}

// Overloaded function to insert a point or face to the octree.
void Octant::add(shared_ptr<Face> face, vec3 facecenter_, float doubleSize)
{
	if (doubleSize > range_) {
		faces_.push_back(std::make_shared<Face>(face));
		return;
	}
	int child = findOctant(facecenter_);
	if (!children_[child])
		children_[child] = populateChild(child);
	children_[child]->add(face, facecenter_, doubleSize);
}

// Moves the points_ from this octant into child octants.
void Octant::split()
{
	for (shared_ptr<Vertex> v : points_) {
		int child = findOctant(v->Position);
		if (!children_[child])
			children_[child] = populateChild(child);
		children_[child]->add(v);
	}

	points_.clear();
}

// Checks if two points_ are distinct (accounting for floating point errors).
bool Octant::pointsNotEqual(vec3 p, vec3 q)
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
unique_ptr<Octant> Octant::populateChild(int i)
{
	float o = range_ / 4.0;
	vec3 c(center_);

	switch (i) {
	case 0: c += vec3(-o, -o, -o);
		break;
	case 1: c += vec3(o, -o, -o);
		break;
	case 2: c += vec3(-o, o, -o);
		break;
	case 3: c += vec3(o, o, -o);
		break;

	case 4: c += vec3(-o, -o, o);
		break;
	case 5: c += vec3(o, -o, o);
		break;
	case 6: c += vec3(-o, o, o);
		break;
	case 7: c += vec3(o, o, o);

	}
	return std::make_unique<Octant>(c, range_ / 2.0);
}

// Finds the lowest node that may contain this point.
tuple<bool, shared_ptr<Vertex>, float> Octant::closestVertex(vec3 p)
{
  bool found = false;
  shared_ptr<Vertex> vertex;
  float minDist = range_;

	if (filled_) { //if filled, traverse down
		int child = findOctant(p);

		if (children_[child]) { 
      return children_[child]->closestVertex(p);
		}

    // if no valid child, search all other children and compare
    for (unique_ptr<Octant>& child : children_) {
      if (child) {
        auto octantPoint = child->closestVertex(p);

        if (std::get<0>(octantPoint) && std::get<2>(octantPoint) < minDist) {
          found = true;
          vertex = std::get<1>(octantPoint);
          minDist = std::get<2>(octantPoint);
        }
      }
    }

    if (found)
      return std::make_tuple(found, vertex, minDist);
	}

  //find closest in points_
  for (shared_ptr<Vertex> v : points_) {
    float distance = glm::distance(p, v->Position);
    if (distance < minDist) {
      found = true;
      minDist = distance;
      vertex = v;
    }
  }

	return std::make_tuple(found, vertex, minDist);
}


Octant::Octant(vec3 c, float r) : center_(c), range_(r), filled_(false)
{
	children_.assign(8, nullptr);
}


Octant::~Octant()
{

}

// 
tuple<bool, vec3, float> Octant::intersects(vec3 rayOrigin, vec3 rayEnd, float doubleSize)
{
	float minDist = range_ * 2;
	bool found = false;
	tuple<bool, vec3, float> intersection;
	intersection = std::make_tuple(found, vec3(0.0f, 0.0f, 0.0f), 0.0f);

	// First, test all faces_ in this FaceCell
	for (shared_ptr<Face> f : faces_) { 
		if (f) {
			bool hit; vec3 inter;
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
	if (doubleSize < range_) {
		vec3 p1 = glm::greaterThanEqual(rayOrigin, center_);
		vec3 p2 = glm::greaterThanEqual(rayEnd, center_);
		if (glm::all(glm::equal(p1, p2))) {
		
      int child = findOctant(rayOrigin);
			if (children_[child]) {
				auto result = children_[child]->intersects(rayOrigin, rayEnd, doubleSize);
				if (std::get<0>(result) && std::get<2>(result) < minDist) {
					found = true;
					minDist = std::get<2>(result);
					intersection = result;
				}
			}
		}
	} else { //just test all faces_ in the region
		for (unique_ptr<Octant> &c : children_) {
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
tuple<bool, shared_ptr<Face>, float> Octant::closestFace(vec3 point) {
  bool found = false;
  shared_ptr<Face> bestFace;
  float bestDist = -1.0f;

  int child = findOctant(point);
  if (children_[child]) {
    auto best = children_[child]->closestFace(point);
    found = true;
    bestFace = std::get<1>(best);
    bestDist = std::get<2>(best);
  }

	for (shared_ptr<Face>& face : faces_) {
		float dist = face->faceDistanceSquared(point);
		if (bestDist < 0.0f || (dist > 0.0f && dist < bestDist)) {
      found = true;
			bestFace = face;
      bestDist = dist;
		}
	}

	// search other octants
	if (bestDist < 0.0f) {
		for (unique_ptr<Octant> &octant : children_) {
			if (octant) {
				auto best = octant->closestFace(point);
        float dist = std::get<2>(best);

				if (bestDist < 0.0f || (dist < bestDist && dist > 0.0f)) {
          found = true;
					bestFace = std::get<1>(best);
          bestDist = dist;
				}
			}
		}
	}

	return std::make_tuple(found, bestFace, bestDist);
}

void Octree::operator()()
{
}

Octree::Octree(const vector<shared_ptr<Vertex>> vertices, vector<shared_ptr<Face>> faces, float range):
  faces_(faces), range_(range), vertices_(vertices), root_(Octant(vec3(0.0f, 0.0f, 0.0f), range_)) {
	//float time = glfwGetTime();
	//0.0284 seconds to build vertex octree
	for (shared_ptr<Vertex> vertex : vertices_) {
		addVertex(vertex);
	}
	//std::cout << "Time to insert vertices: " << glfwGetTime() - time << std::endl;

	//time = glfwGetTime();
	for (shared_ptr<Face> face : faces_) {
		addFace(face);
	}
	//std::cout << "Time to insert faces_: " << glfwGetTime() - time << std::endl;
}


Octree::Octree() : range_(20.0f), root_(Octant(vec3(0.0, 0.0, 0.0), range_))
{
	vertices_.reserve(100);
}


Octree::~Octree()
{
}

// Fills the octree with the given inputs.
void Octree::update(const vector<shared_ptr<Vertex>> vertices, vector<shared_ptr<Face>> faces, float range)
{
	faces_ = faces; 
	range_ = range;
	vertices_ = vertices;

  for (shared_ptr<Vertex> vertex : vertices_) {
    addVertex(vertex);
  }

  for (shared_ptr<Face> face : faces_) {
    addFace(face);
  }
}

// Inserts a point to the octree, enlarging it if necessary, and creating a new placeholder Vertex.
void Octree::addPoint(vec3 p)
{
  shared_ptr<Vertex> vertex;
  vertex->Position = p;
  vertices_.push_back(vertex);
	addVertex(vertex);
}

// Inserts a vertex to the octree, enlarging it if necessary. 
void Octree::addVertex(shared_ptr<Vertex> vertex)
{
	vec3 pos = vertex->Position;
	float newRange = 2.0 * std::max({ pos.x, pos.y, pos.z });

	while (root_.range_ < newRange) {
		enlargeRoot();
	}

	if (!(isnan(pos.x) || isnan(pos.y) || isnan(pos.z)))
		root_.add(vertex);
}

// Inserts a face to the octree, enlarging it if necessary. 
void Octree::addFace(shared_ptr<Face> face) {
	float doubleSize;
	vec3 center_;
	face->size(doubleSize, center_);

	while (doubleSize > root_.range_) {
		enlargeRoot();
	}

	if (doubleSize > face->EPSILON) {
		root_.add(face, center_, doubleSize);
	} else {
		//std::cout << "Invalid faces_ detected!" << std::endl;
	}
}

// Subordinates the root_ octant to a new, larger octant.
void Octree::enlargeRoot()
{
	vec3 center_ = root_.center_;
	float oldRange = root_.range_;
	Octant child = root_;
	root_ = Octant(center_, oldRange * 2.0);

	for (int i = 0; i < 8; i++) {
		root_.children_[i] = root_.populateChild(i);
	}

	for (int i = 0; i < child.children_.size(); i++) {
		if (child.children_[i]) {
      int secondChild = root_.children_[i]->findOctant(child.children_[i]->center_);
      root_.children_[i]->children_[secondChild] = std::move(child.children_[i]);
		}
	}

	for (shared_ptr<Face> face : child.faces_) {
		addFace(face);
	}

	for (shared_ptr<Vertex> point : child.points_) {
		root_.points_.push_back(point);
	}
}

// Returns the position of the closest node to p.
vec3 Octree::getNearestPoint(vec3 p) {
	return std::get<1>(getNearestVertex(p))->Position;
}

// Returns the index of the closest node to p.
tuple<bool, shared_ptr<Vertex>, float> Octree::getNearestVertex(vec3 p)
{
	return root_.closestVertex(p);
}

// Returns the a pointer to the closest face to a point.
shared_ptr<Face> Octree::getNearestFace(vec3 p)
{
	return std::get<1>(root_.closestFace(p));
}

// Finds all the points_ within a radius and over a minimum distance.
// Returns the indices in a vector.
vector<shared_ptr<Vertex>> Octree::findInRadius(vec3 point, float radius, float minD)
{
	return root_.getPoints(point, radius, minD, vertices_);
}

// Returns the closest point of intersection of a line segment with any face.
bool Octree::intersects(vec3 rayOrigin, vec3 rayEnd)
{
	float doubleSize = (2.0 * glm::distance(rayOrigin, rayEnd));
	auto intersection = root_.intersects(rayOrigin, rayEnd, doubleSize);

	return std::get<0>(intersection); 
}

// Returns the closest point of intersection of a ray with any face.
vec3 Octree::rayCast(vec3 rayOrigin, vec3 direction)
{
	vec3 rayEnd = (root_.range_ * 2.0f) * direction + rayOrigin;

	auto intersection = root_.intersects(rayOrigin, rayEnd, root_.range_);

	return std::get<1>(intersection);
}

