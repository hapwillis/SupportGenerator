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

std::vector<int> Cell::getPoints(glm::vec3 point, float radius, float minD, std::vector<Vertex*> &vertices)
{
	if (range > radius * 4) {
		return children[findQuadrant(point)]->getPoints(point, radius, minD, vertices);
	} else {
		if (children.size() > 1) {
			// optionally you could eliminate children here, but that probably just slows it down.
			std::vector<int> verts;
			for (Cell *c : children) {
				std::vector<int> v = c->getPoints(point, radius, minD, vertices);
				verts.insert(verts.end(), v.begin(), v.end());
			}

			return verts;
		} else {
			float d = glm::distance(vertices[index]->Position, point);
			if (d <  radius && d > minD)
				return std::vector<int>(1, index);
			return std::vector<int>();
		}
	}
}

int Cell::findQuadrant(glm::vec3 p)
{
	int i = 0;
	if (p.x >= center.x) { i = i + 1; }
	if (p.y >= center.y) { i = i + 2; }
	if (p.z >= center.z) { i = i + 4; }
	return i;
}

void Cell::add(glm::vec3 p, int pointIndex, std::vector<Vertex*> &vertices)
{
	if (filled) {
		// check for children
		if (children.size() < 1) {
			// avoid duplicate entries
			if (glm::all(glm::equal(vertices[index]->Position, p))) {
				children.reserve(8);
				float r = range / 2.0;
				PopulateChildren(children, center, r);

				children[findQuadrant(p)]->add(vertices[index]->Position, index, vertices);
				children[findQuadrant(p)]->add(p, pointIndex, vertices);
			}
		} else {
			children[findQuadrant(p)]->add(p, pointIndex, vertices);
		}
	} else {
		index = pointIndex;
		filled = true;
	}
}

void Cell::PopulateChildren(std::vector<Cell*> &children, glm::vec3 center, float r)
{
	// TODO: lazy initialization
	float offset = r / 2.0;
	glm::vec3 c = center;
	c -= glm::vec3(offset, offset, offset);
	children[0] = new Cell(c, r);
	c.x += r;
	children[1] = new Cell(c, r);
	c.x -= r;
	c.y += r;
	children[2] = new Cell(c, r);
	c.x += r;
	children[3] = new Cell(c, r);

	c.x -= r;
	c.y -= r;
	c.z += r;
	children[4] = new Cell(c, r);
	c.x += r;
	children[5] = new Cell(c, r);
	c.x -= r;
	c.y += r;
	children[6] = new Cell(c, r);
	c.x += r;
	children[7] = new Cell(c, r);
}

Cell Cell::find(glm::vec3 p)
{
	int child = findQuadrant(p);
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

Octree::Octree(std::vector<Mesh> meshes, int vertNum, int faceNum)
{
	vertices.reserve(vertNum);
	faces.reserve(faceNum);

	float maxX = 0, minX = 0;
	float maxY = 0, minY = 0;
	float maxZ = 0, minZ = 0;

	int meshNum = meshes.size();
	int vIndex = 0;
	int fIndex = 0;
	for (int i = 0; i < meshNum; i++) {
		Mesh *mesh = &meshes[i];
		//unsigned int size = mesh->vertices.size();
		for (auto &v : mesh->vertices) {
			vertices[vIndex] = &v;
			add(vIndex);
			vIndex++;
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

		int fSize = mesh->indices.size();
		for (int j = 0; j < fSize; j +=3) {
			faces[fIndex] = mesh->indices[j];
			faces[fIndex + 1] = mesh->indices[j + 1];
			faces[fIndex + 2] = mesh->indices[j + 2];

			addFace(fIndex);
			fIndex += 3;
		}
	}

	range = std::min({ (maxX - minX), (maxY - minY), (maxZ - minZ) });
	root = new Cell(glm::vec3(0.0, 0.0, 0.0), range);
	faceRoot = new FaceCell(glm::vec3(0.0, 0.0, 0.0), range);
}


Octree::~Octree()
{
	delete root;
	delete faceRoot;
}

void Octree::add(int index)
{
	root->add(vertices[index]->Position, index, vertices);
}

void Octree::addFace(int index) {
	glm::vec3 vertex0, vertex1, vertex2, edge1, edge2, center;

	glm::vec3 vertex0 = vertices[faces[index]]->Position;
	glm::vec3 vertex1 = vertices[faces[index + 1]]->Position;
	glm::vec3 vertex2 = vertices[faces[index + 2]]->Position;

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
	glm::vec3 *p1;
	glm::vec3 *p2;
	float d1 = -1;

	if (leaf.filled && leaf.children.size() < 1) {
		glm::vec3 t = vertices[leaf.index]->Position;
		if (glm::all(glm::equal(t, p))) {
			p1 = &t;
			d1 = glm::distance(t, p);
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
