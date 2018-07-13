#include "Model.h"

// Default uninitialized constructor
Vertex::Vertex() {
	Wireframe = 0.0;
}


Vertex::Vertex(glm::vec3 p, glm::vec3 n, float wireframe) : 
  Position(p), Normal(n), Wireframe(wireframe) {

}


Mesh::Mesh(std::vector<Vertex> vertices, std::vector<unsigned int> indices) {
	this->vertices = vertices;
	this->indices = indices;

	// now that we have all the required data, set the vertex buffers and its attribute pointers.
	setupMesh();
}

// Makes the actual render call and handles VAO binding
void Mesh::Draw(DefaultShader shader) {
	// draw mesh
	glBindVertexArray(VAO);
	glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0);
	glBindVertexArray(0);
}

// Uploads a mesh to the GPU (sets up a VBO etc.)
void Mesh::setupMesh() {
	glGenVertexArrays(1, &VAO);
	glGenBuffers(1, &VBO);
	glGenBuffers(1, &EBO);

	glBindVertexArray(VAO);
	// load data into vertex buffers
	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	// A great thing about structs is that their memory layout is sequential for all its items.
	// The effect is that we can simply pass a pointer to the struct and it translates perfectly to a glm::vec3/2 array which
	// again translates to 3/2 floats which translates to a byte array.
	glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), &vertices[0], GL_STATIC_DRAW);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), &indices[0], GL_STATIC_DRAW);

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


Model::Model(std::string &path) {
	loadModel(path);

	//float time = glfwGetTime();
	//0.131 seconds to remove vertices
	concatenateModelMeshes();
	//std::cout << "Time to simplify Mesh: " << glfwGetTime() - time << std::endl;
}


Model::~Model() {

}

// Draws the render-optimized model data by calling each mesh's Draw()
void Model::Draw(DefaultShader shader) {
	for (unsigned int i = 0; i < meshes.size(); i++)
		meshes[i].Draw(shader);
}

// Returns the radius of tightly fitting bounding sphere
// Useful for approximating collisions and center of mass.
float Model::BoundingSphere() {
	if (boundingRadius == 0.0) {
		// TODO: use Gartner's algorithm: https://people.inf.ethz.ch/gaertner/subdir/texts/own_work/esa99_final.pdf
		boundingRadius = AABBsize() / 2.0;
	}

	return boundingRadius;
}

// Returns the size of an Axis Aligned Bounding Box.
// Specifically a double-oversized cube, which allows for trivial insertion
// into an octree.
float Model::AABBsize() {
	if (AABB <= 0.0) {
		float delta = 0.0;
		for (Vertex *vertex : denseVertices) {
			glm::vec3 v = vertex->Position;
			delta = std::max({ delta, std::abs(v.x), std::abs(v.y), std::abs(v.z) });
		}

		AABB = 2.0 * delta;
	}
	return AABB;
}

// Deletes render-optimized data, which can be highly redundant.
// Until this is called, the model object keeps both a compact and an
// expanded copy.
void Model::clean() {
	// TODO: delete all extra data
}

// Creates and sets up an AssImp importer for a given file path to a model.
void Model::loadModel(std::string &path) {
	// Loading time for AssImp: .067 seconds
	//float time = glfwGetTime();
	Assimp::Importer import;
	import.SetPropertyInteger(AI_CONFIG_PP_RVC_FLAGS, 
		//aiComponent_NORMALS					|
		aiComponent_TANGENTS_AND_BITANGENTS	|
		aiComponent_COLORS					|
		aiComponent_TEXCOORDS				|
		aiComponent_BONEWEIGHTS				|
		aiComponent_ANIMATIONS				|
		aiComponent_TEXTURES				|
		aiComponent_CAMERAS					|
		aiComponent_LIGHTS					|
		aiComponent_MATERIALS);

	// Adding aiProcess_JoinIdenticalVertices increases load time by 5.8 seconds.
	const aiScene *scene = import.ReadFile(path, 
		//aiProcess_GenNormals			|
		aiProcess_Triangulate			| 
		aiProcess_RemoveComponent		|
		aiProcess_FlipUVs);

	if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode)
	{
		std::cout << "ERROR::ASSIMP::" << import.GetErrorString() << std::endl;
		return;
	}
	directory = path.substr(0, path.find_last_of('/'));

	//std::cout << "Time to load: " << glfwGetTime() - time << std::endl;
	// Time for processNode: 0.058 seconds
	//time = glfwGetTime();
	processNode(scene->mRootNode, scene);
	//std::cout << "Time to process: " << glfwGetTime() - time << std::endl;
}


bool Model::exportModel(std::string & path) {
	// TODO: exportModel()
	return false;
}

// Creates Mesh objects from an AssImp aiNode.
void Model::processNode(aiNode *node, const aiScene *scene) {
	// process all the node's meshes (if any)
	for (unsigned int i = 0; i < node->mNumMeshes; i++)
	{
		aiMesh *mesh = scene->mMeshes[node->mMeshes[i]];
		meshes.push_back(processMesh(mesh, scene));
	}
	// then do the same for each of its children
	for (unsigned int i = 0; i < node->mNumChildren; i++)
	{
		processNode(node->mChildren[i], scene);
	}
}

// Converts an AssImp aiMesh to a vector of Vertex objects and indices.
Mesh Model::processMesh(aiMesh *mesh, const aiScene *scene) {
	std::vector<Vertex> vertices (mesh->mNumVertices);
	std::vector<unsigned int> indices(mesh->mNumFaces * 3);

	for (unsigned int i = 0; i < mesh->mNumVertices; i++) {
		glm::vec3 vector;
		Vertex vertex;

		aiVector3D *vert = &(mesh->mVertices[i]);
		vector.x = vert->x;
		vector.y = vert->y;
		vector.z = vert->z;
		vertex.Position = vector;

		aiVector3D *norm = &(mesh->mNormals[i]);
		vector.x = norm->x;
		vector.y = norm->y;
		vector.z = norm->z;
		vertex.Normal = vector;
		vertex.Wireframe = 0.0f;

		vertices[i] = vertex; 
	}

	// process indices
	int index = 0;
	for (unsigned int i = 0; i < mesh->mNumFaces; i++) {
		aiFace face = mesh->mFaces[i];
		index = i * 3;
		indices[index] = face.mIndices[0];
		indices[index + 1] = face.mIndices[1];
		indices[index + 2] = face.mIndices[2];
		// retrieve all indices of the face and store them in the indices vector
	}

	return Mesh(vertices, indices);
}

// Creates a set of unique vertices and a set of unique indices comprising all meshes.
void Model::concatenateModelMeshes() {
	int translateSize = 0, indicesSize = 0;
  // this is over-optimizing
	for (Mesh mesh : meshes) {
		translateSize += mesh.vertices.size();
		indicesSize += mesh.indices.size();
	}

	std::vector<int> translate;
	translate = constructUniqueVertices(translateSize);
	constructUniqueFaces(indicesSize, translate);
}

// Creates a reduced list of vertices and returns a vector mapping between the 
// old and new lists.
std::vector<int> Model::constructUniqueVertices(int size) { 
  //TODO: can be combined with processMesh.
	std::vector<int> translate;
	translate.reserve(size);
	std::unordered_multimap<float, int> vertexMap;
	int vIndex = 0;

	for (int i = 0; i < meshes.size(); i++) {
		Mesh *mesh = &meshes[i];
		for (int j = 0; j < mesh->vertices.size(); j++) {
			Vertex *v = &mesh->vertices[j];
			int index = checkMultimap(v, vertexMap);

			if (index < 1) { 
				//if not found, insert this point and set translate[tIndex] = vIndex
				translate.push_back(vIndex);
				vertexMap.insert(std::pair<float, int>(v->Position.x, vIndex));
				denseVertices.push_back(v);
				vIndex++;
			} else {
				translate.push_back(index);
			}
		}
	}

	return translate;
}

// Checks a multimap for a particular vertex and returns the index of the first duplicate.
// Returns -1 if no duplicate was found, then records the current index.
// Helper function for constructUniqueVertices
int Model::checkMultimap(Vertex *v, std::unordered_multimap<float, int> &vertexMap) {
	auto range = vertexMap.equal_range(v->Position.x);

	//check vertexMap for key, find any equal vertices
	for (auto r = range.first; r != range.second; r++) {
		int index = r->second;
		// if found, set translate[tIndex] = value
		if (pointsEqual(v->Position, denseVertices[index]->Position)) {
			return index;
		}
	}

	return -1;
}

// Reduces this models' indices to unique groups of indices (representing triangles).
// Assumes unique vertices have been removed but the indices haven't been updated,
// so it expects to be passed a vector mapping old to new indices.
void Model::constructUniqueFaces(int size, std::vector<int> &translate) {
	// TODO: this vector and loop can be moved into the following loop
	std::vector<unsigned int> indices;
	indices.reserve(size);
	int offset = 0;
	for (int i = 0; i < meshes.size(); i++) {
		for (int j : meshes[i].indices) {
			indices.push_back(translate[j + offset]);
		}
		offset += meshes[i].vertices.size();
	}

	std::unordered_multimap<int, int> faceMap;
	int brokenfaces = 0;
	for (int i = 0; i < indices.size(); i += 3) {
		bool notFound = true;
		int key = indices[i];
		auto range = faceMap.equal_range(key);

		//check faceMap for key, then find any equal faces
		for (auto r = range.first; r != range.second; r++) {
			int index = r->second;
			if ((indices[i + 1] == indices[index]) && (indices[i + 2] == indices[index + 1])) {
				notFound = false;
				break;
			}
		}

		//if not found, insert this face
		if (notFound) {
			// 
			if (!(key == indices[i + 1] || key == indices[i + 2] || indices[i + 1] == indices[i + 2])) {
				faceMap.insert(std::pair<int, int>(key, i + 1));

				denseIndices.push_back(key);
				denseIndices.push_back(indices[i + 1]);
				denseIndices.push_back(indices[i + 2]);
			} else {
				brokenfaces++;
			}
		}
	}

	std::cout << "Number of broken faces: " << brokenfaces << std::endl;
}

// Readability function for determining if two points are in the same spot.
bool Model::pointsEqual(glm::vec3 p, glm::vec3 q) {
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
