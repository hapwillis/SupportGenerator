#include "Model.h"

Mesh::Mesh(std::vector<Vertex> vertices, std::vector<unsigned int> indices)
{
	this->vertices = vertices;
	this->indices = indices;

	// now that we have all the required data, set the vertex buffers and its attribute pointers.
	setupMesh();
}

void Mesh::Draw(DefaultShader shader)
{
	// draw mesh
	glBindVertexArray(VAO);
	glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0);
	glBindVertexArray(0);
}

void Mesh::setupMesh() 
{
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
	glBindVertexArray(0);
}

Model::Model(std::string &path)
{
	loadModel(path);
}

Model::~Model()
{

}

void Model::Draw(DefaultShader shader)
{
	for (unsigned int i = 0; i < meshes.size(); i++)
		meshes[i].Draw(shader);
}

float Model::BoundingSphere()
{
	// TODO: use Gartner's algorithm: https://people.inf.ethz.ch/gaertner/subdir/texts/own_work/esa99_final.pdf
	float maxX = 0, minX = 0;
	float maxY = 0, minY = 0;
	float maxZ = 0, minZ = 0;

	for (int i = 0; i < meshes.size(); i++) {
		Mesh *mesh = &meshes[i];
		//unsigned int size = mesh->vertices.size();
		for (Vertex v : mesh->vertices) {
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
	}

	return ret;
}

void Model::loadModel(std::string &path)
{
	// Loading time for AssImp: .067 seconds
	Assimp::Importer import;
	// aiProcess_GenNormals to create normals
	const aiScene *scene = import.ReadFile(path, aiProcess_Triangulate | aiProcess_FlipUVs);

	if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode)
	{
		std::cout << "ERROR::ASSIMP::" << import.GetErrorString() << std::endl;
		return;
	}
	directory = path.substr(0, path.find_last_of('/'));

	// Time for processNode: .52 seconds
	float time = glfwGetTime();
	processNode(scene->mRootNode, scene);
	std::cout << "Time to process: " << glfwGetTime() - time << std::endl;
}

void Model::processNode(aiNode *node, const aiScene *scene)
{
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

Mesh Model::processMesh(aiMesh *mesh, const aiScene *scene)
{
	std::vector<Vertex> vertices (mesh->mNumVertices);
	//vertices.reserve(mesh->mNumVertices);
	std::vector<unsigned int> indices(mesh->mNumFaces * 3);
	//indices.reserve(mesh->mNumFaces);

	for (unsigned int i = 0; i < mesh->mNumVertices; i++)
	{
		glm::vec3 vector;
		Vertex vertex;
		// positions
		aiVector3D *vert = &(mesh->mVertices[i]);
		vector.x = vert->x;
		vector.y = vert->y;
		vector.z = vert->z;
		vertex.Position = vector;
		// normals
		aiVector3D *norm = &(mesh->mNormals[i]);
		vector.x = norm->x;
		vector.y = norm->y;
		vector.z = norm->z;
		vertex.Normal = vector;

		vertices[i] = vertex; //still slow
	}

	// process indices
	int index = 0;
	for (unsigned int i = 0; i < mesh->mNumFaces; i++)
	{
		aiFace face = mesh->mFaces[i];
		// retrieve all indices of the face and store them in the indices vector
		for (unsigned int j = 0; j < face.mNumIndices; j++) {
			indices[index] = face.mIndices[j]; // still slow
			index++;
		}
	}

	return Mesh(vertices, indices);
}
