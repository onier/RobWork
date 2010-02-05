#ifndef RW_DRAWABLE_LOADER_3DS_H
#define RW_DRAWABLE_LOADER_3DS_H

#include "Model3D.hpp"

#include <cstdio>
#include <cstring>
#include <vector>
#include <string>

class Loader3DS
{
public:
    static Model3DPtr load(const std::string& name); // Loads a model

private:
    FILE *_bin3ds;			// The binary 3ds file
    Model3D *_model;

    std::string _path; // The path of the model

    Loader3DS();

    Model3DPtr loadModel(const std::string& name);

private:

    // Every chunk in the 3ds file starts with this struct
    struct ChunkHeader {
        unsigned short id;	// The chunk's id
        unsigned long  len;	// The lenght of the chunk
    };

    void IntColorChunkProcessor(long length, long findex, int matindex);
    void FloatColorChunkProcessor(long length, long findex, int matindex);
    // Processes the Main Chunk that all the other chunks exist is
    void MainChunkProcessor(long length, long findex);
    // Processes the model's info
    void EditChunkProcessor(long length, long findex);

    // Processes the model's materials
    void MaterialChunkProcessor(long length, long findex, int matindex);
    // Processes the names of the materials
    void MaterialNameChunkProcessor(long length, long findex, int matindex);
    // Processes the material's diffuse color
    void DiffuseColorChunkProcessor(long length, long findex, int matindex);
    // Processes the material's texture maps
    void TextureMapChunkProcessor(long length, long findex, int matindex);
    // Processes the names of the textures and load the textures
    void MapNameChunkProcessor(long length, long findex, int matindex);

    // Processes the model's geometry
    void ObjectChunkProcessor(long length, long findex, int objindex);
    // Processes the triangles of the model
    void TriangularMeshChunkProcessor(long length, long findex, int objindex);
    // Processes the vertices of the model and loads them
    void VertexListChunkProcessor(long length, long findex, int objindex);
    // Processes the texture cordiantes of the vertices and loads them
    void TexCoordsChunkProcessor(long length, long findex, int objindex);
    // Processes the faces of the model and loads the faces
    void FacesDescriptionChunkProcessor(long length, long findex, int objindex);
    // Processes the materials of the faces and splits them up by material
    void FacesMaterialsListChunkProcessor(
        long length, long findex, int objindex, int subfacesindex);

    // Calculates the normals of the vertices by averaging
    // the normals of the faces that use that vertex
    void CalculateNormals();
};

#endif // RW_DRAWABLE_LOADER_3DS_H
