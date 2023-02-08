#pragma once
#include <iostream>
#include <fstream>
#include <array>

#include "Eigen.h"
#include "MarchingCubes.h"
#include "SimpleMesh.h"

namespace Mesh {
    inline void add_color(const VoxelArray &volume, SimpleMesh &mesh)
    {
        auto &vert = mesh.GetVertices();
        for (unsigned i = 0; i < vert.size(); i++)
        {
            mesh.AddColor(volume.GetColorVal(util::Vec3to4(vert[i])));
        }
    }

    inline void add_point(SimpleMesh& mesh, SimpleMesh::vertex pt, Vector4uc color = Vector4uc{255, 0, 0, 255})
    {
        if (pt.x() != MINF)
        {
            mesh.AddVertex(pt);
            mesh.AddColor(color);
        }
    }

    inline void add_line(SimpleMesh& mesh, SimpleMesh::vertex pt1, SimpleMesh::vertex pt2, Vector4uc color = Vector4uc{255, 0, 0, 255})
    {
        // large triangle: pt1 start, pt2 end
        auto t = pt2;
        t.x() += 2e-2;
        t.y() += 2e-2;
        t.z() += 2e-2;
        mesh.AddFace(mesh.AddVertex(pt1), mesh.AddVertex(pt2), mesh.AddVertex(t));
        mesh.AddColor(color);
        mesh.AddColor(color);
        mesh.AddColor(color);
    }

    inline void export_mesh(SimpleMesh &mesh, const string &outpath, bool colored)
    {
        if (colored)
            if (!mesh.WriteColoredMesh(outpath))
                throw std::runtime_error("ERROR: unable to write output mesh file: " + outpath + " with error: " + strerror(errno));
            else if (!mesh.WriteMesh(outpath))
                throw std::runtime_error("ERROR: unable to write output mesh file: " + outpath + " with error: " + strerror(errno));
    }

    inline void export_mesh(const VoxelArray &volume, const string &outpath, bool colored)
    {
        SimpleMesh mesh;
        for (unsigned int x = 0; x < volume.GetDimX() - 1; x++)
        {
            // std::cerr << "Marching Cubes on slice " << x << " of " << volume.GetDimX() << std::endl;
            for (unsigned int y = 0; y < volume.GetDimY() - 1; y++)
                for (unsigned int z = 0; z < volume.GetDimZ() - 1; z++)
                    MC::ProcessVolumeCell(volume, x, y, z, 0.0f, mesh);
        }
        // write mesh to file
        if (colored)
        {
            add_color(volume, mesh);
            if (!mesh.WriteColoredMesh(outpath))
                throw std::runtime_error("ERROR: unable to write output mesh file: " + outpath + " with error: " + strerror(errno));
        }
        else if (!mesh.WriteMesh(outpath))
            throw std::runtime_error("ERROR: unable to write output mesh file: " + outpath + " with error: " + strerror(errno));
    }
}
