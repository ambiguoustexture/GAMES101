//
// Created by LEI XU on 4/11/19.
//

#ifndef RASTERIZER_TRIANGLE_H
#define RASTERIZER_TRIANGLE_H

#include <eigen3/Eigen/Eigen>

using namespace Eigen;
class Triangle{

public:
    // the original coordinates of the triangle, v0, v1, v2 in counter clockwise order
    Vector3f v[3]; 
    /*Per vertex values*/
    Vector3f color[3]; //color at each vertex;
    Vector2f tex_coords[3]; //texture u,v
    Vector3f normal[3]; //normal vector for each vertex

    // Texture *tex;
    Triangle();
    // set i-th vertex coordinates
    void setVertex(int ind, Vector3f ver);
    
    // set i-th vertex normal vector 
    void setNormal(int ind, Vector3f n); 
    
    // set i-th vertex color
    void setColor(int ind, float r, float g, float b);
    
    // Only one color per triangle 
    Vector3f getColor() const 
    {
        return color[0]*255; 
    } 
    
    // set i-th vertex texture coordinate
    void setTexCoord(int ind, float s, float t);

    std::array<Vector4f, 3> toVector4() const;
};

#endif //RASTERIZER_TRIANGLE_H
