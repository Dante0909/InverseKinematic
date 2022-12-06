#include "IKGLCanvas.h"

#include "IKApplication.h"
#include "Armature.h"

#include <fstream>
#include <iostream>

using namespace nanogui;

namespace
{
    static int numCubeVerts, numSphereVerts;
    static std::vector<float> cubeVerts, sphereVerts;
    static std::vector<float> cubeNormals, sphereNormals;

    static float axesVerts[] = {
        0.0f, 0.0f, 0.0f, 0.5f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.5f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.5f,
    };

    static float axesColors[] = {
        1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f,
    };

    // Load OBJ file
    int loadFile(const std::string& filename, std::vector<float>& _verts, std::vector<float>& _normals)
    {
        // Open the input file
        std::ifstream fs(filename, std::ifstream::in);
        if (!fs.is_open())
        {
            std::cout << "Error: Failed to open file " << filename << " for reading!" << std::endl;
            return false;
        }

        // Read file - first pass count the vertices
        std::string line;
        unsigned int vertCount = 0;
        unsigned int triCount = 0;
        while (std::getline(fs, line))
        {
            if (line[0] == 'v' && line[1] == ' ') ++vertCount;
            else if (line[0] == 'f' && line[1] == ' ') ++triCount;
        }

        _verts.resize(triCount * 9);
        _normals.resize(triCount * 9);

        std::vector<Vector3f> v, vn;

        // Read file - first pass count the vertices
        int vertIndex = 0;
        int normalIndex = 0;
        fs.clear();
        fs.seekg(std::ios::beg);
        while (std::getline(fs, line))
        {
            if (line[0] == 'v' && line[1] == ' ')
            {
                // Vertex! Add it to the list.
                float x, y, z;
                std::stringstream ss(line.substr(2));
                ss >> x >> y >> z;
                v.push_back(Vector3f(x, y, z));
            }
            else if (line[0] == 'v' && line[1] == 'n')
            {
                // Normal! Add it to the list.
                float x, y, z;
                std::stringstream ss(line.substr(3));
                ss >> x >> y >> z;
                vn.push_back(Vector3f(x, y, z));
            }
            else if (line[0] == 'f' && line[1] == ' ')
            {
                std::stringstream ss(line.substr(2));
                std::string tok;

                // First tuple
                for (int k = 0; k < 3; ++k)
                {
                    if (std::getline(ss, tok, '/'))
                    {
                        const int i = std::stoi(tok);
                        memcpy(&_verts[3 * vertIndex++], v[i - 1].v, sizeof(Vector3f::v));
                    }
                    std::getline(ss, tok, '/');
                    if (std::getline(ss, tok, ' '))
                    {
                        const int i = std::stoi(tok);
                        memcpy(&_normals[3 * normalIndex++], vn[i - 1].v, sizeof(Vector3f::v));
                    }
                }
            }
        }

        fs.close();
        return vertIndex;
    }

    static inline Matrix3f computeNormalMatrix(const Matrix4f& m)
    {
        Matrix3f n;
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                n.m[j][i] = m.m[j][i];
            }
        }
        return n;
    }

    static inline Vector3f transformPoint(const Matrix4f& A, const Vector3f& p)
    {
        Vector3f p2(0, 0, 0);
        for (int j = 0; j < 3; ++j)
        {
            for (int i = 0; i < 3; ++i)
            {
                p2.v[j] += A.m[j][i] * p.v[i];
            }
        }

        p2.v[0] += A.m[3][0];
        p2.v[1] += A.m[3][1];
        p2.v[2] += A.m[3][2];
        return p2;
    }

    static inline void computeVectorRotation(const Vector3f& a, const Vector3f& b, Vector3f& axis, float& theta)
    {
        const Vector3f v1 = (nanogui::squared_norm(a) > 1e-4f) ? nanogui::normalize(a) : Vector3f(0.0f, 0.0f, 1.0f);
        const Vector3f v2 = (nanogui::squared_norm(b) > 1e-4f) ? nanogui::normalize(b) : Vector3f(0.0f, 0.0f, 1.0f);
        const float dp = std::min(1.0f, std::max(-1.0f, nanogui::dot(v1, v2)));
        theta = std::acos(dp);
        if (theta < 1e-4f)
        {
            axis.v[0] = 1.0f; axis.v[1] = 0.0f; axis.v[2] = 0.0f;
        }
        else
        {
            axis = nanogui::normalize(nanogui::cross(v1, v2));
        }
    }
}

IKGLCanvas::IKGLCanvas(IKApplication* _app) : Canvas(_app->getWindow()), m_app(_app)
{
    resetCamera();

    // A simple shader to draw triangle meshes with normals.
    m_shader = new Shader(render_pass(),
        // Shader name
        "draw_link",
        // Vertex shader
        R"(#version 410
            uniform mat4 mvMatrix;
            uniform mat4 projMatrix;
            uniform mat3 normalMatrix;
            in vec3 position;
            in vec3 normal;
            out vec3 fPosition;
            out vec3 fNormal;
            void main() {
                vec4 vEyeCoord = mvMatrix * vec4(position, 1.0);
                gl_Position = projMatrix * vEyeCoord;
                fPosition = vEyeCoord.xyz;
                fNormal = normalMatrix * normal;
            })",

        // Fragment shader
        R"(#version 410
            uniform vec4 Kd;
            in vec3 fPosition;
            in vec3 fNormal;
            out vec4 fColor;
            void
            main()
            {
                vec3 normal = normalize(fNormal);
                vec3 LightDirection = normalize(-fPosition);
                vec3 viewDirection = normalize(-fPosition);
                float diffuse = max(0.0, dot(normal, LightDirection));
                fColor = Kd * (diffuse + 0.1);
             })", Shader::BlendMode::AlphaBlend
    );


    // A simple shader to draw triangle meshes with normals.
    m_lineShader = new Shader(render_pass(),
        // Shader name
        "draw_lines",
        // Vertex shader
        R"(#version 410
            uniform mat4 mvMatrix;
            uniform mat4 projMatrix;
            in vec3 position;
            in vec3 color;
            out vec3 fColor;
            void main() {
                gl_Position = projMatrix * mvMatrix * vec4(position, 1.0);
                fColor = color;
            })",

        // Fragment shader
        R"(#version 410
            in vec3 fColor;
            out vec4 outColor;
            void
            main()
            {
                outColor = vec4(fColor, 1.0);
             })"
    );

    // Load cube from OBJ file.
    //
    numCubeVerts = loadFile("cube.obj", cubeVerts, cubeNormals);
    numSphereVerts = loadFile("sphere.obj", sphereVerts, sphereNormals);

    m_shader->set_buffer("position", VariableType::Float32, { (unsigned int)numCubeVerts, 3 }, cubeVerts.data());
    m_shader->set_buffer("normal", VariableType::Float32, { (unsigned int)numCubeVerts, 3 }, cubeNormals.data());

    m_lineShader->set_buffer("position", VariableType::Float32, { 6, 3 }, axesVerts);
    m_lineShader->set_buffer("color", VariableType::Float32, { 6, 3 }, axesColors);
}

void IKGLCanvas::draw_contents()
{
    static const Vector3f zplus(0.0f, 0.0f, 1.0f);
    Vector3f axis;
    float theta;
    float length;
    gti320::Armature* armature = m_app->getArmature();

    {
        const float fov = 1.57f;
        const float n = 0.2f;
        const float f = 20.0f;
        const float aspect = (float)width() / (float)height();
        const Matrix4f projMat = Matrix4f::perspective(fov / 2.0f, n, f, aspect);

        const Vector3f p = transformPoint(Matrix4f::rotate(Vector3f(1.0f, 0.0f, 0.0f), phi[1]) * Matrix4f::rotate(Vector3f(0, 1, 0), phi[0]), Vector3f(0, 0, dist));
        const Matrix4f viewMat = Matrix4f::look_at(p, Vector3f(0, 1.0f, 0), Vector3f(0, 1, 0));

        // Draw the joint axes.
        //
        m_lineShader->set_uniform("projMatrix", projMat);
        m_lineShader->set_buffer("position", VariableType::Float32, { 6, 3 }, axesVerts);
        for (gti320::Link* l : armature->links)
        {
            if (l->isEndEffector())
                continue;

            Matrix4f linkM;
            memcpy(linkM.m, l->M.data(), sizeof(Matrix4f::m));

            Matrix4f modelMat = linkM;
            const Matrix4f mvMat = viewMat * modelMat;
            m_lineShader->set_uniform("mvMatrix", mvMat);

            m_lineShader->begin();
            m_lineShader->draw_array(Shader::PrimitiveType::Line, 0, 6, false);
            m_lineShader->end();
        }

        m_shader->set_uniform("projMatrix", projMat);

        // Draw the armature.
        //
        m_shader->set_uniform("Kd", Vector4f(0.0f, 0.0f, 0.8f, 0.6f));
        m_shader->set_buffer("position", VariableType::Float32, { (unsigned int)numCubeVerts, 3 }, cubeVerts.data());
        m_shader->set_buffer("normal", VariableType::Float32, { (unsigned int)numCubeVerts, 3 }, cubeNormals.data());

        // Draw the armature links.
        //
        for (gti320::Link* l : armature->links)
        {
            Matrix4f linkM, parentM;
            if (l->parent)
            {
                memcpy(parentM.m, l->parent->M.data(), sizeof(parentM.m));

                // Compute orientation of the link 
                // in parent reference frame
                Vector3f trans;
                memcpy(trans.v, l->trans.data(), sizeof(trans.v));
                computeVectorRotation(zplus, trans, axis, theta);

                // Compute length of the link
                length = nanogui::norm(trans);

                Matrix4f modelMat = parentM * Matrix4f::rotate(axis, theta) * Matrix4f::scale(Vector3f(0.16f, 0.16f, length)) * Matrix4f::translate(0.5f * zplus);
                const Matrix4f mvMat = viewMat * modelMat;
                const Matrix3f normalMat = computeNormalMatrix(mvMat);
                m_shader->set_uniform("mvMatrix", mvMat);
                m_shader->set_uniform("normalMatrix", normalMat);

                m_shader->begin();
                m_shader->draw_array(Shader::PrimitiveType::Triangle, 0, 3 * (unsigned int)numCubeVerts, false);
                m_shader->end();
            }
        }

        m_shader->set_uniform("Kd", Vector4f(0.8f, 0.1f, 0.1f, 0.8f));
        m_shader->set_buffer("position", VariableType::Float32, { (unsigned int)numSphereVerts, 3 }, sphereVerts.data());
        m_shader->set_buffer("normal", VariableType::Float32, { (unsigned int)numSphereVerts, 3 }, sphereNormals.data());

        // Draw the armature joints.
        for (gti320::Link* l : armature->links)
        {
            if (l->isEndEffector())
                continue;

            Matrix4f linkM;
            memcpy(linkM.m, l->M.data(), sizeof(Matrix4f::m));

            Matrix4f modelMat = linkM;
            const Matrix4f mvMat = viewMat * modelMat;
            const Matrix3f normalMat = computeNormalMatrix(mvMat);
            m_shader->set_uniform("mvMatrix", mvMat);
            m_shader->set_uniform("normalMatrix", normalMat);

            m_shader->begin();
            m_shader->draw_array(Shader::PrimitiveType::Triangle, 0, 3 * (unsigned int)numSphereVerts, false);
            m_shader->end();
        }

        m_shader->set_uniform("Kd", Vector4f(0.1f, 0.8f, 0.1f, 1.0f));

        // Draw the target.
        {
            const gti320::Vector3f& t = m_app->getTargetPos();
            Matrix4f modelMat = Matrix4f::translate(nanogui::Vector3f(t(0), t(1), t(2)));
            const Matrix4f mvMat = viewMat * modelMat;
            const Matrix3f normalMat = computeNormalMatrix(mvMat);
            m_shader->set_uniform("mvMatrix", mvMat);
            m_shader->set_uniform("normalMatrix", normalMat);

            m_shader->begin();
            m_shader->draw_array(Shader::PrimitiveType::Triangle, 0, 3 * (unsigned int)numSphereVerts, false);
            m_shader->end();
        }

    }

}

bool IKGLCanvas::mouse_drag_event(const Vector2i& p, const Vector2i& rel, int button, int modifiers)
{
    phi[0] = std::max(-1.57f, std::min(1.57f, phi[0] + 0.01f * (float)rel.x()));
    phi[1] = std::max(-1.57f, std::min(1.57f, phi[1] - 0.01f * (float)rel.y()));

    return true;
}

bool IKGLCanvas::scroll_event(const Vector2i& p, const Vector2f& rel)
{
    dist = std::max(2.0f, std::min(8.0f, dist - 0.1f * (float)rel.y()));
    return false;
}

void IKGLCanvas::resetCamera()
{
    phi[0] = phi[1] = 0.0f;
    dist = 4.0f;
}
