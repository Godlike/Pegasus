/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#ifndef PEGASUS_RENDERER_HPP
#define PEGASUS_RENDERER_HPP

#include <glbinding/gl44ext/gl.h>
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include <glm/ext.hpp>

#include <vector>

using namespace gl;

namespace pegasus
{
namespace render
{
using Handle = uint32_t;

namespace mesh
{
struct Mesh
{
    std::vector<GLdouble> vertices;
    std::vector<GLuint> indices;
    glm::mat4 model = glm::mat4(1);
    glm::vec3 color = glm::vec3(1, 0, 0);

    struct BufferData
    {
        GLuint vertexArrayObject = 0;
        GLuint elementBufferObject = 0;
        GLuint vertexBufferObject = 0;
    };
    BufferData bufferData;
};

void Allocate(Mesh& mesh);

void Deallocate(Mesh& mesh);

Mesh Create(std::vector<GLdouble>&& vertices, std::vector<GLuint>&& indices);

Mesh CreateLineSegment(glm::vec3 start, glm::vec3 end);

Mesh CreatePlane(glm::dvec3 normal, double length);

Mesh CreateSphere(double radius, uint32_t depth);

Mesh CreateBox(glm::dvec3 i, glm::dvec3 j, glm::dvec3 k);

void Delete(Mesh& mesh);
} // namespace mesh

namespace asset
{
template < typename T >
struct Asset
{
    Handle id;
    T data;
};

template < typename T >
Handle Make(std::vector<Asset<T>>& data)
{
    for (size_t i = 0; i < data.size(); ++i)
    {
        if (data[i].id == 0)
        {
            data[i].id = static_cast<Handle>(i + 1);
            return data[i].id;
        }
    }

    data.resize(data.size() + 1);
    data.back().id = static_cast<Handle>(data.size());
    return data.back().id;
}

template < typename T >
T& Get(std::vector<Asset<T>>& data, Handle id)
{
    return data[id - 1].data;
}

template < typename T >
void Remove(std::vector<Asset<T>>& data, Handle id)
{
    data[id - 1].id = 0;
}
} // namespace asset

namespace shader
{
struct Shader
{
    GLuint handle;
    GLenum type;
    std::string info;
    bool valid;
};

Shader CompileShader(GLenum type, GLchar const sources[1]);

void DeleteShader(Shader const& shader);

struct Program
{
    struct Handles
    {
        GLuint vertexShader = 0;
        GLuint tesselationControlShader = 0;
        GLuint tesselationEvaluationShader = 0;
        GLuint geometryShader = 0;
        GLuint fragmentShader = 0;
    };

    GLuint handle;
    Handles shaders;
    std::string info;
    bool valid;
};

Program MakeProgram(Program::Handles shaders);
} // namespace shader

class Camera
{
public:
    Camera();

    void SetRatio(float ratio);

    void SetPosition(glm::vec3 position);

    void SetDirection(glm::vec3 direction);

    void SetUp(glm::vec3 up);

    glm::vec3 GetPosition() const;

    glm::vec3 GetDirection() const;

    glm::vec3 GetUp() const;

    glm::mat4 GetView() const;

    glm::mat4 GetProjection() const;

    float speed;

private:
    float m_angle;
    float m_ratio;
    float m_near;
    float m_far;
    glm::vec3 m_position;
    glm::vec3 m_direction;
    glm::vec3 m_up;
    glm::mat4 m_view;
    glm::mat4 m_projection;

    void UpdateView();

    void UpdateProjection();
};

class Input
{
public:
    static Input& GetInstance();

    static void InitializeContext(GLFWwindow* window);

    void AddResizeCallback(std::function<void(GLFWwindow*, int, int)> callback);

    void RemoveResizeCallback(std::function<void(GLFWwindow*, int, int)> callback);

    void AddKeyButtonCallback(std::function<void(GLFWwindow*, int, int, int, int)> callback);

    void RemoveKeyButtonCallback(std::function<void(GLFWwindow*, int, int, int, int)> callback);

    void AddCursoreMoveCallback(std::function<void(GLFWwindow*, double, double)> callback);

    void RemoveCursoreMoveCallback(std::function<void(GLFWwindow*, double, double)> callback);

    void AddMouseButtonCallback(std::function<void(GLFWwindow*, int, int, int)> callback);

    void RemoveMouseButtonCallback(std::function<void(GLFWwindow*, int, int, int)> callback);

private:
    template < typename CallbackType >
    using FunctionVector = std::vector<std::function<CallbackType>>;

    GLFWwindow* m_pWindow;
    FunctionVector<void(GLFWwindow*, int, int)> m_resizeCallbacks;
    FunctionVector<void(GLFWwindow*, int, int, int, int)> m_keyButtonCallbacks;
    FunctionVector<void(GLFWwindow*, double, double)> m_cursorMoveCallbacks;
    FunctionVector<void(GLFWwindow*, int, int, int)> m_mouseButtonCallbacks;

    Input();

    template < typename Ret, typename... Args >
    static size_t GetAddress(std::function<Ret(Args...)> f)
    {
        typedef Ret(FnType)(Args...);
        FnType ** fnPointer = f.template target<FnType*>();
        return reinterpret_cast<size_t>(*fnPointer);
    }

    template < typename Ret, typename... Args >
    static void AddCallback(FunctionVector<Ret(Args...)>& container, std::function<Ret(Args...)> callback)
    {
        for(auto function : container)
        {
            if (GetAddress(function) == GetAddress(callback))
            {
                return;
            }
        }
        container.push_back(callback);
    }

    template < typename Ret, typename... Args >
    static void RemoveCallback(FunctionVector<Ret(Args...)>& container, std::function<Ret(Args...)> callback)
    {
        for(auto it = container.begin(); it != container.end(); ++it)
        {
            if (GetAddress(*it) == GetAddress(callback))
            {
                container.erase(it);
                return;
            }
        }
    }

    static void Resize(GLFWwindow* window, int width, int height);

    static void KeyButton(GLFWwindow* window, int key, int scancode, int action, int mods);

    static void CursorMove(GLFWwindow* window, double xpos, double ypos);

    static void MouseButton(GLFWwindow* window, int button, int action, int mods);
};

class Renderer
{
public:
    Renderer(Renderer const&) = delete;
    Renderer& operator==(Renderer const&) = delete;
    Renderer(Renderer&&) = delete;
    Renderer& operator==(Renderer&&) = delete;

    static Renderer& GetInstance();

    bool IsValid() const;

    void RenderFrame();

    Handle MakeMesh();

    mesh::Mesh& GetMesh(Handle id);

    void RemoveMesh(Handle id);

private:
    struct Window
    {
        GLFWwindow* pWindow;
        int windowWidth = 800;
        int windowHeight = 800;
        int frameBufferWidth;
        int frameBufferHeight;
    };

    bool m_initialized;
    Window m_window;
    Camera m_camera;
    std::vector<asset::Asset<mesh::Mesh>> m_meshes;

    shader::Program m_program;
    GLint m_mvpUniformHandle;
    GLint m_modelUniformHandle;
    GLint m_colorUniformHandle;
    GLint m_lightUniformHandle;
    GLint m_eyeUniformHandle;
    GLchar const* m_pVertexShaderSources =
    R"(
        #version 440

        uniform mat4 model;
        uniform mat4 mvp;

        layout (location = 0) in vec3 aPos;
        layout (location = 1) in vec3 aNormal;

        out vec3 fragPos;
        out vec3 fragNormal;

        void main()
        {
            gl_Position = mvp * vec4(aPos, 1.0);
            fragPos = vec3(model * vec4(aPos, 1.0));
            fragNormal = aNormal;
        }
    )";
    GLchar const* m_pFragmentShaderSources =
    R"(
        #version 440

        uniform vec3 eye;
        uniform vec3 light;
        uniform vec3 color;

        in vec3 fragPos;
        in vec3 fragNormal;
        out vec4 fragColor;

        void main()
        {
            vec3 lightColor = vec3(0.956, 1.000, 0.980);

            float ambientStrength = 0.75;
            vec3 ambient = ambientStrength * lightColor;

            float diff = max(dot(fragNormal, light), 0.0);
            vec3 diffuse = diff * lightColor;

            float specularStrength = 0.5;
            vec3 viewDir = normalize(eye - fragPos);
            vec3 reflectDir = reflect(-light, fragNormal);
            float spec = pow(max(dot(viewDir, reflectDir), 0.0), 16);
            vec3 specular = specularStrength * spec * lightColor;

            fragColor = vec4((ambient + diffuse + specular) * color, 1.0);
        }
    )";

    Renderer();

    ~Renderer();

    void InitializeGlfw();

    void InitializeContext();

    void InitializeCallbacks() const;

    void InitializeShaderProgram();

    static void Resize(GLFWwindow* window, int width, int height);

    static void KeyButton(GLFWwindow* window, int key, int scancode, int action, int mods);

    static void CursorMove(GLFWwindow* window, double xpos, double ypos);
};

namespace primitive
{
class Primitive
{
public:
    Primitive();

    ~Primitive();

    void SetModel(glm::mat4 model) const;

    glm::mat4 GetModel() const;

protected:
    bool m_initialized;
    Renderer* m_pRenderer;
    Handle m_meshHandle;
};

class LineSegment : public Primitive
{
public:
    LineSegment(glm::mat4 model, glm::vec3 color, glm::vec3 start, glm::vec3 end);

    glm::vec3 GetStart() const;

    glm::vec3 GetEnd() const;

private:
    glm::vec3 m_start;
    glm::vec3 m_end;
};

class Plane : public Primitive
{
public:
    Plane(glm::mat4 model, glm::vec3 color, glm::dvec3 normal);

    glm::dvec3 GetNormal() const;

private:
    glm::dvec3 m_normal;
    double m_sideLength;
};

class Sphere : public Primitive
{
public:
    Sphere(glm::mat4 model, glm::dvec3 color, double radius);

    double GetRadius() const;

private:
    double m_radius;
};

class Box : public Primitive
{
public:
    struct Axes
    {
        glm::dvec3 i;
        glm::dvec3 j;
        glm::dvec3 k;
    };

    Box(glm::mat4 model, glm::dvec3 color, Axes axes);

    Axes GetAxes() const;

private:
    Axes m_axes;
};
} // namespace primitive

} // namespace render
} // namesapce pegasus

#endif // PEGASUS_RENDERER_HPP
