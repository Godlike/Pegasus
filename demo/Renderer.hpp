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

Mesh CreatePlane(glm::dvec3 normal, double distance, double length);

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
    for (Asset<T>& asset : data)
    {
        if (asset.id == 0)
        {
            return asset.id;
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
    using Callback = std::function<CallbackType>;
    template < typename CallbackType >
    using CallbackVector = std::vector<Callback<CallbackType>>;

    GLFWwindow* m_pWindow;
    CallbackVector<void(GLFWwindow*, int, int)> m_resizeCallbacks;
    CallbackVector<void(GLFWwindow*, int, int, int, int)> m_keyButtonCallbacks;
    CallbackVector<void(GLFWwindow*, double, double)> m_cursorMoveCallbacks;
    CallbackVector<void(GLFWwindow*, int, int, int)> m_mouseButtonCallbacks;

    Input();

    template < typename CallbackType >
    static void AddCallback(CallbackVector<CallbackType>& container, Callback<CallbackType>& callback)
    {
        for(auto it = container.begin(); it != container.end(); ++it)
        {
            if (it->template target<CallbackType>() == callback.template target<CallbackType>())
            {
                return;
            }
        }
        container.emplace_back(callback);
    }

    template < typename CallbackType >
    static void RemoveCallback(CallbackVector<CallbackType>& container, Callback<CallbackType>& callback)
    {
        for(auto it = container.begin(); it != container.end(); ++it)
        {
            if (it->template target<CallbackType>() == callback.template target<CallbackType>())
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
        int windowWidth = 600;
        int windowHeight = 600;
        int frameBufferWidth;
        int frameBufferHeight;
    };

    bool m_initialized;
    Window m_window;
    Camera m_camera;
    std::vector<asset::Asset<mesh::Mesh>> m_meshes;

    shader::Program m_program;
    GLint m_mvpUniformHandle;
    GLint m_colorUniformHandle;
    GLchar const* m_pVertexShaderSources =
    R"(
        #version 440

        layout (location = 0) in vec3 aPos;
        uniform vec3 color;
        uniform mat4 mvp;
        out vec4 vsColorOut;

        void main()
        {
            gl_Position = mvp * vec4(aPos, 1.0);
            vsColorOut = vec4(color, 1.0);
        }
    )";
    GLchar const* m_pFragmentShaderSources =
    R"(
        #version 440

        in  vec4 vsColorOut;
        out vec4 fragColor;

        void main()
        {
            fragColor = vsColorOut;
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
class Plane
{
public:
	Plane(Renderer& renderer, glm::mat4 model, glm::dvec3 normal, double distance, glm::vec3 color);

    Plane(Plane const&) = delete;

    Plane& operator=(Plane const&) = delete;

    Plane(Plane&& other) noexcept;

    Plane& operator=(Plane&& other) noexcept;

	~Plane();

    friend void swap(Plane& lhs, Plane& rhs) noexcept;

	glm::dvec3 GetNormal() const;

	double GetDistance() const;

    void SetModel(glm::mat4 const& model) const;

    glm::mat4 GetModel() const;

private:
    bool m_isInitialized;
	Renderer* m_pRenderer;
	Handle m_handle;
	glm::dvec3 m_normal;
	double m_distance;
	double m_sideLength;
};

class Sphere
{
public:
    Sphere(Renderer& renderer, glm::mat4 model, double radius, glm::dvec3 color);

    Sphere(Sphere const&) = delete;

    Sphere& operator=(Sphere const&) = delete;

    Sphere(Sphere&& other) noexcept;

    Sphere& operator=(Sphere&& other) noexcept;

    ~Sphere();

    friend void swap(Sphere& lhs, Sphere& rhs) noexcept;

    double GetRadius() const;

    void SetModel(glm::mat4 const& model) const;

    glm::mat4 GetModel() const;

private:
    bool m_isInitialized;
    Renderer* m_pRenderer;
    Handle m_handle;
    double m_radius;
};

class Box
{
public:
    struct Axes
    {
        glm::dvec3 i;
        glm::dvec3 j;
        glm::dvec3 k;
    };

    Box(Renderer& renderer, glm::mat4 model, Axes axes, glm::dvec3 color);

    Box(Box const&) = delete;

    Box& operator=(Box const&) = delete;

    Box(Box&& other) noexcept;

    Box& operator=(Box&& other) noexcept;

    ~Box();

    friend void swap(Box& lhs, Box& rhs) noexcept;

    Axes GetAxes() const;

    void SetModel(glm::mat4 const& model) const;

    glm::mat4 GetModel() const;

private:
    bool m_isInitialized;
    Renderer* m_pRenderer;
    Handle m_handle;
    Axes m_axes;
};
} // namespace primitive

} // namespace render
} // namesapce pegasus

#endif // PEGASUS_RENDERER_HPP
