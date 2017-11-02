/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#ifndef PEGASUS_DEMO_RENDER_HPP
#define PEGASUS_DEMO_RENDER_HPP

#include "demo/Gl.hpp"
#include <glm/ext.hpp>
#include <vector>

namespace pegasus
{
namespace render
{
using Handle = uint32_t;

namespace mesh
{
/** Stores mesh data and GPU OpenGL handle information */
struct Mesh
{
    std::vector<GLdouble> vertices;
    std::vector<GLuint> indices;
    glm::mat4 model = glm::mat4(1);
    glm::vec3 color = glm::vec3(1, 0, 0);

    /** Stores OpenGL vertex array and vertex buffer handles */
    struct BufferData
    {
        GLuint vertexArrayObject = 0;
        GLuint elementBufferObject = 0;
        GLuint vertexBufferObject = 0;
    };
    BufferData bufferData;
};

/**
 * @brief Copies mesh data to the GPU memory and updates mesh with its handles
 * @param[in,out] mesh holds vertex data and OpenGL handles
 */
void Allocate(Mesh& mesh);

/**
 * @brief Removes given mesh from the GPU memory and updates its handles
 * @param[in,out] mesh holds vertex data and OpenGL handles
 */
void Deallocate(Mesh& mesh);

/**
 * @brief Makes mesh and copies its data to the GPU memory
 * @param[in] vertices vertex data
 * @param[in] indices index data
 * @return created mesh
 */
Mesh Create(std::vector<GLdouble>&& vertices, std::vector<GLuint>&& indices);

/**
 * @brief Makes line segment mesh and copies its data to the GPU memory
 * @param[in] start first point of the line segment
 * @param[in] end second point of the line segment
 * @return created mesh
 */
Mesh CreateLineSegment(glm::vec3 start, glm::vec3 end);

/**
 * @brief Makes plane mesh and copies its data to the GPU memory
 * @param[in] normal plane normal
 * @param[in] length length of the side of the plane
 * @return created mesh
 */
Mesh CreatePlane(glm::dvec3 normal, double length);

/**
 * @brief Makes ICO sphere mesh and copies its data to the GPU memory
 * @param[in] radius sphere's radius
 * @param[in] depth sphere's level of the details
 * @return created mesh
 */
Mesh CreateSphere(double radius, uint32_t depth);

/**
 * @brief Makes box mesh and copies its data to the GPU memory
 * @param[in] i orthogonal basis vector of the box
 * @param[in] j orthogonal basis vector of the box
 * @param[in] k orthogonal basis vector of the box
 * @return created mesh
 */
Mesh CreateBox(glm::dvec3 i, glm::dvec3 j, glm::dvec3 k);

/**
 * @brief Deallocates GPU and local memory used by the mesh
 * @param[in] mesh reference to the mesh
 */
void Delete(Mesh& mesh);
} // namespace mesh

namespace shader
{
/** Stores shader's program data */
struct Shader
{
    GLuint handle;
    GLenum type;
    std::string info;
    bool valid;
};

/**
 * @brief Compiles shader
 * @param[in] type OpenGL shader type
 * @param[in] sources shader's source code
 * @return shader handle
 */
Shader CompileShader(GLenum type, GLchar const sources[1]);

/**
 * @brief Deletes given shader
 * @param shader reference to the handle data
 */
void DeleteShader(Shader const& shader);

/** Stores OpenGL shader program data */
struct Program
{
    /** Stores compiled shader handles */
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

/**
 * @brief Creates program and links given @p shaders to it
 * @param shaders handles to the shaders
 * @return shader program handle data
 */
Program MakeProgram(Program::Handles shaders);
} // namespace shader

/** Stores data to generate view projection matrices */
class Camera
{
public:
    /** Construct default initialized camera */
    Camera();

    /**
     * @brief Sets aspect ration of the screen
     * @param ratio width Creates an object describing ad by height
     */
    void SetRatio(float ratio);

    /**
     * @brief Sets position of the camera to be used in the view matrix calculation
     * @param position eye location
     */
    void SetPosition(glm::vec3 position);

    /**
     * @brief Sets direction of the camera to be used in the view matrix calculation
     * @param direction normalized unit vector
     */
    void SetDirection(glm::vec3 direction);

    /**
     * @brief Sets vertical axis of the camera to be used in the view matrix calculation
     * @param up normalized unit vector
     */
    void SetUp(glm::vec3 up);

    /**
     * @brief Returns camera position
     * @return eye location
     */
    glm::vec3 GetPosition() const;

    /**
     * @brief Returns camera direction
     * @return normalized unit vector
     */
    glm::vec3 GetDirection() const;

    /**
     * @brief Returns camera vertical axis vector
     * @return normalized unit vector
     */
    glm::vec3 GetUp() const;

    /**
     * @brief Returns calculated view matrix
     * @return transformation of the space from the camera perspective
     */
    glm::mat4 GetView() const;

    /**
     * @brief Returns calculated projection matrix
     * @return transformation of the space into the camera frustum
     */
    glm::mat4 GetProjection() const;

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

    /** Recalculates view matrix */
    void UpdateView();

    /** Recalculates projection matrix */
    void UpdateProjection();
};

/** Singleton class for handling input events */
class Input
{
public:
    /**
     * @brief Returns a reference to the singleton instance of the class
     * @return reference to the instance
     */
    static Input& GetInstance();

    /**
     * @brief Initializes input with the given context
     * @param[in] window callbacks context pointer
     */
    static void InitializeContext(GLFWwindow* window);

    /**
     * @brief Pushes additional callback for the Resize event
     * @param callback function object containing the pointer to the callback function
     */
    void AddResizeCallback(void(*callback)(GLFWwindow*, int, int));

    /**
     * @brief Removes given callback function from the Resize event call list
     * @param callback pointer to the callback function to be removed
     */
    void RemoveResizeCallback(void(*callback)(GLFWwindow*, int, int));

    /**
     * @brief Pushes additional callback for the KeyButton event
     * @param callback function object containing the pointer to the callback function
     */
    void AddKeyButtonCallback(void(*callback)(GLFWwindow*, int, int, int, int));

    /**
     * @brief Removes given callback function from the KeyButton event call list
     * @param callback pointer to the callback function to be removed
     */
    void RemoveKeyButtonCallback(void(*callback)(GLFWwindow*, int, int, int, int));

    /**
     * @brief Pushes additional callback for the CursorMove event
     * @param callback function object containing the pointer to the callback function
     */
    void AddCursorMoveCallback(void(*callback)(GLFWwindow*, double, double));

    /**
     * @brief Removes given callback function from the CursorMove event call list
     * @param callback pointer to the callback function to be removed
     */
    void RemoveCursorMoveCallback(void(*callback)(GLFWwindow*, double, double));

    /**
     * @brief Pushes additional callback for the MouseButton event
     * @param callback function object containing the pointer to the callback function
     */
    void AddMouseButtonCallback(void(*callback)(GLFWwindow*, int, int, int));

    /**
     * @brief Removes given callback function from the MouseButton event call list
     * @param callback pointer to the callback function to be removed
     */
    void RemoveMouseButtonCallback(void(*callback)(GLFWwindow*, int, int, int));

private:
    GLFWwindow* m_pWindow;
    std::vector<void(*)(GLFWwindow*, int, int)> m_resizeCallbacks;
    std::vector<void(*)(GLFWwindow*, int, int, int, int)> m_keyButtonCallbacks;
    std::vector<void(*)(GLFWwindow*, double, double)> m_cursorMoveCallbacks;
    std::vector<void(*)(GLFWwindow*, int, int, int)> m_mouseButtonCallbacks;

    /**
     * @brief Constructs default initialized instance of the callback system
     */
    Input();

    /**
     * @brief Pushes new callback to the given function vector
     * @tparam Ret return type of the function
     * @tparam Args arguments type of the function
     * @param container vector of the function objects
     * @param callback new callback
     */
    template < typename Ret, typename... Args >
    static void AddCallback(std::vector<Ret(*)(Args...)>& container, Ret(*callback)(Args...))
    {
        for(Ret(*f)(Args...) : container)
        {
            if (f == callback)
            {
                return;
            }
        }
        container.push_back(callback);
    }

    /**
     * @brief Removes given callback from callback container
     * @tparam Ret return type of the function
     * @tparam Args arguments type of the function
     * @param container container of the function objects
     * @param callback callback to be deleted
     */
    template < typename Ret, typename... Args >
    static void RemoveCallback(std::vector<Ret(*)(Args...)>& container, Ret(*callback)(Args...))
    {
        for(auto it = container.begin(); it != container.end(); ++it)
        {
            if (*it == callback)
            {
                container.erase(it);
                return;
            }
        }
    }

    /**
     * @brief Initial resize callback that calls user provided resize callbacks
     * @param[in] window GLFW window pointer for the callback
     * @param[in] width new width of the screen
     * @param[in] height new height of the screen
     */
    static void Resize(GLFWwindow* window, int width, int height);

    /**
     * @brief Initial keybutton callback that calls user provided keybutton callbacks
     * @param[in] window GLFW window pointer for the callback
     * @param[in] key button id
     * @param[in] scancode
     * @param[in] action action type
     * @param[in] mods
     */
    static void KeyButton(GLFWwindow* window, int key, int scancode, int action, int mods);

    /**
     * @brief Initial cursormove callback that calls user provided cursormove callbacks
     * @param[in] window GLFW window pointer for the callback
     * @param[in] xpos new mouse x screen coordinate
     * @param[in] ypos new mouse y screen coordinate
     */
    static void CursorMove(GLFWwindow* window, double xpos, double ypos);

    /**
     * @brief Initial mousebutton callback that calls user provided mousebutton callbacks
     * @param[in] window GLFW window pointer for the callback
     * @param[in] button button id
     * @param[in] action action type
     * @param[in] mods
     */
    static void MouseButton(GLFWwindow* window, int button, int action, int mods);
};

/** Represent singleton instance of the renderer */
class Renderer
{
public:
    Renderer(Renderer const&) = delete;
    Renderer& operator==(Renderer const&) = delete;
    Renderer(Renderer&&) = delete;
    Renderer& operator==(Renderer&&) = delete;

    /**
     * @brief Returns instance of the singleton renderer
     *
     * @attention First time must be called from the main thread
     * @return reference to the renderer
     */
    static Renderer& GetInstance();

    /**
     * @brief Checks if the renderer is in the valid state
     * @return @c true if renderer is initialized and the window is opened, @c false otherwise
     */
    bool IsValid() const;

    /**
     * @brief Draws new frame and swaps buffers
     */
    void RenderFrame();

    /**
     * @brief Finds free or creates new mesh
     * @return handle of the mesh
     */
    Handle MakeMesh();

    /**
     * @brief Return reference to the mesh associated with the handle
     * @param id handle of the mesh
     * @return reference to the mesh
     */
    mesh::Mesh& GetMesh(Handle id);

    /**
     * @brief Marks mesh associated with the handle as free
     * @param id mesh handle
     */
    void RemoveMesh(Handle id);

private:
    /** Stores GLFW window handler and window related information */
    struct Window
    {
        GLFWwindow* pWindow;
        int windowWidth = 800;
        int windowHeight = 800;
        int frameBufferWidth;
        int frameBufferHeight;
    };

    /**
     * @brief Represents a unit of storage with id
     * @tparam T storage data type
     */
    template < typename T >
    struct Asset
    {
        Handle id;
        T data;
    };

    bool m_initialized;
    Window m_window;
    Camera m_camera;
    std::vector<Asset<mesh::Mesh>> m_meshes;

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

    /**
     * @brief Construct default initialized render instance
     * @attention must be called from the main thread
     */
    Renderer();

    /**
     * @brief Deinitializes GLFW
     */
    ~Renderer();

    /**
     * @brief Initializes GLFW library
     * @attention must be called from the main thread
     */
    void InitializeGlfw();

    /**
     * @brief Initializes GLFW context
     * @attention must be called from the main thread
     */
    void InitializeContext();

    /**
     * @brief Initializes GLFW callbacks
     * @attention must be called from the main thread
     */
    void InitializeCallbacks() const;

    /**
     * @brief Compiles and links the shader program
     */
    void InitializeShaderProgram();

    /**
     * @brief Returns a handle for @p T data
     *
     * Searches for an empty handle in @p data. If there are no empty handles, creates a new one.
     *
     * @tparam T storage data type
     * @param[in,out] data reference to the vector of assets
     * @return handle to the free element
     */
    template < typename T >
    Handle MakeAsset(std::vector<Asset<T>>& data)
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

    /**
     * @brief Return reference to the data assigned to the given vector
     *
     * @attention Reference might be invalidated after the push to the given vector
     *
     * @tparam T storage data type
     * @param[in] data assets vector
     * @param[in] id handle of the data
     * @return reference to the data
     */
    template < typename T >
    T& GetAsset(std::vector<Asset<T>>& data, Handle id)
    {
        return data[id - 1].data;
    }

    /**
     * @brief Marks element as free
     * @tparam T storage data type
     * @param[in,out] data vector of assets
     * @param[in] id handle to the element
     */
    template < typename T >
    void RemoveAsset(std::vector<Asset<T>>& data, Handle id)
    {
        data[id - 1].id = 0;
    }

    /**
     * @brief Resize callback function
     * @param[in] window pointer to the current GLFW context window
     * @param[in] width new width of the window
     * @param[in] height
     */
    static void Resize(GLFWwindow* window, int width, int height);

    /**
     * @brief KeyButton callback function
     * @param[in] window pointer to the current GLFW context window
     * @param[in] key button id
     * @param[in] scancode
     * @param[in] action action type
     * @param[in] mods
     */
    static void KeyButton(GLFWwindow* window, int key, int scancode, int action, int mods);

    /**
     * @brief CursorMove callback function
     * @param[in] window pointer to the current GLFW context window
     * @param[in] xpos new mouse x value in the window coordinates
     * @param[in] ypos new mouse y value in the window coordinates
     */
    static void CursorMove(GLFWwindow* window, double xpos, double ypos);
};

/** Hight level primitive initialization class for the render */
class Primitive
{
public:
    /** Allocates memory for the mesh data in the renderer */
    Primitive();

    /** Deallocates mesh memory */
    ~Primitive();

    /**
     * @brief Sets model matrix for the given primitive's mesh
     * @param model stores local to world space mesh transformation stores local to world space mesh transformation
     */
    void SetModel(glm::mat4 model) const;

    /**
     * @brief Returns model matrix of the mesh
     * @return model matrix
     */
    glm::mat4 GetModel() const;

protected:
    Renderer* m_pRenderer;
    Handle m_meshHandle;
};

/** Line segment mesh data container */
class LineSegment : public Primitive
{
public:
    /**
     * @brief Creates new line segment mesh in the renderer
     * @param model stores local to world space mesh transformation
     * @param color primitive's color in the float rgb format
     * @param start first point of the line segment
     * @param end second point of the line segment
     */
    LineSegment(glm::mat4 model, glm::vec3 color, glm::vec3 start, glm::vec3 end);

    /**
     * @brief Returns first point of the line segment in the model space
     * @return point of the line segment
     */
    glm::vec3 GetStart() const;

    /**
     * @brief Returns second point of the line segment in the model space
     * @return point of the line segment
     */
    glm::vec3 GetEnd() const;

private:
    glm::vec3 m_start;
    glm::vec3 m_end;
};

/** Plane mesh data container */
class Plane : public Primitive
{
public:
    /**
     * @brief Creates new plane in the renderer
     * @param model stores local to world space mesh transformation
     * @param color primitive's color in the float rgb format
     * @param normal plane normal unit vector
     */
    Plane(glm::mat4 model, glm::vec3 color, glm::vec3 normal);

    /**
     * @brief Returns plane normal
     * @return normalized unit vector
     */
    glm::vec3 GetNormal() const;

private:
    glm::vec3 m_normal;
    double m_sideLength;
};

/** ICO sphere mesh data container */
class Sphere : public Primitive
{
public:
    /**
     * @brief Creates new ico sphere mesh in the renderer
     * @param model stores local to world space mesh transformation
     * @param color primitive's color in the float rgb format
     * @param radius sphere's radius
     */
    Sphere(glm::mat4 model, glm::vec3 color, double radius);

    /**
     * @brief Returns sphere's radius
     * @return sphere's radius
     */
    double GetRadius() const;

private:
    double m_radius;
};

/** Box mesh data container */
class Box : public Primitive
{
public:
    /** Stores orthogonal basis axes of the box */
    struct Axes
    {
        glm::vec3 i;
        glm::vec3 j;
        glm::vec3 k;
    };

    /**
     * @brief Creates new box mesh in the renderer
     * @param model stores local to world space mesh transformation
     * @param color primitive's color in the float rgb format
     * @param axes basis axes of the box
     */
    Box(glm::mat4 model, glm::vec3 color, Axes axes);

    /**
     * @brief Returns axes of the box
     * @return orthogonal axes basis
     */
    Axes GetAxes() const;

private:
    Axes m_axes;
};

} // namespace render
} // namespace pegasus

#endif // PEGASUS_DEMO_RENDER_HPP
