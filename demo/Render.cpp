/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#include "demo/Render.hpp"
#include <Epona/Analysis.hpp>
#include <imgui.h>
#include <imgui_impl_glfw_gl3.h>

using namespace pegasus;
using namespace render;

void mesh::Allocate(Mesh& mesh)
{
    //Generate VBO and EBO
    glGenVertexArrays(1, &mesh.bufferData.vertexArrayObject);
    glGenBuffers(1, &mesh.bufferData.vertexBufferObject);
    glGenBuffers(1, &mesh.bufferData.elementBufferObject);

    //Initialize VBO and EBO data
    glBindVertexArray(mesh.bufferData.vertexArrayObject);

    glBindBuffer(GL_ARRAY_BUFFER, mesh.bufferData.vertexBufferObject);
    glBufferData(GL_ARRAY_BUFFER, sizeof(GLdouble) * mesh.vertices.size(), &mesh.vertices.front(), GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mesh.bufferData.elementBufferObject);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(GLuint) * mesh.indices.size(), &mesh.indices.front(), GL_STATIC_DRAW);

    //Initialize VBO arguments
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_DOUBLE, GL_FALSE, sizeof(GLdouble) * 6,
        reinterpret_cast<void*>(sizeof(GLdouble) * 0));

    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_DOUBLE, GL_FALSE, sizeof(GLdouble) * 6,
        reinterpret_cast<void*>(sizeof(GLdouble) * 3));

    glBindVertexArray(0);
}

void mesh::Deallocate(Mesh& mesh)
{
    glDeleteVertexArrays(1, &mesh.bufferData.vertexArrayObject);
    glDeleteBuffers(1, &mesh.bufferData.vertexBufferObject);
    glDeleteBuffers(1, &mesh.bufferData.elementBufferObject);
}

mesh::Mesh mesh::Create(std::vector<GLdouble>&& vertices, std::vector<GLuint>&& indices)
{
    Mesh mesh{ std::move(vertices), std::move(indices) };
    Allocate(mesh);
    return mesh;
}

mesh::Mesh mesh::CreateLineSegment(glm::vec3 start, glm::vec3 end)
{
    Mesh mesh;

    mesh.vertices = {
        start.x, start.y, start.z, 1, 0, 0,
        end.x, end.y, end.z, 0, 1, 0,
        end.x, end.y, end.z, 0, 0, 1,
    };
    mesh.indices = { 0, 1, 2 };
    Allocate(mesh);

    return mesh;
}

mesh::Mesh mesh::CreatePlane(glm::dvec3 normal, double length)
{
    Mesh mesh;

    glm::dvec3 const i = epona::CalculateOrthogonalVector(normal) * (length / 2.0);
    glm::dvec3 const j = glm::normalize(glm::cross(i, normal)) * (length / 2.0);

    mesh.vertices = {
        ( i + j).x, ( i + j).y, ( i + j).z, normal.x, normal.y, normal.z,
        (-i + j).x, (-i + j).y, (-i + j).z, normal.x, normal.y, normal.z,
        ( i - j).x, ( i - j).y, ( i - j).z, normal.x, normal.y, normal.z,
        (-i - j).x, (-i - j).y, (-i - j).z, normal.x, normal.y, normal.z,
    };
    mesh.indices = { 0, 1, 2, 1, 2, 3 };
    Allocate(mesh);

    return mesh;
}

mesh::Mesh mesh::CreateTriangle(glm::dvec3 a, glm::dvec3 b, glm::dvec3 c)
{
    Mesh mesh;
    glm::dvec3 const normal = glm::normalize(glm::cross(a - b, a - c));
    mesh.vertices = { 
        a.x, a.y, a.z, normal.x, normal.y, normal.z,
        b.x, b.y, b.z, normal.x, normal.y, normal.z,
        c.x, c.y, c.z, normal.x, normal.y, normal.z,
    };
    mesh.indices = { 0, 1, 2 };
    Allocate(mesh);

    return mesh;
}

mesh::Mesh mesh::CreateSphere(double radius, uint32_t depth)
{
    //Initial hexahedron
    Mesh mesh;
    mesh.vertices = {
        0, 0, radius,
        0, radius, 0,
        radius, 0, 0,
        0, -radius, 0,
        -radius, 0, 0,
        0, 0, -radius,
    };
    mesh.indices = {
        0, 1, 2, 0, 2, 3, 0, 3, 4, 0, 1, 4,
        1, 2, 5, 2, 3, 5, 3, 4, 5, 1, 4, 5,
    };

    //Subdivide
    for(uint32_t i = 0; i < depth; ++i)
    {
        for (int64_t j = mesh.indices.size() - 3; j >= 0; j-=3)
        {
            glm::u32vec3 indices(mesh.indices[j], mesh.indices[j + 1], mesh.indices[j + 2]);
            glm::dvec3 const a(mesh.vertices[indices[0]*3],mesh.vertices[indices[0]*3+1], mesh.vertices[indices[0]*3+2]);
            glm::dvec3 const b(mesh.vertices[indices[1]*3],mesh.vertices[indices[1]*3+1], mesh.vertices[indices[1]*3+2]);
            glm::dvec3 const c(mesh.vertices[indices[2]*3],mesh.vertices[indices[2]*3+1], mesh.vertices[indices[2]*3+2]);
            glm::dvec3 const ab = glm::normalize(a + b) * radius;
            glm::dvec3 const bc = glm::normalize(b + c) * radius;
            glm::dvec3 const ca = glm::normalize(c + a) * radius;

            GLuint const index = static_cast<GLuint>(mesh.vertices.size()) / 3 - 1;
            mesh.indices.insert(mesh.indices.end(), {
                indices[0], index + 1, index + 3,
                indices[1], index + 1, index + 2,
                indices[2], index + 2, index + 3,
                index + 1, index + 2, index + 3
            });
            mesh.indices.erase(mesh.indices.begin() + j, mesh.indices.begin() + j + 3);

            mesh.vertices.insert(mesh.vertices.end(), { ab.x, ab.y, ab.z, bc.x, bc.y, bc.z, ca.x, ca.y, ca.z });
        }
    }

    //Insert normals
    mesh.vertices.reserve(mesh.vertices.size() * 2);
    for(size_t i = 0; i < mesh.vertices.size(); i+=6)
    {
        glm::dvec3 const normal = glm::normalize(glm::dvec3{
            mesh.vertices[i+0], mesh.vertices[i+1], mesh.vertices[i+2]
        });

        mesh.vertices.insert(mesh.vertices.begin() + i + 3, normal.x);
        mesh.vertices.insert(mesh.vertices.begin() + i + 4, normal.y);
        mesh.vertices.insert(mesh.vertices.begin() + i + 5, normal.z);
    }
    Allocate(mesh);

    return mesh;
}

mesh::Mesh mesh::CreateBox(glm::dvec3 i, glm::dvec3 j, glm::dvec3 k)
{
    Mesh mesh;
    glm::dvec3 const iNormal = glm::normalize(i);
    glm::dvec3 const jNormal = glm::normalize(j);
    glm::dvec3 const kNormal = glm::normalize(k);
    mesh.vertices = {
        //top
        ( i + j + k).x, ( i + j + k).y, ( i + j + k).z, kNormal.x, kNormal.y, kNormal.z,
        (-i + j + k).x, (-i + j + k).y, (-i + j + k).z, kNormal.x, kNormal.y, kNormal.z,
        ( i - j + k).x, ( i - j + k).y, ( i - j + k).z, kNormal.x, kNormal.y, kNormal.z,
        (-i - j + k).x, (-i - j + k).y, (-i - j + k).z, kNormal.x, kNormal.y, kNormal.z,

        //right
        ( i + j + k).x, ( i + j + k).y, ( i + j + k).z, jNormal.x, jNormal.y, jNormal.z,
        (-i + j + k).x, (-i + j + k).y, (-i + j + k).z, jNormal.x, jNormal.y, jNormal.z,
        ( i + j - k).x, ( i + j - k).y, ( i + j - k).z, jNormal.x, jNormal.y, jNormal.z,
        (-i + j - k).x, (-i + j - k).y, (-i + j - k).z, jNormal.x, jNormal.y, jNormal.z,

        //left
        ( i - j + k).x, ( i - j + k).y, ( i - j + k).z, -jNormal.x, -jNormal.y, -jNormal.z,
        (-i - j + k).x, (-i - j + k).y, (-i - j + k).z, -jNormal.x, -jNormal.y, -jNormal.z,
        ( i - j - k).x, ( i - j - k).y, ( i - j - k).z, -jNormal.x, -jNormal.y, -jNormal.z,
        (-i - j - k).x, (-i - j - k).y, (-i - j - k).z, -jNormal.x, -jNormal.y, -jNormal.z,

        //front
        ( i + j + k).x, ( i + j + k).y, ( i + j + k).z, iNormal.x, iNormal.y, iNormal.z,
        ( i - j + k).x, ( i - j + k).y, ( i - j + k).z, iNormal.x, iNormal.y, iNormal.z,
        ( i + j - k).x, ( i + j - k).y, ( i + j - k).z, iNormal.x, iNormal.y, iNormal.z,
        ( i - j - k).x, ( i - j - k).y, ( i - j - k).z, iNormal.x, iNormal.y, iNormal.z,

        //back
        (-i + j + k).x, (-i + j + k).y, (-i + j + k).z, -iNormal.x, -iNormal.y, -iNormal.z,
        (-i - j + k).x, (-i - j + k).y, (-i - j + k).z, -iNormal.x, -iNormal.y, -iNormal.z,
        (-i + j - k).x, (-i + j - k).y, (-i + j - k).z, -iNormal.x, -iNormal.y, -iNormal.z,
        (-i - j - k).x, (-i - j - k).y, (-i - j - k).z, -iNormal.x, -iNormal.y, -iNormal.z,

        //bottom
        ( i + j - k).x, ( i + j - k).y, ( i + j - k).z, -kNormal.x, -kNormal.y, -kNormal.z,
        (-i + j - k).x, (-i + j - k).y, (-i + j - k).z, -kNormal.x, -kNormal.y, -kNormal.z,
        ( i - j - k).x, ( i - j - k).y, ( i - j - k).z, -kNormal.x, -kNormal.y, -kNormal.z,
        (-i - j - k).x, (-i - j - k).y, (-i - j - k).z, -kNormal.x, -kNormal.y, -kNormal.z,
    };
    mesh.indices = {
        0, 1, 2, 1, 2, 3,       //top
        4, 5, 6, 5, 6, 7,       //right
        8, 9, 10, 9, 10, 11,    //left
        12, 13, 14, 13, 14, 15, //front
        16, 17, 18, 17, 18, 19, //back
        20, 21, 22, 21, 22, 23, //bottom
    };
    Allocate(mesh);

    return mesh;
}

mesh::Mesh mesh::CreateTriangleCollection(std::vector<glm::mat3> triangles)
{
    Mesh mesh;
    for (size_t i = 0; i < triangles.size(); ++i)
    {
        glm::dvec3 const normal = glm::normalize(
            glm::cross(triangles[i][0] - triangles[i][1], triangles[i][0] - triangles[i][2])
        );

        mesh.vertices.insert(mesh.vertices.end(), { 
            triangles[i][0].x, triangles[i][0].y, triangles[i][0].z, normal.x, normal.y, normal.z, 
            triangles[i][1].x, triangles[i][1].y, triangles[i][1].z, normal.x, normal.y, normal.z, 
            triangles[i][2].x, triangles[i][2].y, triangles[i][2].z, normal.x, normal.y, normal.z,
        });
        mesh.indices.insert(mesh.indices.end(), { 
            static_cast<GLuint>(i * 3 + 0), 
            static_cast<GLuint>(i * 3 + 1), 
            static_cast<GLuint>(i * 3 + 2)
        });
    }
    Allocate(mesh);

    return mesh;
}

void mesh::Delete(Mesh& mesh)
{
    std::vector<GLuint>().swap(mesh.indices);
    std::vector<GLdouble>().swap(mesh.vertices);
    mesh.model = glm::dmat4();
    Deallocate(mesh);
}

shader::Shader shader::CompileShader(GLenum type, const GLchar sources[1])
{
    Shader result{glCreateShader(type), type};

    glShaderSource(result.handle, 1, &sources, nullptr);
    glCompileShader(result.handle);

    GLint success = 0;
    glGetShaderiv(result.handle, GL_COMPILE_STATUS, &success);
    result.valid = static_cast<bool>(success);

    if (!result.valid)
    {
        result.info.resize(512);
        glGetShaderInfoLog(result.handle, static_cast<GLsizei>(result.info.size()), nullptr, &result.info.front());
    }

    return result;
}

void shader::DeleteShader(Shader const& shader)
{
    glDeleteShader(shader.handle);
}

shader::Program shader::MakeProgram(Program::Handles shaders)
{
    Program result{glCreateProgram(), shaders};

    glAttachShader(result.handle, shaders.vertexShader);
    if (shaders.tesselationControlShader) {
        glAttachShader(result.handle, shaders.tesselationControlShader);
    }
    if (shaders.tesselationEvaluationShader) {
        glAttachShader(result.handle, shaders.tesselationEvaluationShader);
    }
    if (shaders.geometryShader) {
        glAttachShader(result.handle, shaders.geometryShader);
    }
    glAttachShader(result.handle, shaders.fragmentShader);

    glLinkProgram(result.handle);

    GLint success = 0;
    glGetProgramiv(result.handle, GL_LINK_STATUS, &success);
    result.valid = static_cast<bool>(success);

    if (!result.valid)
    {
        result.info.resize(512);
        glGetProgramInfoLog(result.handle, static_cast<GLsizei>(result.info.size()), nullptr, &result.info.front());
    }

    return result;
}

Camera::Camera()
    : m_angle(60.0f)
    , m_ratio(1.0f)
    , m_near(0.1f)
    , m_far(1000.0f)
    , m_position(0.0f, 0.0f, 0.0f)
    , m_direction(1.0f, 0.0f, 0.0)
    , m_up(0.0f, 1.0f, 0.0f)
    , m_view(1.0f)
    , m_projection(1.0f)
{
    UpdateView();
    UpdateProjection();
}

void Camera::SetRatio(float ratio)
{
    m_ratio = ratio;
    UpdateProjection();
}

void Camera::SetPosition(glm::vec3 position)
{
    m_position = position;
    UpdateView();
}

void Camera::SetDirection(glm::vec3 direction)
{
    m_direction = direction;
    UpdateView();
}

void Camera::SetUp(glm::vec3 up)
{
    m_up = up;
    UpdateView();
}

glm::vec3 Camera::GetPosition() const
{
    return m_position;
}

glm::vec3 Camera::GetDirection() const
{
    return m_direction;
}

glm::vec3 Camera::GetUp() const
{
    return m_up;
}

glm::mat4 Camera::GetView() const
{
    return m_view;
}

glm::mat4 Camera::GetProjection() const
{
    return m_projection;
}

void Camera::UpdateView()
{
    m_view = glm::lookAt(m_position, m_position + m_direction, m_up);
}

void Camera::UpdateProjection()
{
    m_projection = glm::perspective(glm::radians(m_angle), m_ratio, m_near, m_far);
}

Input& Input::GetInstance()
{
    static Input input;
    return input;
}

void Input::InitializeContext(GLFWwindow* window)
{
    Input& input = GetInstance();
    input.m_pWindow = window;

    glfwSetWindowSizeCallback(input.m_pWindow, &Input::Resize);
    glfwSetKeyCallback(input.m_pWindow, &Input::KeyButton);
    glfwSetCursorPosCallback(input.m_pWindow, &Input::CursorMove);
    glfwSetMouseButtonCallback(input.m_pWindow, &Input::MouseButton);
}

void Input::AddResizeCallback(void(*callback)(GLFWwindow*, int, int))
{
    AddCallback(m_resizeCallbacks, callback);
}

void Input::RemoveResizeCallback(void(*callback)(GLFWwindow*, int, int))
{
    RemoveCallback(m_resizeCallbacks, callback);
}

void Input::AddKeyButtonCallback(void(*callback)(GLFWwindow*, int, int, int, int))
{
    AddCallback(m_keyButtonCallbacks, callback);
}

void Input::RemoveKeyButtonCallback(void(*callback)(GLFWwindow*, int, int, int, int))
{
    RemoveCallback(m_keyButtonCallbacks, callback);
}

void Input::AddCursorMoveCallback(void(*callback)(GLFWwindow*, double, double))
{
    AddCallback(m_cursorMoveCallbacks, callback);
}

void Input::RemoveCursorMoveCallback(void(*callback)(GLFWwindow*, double, double))
{
    RemoveCallback(m_cursorMoveCallbacks, callback);
}

void Input::AddMouseButtonCallback(void(*callback)(GLFWwindow*, int, int, int))
{
    AddCallback(m_mouseButtonCallbacks, callback);
}

void Input::RemoveMouseButtonCallback(void(*callback)(GLFWwindow*, int, int, int))
{
    RemoveCallback(m_mouseButtonCallbacks, callback);
}

Input::Input()
    : m_pWindow(nullptr)
{
}

void Input::Resize(GLFWwindow* window, int width, int height)
{
    Input& input = GetInstance();
    for (auto const& callback : input.m_resizeCallbacks)
    {
        callback(window, width, height);
    }
}

void Input::KeyButton(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    Input& input = GetInstance();
    for (auto const& callback : input.m_keyButtonCallbacks)
    {
        callback(window, key, scancode, action, mods);
    }
}

void Input::CursorMove(GLFWwindow* window, double xpos, double ypos)
{
    Input& input = GetInstance();
    for (auto const& callback : input.m_cursorMoveCallbacks)
    {
        callback(window, xpos, ypos);
    }
}

void Input::MouseButton(GLFWwindow* window, int button, int action, int mods)
{
    Input& input = GetInstance();
    for (auto const& callback : input.m_mouseButtonCallbacks)
    {
        callback(window, button, action, mods);
    }
}

Renderer& Renderer::GetInstance()
{
    static Renderer instace;
    return instace;
}

bool Renderer::IsValid() const
{
    return m_initialized && (GLFW_FALSE == glfwWindowShouldClose(m_window.pWindow));
}

void Renderer::RenderFrame()
{
    //Poll events and initilize imgui
    glfwPollEvents();
    ImGui_ImplGlfwGL3_NewFrame();

    //Clear buffer
    glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    //Render primitives
    glUseProgram(m_program.handle);
    for (Asset<mesh::Mesh>& mesh : m_meshes)
    {
        glBindVertexArray(mesh.data.bufferData.vertexArrayObject);

        glm::mat4 const viewProjection = m_camera.GetProjection() * m_camera.GetView();
        glm::mat4 const modelViewProjection = viewProjection * mesh.data.model;
        glm::vec3 const light{ glm::normalize(glm::vec3{ -0.5, 1.0, 0.2 }) };

        glUniform3fv(m_lightUniformHandle, 1, glm::value_ptr(light));
        glUniform3fv(m_colorUniformHandle, 1, glm::value_ptr(mesh.data.color));
        glUniform3fv(m_eyeUniformHandle, 1, glm::value_ptr(m_camera.GetPosition()));
        glUniformMatrix4fv(m_modelUniformHandle, 1, GL_FALSE, glm::value_ptr(mesh.data.model));
        glUniformMatrix4fv(m_mvpUniformHandle, 1, GL_FALSE, glm::value_ptr(modelViewProjection));

        if (PrimitiveRenderType::SOLID == primitiveRenderType 
            || PrimitiveRenderType::WIRE_SOLID == primitiveRenderType)
        {
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(mesh.data.indices.size()), GL_UNSIGNED_INT, nullptr);
        }

        if (PrimitiveRenderType::WIRE == primitiveRenderType
            || PrimitiveRenderType::WIRE_SOLID == primitiveRenderType)
        {
            glUniform3fv(m_colorUniformHandle, 1, glm::value_ptr(glm::vec3(0.2, 0.2, 0.2)));
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
            glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(mesh.data.indices.size()), GL_UNSIGNED_INT, nullptr);
        }
    }
   
    //Draw UI if possible
    if (drawUiCallback)
    {
        drawUiCallback();
    }

    //Render imgui interface
    ImGui::Render();
    ImGui_ImplGlfwGL3_RenderDrawData(ImGui::GetDrawData());

    //Swap buffers
    glfwSwapBuffers(m_window.pWindow);
}

Handle Renderer::MakeMesh()
{
    return MakeAsset(m_meshes);
}

mesh::Mesh& Renderer::GetMesh(Handle id)
{
    return GetAsset(m_meshes, id);
}

void Renderer::RemoveMesh(Handle id)
{
    RemoveAsset(m_meshes, id);
}

Renderer::Renderer()
    : m_initialized(false)
{
    InitializeGlfw();
    InitializeContext();
    InitializeImgui();
    InitializeCallbacks();
    InitializeShaderProgram();

    m_camera.SetPosition(glm::vec3(1, 1, 1) * 15.f);
    m_camera.SetDirection(glm::normalize(glm::dvec3{-1, -1, -1}));
}

Renderer::~Renderer()
{
    //Deinitialize inputs
    Input& input = Input::GetInstance();
    input.RemoveCursorMoveCallback(&Renderer::CursorMove);
    input.RemoveResizeCallback(&Renderer::Resize);
    input.RemoveKeyButtonCallback(&Renderer::KeyButton);

    //Deinitialize imgui
    ImGui_ImplGlfwGL3_Shutdown();
    ImGui::DestroyContext();

    //Deinitialize glfw
    glfwTerminate();
}

void Renderer::InitializeGlfw()
{
    m_initialized = (GLFW_TRUE == glfwInit());
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 4);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_SAMPLES, 4);
}

void pegasus::render::Renderer::InitializeImgui()
{
    ImGui::CreateContext();
    ImGui_ImplGlfwGL3_Init(m_window.pWindow, true);   
    ImGui::StyleColorsClassic();
    ImGuiIO& io = ImGui::GetIO();
    io.NavFlags |= ImGuiNavFlags_EnableKeyboard;
}

void Renderer::InitializeContext()
{
    m_window.pWindow = glfwCreateWindow(m_window.windowWidth, m_window.windowHeight, "Pegasus", nullptr, nullptr);
    glfwMakeContextCurrent(m_window.pWindow);
    glfwGetFramebufferSize(m_window.pWindow, &m_window.frameBufferWidth, &m_window.frameBufferHeight);
    glbinding::Binding::initialize();
    glEnable(GL_DEPTH_TEST);
    glViewport(0, 0, m_window.frameBufferWidth, m_window.frameBufferHeight);
}

void Renderer::InitializeCallbacks() const
{
    Input& input = Input::GetInstance();
    input.InitializeContext(m_window.pWindow);

    input.AddCursorMoveCallback(&Renderer::CursorMove);
    input.AddResizeCallback(&Renderer::Resize);
    input.AddKeyButtonCallback(&Renderer::KeyButton);
}

void Renderer::InitializeShaderProgram()
{
    shader::Shader const vertexShader{
        shader::CompileShader(GL_VERTEX_SHADER, m_pVertexShaderSources)
    };
    shader::Shader const fragmentShader{
        shader::CompileShader(GL_FRAGMENT_SHADER, m_pFragmentShaderSources)
    };
    if (!vertexShader.valid || !fragmentShader.valid)
    {
        m_initialized = false;
    }

    shader::Program::Handles const shaders{
        vertexShader.handle, 0, 0, 0, fragmentShader.handle
    };
    m_program = shader::MakeProgram(shaders);
    m_mvpUniformHandle = glGetUniformLocation(m_program.handle, "mvp");
    m_modelUniformHandle = glGetUniformLocation(m_program.handle, "model");
    m_colorUniformHandle = glGetUniformLocation(m_program.handle, "color");
    m_lightUniformHandle = glGetUniformLocation(m_program.handle, "light");
    m_eyeUniformHandle = glGetUniformLocation(m_program.handle, "eye");
    if (   -1 == m_mvpUniformHandle
        || -1 == m_modelUniformHandle
        || -1 == m_colorUniformHandle
        || -1 == m_lightUniformHandle
        || -1 == m_eyeUniformHandle)
    {
        m_initialized = false;
    }
}

void Renderer::Resize(GLFWwindow* /*window*/, int width, int height)
{
    Window& window = GetInstance().m_window;
    Camera& camera = GetInstance().m_camera;

    window.windowHeight = height;
    window.windowWidth = width;
    glfwSetWindowSize(window.pWindow, width, height);
    glfwGetFramebufferSize(window.pWindow, &window.frameBufferWidth, &window.frameBufferHeight);
    glViewport(0, 0, window.frameBufferWidth, window.frameBufferHeight);
    camera.SetRatio(float(window.frameBufferWidth) / float(window.frameBufferHeight));
}

void Renderer::KeyButton(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    Renderer& renderer = GetInstance();
    Camera& camera = GetInstance().m_camera;
    glm::vec3 const left = glm::normalize(glm::cross(camera.GetUp(), camera.GetDirection()));
    GLint const nextCursorMode =
        (glfwGetInputMode(window, GLFW_CURSOR) == GLFW_CURSOR_DISABLED)
            ? GLFW_CURSOR_NORMAL : GLFW_CURSOR_DISABLED;

    switch (key)
    {
        case GLFW_KEY_W:
            if (action == GLFW_PRESS || action == GLFW_REPEAT) {
                camera.SetPosition(camera.GetPosition() + (camera.GetDirection() * 0.2f));
            }
            break;
        case GLFW_KEY_S:
            if (action == GLFW_PRESS || action == GLFW_REPEAT) {
                camera.SetPosition(camera.GetPosition() + (-camera.GetDirection() * 0.2f));
            }
            break;
        case GLFW_KEY_D:
            if (action == GLFW_PRESS || action == GLFW_REPEAT) {
                camera.SetPosition(camera.GetPosition() + (-left * 0.2f));
            }
            break;
        case GLFW_KEY_A:
            if (action == GLFW_PRESS || action == GLFW_REPEAT) {
                camera.SetPosition(camera.GetPosition() + (left * 0.2f));
            }
            break;
        case GLFW_KEY_SPACE:
            if (action == GLFW_PRESS || action == GLFW_REPEAT) {
                camera.SetPosition(camera.GetPosition() + (camera.GetUp() * 0.2f));
            }
            break;
        case GLFW_KEY_LEFT_SHIFT:
            if (action == GLFW_PRESS || action == GLFW_REPEAT) {
                camera.SetPosition(camera.GetPosition() + (-camera.GetUp() * 0.2f));
            }
            break;
        case GLFW_KEY_F:
            if (action == GLFW_RELEASE) {
                glfwSetInputMode(window, GLFW_CURSOR, nextCursorMode);
                renderer.freeLook = !renderer.freeLook;
            }
            break;
        default:
            break;
    }
}

void Renderer::CursorMove(GLFWwindow* window, double xpos, double ypos)
{
    if (!GetInstance().freeLook)
    {
        return;
    }

    Camera& camera = GetInstance().m_camera;

    static float lastX = static_cast<float>(xpos);
    static float lastY = static_cast<float>(ypos);

    float const sensitivity = 0.1f;
    float const xoffset = (static_cast<float>(xpos) - lastX) * sensitivity;
    float const yoffset = (lastY - static_cast<float>(ypos)) * sensitivity;
    lastX = static_cast<float>(xpos);
    lastY = static_cast<float>(ypos);

    static double yaw = 0;
    static double pitch = 0;
    yaw += xoffset;
    pitch += yoffset;

    if (pitch > 89.0f) {
        pitch = 89.0f;
    }
    if (pitch < -89.0f) {
        pitch = -89.0f;
    }

    glm::vec3 const direction(
        glm::cos(glm::radians(yaw)) * glm::cos(glm::radians(pitch)),
        glm::sin(glm::radians(pitch)),
        glm::sin(glm::radians(yaw)) * glm::cos(glm::radians(pitch))
    );

    camera.SetDirection(glm::normalize(direction));
}

Primitive::Primitive()
    : m_pRenderer(&Renderer::GetInstance())
    , m_meshHandle(m_pRenderer->MakeMesh())
{
}

Primitive::~Primitive()
{
    mesh::Mesh& mesh = m_pRenderer->GetMesh(m_meshHandle);
    mesh::Delete(mesh);
    m_pRenderer->RemoveMesh(m_meshHandle);
}

void Primitive::SetModel(glm::mat4 model) const
{
    m_pRenderer->GetMesh(m_meshHandle).model = model;
}

glm::mat4 Primitive::GetModel() const
{
    return m_pRenderer->GetMesh(m_meshHandle).model;
}

LineSegment::LineSegment(glm::mat4 model, glm::vec3 color, glm::vec3 start, glm::vec3 end)
    : m_start(start)
    , m_end(end)
{
    mesh::Mesh& mesh = m_pRenderer->GetMesh(m_meshHandle);
    mesh = mesh::CreateLineSegment(start, end);
    mesh.color = color;
    mesh.model = model;
}

glm::vec3 LineSegment::GetStart() const
{
    return m_start;
}

glm::vec3 LineSegment::GetEnd() const
{
    return m_end;
}

Plane::Plane(glm::mat4 model, glm::vec3 color, glm::vec3 normal)
    : m_normal(normal)
    , m_sideLength(35.0)
{
    mesh::Mesh& mesh = m_pRenderer->GetMesh(m_meshHandle);
    mesh = mesh::CreatePlane(m_normal, m_sideLength);
    mesh.model = model;
    mesh.color = color;
}

glm::vec3 Plane::GetNormal() const
{
    return m_normal;
}

Sphere::Sphere(glm::mat4 model, glm::vec3 color, double radius)
    : m_radius(radius)
{
    mesh::Mesh& mesh = m_pRenderer->GetMesh(m_meshHandle);
    mesh = mesh::CreateSphere(m_radius, 3);
    mesh.model = model;
    mesh.color = color;
}

double Sphere::GetRadius() const
{
    return m_radius;
}

Triangle::Triangle(glm::mat4 model, glm::vec3 color, glm::vec3 a, glm::vec3 b, glm::vec3 c)
{
    mesh::Mesh& mesh = m_pRenderer->GetMesh(m_meshHandle);
    mesh = mesh::CreateTriangle(a, b, c);
    mesh.model = model;
    mesh.color = color;
}

glm::mat3 Triangle::GetVertices() const
{
    return { m_a, m_b, m_c };
}

Box::Box(glm::mat4 model, glm::vec3 color, Axes axes)
{
    mesh::Mesh& mesh = m_pRenderer->GetMesh(m_meshHandle);
    mesh = mesh::CreateBox(axes.i, axes.j, axes.k);
    mesh.model = model;
    mesh.color = color;
}

Box::Axes Box::GetAxes() const
{
    return m_axes;
}

TriangleCollection::TriangleCollection(glm::mat4 model, glm::vec3 color, std::vector<glm::mat3> const & triangles)
{
    mesh::Mesh& mesh = m_pRenderer->GetMesh(m_meshHandle);
    mesh = mesh::CreateTriangleCollection(triangles);
    mesh.model = model;
    mesh.color = color;
}

std::vector<glm::mat3> TriangleCollection::GetTriangles() const
{
    return m_triangles;
}
