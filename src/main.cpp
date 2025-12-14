#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <GL/glew.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "implot.h"
#include <GLFW/glfw3.h>

#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>
#include <chrono>
#include <algorithm>
#include <cstring>

// --- SHADERY DLA POINT CLOUD ---
const char* viz_vertex_shader = R"(
#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aColor;
uniform mat4 projection;
uniform mat4 view;
out vec3 vertexColor;
void main() {
    gl_Position = projection * view * vec4(aPos, 1.0);
    gl_PointSize = 2.0;
    vertexColor = aColor;
}
)";

const char* viz_fragment_shader = R"(
#version 330 core
in vec3 vertexColor;
out vec4 FragColor;
void main() {
    FragColor = vec4(vertexColor, 1.0);
}
)";

// Shader dla siatki (Grid)
const char* grid_vertex_shader = R"(
#version 330 core
layout (location = 0) in vec3 aPos;
uniform mat4 projection;
uniform mat4 view;
void main() {
    gl_Position = projection * view * vec4(aPos, 1.0);
}
)";

const char* grid_fragment_shader = R"(
#version 330 core
out vec4 FragColor;
void main() {
    FragColor = vec4(0.4, 0.4, 0.4, 1.0);
}
)";

GLuint createShaderProgram(const char* vSrc, const char* fSrc) {
    GLuint v = glCreateShader(GL_VERTEX_SHADER); glShaderSource(v, 1, &vSrc, NULL); glCompileShader(v);
    GLuint f = glCreateShader(GL_FRAGMENT_SHADER); glShaderSource(f, 1, &fSrc, NULL); glCompileShader(f);
    GLuint p = glCreateProgram(); glAttachShader(p, v); glAttachShader(p, f); glLinkProgram(p);
    glDeleteShader(v); glDeleteShader(f);
    return p;
}

// --- STRUKTURA KAMERY 3D ---
struct Camera3D {
    float yaw = 135.0f, pitch = 30.0f, dist = 50.0f;
    float target[3] = {0,0,0};
    bool dragging = false, panning = false;
    double lx=0, ly=0;
    ImVec2 last_mouse_pos;
    bool mouse_over_window = false;

    void update(ImVec2 mouse_pos, bool left_down, bool right_down, ImVec2 mouse_delta) {
        // Obrót (LPM)
        if(left_down && mouse_over_window) {
            yaw += mouse_delta.x * 0.5f;
            pitch += mouse_delta.y * 0.5f;
            pitch = std::max(-89.0f, std::min(89.0f, pitch));
        }

        // Przesuwanie (PPM)
        if(right_down && mouse_over_window) {
            float dx = mouse_delta.x * 0.05f;
            float dy = mouse_delta.y * 0.05f;
            float rad = glm::radians(yaw);
            target[0] -= (dx * cos(rad) - dy * sin(rad));
            target[2] -= (dx * sin(rad) + dy * cos(rad));
        }
    }
    
    void scroll(float delta) {
        if(mouse_over_window) {
            dist -= delta * 2.0f;
            if(dist < 0.1f) dist = 0.1f;
        }
    }
    
    glm::mat4 getView() {
        float ry = glm::radians(yaw), rp = glm::radians(pitch);
        float cx = target[0] + dist * cos(rp) * sin(ry);
        float cy = target[1] + dist * sin(rp);
        float cz = target[2] + dist * cos(rp) * cos(ry);
        return glm::lookAt(glm::vec3(cx,cy,cz), glm::vec3(target[0],target[1],target[2]), glm::vec3(0,1,0));
    }
};

class ImGuiVisualizerNode : public rclcpp::Node
{
public:
    ImGuiVisualizerNode() : Node("imgui_visualizer")
    {
        this->declare_parameter<std::string>("vo_pose_topic", "/vo_pose");
        this->declare_parameter<std::string>("image_track_topic", "/image_track");
        this->declare_parameter<std::string>("point_cloud_topic", "/point_cloud");
        
        // QoS Best Effort dla PointCloud
        rclcpp::QoS qos(10);
        qos.keep_last(5);
        qos.best_effort();
        qos.durability_volatile();

        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            this->get_parameter("vo_pose_topic").as_string(), 10,
            std::bind(&ImGuiVisualizerNode::poseCallback, this, std::placeholders::_1));

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            this->get_parameter("image_track_topic").as_string(), 10,
            std::bind(&ImGuiVisualizerNode::imageCallback, this, std::placeholders::_1));
            
        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            this->get_parameter("point_cloud_topic").as_string(), qos,
            std::bind(&ImGuiVisualizerNode::pointCloudCallback, this, std::placeholders::_1));
    }

    // Gettery 
    float getCameraX() const { return camera_x_; }
    float getCameraY() const { return camera_y_; }
    float getCameraZ() const { return camera_z_; }
    float getVelocityLinear() const { return velocity_linear_; }
    float getDistanceLinear() const { return distance_linear_; }
    
    bool hasImage() const { return has_image_; }
    
    void updateImageTexture() {
        if(has_new_image_ && !current_image_mat_.empty()) {
             if (image_texture_ == 0) {
                glGenTextures(1, &image_texture_);
                glBindTexture(GL_TEXTURE_2D, image_texture_);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            }
            glBindTexture(GL_TEXTURE_2D, image_texture_);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, image_width_, image_height_, 
                         0, GL_RGB, GL_UNSIGNED_BYTE, current_image_mat_.data);
            has_new_image_ = false;
        }
    }
    
    GLuint getImageTexture() const { return image_texture_; }
    int getImageWidth() const { return image_width_; }
    int getImageHeight() const { return image_height_; }
    
    const std::vector<float>& getTrajectoryX() const { return trajectory_x_; }
    const std::vector<float>& getTrajectoryZ() const { return trajectory_z_; }
    
    // Logo
    bool loadLogo(const std::string& logo_path) {
        cv::Mat logo_img = cv::imread(logo_path, cv::IMREAD_UNCHANGED);
        if (logo_img.empty()) return false;
        
        cv::Mat logo_converted;
        bool has_alpha = (logo_img.channels() == 4);
        if (has_alpha) cv::cvtColor(logo_img, logo_converted, cv::COLOR_BGRA2RGBA);
        else cv::cvtColor(logo_img, logo_converted, cv::COLOR_BGR2RGB);
        
        logo_width_ = logo_converted.cols;
        logo_height_ = logo_converted.rows;
        
        glGenTextures(1, &logo_texture_);
        glBindTexture(GL_TEXTURE_2D, logo_texture_);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        
        GLint format = has_alpha ? GL_RGBA : GL_RGB;
        glTexImage2D(GL_TEXTURE_2D, 0, format, logo_width_, logo_height_, 0, format, GL_UNSIGNED_BYTE, logo_converted.data);
        
        has_logo_ = true;
        return true;
    }
    
    bool hasLogo() const { return has_logo_; }
    GLuint getLogoTexture() const { return logo_texture_; }
    int getLogoWidth() const { return logo_width_; }
    int getLogoHeight() const { return logo_height_; }
    float getLogoScale() const { return logo_scale_; }
    float getLogoAlpha() const { return logo_alpha_; }
    
    void saveTrajectoryToCSV(const std::string& filename) {
        std::ofstream file(filename);
        if (file.is_open()) {
            file << "x,z\n";
            for (size_t i = 0; i < trajectory_x_.size(); ++i)
                file << trajectory_x_[i] << "," << trajectory_z_[i] << "\n";
            file.close();
        }
    }

    // Point Cloud Data Access
    std::vector<float> cloud_points_buffer;
    std::vector<float> cloud_colors_buffer;
    bool new_cloud_available = false;
    size_t getCloudSize() const { return cloud_points_buffer.size() / 3; }
    
    float center_x=0, center_y=0, center_z=0;

private:
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        auto current_time = this->now();
        if (last_pose_time_.seconds() > 0.0) {
            double dt = (current_time - last_pose_time_).seconds();
            if (dt > 0.0001) {
                float dx = msg->pose.position.x - last_x_;
                float dy = msg->pose.position.y - last_y_;
                float dz = msg->pose.position.z - last_z_;
                velocity_linear_ = std::sqrt(dx*dx + dy*dy + dz*dz) / dt;
                distance_linear_ += std::sqrt(dx*dx + dy*dy + dz*dz);
            }
        }
        last_pose_time_ = current_time;
        last_x_ = msg->pose.position.x;
        last_y_ = msg->pose.position.y;
        last_z_ = msg->pose.position.z;
        camera_x_ = msg->pose.position.x;
        camera_y_ = msg->pose.position.y;
        camera_z_ = msg->pose.position.z;
        trajectory_x_.push_back(camera_x_);
        trajectory_z_.push_back(camera_z_);
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            cv::cvtColor(cv_ptr->image, current_image_mat_, cv::COLOR_BGR2RGB);
            image_width_ = current_image_mat_.cols;
            image_height_ = current_image_mat_.rows;
            has_image_ = true;
            has_new_image_ = true;
        } catch (...) {}
    }
    
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        int x_off=-1, y_off=-1, z_off=-1, rgb_off=-1;
        for(const auto& f : msg->fields) {
            if(f.name=="x") x_off=f.offset;
            else if(f.name=="y") y_off=f.offset;
            else if(f.name=="z") z_off=f.offset;
            else if(f.name=="rgb") rgb_off=f.offset;
        }
        if(x_off<0) return;

        size_t n = msg->width * msg->height;
        std::vector<float> pts; pts.reserve(n*3);
        std::vector<float> cols; cols.reserve(n*3);
        
        double sx=0, sy=0, sz=0;
        int valid=0;

        for(size_t i=0; i<n; ++i) {
            size_t ptr = i * msg->point_step;
            float x = *(float*)&msg->data[ptr + x_off];
            float y = *(float*)&msg->data[ptr + y_off];
            float z = *(float*)&msg->data[ptr + z_off];

            if(std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
                pts.push_back(x); pts.push_back(y); pts.push_back(z);
                sx+=x; sy+=y; sz+=z; valid++;

                if(rgb_off>=0) {
                    uint32_t c = *(uint32_t*)&msg->data[ptr + rgb_off];
                    cols.push_back(((c>>16)&0xFF)/255.0f);
                    cols.push_back(((c>>8)&0xFF)/255.0f);
                    cols.push_back((c&0xFF)/255.0f);
                } else {
                    cols.push_back(1.0f); cols.push_back(1.0f); cols.push_back(1.0f);
                }
            }
        }

        if(valid > 0) {
            center_x = sx/valid; center_y = sy/valid; center_z = sz/valid;
        }
        
        cloud_points_buffer = std::move(pts);
        cloud_colors_buffer = std::move(cols);
        new_cloud_available = true;
    }

    float camera_x_ = 0, camera_y_ = 0, camera_z_ = 0;
    float velocity_linear_ = 0, distance_linear_ = 0;
    rclcpp::Time last_pose_time_ = rclcpp::Time(0);
    float last_x_ = 0, last_y_ = 0, last_z_ = 0;
    
    bool has_image_ = false;
    bool has_new_image_ = false;
    cv::Mat current_image_mat_;
    GLuint image_texture_ = 0;
    int image_width_ = 0, image_height_ = 0;

    std::vector<float> trajectory_x_, trajectory_z_;
    
    bool has_logo_ = false;
    GLuint logo_texture_ = 0;
    int logo_width_ = 0, logo_height_ = 0;
    float logo_scale_ = 0.3f;
    float logo_alpha_ = 0.15f;
    
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
};

// --- KLASA DO RENDEROWANIA POINT CLOUD DO TEKSTURY ---
class PointCloudRenderer {
public:
    PointCloudRenderer() {}
    
    bool init(int width, int height) {
        width_ = width;
        height_ = height;
        
        // Shadery
        prog_viz_ = createShaderProgram(viz_vertex_shader, viz_fragment_shader);
        prog_grid_ = createShaderProgram(grid_vertex_shader, grid_fragment_shader);
        
        // VAO/VBO dla punktów
        glGenVertexArrays(1, &vao_pts_);
        glGenBuffers(1, &vbo_pts_);
        glGenBuffers(1, &cbo_pts_);
        
        // VAO/VBO dla siatki
        glGenVertexArrays(1, &vao_grid_);
        std::vector<float> gridV;
        int gSize = 100;
        for(int i=-gSize; i<=gSize; i+=5) {
            gridV.push_back(i); gridV.push_back(0); gridV.push_back(-gSize);
            gridV.push_back(i); gridV.push_back(0); gridV.push_back(gSize);
            gridV.push_back(-gSize); gridV.push_back(0); gridV.push_back(i);
            gridV.push_back(gSize); gridV.push_back(0); gridV.push_back(i);
        }
        glGenBuffers(1, &vbo_grid_);
        glBindVertexArray(vao_grid_);
        glBindBuffer(GL_ARRAY_BUFFER, vbo_grid_);
        glBufferData(GL_ARRAY_BUFFER, gridV.size()*sizeof(float), gridV.data(), GL_STATIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
        glEnableVertexAttribArray(0);
        grid_vertices_ = gridV.size() / 3;
        
        // Framebuffer i tekstura
        glGenFramebuffers(1, &fbo_);
        glGenTextures(1, &texture_);
        glGenRenderbuffers(1, &rbo_depth_);
        
        resize(width, height);
        
        return true;
    }
    
    void resize(int width, int height) {
        width_ = width;
        height_ = height;
        
        // Tekstura koloru
        glBindTexture(GL_TEXTURE_2D, texture_);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width_, height_, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        
        // Renderbuffer głębi
        glBindRenderbuffer(GL_RENDERBUFFER, rbo_depth_);
        glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, width_, height_);
        
        // Framebuffer
        glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, texture_, 0);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, rbo_depth_);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }
    
    void render(Camera3D& cam, ImGuiVisualizerNode* node) {
        // Renderuj do framebuffera
        glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
        glViewport(0, 0, width_, height_);
        glClearColor(0.15f, 0.15f, 0.15f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_PROGRAM_POINT_SIZE);
        
        glm::mat4 proj = glm::perspective(glm::radians(45.0f), (float)width_/height_, 0.1f, 10000.0f);
        glm::mat4 view = cam.getView();
        
        // Rysuj siatkę
        glUseProgram(prog_grid_);
        glUniformMatrix4fv(glGetUniformLocation(prog_grid_, "projection"), 1, GL_FALSE, glm::value_ptr(proj));
        glUniformMatrix4fv(glGetUniformLocation(prog_grid_, "view"), 1, GL_FALSE, glm::value_ptr(view));
        glBindVertexArray(vao_grid_);
        glDrawArrays(GL_LINES, 0, grid_vertices_);
        
        // Aktualizuj dane punktów jeśli są nowe
        if(node->new_cloud_available) {
            glBindVertexArray(vao_pts_);
            glBindBuffer(GL_ARRAY_BUFFER, vbo_pts_);
            glBufferData(GL_ARRAY_BUFFER, node->cloud_points_buffer.size()*4, 
                         node->cloud_points_buffer.data(), GL_DYNAMIC_DRAW);
            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
            glEnableVertexAttribArray(0);
            
            glBindBuffer(GL_ARRAY_BUFFER, cbo_pts_);
            glBufferData(GL_ARRAY_BUFFER, node->cloud_colors_buffer.size()*4, 
                         node->cloud_colors_buffer.data(), GL_DYNAMIC_DRAW);
            glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);
            glEnableVertexAttribArray(1);
            
            node->new_cloud_available = false;
            
            // Auto-center
            if(cam.target[0]==0 && cam.target[1]==0) {
                cam.target[0]=node->center_x;
                cam.target[1]=node->center_y;
                cam.target[2]=node->center_z;
            }
        }
        
        // Rysuj punkty
        if(node->getCloudSize() > 0) {
            glUseProgram(prog_viz_);
            glUniformMatrix4fv(glGetUniformLocation(prog_viz_, "projection"), 1, GL_FALSE, glm::value_ptr(proj));
            glUniformMatrix4fv(glGetUniformLocation(prog_viz_, "view"), 1, GL_FALSE, glm::value_ptr(view));
            glBindVertexArray(vao_pts_);
            glDrawArrays(GL_POINTS, 0, node->getCloudSize());
        }
        
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }
    
    GLuint getTexture() const { return texture_; }
    
    ~PointCloudRenderer() {
        if(fbo_) glDeleteFramebuffers(1, &fbo_);
        if(texture_) glDeleteTextures(1, &texture_);
        if(rbo_depth_) glDeleteRenderbuffers(1, &rbo_depth_);
        if(vao_pts_) glDeleteVertexArrays(1, &vao_pts_);
        if(vbo_pts_) glDeleteBuffers(1, &vbo_pts_);
        if(cbo_pts_) glDeleteBuffers(1, &cbo_pts_);
        if(vao_grid_) glDeleteVertexArrays(1, &vao_grid_);
        if(vbo_grid_) glDeleteBuffers(1, &vbo_grid_);
        if(prog_viz_) glDeleteProgram(prog_viz_);
        if(prog_grid_) glDeleteProgram(prog_grid_);
    }

private:
    GLuint fbo_ = 0, texture_ = 0, rbo_depth_ = 0;
    GLuint vao_pts_ = 0, vbo_pts_ = 0, cbo_pts_ = 0;
    GLuint vao_grid_ = 0, vbo_grid_ = 0;
    GLuint prog_viz_ = 0, prog_grid_ = 0;
    int width_ = 800, height_ = 600;
    int grid_vertices_ = 0;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImGuiVisualizerNode>();

    if (!glfwInit()) return -1;
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    
    GLFWwindow* window = glfwCreateWindow(1600, 1000, "OV2SLAM PowerViz", NULL, NULL);
    if (!window) return -1;

    glfwMakeContextCurrent(window);
    glewExperimental = GL_TRUE;
    glewInit();
    
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;

    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");
    
    node->loadLogo("/ws/src/imgui_app/logo.png");
    
    ImGuiStyle& style = ImGui::GetStyle();
    style.Colors[ImGuiCol_WindowBg].w = 0.85f;
    
    // Inicjalizacja renderera Point Cloud
    PointCloudRenderer pc_renderer;
    pc_renderer.init(800, 600);
    
    Camera3D cam3d;
    ImVec2 last_mouse_pos(0, 0);

    while (!glfwWindowShouldClose(window))
    {
        rclcpp::spin_some(node);
        glfwPollEvents();

        node->updateImageTexture();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // LOGO W TLE 
        if (node->hasLogo()) {
            ImGui::SetNextWindowPos(ImVec2(0, 0));
            ImGui::SetNextWindowSize(ImGui::GetMainViewport()->Size);
            ImGui::SetNextWindowBgAlpha(node->getLogoAlpha());
            ImGui::Begin("Background", nullptr, 
                ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoInputs | 
                ImGuiWindowFlags_NoBackground | ImGuiWindowFlags_NoBringToFrontOnFocus);
            
            ImVec2 sz = ImGui::GetWindowSize();
            float lh = sz.y * node->getLogoScale();
            float aspect = (float)node->getLogoWidth() / (float)node->getLogoHeight();
            float lw = lh * aspect;
            
            ImGui::SetCursorPos(ImVec2((sz.x - lw)*0.5f, (sz.y - lh)*0.5f));
            ImGui::Image((void*)(intptr_t)node->getLogoTexture(), ImVec2(lw, lh), 
                        ImVec2(0,0), ImVec2(1,1), ImVec4(1,1,1,1.0f), ImVec4(0,0,0,0));
            ImGui::End();
        }

        ImGui::DockSpaceOverViewport(0, ImGui::GetMainViewport());

        // OKNO 1: POZYCJA KAMERY
        ImGui::Begin("Camera Position");
        ImGui::Text("Position:");
        ImGui::Text("  X: %.3f m", node->getCameraX());
        ImGui::Text("  Y: %.3f m", node->getCameraY());
        ImGui::Text("  Z: %.3f m", node->getCameraZ());
        ImGui::Separator();
        ImGui::Text("Velocity:");
        ImGui::Text("  Linear: %.3f m/s", node->getVelocityLinear());
        ImGui::Separator();
        ImGui::Text("Distance:");
        ImGui::Text("  Linear: %.3f m", node->getDistanceLinear());
        ImGui::Separator();
        ImGui::Text("Point Cloud:");
        ImGui::Text("  Points: %zu", node->getCloudSize());
        ImGui::Text("  Center: %.1f %.1f %.1f", node->center_x, node->center_y, node->center_z);
        ImGui::End();
        
        // OKNO 2: OBRAZ Z KAMERY
        ImGui::Begin("Image Track");
        if (node->hasImage()) {
            float avail = ImGui::GetContentRegionAvail().x;
            float w = node->getImageWidth(); float h = node->getImageHeight();
            if(w > avail) { float s = avail/w; w*=s; h*=s; }
            ImGui::Image((void*)(intptr_t)node->getImageTexture(), ImVec2(w, h));
        } else {
            ImGui::Text("Waiting for image...");
        }
        ImGui::End();

        // OKNO 3: TRAJEKTORIA
        ImGui::Begin("Trajectory XZ");
        const auto& traj_x = node->getTrajectoryX();
        const auto& traj_z = node->getTrajectoryZ();
        
        if (ImGui::Button("Save Trajectory to CSV")) {
            std::string filename = "/ws/trajectories/trajectory_" + 
                std::to_string(std::chrono::system_clock::now().time_since_epoch().count()) + 
                ".csv";
            node->saveTrajectoryToCSV(filename);
        }
        ImGui::SameLine();
        ImGui::Text("Points: %zu", traj_x.size());
        
        if (traj_x.size() > 1) {
            if (ImPlot::BeginPlot("Camera Trajectory (Top View)", ImVec2(-1, -1))) {
                ImPlot::SetupAxis(ImAxis_X1, "X [m]");
                ImPlot::SetupAxis(ImAxis_Y1, "Z [m]");
                ImPlot::SetupAxisLimits(ImAxis_X1, -10, 10, ImGuiCond_Once);
                ImPlot::SetupAxisLimits(ImAxis_Y1, -10, 10, ImGuiCond_Once);
                
                ImPlot::PlotLine("Path", traj_x.data(), traj_z.data(), traj_x.size());
                
                if (!traj_x.empty()) {
                    float current_x = traj_x.back();
                    float current_z = traj_z.back();
                    ImPlot::PlotScatter("Current", &current_x, &current_z, 1);
                }
                
                ImPlot::EndPlot();
            }
        } else {
            ImGui::Text("Collecting trajectory data...");
        }
        ImGui::End();

        // OKNO 4: POINT CLOUD 3D
        ImGui::Begin("3D Point Cloud View");
        
        ImVec2 region = ImGui::GetContentRegionAvail();
        
        // Sprawdź czy region jest prawidłowy (nie zerowy)
        if(region.x > 50 && region.y > 50) {
            pc_renderer.resize((int)region.x, (int)region.y);
            
            // Renderuj Point Cloud
            pc_renderer.render(cam3d, node.get());
            
            // Wyświetl jako teksturę (INVISIBLE_BUTTON przechwytuje kliknięcia myszy)
            ImGui::InvisibleButton("##pc_canvas", region);
            ImVec2 p_min = ImGui::GetItemRectMin();
            ImVec2 p_max = ImGui::GetItemRectMax();
            ImDrawList* draw_list = ImGui::GetWindowDrawList();
            draw_list->AddImage((void*)(intptr_t)pc_renderer.getTexture(), 
                               p_min, p_max, ImVec2(0,1), ImVec2(1,0));
            
            // Sprawdź czy mysz jest nad obrazem
            bool is_over_image = ImGui::IsItemHovered();
            
            cam3d.mouse_over_window = is_over_image;
            
            // Obsługa myszy - tylko gdy jest nad obrazem
            ImVec2 mouse_pos = ImGui::GetMousePos();
            if(is_over_image) {
                ImVec2 mouse_delta(mouse_pos.x - last_mouse_pos.x, mouse_pos.y - last_mouse_pos.y);
                bool left_down = ImGui::IsMouseDown(ImGuiMouseButton_Left);
                bool right_down = ImGui::IsMouseDown(ImGuiMouseButton_Right);
                
                cam3d.update(mouse_pos, left_down, right_down, mouse_delta);
                
                // Scroll dla zoomu
                float wheel = ImGui::GetIO().MouseWheel;
                if(wheel != 0.0f) {
                    cam3d.scroll(wheel);
                }
            }
            last_mouse_pos = mouse_pos;
        } else {
            // Okno jest za małe lub jeszcze się inicjalizuje
            ImGui::Text("Resize window to view Point Cloud...");
        }
        
        ImGui::Text("Controls:");
        ImGui::BulletText("Left Mouse: Rotate");
        ImGui::BulletText("Right Mouse: Pan");
        ImGui::BulletText("Scroll: Zoom");
        
        ImGui::End();

        // RENDER GUI
        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(0.1f, 0.1f, 0.12f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        
        glfwSwapBuffers(window);
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();
    
    glfwDestroyWindow(window);
    glfwTerminate();

    rclcpp::shutdown();
    return 0;
}
