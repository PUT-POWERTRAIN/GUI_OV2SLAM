#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

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

class ImGuiVisualizerNode : public rclcpp::Node
{
public:
    ImGuiVisualizerNode() : Node("imgui_visualizer")
    {
        this->declare_parameter<std::string>("vo_pose_topic", "/vo_pose");
        this->declare_parameter<std::string>("image_track_topic", "/image_track");
        this->declare_parameter<std::string>("point_cloud_topic", "/point_cloud");
        
        std::string vo_pose_topic = this->get_parameter("vo_pose_topic").as_string();
        std::string image_track_topic = this->get_parameter("image_track_topic").as_string();
        std::string point_cloud_topic = this->get_parameter("point_cloud_topic").as_string();
        
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            vo_pose_topic, 10,
            std::bind(&ImGuiVisualizerNode::poseCallback, this, std::placeholders::_1));

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            image_track_topic, 10,
            std::bind(&ImGuiVisualizerNode::imageCallback, this, std::placeholders::_1));
            
        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            point_cloud_topic, 10,
            std::bind(&ImGuiVisualizerNode::pointCloudCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "ImGui Visualizer Node started!");
    }

    float getCameraX() const { return camera_x_; }
    float getCameraY() const { return camera_y_; }
    float getCameraZ() const { return camera_z_; }
    float getVelocityLinear() const { return velocity_linear_; }
    float getDistanceLinear() const { return distance_linear_; }
    
    bool hasImage() const { return has_image_; }
    GLuint getImageTexture() const { return image_texture_; }
    int getImageWidth() const { return image_width_; }
    int getImageHeight() const { return image_height_; }
    
    const std::vector<float>& getTrajectoryX() const { return trajectory_x_; }
    const std::vector<float>& getTrajectoryZ() const { return trajectory_z_; }
    
    bool loadLogo(const std::string& logo_path)
    {
        cv::Mat logo_img = cv::imread(logo_path, cv::IMREAD_UNCHANGED);
        if (logo_img.empty()) {
            RCLCPP_WARN(this->get_logger(), "Failed to load logo: %s", logo_path.c_str());
            return false;
        }
        
        cv::Mat logo_converted;
        bool has_alpha = (logo_img.channels() == 4);
        
        if (has_alpha) {
            cv::cvtColor(logo_img, logo_converted, cv::COLOR_BGRA2RGBA);
        } else {
            cv::cvtColor(logo_img, logo_converted, cv::COLOR_BGR2RGB);
        }
        
        logo_width_ = logo_converted.cols;
        logo_height_ = logo_converted.rows;
        
        glGenTextures(1, &logo_texture_);
        glBindTexture(GL_TEXTURE_2D, logo_texture_);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        
        GLint format = has_alpha ? GL_RGBA : GL_RGB;
        glTexImage2D(GL_TEXTURE_2D, 0, format, logo_width_, logo_height_, 
                     0, format, GL_UNSIGNED_BYTE, logo_converted.data);
        
        has_logo_ = true;
        RCLCPP_INFO(this->get_logger(), "Logo loaded successfully (%s)", 
                    has_alpha ? "with alpha" : "without alpha");
        return true;
    }
    
    bool hasLogo() const { return has_logo_; }
    GLuint getLogoTexture() const { return logo_texture_; }
    int getLogoWidth() const { return logo_width_; }
    int getLogoHeight() const { return logo_height_; }
    float getLogoScale() const { return logo_scale_; }
    float getLogoAlpha() const { return logo_alpha_; }
    
    const std::vector<float>& getPointCloudX() const { return cloud_x_; }
    const std::vector<float>& getPointCloudY() const { return cloud_y_; }
    const std::vector<float>& getPointCloudZ() const { return cloud_z_; }
    const std::vector<uint32_t>& getPointCloudColors() const { return cloud_colors_; }
    size_t getPointCloudSize() const { return cloud_x_.size(); }
    
    float& getCameraAngleH() { return camera_angle_h_; }
    float& getCameraAngleV() { return camera_angle_v_; }
    float& getCameraDistance() { return camera_distance_; }
    
    void saveTrajectoryToCSV(const std::string& filename)
    {
        std::ofstream file(filename);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", filename.c_str());
            return;
        }
        
        file << "x,z\n";
        for (size_t i = 0; i < trajectory_x_.size(); ++i) {
            file << trajectory_x_[i] << "," << trajectory_z_[i] << "\n";
        }
        
        file.close();
        RCLCPP_INFO(this->get_logger(), "Trajectory saved to: %s (%zu points)", 
                    filename.c_str(), trajectory_x_.size());
    }

private:
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
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
        
        // Dodaj punkt do trajektorii
        trajectory_x_.push_back(camera_x_);
        trajectory_z_.push_back(camera_z_);
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            cv::Mat rgb_image;
            cv::cvtColor(cv_ptr->image, rgb_image, cv::COLOR_BGR2RGB);
            
            image_width_ = rgb_image.cols;
            image_height_ = rgb_image.rows;
            
            // Utwórz teksturę OpenGL jeśli nie istnieje
            if (image_texture_ == 0) {
                glGenTextures(1, &image_texture_);
                glBindTexture(GL_TEXTURE_2D, image_texture_);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            }
            
            // Aktualizuj teksturę
            glBindTexture(GL_TEXTURE_2D, image_texture_);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, image_width_, image_height_, 
                         0, GL_RGB, GL_UNSIGNED_BYTE, rgb_image.data);
            
            has_image_ = true;
            
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }
    
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        cloud_x_.clear();
        cloud_y_.clear();
        cloud_z_.clear();
        cloud_colors_.clear();
        
        // Znajdź offsety pól
        int x_offset = -1, y_offset = -1, z_offset = -1, rgb_offset = -1;
        
        for (const auto& field : msg->fields) {
            if (field.name == "x") x_offset = field.offset;
            else if (field.name == "y") y_offset = field.offset;
            else if (field.name == "z") z_offset = field.offset;
            else if (field.name == "rgb") rgb_offset = field.offset;
        }
        
        if (x_offset < 0 || y_offset < 0 || z_offset < 0) {
            RCLCPP_WARN(this->get_logger(), "Point cloud missing x, y, or z fields");
            return;
        }
        
        size_t num_points = msg->width * msg->height;
        cloud_x_.reserve(num_points);
        cloud_y_.reserve(num_points);
        cloud_z_.reserve(num_points);
        cloud_colors_.reserve(num_points);
        
        // Parsuj punkty
        for (size_t i = 0; i < num_points; ++i) {
            size_t point_offset = i * msg->point_step;
            
            float x, y, z;
            memcpy(&x, &msg->data[point_offset + x_offset], sizeof(float));
            memcpy(&y, &msg->data[point_offset + y_offset], sizeof(float));
            memcpy(&z, &msg->data[point_offset + z_offset], sizeof(float));
            
            // Sprawdź czy punkt jest poprawny
            if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
                cloud_x_.push_back(x);
                cloud_y_.push_back(y);
                cloud_z_.push_back(z);
                
                // Odczytaj kolor jeśli dostępny
                if (rgb_offset >= 0) {
                    uint32_t rgb;
                    memcpy(&rgb, &msg->data[point_offset + rgb_offset], sizeof(uint32_t));
                    cloud_colors_.push_back(rgb);
                } else {
                    cloud_colors_.push_back(0xFFFFFFFF);
                }
            }
        }
        
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Point cloud: %zu points", cloud_x_.size());
    }

    float camera_x_ = 0.0f;
    float camera_y_ = 0.0f;
    float camera_z_ = 0.0f;
    float velocity_linear_ = 0.0f;
    float distance_linear_ = 0.0f;
    
    rclcpp::Time last_pose_time_ = rclcpp::Time(0);
    float last_x_ = 0.0f;
    float last_y_ = 0.0f;
    float last_z_ = 0.0f;
    
    // Image
    bool has_image_ = false;
    GLuint image_texture_ = 0;
    int image_width_ = 0;
    int image_height_ = 0;

    // Trajectory
    std::vector<float> trajectory_x_;
    std::vector<float> trajectory_z_;
    
    // Logo
    bool has_logo_ = false;
    GLuint logo_texture_ = 0;
    int logo_width_ = 0;
    int logo_height_ = 0;
    float logo_scale_ = 0.3f;  // Rozmiar logo (0.1 - 1.0, gdzie 1.0 = 100% wysokości ekranu)
    float logo_alpha_ = 0.15f;  // Przezroczystość (0.0 - 1.0, gdzie 1.0 = nieprzezroczyste)
    
    // Point Cloud
    std::vector<float> cloud_x_;
    std::vector<float> cloud_y_;
    std::vector<float> cloud_z_;
    std::vector<uint32_t> cloud_colors_;
    
    // Kamera 3D
    float camera_angle_h_ = 45.0f;   // Kąt poziomy (stopnie)
    float camera_angle_v_ = 30.0f;   // Kąt pionowy (stopnie)
    float camera_distance_ = 20.0f;  // Odległość od centrum

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImGuiVisualizerNode>();

    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return -1;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWwindow* window = glfwCreateWindow(1920, 1080, "OV2SLAM PowerViz", NULL, NULL);
    if (window == NULL) {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;

    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");
    
    // Załaduj logo
    node->loadLogo("/ws/src/imgui_app/logo.png");
    
    // Ustaw przezroczyste tło dla okien ImGui
    ImGuiStyle& style = ImGui::GetStyle();
    style.Colors[ImGuiCol_WindowBg].w = 0.85f; // 85% nieprzezroczystości okien

    ImVec4 clear_color = ImVec4(0.1f, 0.1f, 0.12f, 1.00f);

    while (!glfwWindowShouldClose(window))
    {
        rclcpp::spin_some(node);
        glfwPollEvents();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // Okno tła z logo (pełnoekranowe, bez ramek, nieinteraktywne)
        if (node->hasLogo()) {
            ImGui::SetNextWindowPos(ImVec2(0, 0));
            ImGui::SetNextWindowSize(ImGui::GetMainViewport()->Size);
            ImGui::SetNextWindowBgAlpha(node->getLogoAlpha()); // Przezroczystość całego okna
            ImGui::Begin("Background", nullptr, 
                ImGuiWindowFlags_NoTitleBar | 
                ImGuiWindowFlags_NoResize | 
                ImGuiWindowFlags_NoMove | 
                ImGuiWindowFlags_NoScrollbar | 
                ImGuiWindowFlags_NoScrollWithMouse |
                ImGuiWindowFlags_NoCollapse |
                ImGuiWindowFlags_NoBringToFrontOnFocus |
                ImGuiWindowFlags_NoFocusOnAppearing |
                ImGuiWindowFlags_NoNav |
                ImGuiWindowFlags_NoBackground |
                ImGuiWindowFlags_NoDocking);
            
            ImVec2 viewport_size = ImGui::GetWindowSize();
            float logo_height = viewport_size.y * node->getLogoScale();
            float aspect = (float)node->getLogoWidth() / (float)node->getLogoHeight();
            float logo_width = logo_height * aspect;
            
            ImVec2 pos(
                (viewport_size.x - logo_width) * 0.5f,
                (viewport_size.y - logo_height) * 0.5f
            );
            
            ImGui::SetCursorPos(pos);
            ImGui::Image(
                (void*)(intptr_t)node->getLogoTexture(),
                ImVec2(logo_width, logo_height),
                ImVec2(0, 0),
                ImVec2(1, 1)
            );
            
            ImGui::End();
        }

        ImGui::DockSpaceOverViewport(0, ImGui::GetMainViewport());

        // Okno z pozycją kamery
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
        ImGui::End();
        
        // Okno z obrazem
        ImGui::Begin("Image Track");
        if (node->hasImage()) {
            ImVec2 imageSize(node->getImageWidth(), node->getImageHeight());
            ImGui::Image((void*)(intptr_t)node->getImageTexture(), imageSize);
        } else {
            ImGui::Text("Waiting for image...");
        }
        ImGui::End();

        // Okno z trajektorią XZ
        ImGui::Begin("Trajectory XZ");
        const auto& traj_x = node->getTrajectoryX();
        const auto& traj_z = node->getTrajectoryZ();
        
        // Przycisk zapisu trajektorii
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
                
                // Rysuj aktualną pozycję jako punkt
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
        
        // Okno z Point Cloud (wizualizacja 3D)
        ImGui::Begin("Point Cloud");
        size_t num_points = node->getPointCloudSize();
        ImGui::Text("Points: %zu", num_points);
        
        // Kontrolki kamery
        ImGui::SliderFloat("Angle H", &node->getCameraAngleH(), 0.0f, 360.0f);
        ImGui::SliderFloat("Angle V", &node->getCameraAngleV(), -89.0f, 89.0f);
        ImGui::SliderFloat("Distance", &node->getCameraDistance(), 5.0f, 100.0f);
        
        if (num_points > 0) {
            const auto& cloud_x = node->getPointCloudX();
            const auto& cloud_y = node->getPointCloudY();
            const auto& cloud_z = node->getPointCloudZ();
            
            float angle_h_rad = node->getCameraAngleH() * 3.14159f / 180.0f;
            float angle_v_rad = node->getCameraAngleV() * 3.14159f / 180.0f;
            
            // Projekcja 3D na 2D (izometryczna)
            if (ImPlot::BeginPlot("Point Cloud 3D", ImVec2(-1, -1))) {
                ImPlot::SetupAxis(ImAxis_X1, "");
                ImPlot::SetupAxis(ImAxis_Y1, "");
                
                // Próbkowanie punktów
                size_t step = std::max(size_t(1), num_points / 3000);
                std::vector<float> proj_x, proj_y;
                
                for (size_t i = 0; i < num_points; i += step) {
                    // Obrót kamery
                    float x = cloud_x[i];
                    float y = cloud_y[i];
                    float z = cloud_z[i];
                    
                    // Obrót wokół osi Y (poziomy)
                    float x_rot = x * cos(angle_h_rad) - z * sin(angle_h_rad);
                    float z_rot = x * sin(angle_h_rad) + z * cos(angle_h_rad);
                    
                    // Obrót wokół osi X (pionowy)
                    float y_rot = y * cos(angle_v_rad) - z_rot * sin(angle_v_rad);
                    float z_final = y * sin(angle_v_rad) + z_rot * cos(angle_v_rad);
                    
                    // Projekcja perspektywiczna
                    float distance = node->getCameraDistance();
                    float scale = distance / (distance + z_final);
                    
                    proj_x.push_back(x_rot * scale);
                    proj_y.push_back(z_final * scale);
                }
                
                ImPlot::PlotScatter("Points", proj_x.data(), proj_y.data(), proj_x.size());
                
                // Rysuj osie układu współrzędnych
                std::vector<float> axis_x = {0, 0, 0};
                std::vector<float> axis_y = {0, 0, 0};
                std::vector<float> axis_len_x = {1, 0, 0};
                std::vector<float> axis_len_y = {0, 1, 0};
                
                // Oś X (czerwona)
                for (size_t i = 0; i < 3; ++i) {
                    float x_rot = axis_len_x[i] * cos(angle_h_rad);
                    float z_rot = axis_len_x[i] * sin(angle_h_rad);
                    float y_rot = axis_len_y[i] * cos(angle_v_rad);
                    
                    axis_x[i] = x_rot;
                    axis_y[i] = z_rot;
                }
                
                ImPlot::EndPlot();
            }
        } else {
            ImGui::Text("Waiting for point cloud data...");
        }
        ImGui::End();

        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w);
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
