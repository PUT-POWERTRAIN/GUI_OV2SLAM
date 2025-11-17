#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
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

class ImGuiVisualizerNode : public rclcpp::Node
{
public:
    ImGuiVisualizerNode() : Node("imgui_visualizer")
    {
        this->declare_parameter<std::string>("vo_pose_topic", "/vo_pose");
        this->declare_parameter<std::string>("image_track_topic", "/image_track");
        
        std::string vo_pose_topic = this->get_parameter("vo_pose_topic").as_string();
        std::string image_track_topic = this->get_parameter("image_track_topic").as_string();
        
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            vo_pose_topic, 10,
            std::bind(&ImGuiVisualizerNode::poseCallback, this, std::placeholders::_1));

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            image_track_topic, 10,
            std::bind(&ImGuiVisualizerNode::imageCallback, this, std::placeholders::_1));

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

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
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

    GLFWwindow* window = glfwCreateWindow(1920, 1080, "OV2SLAM Visualizer", NULL, NULL);
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

    ImVec4 clear_color = ImVec4(0.1f, 0.1f, 0.12f, 1.00f);

    while (!glfwWindowShouldClose(window))
    {
        rclcpp::spin_some(node);
        glfwPollEvents();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::DockSpaceOverViewport(0, ImGui::GetMainViewport());

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
        // Nowe okno z obrazem
        ImGui::Begin("Image Track");
        if (node->hasImage()) {
            ImVec2 imageSize(node->getImageWidth(), node->getImageHeight());
            ImGui::Image((void*)(intptr_t)node->getImageTexture(), imageSize);
        } else {
            ImGui::Text("Waiting for image...");
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
