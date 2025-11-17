#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/path.hpp>
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
#include <thread> 
#include <atomic> 
#include <rclcpp/executors.hpp>

class ImGuiVisualizerNode : public rclcpp::Node
{
public:
    ImGuiVisualizerNode() : Node("imgui_visualizer")
    {
        this->declare_parameter<std::string>("vo_pose_topic", "/vo_pose");
        this->declare_parameter<std::string>("image_track_topic", "/image_track");
        
        std::string vo_pose_topic = this->get_parameter("vo_pose_topic").as_string();
        std::string image_track_topic = this->get_parameter("image_track_topic").as_string();
        

        pose_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::Reentrant);
        image_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::Reentrant);

        
        auto pose_options = rclcpp::SubscriptionOptions();
        pose_options.callback_group = pose_group_;
        
        rclcpp::QoS pose_qos(rclcpp::KeepLast(10));
        pose_qos.reliable();

        
        auto image_options = rclcpp::SubscriptionOptions();
        image_options.callback_group = image_group_;
       
        rclcpp::QoS image_qos(rclcpp::KeepLast(1));
        image_qos.best_effort();
        
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            vo_pose_topic,
            pose_qos, 
            std::bind(&ImGuiVisualizerNode::poseCallback, this, std::placeholders::_1),
            pose_options);
            
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            image_track_topic,
            image_qos,
            std::bind(&ImGuiVisualizerNode::imageCallback, this, std::placeholders::_1),
            image_options);
        
        
        
        RCLCPP_INFO(this->get_logger(), "ImGui Visualizer Node started!");
        RCLCPP_INFO(this->get_logger(), "Subscribed to pose topic: %s", vo_pose_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Subscribed to image topic: %s", image_track_topic.c_str());

    }
    
    rclcpp::CallbackGroup::SharedPtr get_pose_group() {
        return pose_group_;
    }
    
    rclcpp::CallbackGroup::SharedPtr get_image_group() {
        return image_group_;
    }
    
    float getCameraX() const { 
        std::lock_guard<std::mutex> lock(pose_mutex_);
        return camera_x_; 
    }
    float getCameraY() const { 
        std::lock_guard<std::mutex> lock(pose_mutex_);
        return camera_y_; 
    }
    float getCameraZ() const { 
        std::lock_guard<std::mutex> lock(pose_mutex_);
        return camera_z_; 
    }
    float getVelocityLinear() const { 
        std::lock_guard<std::mutex> lock(pose_mutex_);
        return velocity_linear_; 
    }
    float getDistanceLinear() const { 
        std::lock_guard<std::mutex> lock(pose_mutex_);
        return distance_linear_; 
    }

    bool hasImage() const { return has_image_; }
    GLuint getImageTexture() const { return image_texture_; }
    int getImageWidth() const { return image_width_; }
    int getImageHeight() const { return image_height_; }
    
    void updateTexture() {
        std::lock_guard<std::mutex> lock(image_mutex_);
        
        if (image_needs_update_ && !latest_image_.empty()) {
            if (image_texture_ == 0) {
                glGenTextures(1, &image_texture_);
                glBindTexture(GL_TEXTURE_2D, image_texture_);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            }
            
            glBindTexture(GL_TEXTURE_2D, image_texture_);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 
                         latest_image_.cols, latest_image_.rows,
                         0, GL_RGB, GL_UNSIGNED_BYTE, latest_image_.data);
            
            has_image_ = true;
            image_width_ = latest_image_.cols;
            image_height_ = latest_image_.rows;
            image_needs_update_ = false;
        }
    }


private:
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        
        auto current_time = this->now();
        
        // Debug - wypisz co 100 wiadomości
        static int msg_count = 0;
        msg_count++;
        if (msg_count % 100 == 0) {
            RCLCPP_INFO(this->get_logger(), "Received %d pose messages. Current pos: [%.3f, %.3f, %.3f]", 
                        msg_count, msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        }
        
        // Najpierw aktualizuj pozycję (to musi być zawsze)
        camera_x_ = msg->pose.position.x;
        camera_y_ = msg->pose.position.y;
        camera_z_ = msg->pose.position.z;
        
        // Potem oblicz prędkość i dystans
        if (last_pose_time_.seconds() > 0.0) {
            double dt_local = (current_time - last_pose_time_).seconds();

            if (dt_local > 0.000001) {  
                float dx = msg->pose.position.x - last_x_;
                float dy = msg->pose.position.y - last_y_;
                float dz = msg->pose.position.z - last_z_;

                float delta_distance = std::sqrt(dx*dx + dy*dy + dz*dz);
                float current_velocity = delta_distance / dt_local;

                velocity_linear_ = current_velocity;
                distance_linear_ = distance_linear_ + delta_distance;
            }
        }
        
        // Na końcu aktualizuj poprzednie wartości
        last_pose_time_ = current_time;
        last_x_ = msg->pose.position.x;
        last_y_ = msg->pose.position.y;
        last_z_ = msg->pose.position.z;
    }
  
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            cv::Mat rgb_image;
            cv::cvtColor(cv_ptr->image, rgb_image, cv::COLOR_BGR2RGB);
            
            std::lock_guard<std::mutex> lock(image_mutex_);
            latest_image_ = rgb_image.clone();
            image_needs_update_ = true;
            
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }
    
    std::mutex image_mutex_;
    cv::Mat latest_image_;
    bool image_needs_update_ = false;

    mutable std::mutex pose_mutex_;  // mutable bo używamy w const getterach
    float camera_x_ = 0.0f;
    float camera_y_ = 0.0f;
    float camera_z_ = 0.0f;
    float velocity_linear_ = 0.0f;
    float distance_linear_ = 0.0f;    
    
    rclcpp::Time last_pose_time_ = rclcpp::Time(0);
    float last_x_ = 0.0f;
    float last_y_ = 0.0f;
    float last_z_ = 0.0f;
    double dt = 0;

    // Image
    bool has_image_ = false;
    GLuint image_texture_ = 0;
    int image_width_ = 0;
    int image_height_ = 0;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::CallbackGroup::SharedPtr pose_group_;
    rclcpp::CallbackGroup::SharedPtr image_group_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImGuiVisualizerNode>();
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    std::thread ros_thread([&executor, &node]() {
        RCLCPP_INFO(node->get_logger(), "Starting ROS Multi-Threaded Executor spin...");
        executor.spin();
        RCLCPP_INFO(node->get_logger(), "ROS executor spin thread finished.");
    });

    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return -1;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWwindow* window = glfwCreateWindow(1600, 900, "OV2SLAM Visualizer", NULL, NULL);
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
        glfwPollEvents();
        node->updateTexture();

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

    RCLCPP_INFO(node->get_logger(), "Shutting down ROS...");
    rclcpp::shutdown();
    if (ros_thread.joinable()) {
        ros_thread.join();
    }
    
    return 0;
}
