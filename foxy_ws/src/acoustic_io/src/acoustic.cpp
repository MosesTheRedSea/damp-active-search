#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include <portaudio.h>
#include <format>
#include <sndfile.h>      // libsndfile for WAV loading — apt install libsndfile1-dev
#include <vector>
#include <string>
#include <thread>
#include <mutex>
#include <complex>
#include <fft.h> 


// Workflow
// 1. Play The Chircp & Record Impulse Response
// 2. Notify current_status -> we are recording
// 3. Save the Impulse Reponse to a output_directory
// 4. Update the recording_complete output_directory
//
//

class Acoustic : public rclcpp::Node, state(State::IDLE)
{

public:

    Acoustic() : Node("acoustic") 
    {   
        this->declare_parameter("excitation_path", "excitation.wav");
        this->declare_parameter("recording_output_dir", "recordings");
        this->delcare_parameter("channels", 16);
        this->declare_parameter("repeat", 1);
        this->declare_parameter("sleep_duration", 3);
        this->declare_parameter("end_sample", 6000);
        this->delcare_parameter("mic_device", 1);
        this->delcare_parameter("speaker_device", 13);
        
        excitation_path = this->get_parameter('excitation_path').as_string();
        output_directory = this->get_parameter('recording_output_dir').as_string();
        channels = this->get_parameter('channels').as_string();
        repeat = this->get_parameter('repeat').as_string();

        sleep_duration = this->get_parameter('sleep_duration').as_string();
        end_sample = this->get_parameter('end_sample').as_string();
        mic_device = this->get_parameter('mic_device').as_string();
        speaker_device = this->get_parameter('speaker_device').as_string();
        
        // Publisher will let us know when it's done recording
        recording_complete = this->create_publisher<std_msgs::msg::Bool>("/recording_complete", 10);
        
        // This Publisher knows the current status
        current_status = this->create_publisher<std_msgs::msg::Bool>("/current_stattus", 10); 
        RCLCPP_INFO(get_logger(), "recording setup completed...")

    }

      
private:

    rclcpp::Publisher<std_msgs::msg::Bool>::Shared_Ptr recording_complete;
    rclcpp::Publisher<std_msgs::msg::Bool>::Shared_Ptr current_status;


}


void list_audio_devices() {
  
  



}

void record_impulse_response() {




}

// This method will play the excitation file chirp
//void play_chirp() {

//  RCLCPP_INFO(get_logger(), "Chirp Finished PLaying")
//}

// This method will use the 16 microphone array to record the impulse response
//void record_reverb() {
    


//    RCLCPP_INFO(get_logger(), "Finished Recording Impulse Reponse...")
//}


// This method will receive the impulse response and save the graph's it receives
void compute_and_save_ir() {

    
    

    
    std::string output_dir = get_parameter("recording_output_dir")
    
    RCLCPP_INFO(get_logger(), std::format("Impulse Response Graph Saved to %s...", output_dir))

}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Collector>());
  rclcpp::shutdown();
  return 0;
}
