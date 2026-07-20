#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include <portaudio.h>
#include <sndfile.h>
#include <fftw3.h>
#include <vector>
#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include <cstring>
#include <cstdint>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <iomanip>

namespace fs = std::filesystem;

class Acoustic : public rclcpp::Node
{
public:
  Acoustic() : Node("acoustic")
  {
  
    this->declare_parameter("dataset_directory", std::string(""));
    this->declare_parameter("room_directory", std::string(""));
    this->declare_parameter("excitation_path", std::string("excitation.wav"));
    this->declare_parameter("occlusion_type", std::string(""));
    this->declare_parameter("occlusion_distance", std::string(""));
    this->declare_parameter("occluded_type", std::string(""));
    this->declare_parameter("occluded_distance", std::string(""));
    this->declare_parameter("base_filename", std::string(""));

    this->declare_parameter("channels", 16);
    this->declare_parameter("repeat", 1);     
    this->declare_parameter("sleep_duration", 3);
    this->declare_parameter("start_sample", 21300);
    this->declare_parameter("end_sample", 22000);
    this->declare_parameter("nperseg", 256);
    this->declare_parameter("noverlap",  128);
    this->declare_parameter("mic_device", 1);
    this->declare_parameter("speaker_device", 13);
    this->declare_parameter("save_ir_spec", true);  

    dataset_directory = this->get_parameter("dataset_directory").as_string();
    room_directory = this->get_parameter("room_directory").as_string();
    excitation_path = this->get_parameter("excitation_path").as_string();
    occlusion_type = this->get_parameter("occlusion_type").as_string();
    occlusion_distance = this->get_parameter("occlusion_distance").as_string();
    occluded_type = this->get_parameter("occluded_type").as_string();
    occluded_distance = this->get_parameter("occluded_distance").as_string();

    base_filename = this->get_parameter("base_filename").as_string();
    channels = this->get_parameter("channels").as_int();
    repeat = this->get_parameter("repeat").as_int();
    sleep_duration = this->get_parameter("sleep_duration").as_int();
    start_sample = this->get_parameter("start_sample").as_int();
    end_sample = this->get_parameter("end_sample").as_int();

    nperseg = this->get_parameter("nperseg").as_int();
    noverlap = this->get_parameter("noverlap").as_int();
    mic_device  = this->get_parameter("mic_device").as_int();
    speaker_device  = this->get_parameter("speaker_device").as_int();
    save_ir_spec  = this->get_parameter("save_ir_spec").as_bool();

    Pa_Initialize();
    listAudioDevices();
    load_excitation();

    start_recording = this->create_subscription<std_msgs::msg::Bool>("/start_record", 10, std::bind(&Acoustic::triggerCallback, this, std::placeholders::_1));
    complete_recording = this->create_publisher<std_msgs::msg::Bool>("/recording_complete", 10);
    current_status  = this->create_publisher<std_msgs::msg::String>("/status", 10);

    saved_recording = this->create_publisher<std_msgs::msg::String>("/recording_saved", 10);
    RCLCPP_INFO(this->get_logger(), "acoustic recorder ready — %d channels, %d repeats...", channels, repeat);

  }

  ~Acoustic() override { Pa_Terminate(); }

private:
  
  // publishes current status for the recording
  void publishStatus(const std::string& msg)
  {
    std_msgs::msg::String m;
    m.data = msg;
    current_status->publish(m);
  }
  
  
  static std::string strip(const std::string& s)
  {
    size_t a = s.find_first_not_of(" \t\n\r");
    if (a == std::string::npos) return "";
    size_t b = s.find_last_not_of(" \t\n\r");
    return s.substr(a, b - a + 1);
  } 
  

  static std::string nowTimestamp()
  {
    auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::tm tm_local{};
    localtime_r(&t, &tm_local);
    std::ostringstream oss;
    oss << std::put_time(&tm_local, "%Y%m%d_%H%M%S");
    return oss.str();
  }
  
  // Creates the Directory to save the impulse response to
  std::string buildTargetDirectory()
  {
    std::string distance_dir = strip(occlusion_distance + "-" + occluded_distance);
    std::string target = dataset_directory + "/" + room_directory + "/" +occlusion_type + "/" + occluded_type + "/" + distance_dir;
    fs::create_directories(target);
    return target;
  }

  void load_excitation()
  {
    SF_INFO info{};
    SNDFILE* f = sf_open(excitation_path.c_str(), SFM_READ, &info);

    if (!f) {
      RCLCPP_ERROR(this->get_logger(), "Cannot open excitation: %s", excitation_path.c_str());
      return;
    }

    fs = info.samplerate;

    std::vector<float> excitation;

    if (info.channels > 1) {
      std::vector<float> interleaved(static_cast<size_t>(info.frames) * info.channels);
      sf_read_float(f, interleaved.data(), interleaved.size());

      excitation.resize(info.frames);
      for (sf_count_t i = 0; i < info.frames; ++i) {
        excitation[i] = interleaved[static_cast<size_t>(i) * info.channels];
      }
    } else {
      excitation.resize(info.frames);
      sf_read_float(f, excitation.data(), info.frames);
    }
    sf_close(f);

    this->excitation = excitation;
    N = static_cast<int>(excitation.size());

    inv_filter.resize(N);

    for (int i = 0; i < N; ++i) {
      inv_filter[i] = excitation[N - 1 - i];
    }

    RCLCPP_INFO(this->get_logger(), "Excitation loaded — N=%d samples at %d Hz", N, fs);

  }

  void listAudioDevices()
  {
    int n = Pa_GetDeviceCount();
    for (int i = 0; i < n; ++i) {
      const PaDeviceInfo* d = Pa_GetDeviceInfo(i);
      RCLCPP_INFO(this->get_logger(), "[%d] %s — in:%d out:%d", i, d->name, d->maxInputChannels, d->maxOutputChannels);
    }
  }

  void triggerCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data && !recording) {
      std::thread([this]{ runRecordingSession(); }).detach();
    }
  }

  void runRecordingSession()
  {
    recording = true;
    publishStatus("recording_started");

    size_t n_samples = excitation.size();
    if (n_samples == 0) {
      RCLCPP_ERROR(this->get_logger(), "No excitation loaded — aborting session");
      recording = false;
      return;
    }
    recorded.assign(n_samples * channels, 0.0f);

    std::string target_directory = buildTargetDirectory();

    StreamData data;
    data.excitation = excitation.data();
    data.exc_len = n_samples;
    data.recorded = recorded.data();
    data.channels = channels;
    data.play_pos = 0;
    data.rec_pos = 0;

    PaStreamParameters in_params{};
    in_params.device = mic_device;
    in_params.channelCount = channels;
    in_params.sampleFormat = paFloat32;
    in_params.suggestedLatency = Pa_GetDeviceInfo(mic_device)->defaultLowInputLatency;

    PaStreamParameters out_params{};
    out_params.device = speaker_device;
    out_params.channelCount = 1;
    out_params.sampleFormat = paFloat32;
    out_params.suggestedLatency = Pa_GetDeviceInfo(speaker_device)->defaultLowOutputLatency;

    PaStream* stream = nullptr;
    PaError err = Pa_OpenStream(&stream, &in_params, &out_params, fs, 256, paClipOff, &streamCallback, &data);

    if (err != paNoError) {
      RCLCPP_ERROR(this->get_logger(), "Pa_OpenStream failed: %s", Pa_GetErrorText(err));
      recording = false;
      return;
    }

    for (int i = 0; i < repeat; ++i) {
      RCLCPP_INFO(this->get_logger(), "--- Recording %d/%d ---", i + 1, repeat);

      data.play_pos = 0;
      data.rec_pos  = 0;
      std::fill(recorded.begin(), recorded.end(), 0.0f);

      std::string timestamp = nowTimestamp();
      std::string base = timestamp + "_" + base_filename + "_" + std::to_string(i + 1);
      std::string recorded_path = target_directory + "/" + base + ".wav";

      Pa_StartStream(stream);
      while (data.play_pos < n_samples) Pa_Sleep(10);
      Pa_Sleep(300);
      Pa_StopStream(stream);

      saveRecording(recorded_path);
      RCLCPP_INFO(this->get_logger(), "Saved %d-channel recording to: %s", channels, recorded_path.c_str());

      std::vector<std::vector<float>> irs = compute_ir(recorded, channels);
      std::vector<std::vector<std::vector<float>>> spectrograms(channels);

      for (int ch = 0; ch < channels; ++ch) {
        spectrograms[ch] = compute_spectrogram(irs[ch]);
      }

      if (save_ir_spec) {
        save_ir_npy(target_directory + "/" + base + "_ir.npy", irs);
        save_spec_npy(target_directory + "/" + base + "_spec.npy", spectrograms);
        RCLCPP_INFO(this->get_logger(), "Saved ir/spec: %s_ir.npy, %s_spec.npy", base.c_str(), base.c_str());
      }

      std_msgs::msg::String saved_msg;
      saved_msg.data = recorded_path;
      saved_recording->publish(saved_msg);

      publishStatus("repeat_" + std::to_string(i + 1) + "_done");

      if (i < repeat - 1) {
        RCLCPP_INFO(this->get_logger(), "Waiting %d seconds before next recording...", sleep_duration);
        std::this_thread::sleep_for(std::chrono::seconds(sleep_duration));
      }
    }

    Pa_CloseStream(stream);
    recording = false;

    std_msgs::msg::Bool msg;
    msg.data = true;
    complete_recording->publish(msg);
    publishStatus("recording_complete");

    RCLCPP_INFO(this->get_logger(), "Recording session complete");
  }

  struct StreamData {
    const float* excitation;
    size_t exc_len;
    float* recorded;
    int channels;
    std::atomic<size_t> play_pos;
    std::atomic<size_t> rec_pos;
  };

  static int streamCallback(
    const void* input, void* output, unsigned long frame_count,
    const PaStreamCallbackTimeInfo*, PaStreamCallbackFlags, void* user_data)
  {
    auto* d   = static_cast<StreamData*>(user_data);
    auto* in  = static_cast<const float*>(input);
    auto* out = static_cast<float*>(output);

    for (unsigned long f = 0; f < frame_count; ++f) {
      out[f] = (d->play_pos < d->exc_len) ? d->excitation[d->play_pos++] : 0.0f;
    }

    size_t rec_capacity = d->exc_len * static_cast<size_t>(d->channels);
    size_t rec_pos      = d->rec_pos.load();

    if (in != nullptr && rec_pos < rec_capacity) {

      size_t samples_to_copy = frame_count * static_cast<size_t>(d->channels);
      size_t max_copy = std::min(samples_to_copy, rec_capacity - rec_pos);
      std::memcpy(d->recorded + rec_pos, in, max_copy * sizeof(float));
      d->rec_pos += max_copy;
    }

    bool done = (d->play_pos >= d->exc_len) && (d->rec_pos.load() >= rec_capacity);
    return done ? paComplete : paContinue;
  }

  void saveRecording(const std::string& path)
  {
    SF_INFO info{};
    info.samplerate = fs;
    info.channels = channels;
    info.format = SF_FORMAT_WAV | SF_FORMAT_FLOAT;

    SNDFILE* f = sf_open(path.c_str(), SFM_WRITE, &info);
    if (!f) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open %s for writing", path.c_str());
      return;
    }
    sf_write_float(f, recorded.data(), recorded.size());
    sf_close(f);
  }

  //  Full linear convolution via FFT: fftconvolve(a, b, mode='full') 
  std::vector<float> fftconvolve_full(const std::vector<float>& a, const std::vector<float>& b)
  {
    const size_t n_fft     = a.size() + b.size() - 1;
    const size_t n_complex = n_fft / 2 + 1;

    double* a_pad = fftw_alloc_real(n_fft);
    double* b_pad = fftw_alloc_real(n_fft);

    std::fill(a_pad, a_pad + n_fft, 0.0);
    std::fill(b_pad, b_pad + n_fft, 0.0);

    for (size_t i = 0; i < a.size(); ++i) a_pad[i] = static_cast<double>(a[i]);
    for (size_t i = 0; i < b.size(); ++i) b_pad[i] = static_cast<double>(b[i]);

    fftw_complex* a_freq = fftw_alloc_complex(n_complex);
    fftw_complex* b_freq = fftw_alloc_complex(n_complex);
    fftw_complex* prod   = fftw_alloc_complex(n_complex);
    double* conv_result  = fftw_alloc_real(n_fft);

    fftw_plan p_a = fftw_plan_dft_r2c_1d(static_cast<int>(n_fft), a_pad, a_freq, FFTW_ESTIMATE);
    fftw_plan p_b = fftw_plan_dft_r2c_1d(static_cast<int>(n_fft), b_pad, b_freq, FFTW_ESTIMATE);
    fftw_execute(p_a);
    fftw_execute(p_b);

    for (size_t i = 0; i < n_complex; ++i) {
      double re = a_freq[i][0] * b_freq[i][0] - a_freq[i][1] * b_freq[i][1];
      double im = a_freq[i][0] * b_freq[i][1] + a_freq[i][1] * b_freq[i][0];
      prod[i][0] = re;
      prod[i][1] = im;
    }

    fftw_plan p_inv = fftw_plan_dft_c2r_1d(static_cast<int>(n_fft), prod, conv_result, FFTW_ESTIMATE);
    fftw_execute(p_inv);
    for (size_t i = 0; i < n_fft; ++i) conv_result[i] /= static_cast<double>(n_fft);

    std::vector<float> result(n_fft);
    for (size_t i = 0; i < n_fft; ++i) result[i] = static_cast<float>(conv_result[i]);

    fftw_destroy_plan(p_a);
    fftw_destroy_plan(p_b);
    fftw_destroy_plan(p_inv);
    fftw_free(a_pad);
    fftw_free(b_pad);
    fftw_free(a_freq);
    fftw_free(b_freq);
    fftw_free(prod);
    fftw_free(conv_result);
    return result;
  }
 
  std::vector<std::vector<float>> compute_ir(const std::vector<float>& data_interleaved, int n_channels)
  {
    std::vector<std::vector<float>> irs(n_channels);
    size_t n_rec = data_interleaved.size() / static_cast<size_t>(n_channels);

    for (int ch = 0; ch < n_channels; ++ch) {
      std::vector<float> channel_data(n_rec);
      for (size_t i = 0; i < n_rec; ++i) {
        channel_data[i] = data_interleaved[i * n_channels + ch];
      }

      std::vector<float> ir_full = fftconvolve_full(channel_data, inv_filter);

      // ir_full = ir_full[N:N*2]
      size_t lo = static_cast<size_t>(N);
      size_t hi = std::min(ir_full.size(), static_cast<size_t>(N) * 2);
      std::vector<float> windowed(ir_full.begin() + std::min(lo, ir_full.size()), ir_full.begin() + hi);

      // irs.append(ir_full[start_sample:end_sample])
      int s = std::max(0, start_sample);
      int e = std::min(static_cast<int>(windowed.size()), end_sample);
      if (s > e) s = e;
      irs[ch] = std::vector<float>(windowed.begin() + s, windowed.begin() + e);
    }
    return irs;
  }


  static std::vector<double> tukey_window(int n, double alpha)
  {
    std::vector<double> w(n, 1.0);
    if (n == 1) return w;
    double edge = alpha * (n - 1) / 2.0;

    for (int i = 0; i < n; ++i) {
      if (i < edge) {
        w[i] = 0.5 * (1.0 + std::cos(M_PI * (2.0 * i / (alpha * (n - 1)) - 1.0)));
      } else if (i > (n - 1) - edge) {
        w[i] = 0.5 * (1.0 + std::cos(M_PI * (2.0 * i / (alpha * (n - 1)) - 2.0 / alpha + 1.0)));
      } else {
        w[i] = 1.0;
      }
    }
    return w;
  }


  std::vector<std::vector<float>> compute_spectrogram(const std::vector<float>& ir)
  {

    int nperseg_curr  = nperseg;
    int noverlap_curr = noverlap;
    int hop_curr = nperseg_curr - noverlap_curr;
    int n_freq_curr   = nperseg_curr / 2 + 1;

    std::vector<double> window = tukey_window(nperseg_curr, 0.25);
    double win_energy = 0.0;
    for (double w : window) win_energy += w * w;

    int n_ir = static_cast<int>(ir.size());
    int n_segments = (n_ir >= nperseg_curr) ? (1 + (n_ir - nperseg_curr) / hop_curr) : 0;

    std::vector<std::vector<float>> Sxx(n_freq_curr, std::vector<float>(std::max(n_segments, 0), 0.0f));
    if (n_segments == 0) return Sxx;

    double* seg_buf     = fftw_alloc_real(nperseg_curr);
    fftw_complex* freq_buf = fftw_alloc_complex(n_freq_curr);
    fftw_plan plan = fftw_plan_dft_r2c_1d(nperseg_curr, seg_buf, freq_buf, FFTW_ESTIMATE);

    for (int t = 0; t < n_segments; ++t) {
      int start = t * hop_curr;
      for (int i = 0; i < nperseg_curr; ++i) {
        seg_buf[i] = static_cast<double>(ir[start + i]) * window[i];
      }
      fftw_execute(plan);

      for (int f = 0; f < n_freq_curr; ++f) {
        double power = freq_buf[f][0] * freq_buf[f][0] + freq_buf[f][1] * freq_buf[f][1];
        double psd = power / (static_cast<double>(fs) * win_energy);
        // one-sided: double all bins except DC, and Nyquist when nperseg is even
        bool is_nyquist = (nperseg_curr % 2 == 0) && (f == n_freq_curr - 1);
        if (f != 0 && !is_nyquist) psd *= 2.0;
        Sxx[f][t] = static_cast<float>(psd);
      }
    }

    fftw_destroy_plan(plan);
    fftw_free(seg_buf);
    fftw_free(freq_buf);
    return Sxx;
  }

  
  void save_npy(const std::string& path, const std::vector<float>& flat_data, const std::vector<size_t>& shape)
  {
    std::ofstream f(path, std::ios::binary);
    if (!f) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open %s for writing", path.c_str());
      return;
    }

    const char magic[6] = {'\x93', 'N', 'U', 'M', 'P', 'Y'};
    f.write(magic, 6);
    uint8_t major = 1, minor = 0;
    f.write(reinterpret_cast<char*>(&major), 1);
    f.write(reinterpret_cast<char*>(&minor), 1);

    std::ostringstream shape_str;
    shape_str << "(";
    for (size_t i = 0; i < shape.size(); ++i) {
      shape_str << shape[i];
      if (shape.size() == 1 || i < shape.size() - 1) shape_str << ", ";
    }

    shape_str << ")";

    std::ostringstream header;
    header << "{'descr': '<f4', 'fortran_order': False, 'shape': " << shape_str.str() << ", }";

    size_t unpadded = 10 + header.str().size() + 1;
    size_t pad = (64 - (unpadded % 64)) % 64;
    header << std::string(pad, ' ') << '\n';

    uint16_t header_len = static_cast<uint16_t>(header.str().size());

    f.write(reinterpret_cast<char*>(&header_len), 2);
    f.write(header.str().c_str(), header_len);

    f.write(reinterpret_cast<const char*>(flat_data.data()), flat_data.size() * sizeof(float));
  }

  // irs: [channels][ir_len]  ->  saved as shape (channels, ir_len)
  void save_ir_npy(const std::string& path, const std::vector<std::vector<float>>& irs)
  {
    size_t n_ch = irs.size();
    size_t ir_len = n_ch > 0 ? irs[0].size() : 0;
    std::vector<float> flat(n_ch * ir_len);
    for (size_t ch = 0; ch < n_ch; ++ch) {
      std::copy(irs[ch].begin(), irs[ch].end(), flat.begin() + ch * ir_len);
    }

    save_npy(path, flat, {n_ch, ir_len});
  }

  // spectrograms: [channels][freq][time] -> saved as shape (channels, freq, time)
  void save_spec_npy(const std::string& path, const std::vector<std::vector<std::vector<float>>>& spectrograms)
  {
    size_t n_ch = spectrograms.size();
    size_t n_freq = n_ch > 0 ? spectrograms[0].size() : 0;
    size_t n_time = (n_ch > 0 && n_freq > 0) ? spectrograms[0][0].size() : 0;

    std::vector<float> flat(n_ch * n_freq * n_time);
    for (size_t ch = 0; ch < n_ch; ++ch) {
      for (size_t fbin = 0; fbin < n_freq; ++fbin) {
        for (size_t tbin = 0; tbin < n_time; ++tbin) {
          flat[ch * n_freq * n_time + fbin * n_time + tbin] = spectrograms[ch][fbin][tbin];
        }
      }
    }
    save_npy(path, flat, {n_ch, n_freq, n_time});
  }

  std::string dataset_directory, room_directory, excitation_path;
  std::string occlusion_type, occlusion_distance, occluded_type, occluded_distance;
  std::string base_filename;
  int channels, repeat, sleep_duration, start_sample, end_sample;
  int nperseg, noverlap;
  int mic_device, speaker_device, fs, N;
  bool save_ir_spec;

  std::vector<float> excitation;
  std::vector<float> inv_filter;
  std::vector<float> recorded;
  std::atomic<bool>  recording{false};

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr   start_recording;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr      complete_recording;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr    current_status;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr    saved_recording;
};


int main(int argc, char** argv)
{

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Acoustic>());
  rclcpp::shutdown();
  return 0;

}
