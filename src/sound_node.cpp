#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <fstream>
#include <alsa/asoundlib.h>
#define SOUND_DEVICE "hw:2,0"

bool playAudio(const char* filepath) {
    snd_pcm_t* pcmHandle;
    snd_pcm_hw_params_t* params;
    unsigned int sampleRate = 44100;
    int dir;
    snd_pcm_uframes_t frames;
    int channels = 2;
    if (snd_pcm_open(&pcmHandle, SOUND_DEVICE, SND_PCM_STREAM_PLAYBACK, 0) < 0) {
        ROS_ERROR("Error: Unable to open PCM device");
        return false;
    }
    snd_pcm_hw_params_malloc(&params);
    snd_pcm_hw_params_any(pcmHandle, params);
    snd_pcm_hw_params_set_access(pcmHandle, params, SND_PCM_ACCESS_RW_INTERLEAVED);
    snd_pcm_hw_params_set_format(pcmHandle, params, SND_PCM_FORMAT_S16_LE);
    snd_pcm_hw_params_set_channels(pcmHandle, params, channels);
    snd_pcm_hw_params_set_rate_near(pcmHandle, params, &sampleRate, &dir);
    frames = 32;
    snd_pcm_hw_params_set_period_size_near(pcmHandle, params, &frames, &dir);
    if (snd_pcm_hw_params(pcmHandle, params) < 0) {
        ROS_ERROR("Error: Unable to set hardware parameters.");
        snd_pcm_close(pcmHandle);
        return false;
    }
    snd_pcm_hw_params_get_period_size(params, &frames, &dir);
    int bufferSize = frames * channels * 2;
    char* buffer = new char[bufferSize];
    std::ifstream audioFile(filepath, std::ios::binary);
    if (!audioFile) {
        ROS_ERROR("Error: Unable to open audio file");
        snd_pcm_close(pcmHandle);
        return false;
    }
    while (!audioFile.eof() || snd_pcm_avail_update(pcmHandle) > 0) {
        audioFile.read(buffer, bufferSize);
        std::streamsize bytesRead = audioFile.gcount();
        if (bytesRead < bufferSize) {
            std::fill(buffer + bytesRead, buffer + bufferSize, 0);
        }
        int pcm = snd_pcm_writei(pcmHandle, buffer, bufferSize / (channels * 2));
        if (pcm < 0) {
            if (pcm == -EPIPE) {
                snd_pcm_prepare(pcmHandle);
            } else {
                ROS_ERROR("Error: %s", snd_strerror(pcm));
                break;
            }
        }
    }
    delete[] buffer;
    audioFile.close();
    snd_pcm_drain(pcmHandle);
    snd_pcm_close(pcmHandle);
    ROS_INFO("Playback finished.");
    return true;
}

void playCallback(const std_msgs::UInt8& msg) {
  if (msg.data == 0)
    playAudio("/home/khadas/sound/start.wav");
  else if(msg.data == 1)
    playAudio("/home/khadas/sound/reach.wav");
  else if(msg.data == 2)
    playAudio("/home/khadas/sound/cube.wav");
  else if(msg.data == 3)
    playAudio("/home/khadas/sound/long.wav");
  else if(msg.data == 4)
    playAudio("/home/khadas/sound/circle.wav");
  else if(msg.data == 5)
    playAudio("/home/khadas/sound/mission_end.wav");
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "sound_node");
  ros::NodeHandle node_handle;
  ros::Subscriber subscriber = node_handle.subscribe("/play", 2, playCallback);
  ros::spin();
}
