#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <fstream>
#include <alsa/asoundlib.h>
#define SOUND_DEVICE "hw:1,0"

bool playAudio(const char* filepath) {
  snd_pcm_t* pcmHandle;
  snd_pcm_hw_params_t* params;
  unsigned int sampleRate = 44100; // 默认采样率
  int dir;
  snd_pcm_uframes_t frames;
  constexpr int channels = 2; // 默认双声道
  // 缓冲区大小
  // 打开音频设备
  if (snd_pcm_open(&pcmHandle, SOUND_DEVICE, SND_PCM_STREAM_PLAYBACK, 0) < 0)
    return false;
  // 分配硬件参数结构体
  snd_pcm_hw_params_malloc(&params);
  snd_pcm_hw_params_any(pcmHandle, params);
  // 设置访问类型为交错模式
  snd_pcm_hw_params_set_access(pcmHandle, params, SND_PCM_ACCESS_RW_INTERLEAVED);
  // 设置音频格式（16位小端）
  snd_pcm_hw_params_set_format(pcmHandle, params, SND_PCM_FORMAT_S16_LE);
  // 设置声道数
  snd_pcm_hw_params_set_channels(pcmHandle, params, channels);
  // 设置采样率
  snd_pcm_hw_params_set_rate_near(pcmHandle, params, &sampleRate, &dir);
  // 设置每帧的样本数量
  frames = 32;
  snd_pcm_hw_params_set_period_size_near(pcmHandle, params, &frames, &dir);
  // 应用参数到设备
  if (snd_pcm_hw_params(pcmHandle, params) < 0) {
    snd_pcm_close(pcmHandle);
    return false;
  }
  // 获取帧大小
  snd_pcm_hw_params_get_period_size(params, &frames, &dir);
  // 打开音频文件
  std::ifstream audioFile(filepath, std::ios::binary);
  if (!audioFile) {
    snd_pcm_close(pcmHandle);
    return false;
  }
  // 分配缓冲区
  const int bufferSize = frames * channels * 2; // 每帧占用2字节（16位）
  const auto buffer = new char[bufferSize];
  // 播放音频
  while (!audioFile.eof()) {
    audioFile.read(buffer, bufferSize);
    const std::streamsize bytesRead = audioFile.gcount();
    if (bytesRead > 0) {
      const int pcm = snd_pcm_writei(pcmHandle, buffer, bytesRead / (channels * 2));
      if (pcm < 0)
        snd_pcm_prepare(pcmHandle); // 恢复流
    }
  }
  // 清理资源
  delete[] buffer;
  audioFile.close();
  snd_pcm_close(pcmHandle);
  return true;
}

void playCallback(const std_msgs::UInt8& msg) {
  if (msg.data == 0)
    playAudio("/home/khadas/sound/start.wav");
  else if(msg.data == 1)
    playAudio("/home/khadas/reach.wav");
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
