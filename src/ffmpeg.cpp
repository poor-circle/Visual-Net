#include <cstdlib>
#include <cstring>
#include <cstdio>
namespace ffmpeg
{
	constexpr int MAXBUFLEN = 256;
	const char ffmpegPath[] = "ffmpeg\\bin\\";
	const char tmpPath[] = "tmpdir";
	int VideotoImage(const char* videoPath,//将路径为videoPath的视频
		const char* imagePath,             //保存到目录imagePath中
		const char* imageFormat);          //拆解成后缀名为imageFormat的帧图像
	int ImagetoVideo(const char* imagePath,//将所在文件夹的路径为imagePath
		const char* imageFormat,           //后缀名为imageFormat的图像
		const char* videoPath,             //合成的视频保存到路径videopath
		unsigned rawFrameRates = 30,       //将原图片序列视为多少帧率（输出时长等于总帧数除以源帧率）
		unsigned outputFrameRates = 30,    //实际输出帧率（如果大于源帧率则插帧，小于源帧率则掉帧）
		unsigned kbps = 0);                //帧率为kbps，默认为0，即ffmpeg自动决定帧率
//掉帧将会严重影响信息传输，故通常不应该出现输出帧率小于源帧率的情况。
	int ImagetoVideo(const char* imagePath,
		const char* imageFormat,
		const char* videoPath,
		unsigned rawFrameRates,
		unsigned outputFrameRates,
		unsigned kbps)
	{
		char BUF[MAXBUFLEN];
		if (kbps)
			snprintf(BUF, MAXBUFLEN,
				"\"%s\"ffmpeg.exe -r %u  -f image2 -i %s\\%%05d.%s -b:v %uK -vcodec libx264  -r %u %s",
				ffmpegPath, rawFrameRates, imagePath, imageFormat, kbps, outputFrameRates, videoPath);
		else
			snprintf(BUF, MAXBUFLEN,
				"\"%s\"ffmpeg.exe -r %u -f image2 -i %s\\%%05d.%s  -vcodec libx264 -r %u %s",
				ffmpegPath, rawFrameRates, imagePath, imageFormat, outputFrameRates, videoPath);
		return system(BUF);
	}
	int VideotoImage(const char* videoPath,
		const char* imagePath,
		const char* imageFormat)
	{
		char BUF[MAXBUFLEN];
		snprintf(BUF, MAXBUFLEN, "md %s", imagePath); //生成文件目录
		system(BUF);
		snprintf(BUF, MAXBUFLEN,
			"\"%s\"ffmpeg.exe -i %s -q:v 2 -f image2  %s\\%%05d.%s",
			ffmpegPath, videoPath, imagePath, imageFormat);
		return system(BUF);
	}
	int test(void)
	{
		bool tag = VideotoImage("test.mp4", tmpPath, "png");
		if (tag)
			return tag;
		tag = ImagetoVideo(tmpPath, "png", "out.mp4", 30, 30);
		return tag;
	}
} // namespace ffmpeg
