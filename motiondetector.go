// Example program that uses blakjack/webcam library
// for working with V4L2 devices.
// The application reads frames from device and writes them to stdout
// If your device supports motion formats (e.g. H264 or MJPEG) you can
// use it's output as a video stream.
// Example usage: go run stdout_streamer.go | vlc -
package main

import (
	"fmt"
	"github.com/blackjack/webcam"
	"image"
	"image/jpeg"
	"log"
	"os"
	"sort"
	"time"
)

type FrameSizes []webcam.FrameSize

// Global variables
var (
	cam              *webcam.Webcam
	testmode         bool = false
	nowRecording     bool = false
	recordingStarted time.Time
	recordingDir     string
	photoNumber      int
	lastMove         time.Time
	YUVimage         []*image.YCbCr
	size             webcam.FrameSize
	subSampleRatio   image.YCbCrSubsampleRatio
	frameNr          int = 0
	currentImage     int = 0
	previousImage    int = 1
	nPixels          float64
	MotionDetected   bool
)

// Global constants
const (
	videoCam                                          = "/dev/video0"
	TIMELAG                             time.Duration = 500 * time.Millisecond
	FRAMES_TO_WAIT                                    = 40
	IND_PIXEL_THRESHOLD                               = 40
	MOTION_DETECT_THRESHOLD                           = 1500
	INACTIVITY_PERIOD_TO_STOP_RECORDING               = 20 * time.Second
	TIMEOUT                                           = 5 // Timeout when waiting for frames
)

// Utility functions
func readChoice(s string) int {
	var i int
	for true {
		print(s)
		_, err := fmt.Scanf("%d\n", &i)
		if err != nil || i < 1 {
			println("Invalid input. Try again")
		} else {
			break
		}
	}
	return i
}

func (slice FrameSizes) Len() int {
	return len(slice)
}

//For sorting purposes
func (slice FrameSizes) Less(i, j int) bool {
	ls := slice[i].MaxWidth * slice[i].MaxHeight
	rs := slice[j].MaxWidth * slice[j].MaxHeight
	return ls < rs
}

//For sorting purposes
func (slice FrameSizes) Swap(i, j int) {
	slice[i], slice[j] = slice[j], slice[i]
}

// Cam configuration and opening

func configure() {
	if len(os.Args) >= 2 && os.Args[1] == "-t" {
		testmode = true
	} else {
		testmode = false
	}
	var err error
	cam, err = webcam.Open(videoCam)
	if err != nil {
		panic(err.Error())
	}

	format_desc := cam.GetSupportedFormats()
	var format webcam.PixelFormat
	for f, thisFormat := range format_desc {
		if thisFormat[0:4] == "YUYV" {
			format = f
		}
	}

	format_chosen := format_desc[format]

	var subsampleratio_chosen string

	subsampleratio_chosen = format_chosen[len(format_chosen)-5:]
	switch subsampleratio_chosen {
	case "4:1:0":
		subSampleRatio = image.YCbCrSubsampleRatio410
	case "4:1:1":
		subSampleRatio = image.YCbCrSubsampleRatio411
	case "4:2:0":
		subSampleRatio = image.YCbCrSubsampleRatio420
	case "4:2:2":
		subSampleRatio = image.YCbCrSubsampleRatio422
	case "4:4:0":
		subSampleRatio = image.YCbCrSubsampleRatio440
	case "4:4:4":
		subSampleRatio = image.YCbCrSubsampleRatio444
	default:
		log.Panic("Unknown subsample ratio:", subsampleratio_chosen)
	}
	fmt.Printf("Format is %v\n", format_chosen)

	frames := FrameSizes(cam.GetSupportedFrameSizes(format))
	sort.Sort(frames)

	size = frames[len(frames)-1]
	fmt.Printf("Size is %v\n", size.GetString())

	_, _, _, err = cam.SetImageFormat(format, uint32(size.MaxWidth), uint32(size.MaxHeight))
	if err != nil {
		panic(err.Error())
	}

	_, _, _, err = cam.SetImageFormat(format, uint32(size.MaxWidth), uint32(size.MaxHeight))
	if err != nil {
		panic(err.Error())
	}

	YUVimage = []*image.YCbCr{
		image.NewYCbCr(image.Rect(0, 0, int(size.MaxWidth), int(size.MaxHeight)), subSampleRatio),
		image.NewYCbCr(image.Rect(0, 0, int(size.MaxWidth), int(size.MaxHeight)), subSampleRatio),
	}

	nPixels = float64(size.MaxWidth) * float64(size.MaxHeight)

	return
}

////////////////////////////////////
// Main function
////////////////////////////////////

func main() {

	configure()

	println("Press Enter to start surveillance")
	fmt.Scanf("\n")

	err := cam.StartStreaming()
	if err != nil {
		panic(err.Error())
	}

	for { // infinite for
		err = cam.WaitForFrame(TIMEOUT)

		switch err.(type) {
		case nil:
		case *webcam.Timeout:
			fmt.Fprint(os.Stderr, err.Error())
			continue
		default:
			panic(err.Error())
		}

		frame, err := cam.ReadFrame()
		if err != nil || len(frame) == 0 {
			log.Panicf("Error reading frame after WaitForFrame, err was [%v]", err)
		}
		for {
			lastframe := frame
			frame, err = cam.ReadFrame()
			if err != nil || len(frame) == 0 {
				frame = lastframe
				break
			}
		}

		for i := range YUVimage[currentImage].Cb {
			YUVimage[currentImage].Y[2*i] = frame[4*i]
			YUVimage[currentImage].Y[2*i+1] = frame[4*i+2]
			YUVimage[currentImage].Cb[i] = frame[4*i+1]
			YUVimage[currentImage].Cr[i] = frame[4*i+3]
		}
		//cam.StopStreaming()
		DiffSq := make([]float64, len(YUVimage[0].Cb))

		// We first calculate diffs and avg diff
		AvgDifferenceSq := float64(0)
		for i := range YUVimage[0].Cb {
			DifY1 := float64(YUVimage[currentImage].Y[2*i]) - float64(YUVimage[previousImage].Y[2*i])
			DifY2 := float64(YUVimage[currentImage].Y[2*i+1]) - float64(YUVimage[previousImage].Y[2*i+1])
			DifCb := float64(YUVimage[currentImage].Cb[i]) - float64(YUVimage[previousImage].Cb[i])
			DifCr := float64(YUVimage[currentImage].Cr[i]) - float64(YUVimage[previousImage].Cr[i])
			DiffSq[i] += DifY1*DifY1 + DifY2*DifY2 + DifCb*DifCb + DifCr*DifCr
			AvgDifferenceSq += DiffSq[i]
		}
		AvgDifferenceSq /= nPixels

		// Now calculating background noise
		BackgroundNoiseSq := float64(0)
		nSamplesInNoise := float64(0)
		for i := range YUVimage[0].Cb {
			if DiffSq[i] < AvgDifferenceSq {
				BackgroundNoiseSq += DiffSq[i]
				nSamplesInNoise++
			}
		}
		if nSamplesInNoise > 0 {
			BackgroundNoiseSq /= nSamplesInNoise
		}

		// Now calculating pixels in motion
		MovingPixels := float64(0)
		for i := range YUVimage[0].Cb {
			if DiffSq[i] > BackgroundNoiseSq+4*IND_PIXEL_THRESHOLD*IND_PIXEL_THRESHOLD {
				MovingPixels++
			}
		}
		MovingPixels = 1000000.0 * MovingPixels / nPixels // Now in number of pixels per million

		if MovingPixels > MOTION_DETECT_THRESHOLD {
			MotionDetected = true
		} else {
			MotionDetected = false
		}

		if testmode {
			fmt.Printf("Moving pixels are %.1f (ppm)\n", MovingPixels)
		}

		if !nowRecording && frameNr > FRAMES_TO_WAIT && MotionDetected {
			nowRecording = true
			recordingStarted = time.Now()
			if testmode {
				fmt.Println(">> Motion detected, now recording!")
			} else {
				recordingDir = fmt.Sprintf(
					"Frames-%04d-%02d-%02d-%02d-%02d-%02d-%02d",
					recordingStarted.Year(),
					recordingStarted.Month(),
					recordingStarted.Day(),
					recordingStarted.Hour(),
					recordingStarted.Minute(),
					recordingStarted.Second(),
					recordingStarted.Nanosecond(),
				)
				os.Mkdir(recordingDir, 0755)
				photoNumber = 0
			}
		}

		if MotionDetected {
			lastMove = time.Now()
		}

		if nowRecording && !MotionDetected && time.Since(lastMove) > INACTIVITY_PERIOD_TO_STOP_RECORDING {
			nowRecording = false
			if testmode {
				fmt.Println(">> Not recording anymore")
			}
		}

		if !testmode && nowRecording {
			file, err := os.Create(recordingDir + fmt.Sprintf("/%v.jpg", photoNumber))
			if err != nil {
				panic(err.Error())
			}
			jpeg.Encode(file, YUVimage[currentImage], &jpeg.Options{Quality: 100})
			err = file.Close()
			if err != nil {
				panic(err.Error())
			}
			photoNumber++
		}

		currentImage = (currentImage + 1) % 2
		previousImage = (currentImage + 1) % 2

		frameNr++
		time.Sleep(TIMELAG)

	} // Infinite for
}
