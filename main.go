package main

/*
	Some of the code in here was taken and adapted from
	- https://github.com/hybridgroup/gobot/blob/master/examples/tello_opencv.go
	- https://github.com/hybridgroup/gobot/blob/master/examples/tello_ps3.go
*/

import (
	"flag"
	"fmt"
	"image"
	"image/color"
	"io"
	"net/http"
	"os/exec"
	"strconv"
	"sync/atomic"
	"time"

	"github.com/mattn/go-mjpeg"
	"github.com/pkg/browser"
	"github.com/sirupsen/logrus"
	"gobot.io/x/gobot"
	"gobot.io/x/gobot/platforms/dji/tello"
	"gobot.io/x/gobot/platforms/joystick"
	"gocv.io/x/gocv"
)

const (
	frameX    = 960
	frameY    = 720
	frameSize = frameX * frameY * 3
)

type stickPair struct {
	x float64
	y float64
}

var (
	leftX        atomic.Value
	leftY        atomic.Value
	rightX       atomic.Value
	rightY       atomic.Value
	batteryLevel atomic.Value
	flightData   atomic.Value

	// command line parameters
	outputFile     string
	outputCodec    string
	controller     string
	listenPort     string
	streamPort     int
	faceClassifier string // not used because it is slow as shit
)

const offset = 32767.0

func init() {
	flag.StringVar(&outputFile, "output", "flight.avi", "Where to write the recorded video of the flight")
	flag.StringVar(&outputCodec, "codec", "MPEG", "Output codec, you probably should not touch that")
	flag.StringVar(&controller, "controller", "logitech.json", "Which controller definition to use")
	flag.StringVar(&listenPort, "port", "6666", "Which port to listen on")
	flag.IntVar(&streamPort, "stream-port", 8080, "Which port to listen on for the mjpeg stream")
	flag.StringVar(&faceClassifier, "face-classifier", "/usr/share/opencv4/haarcascades/haarcascade_frontalface_alt.xml", "Which face classifier to use")
}

func main() {
	flag.Parse()

	drone := tello.NewDriver(listenPort)
	joystickAdaptor := joystick.NewAdaptor()
	stick := joystick.NewDriver(joystickAdaptor, controller)

	ffmpeg := exec.Command("ffmpeg", "-hwaccel", "auto", "-hwaccel_device", "opencl", "-i", "pipe:0",
		"-pix_fmt", "bgr24", "-s", strconv.Itoa(frameX)+"x"+strconv.Itoa(frameY), "-f", "rawvideo", "pipe:1",
	)

	ffmpegIn, _ := ffmpeg.StdinPipe()
	ffmpegOut, _ := ffmpeg.StdoutPipe()
	defer ffmpeg.Process.Kill()

	classifier := gocv.NewCascadeClassifier()
	defer classifier.Close()

	if !classifier.Load(faceClassifier) {
		logrus.Fatalf("Error reading cascade file: %v", faceClassifier)
	}

	output, err := gocv.VideoWriterFile(outputFile, outputCodec, 25, frameX, frameY, true)
	if err != nil {
		logrus.WithError(err).Fatal("Could not open the output flight footage file")
	}
	defer output.Close()

	stream := mjpeg.NewStreamWithInterval(time.Millisecond * 50)
	http.HandleFunc("/mjpeg", stream.ServeHTTP)

	http.HandleFunc("/", func(w http.ResponseWriter, r *http.Request) {
		w.Header().Set("Content-Type", "text/html")
		w.Write([]byte(`<img src="/mjpeg" />`))
	})

	server := &http.Server{Addr: fmt.Sprintf(":%d", streamPort)}
	go func() {
		server.ListenAndServe()
	}()

	browser.OpenURL(fmt.Sprintf("http://localhost:%d", streamPort))

	work := func() {
		leftX.Store(float64(0.0))
		leftY.Store(float64(0.0))
		rightX.Store(float64(0.0))
		rightY.Store(float64(0.0))

		if err := ffmpeg.Start(); err != nil {
			fmt.Println(err)
			return
		}

		drone.On(tello.ConnectedEvent, func(data interface{}) {
			drone.StartVideo()
			drone.SetVideoEncoderRate(tello.VideoBitRate1M)
			drone.SetExposure(0)

			gobot.Every(100*time.Millisecond, func() {
				drone.StartVideo()
			})
		})

		drone.On(tello.VideoFrameEvent, func(data interface{}) {
			pkt := data.([]byte)
			if _, err := ffmpegIn.Write(pkt); err != nil {
				fmt.Println(err)
			}
		})

		drone.On(tello.FlightDataEvent, func(data interface{}) {
			fd := data.(*tello.FlightData)
			flightData.Store(fd)
		})
	}

	robot := gobot.NewRobot("drone",
		[]gobot.Connection{},
		[]gobot.Device{drone},
		work,
	)

	robot.Start(false)

	workStick := func() {
		stick.On(joystick.SquarePress, func(data interface{}) {
			drone.LeftFlip()
		})

		stick.On(joystick.TrianglePress, func(data interface{}) {
			drone.TakeOff()
		})
		stick.On(joystick.CirclePress, func(data interface{}) {
			drone.RightFlip()
		})
		stick.On(joystick.XPress, func(data interface{}) {
			drone.Land()
		})
		stick.On(joystick.StartPress, func(data interface{}) {
		})
		stick.On(joystick.SelectPress, func(data interface{}) {
		})

		stick.On(joystick.LeftX, func(data interface{}) {
			value := float64(data.(int16))
			leftX.Store(value)
		})

		stick.On(joystick.LeftY, func(data interface{}) {
			value := float64(data.(int16))
			leftY.Store(value)
		})

		stick.On(joystick.RightX, func(data interface{}) {
			value := float64(data.(int16))
			rightX.Store(value)
		})

		stick.On(joystick.RightY, func(data interface{}) {
			value := float64(data.(int16))
			rightY.Store(value)
		})

		gobot.Every(50*time.Millisecond, func() {
			rightStick := getRightStick()

			switch {
			case rightStick.y < -10:
				drone.Forward(tello.ValidatePitch(rightStick.y, offset))
			case rightStick.y > 10:
				drone.Backward(tello.ValidatePitch(rightStick.y, offset))
			default:
				drone.Forward(0)
			}

			switch {
			case rightStick.x > 10:
				drone.Right(tello.ValidatePitch(rightStick.x, offset))
			case rightStick.x < -10:
				drone.Left(tello.ValidatePitch(rightStick.x, offset))
			default:
				drone.Right(0)
			}
		})

		gobot.Every(50*time.Millisecond, func() {
			leftStick := getLeftStick()
			switch {
			// the -1000 and +1000 values are so that you dont accidentally go
			// up and down when trying to rotate the drone
			case leftStick.y < -1000:
				drone.Up(tello.ValidatePitch(leftStick.y, offset))
			case leftStick.y > 1000:
				drone.Down(tello.ValidatePitch(leftStick.y, offset))
			default:
				drone.Up(0)
			}

			switch {
			case leftStick.x > 20:
				drone.Clockwise(tello.ValidatePitch(leftStick.x, offset))
			case leftStick.x < -20:
				drone.CounterClockwise(tello.ValidatePitch(leftStick.x, offset))
			default:
				drone.Clockwise(0)
			}
		})
	}

	robotStick := gobot.NewRobot("joystick",
		[]gobot.Connection{joystickAdaptor},
		[]gobot.Device{stick},
		workStick,
	)

	robotStick.Start(false)

	for {
		buf := make([]byte, frameSize)
		if _, err := io.ReadFull(ffmpegOut, buf); err != nil {
			fmt.Println(err)
			continue
		}

		img, err := gocv.NewMatFromBytes(frameY, frameX, gocv.MatTypeCV8UC3, buf)
		if img.Empty() {
			continue
		}

		if err != nil {
			logrus.WithError(err).Error("could not decode image")
			continue
		}

		fd := flightData.Load().(*tello.FlightData)

		boolToStatus := map[bool]string{
			true:  "OK",
			false: "HIGH",
		}

		boolToYesNo := map[bool]string{
			true:  "YES",
			false: "NOPE",
		}

		gocv.PutText(&img,
			fmt.Sprintf("Battery: %3d%%", fd.BatteryPercentage),
			image.Point{
				X: 10,
				Y: 20,
			},
			gocv.FontHersheyPlain,
			1.5,
			color.RGBA{
				R: 0,
				G: 255,
				B: 0,
				A: 0,
			},
			2,
		)

		gocv.PutText(&img,
			fmt.Sprintf("Temperature: %s", boolToStatus[!fd.TemperatureHigh]),
			image.Point{
				X: 10,
				Y: 40,
			},
			gocv.FontHersheyPlain,
			1.5,
			color.RGBA{
				R: 0,
				G: 255,
				B: 0,
				A: 0,
			},
			2,
		)

		gocv.PutText(&img,
			fmt.Sprintf("Pressure: %s", boolToStatus[!fd.PressureState]),
			image.Point{
				X: 10,
				Y: 60,
			},
			gocv.FontHersheyPlain,
			1.5,
			color.RGBA{
				R: 0,
				G: 255,
				B: 0,
				A: 0,
			},
			2,
		)

		gocv.PutText(&img,
			fmt.Sprintf("Ground speed: %.2fm/s", fd.GroundSpeed()),
			image.Point{
				X: 10,
				Y: 80,
			},
			gocv.FontHersheyPlain,
			1.5,
			color.RGBA{
				R: 0,
				G: 255,
				B: 0,
				A: 0,
			},
			2,
		)

		gocv.PutText(&img,
			fmt.Sprintf("Air speed: %.2fm/s", fd.AirSpeed()),
			image.Point{
				X: 10,
				Y: 100,
			},
			gocv.FontHersheyPlain,
			1.5,
			color.RGBA{
				R: 0,
				G: 255,
				B: 0,
				A: 0,
			},
			2,
		)

		gocv.PutText(&img,
			fmt.Sprintf("Height: %.2f", float32(fd.Height)/10.0),
			image.Point{
				X: 10,
				Y: 120,
			},
			gocv.FontHersheyPlain,
			1.5,
			color.RGBA{
				R: 0,
				G: 255,
				B: 0,
				A: 0,
			},
			2,
		)

		gocv.PutText(&img,
			fmt.Sprintf("Flying: %s", boolToYesNo[fd.Flying]),
			image.Point{
				X: 10,
				Y: 140,
			},
			gocv.FontHersheyPlain,
			1.5,
			color.RGBA{
				R: 0,
				G: 255,
				B: 0,
				A: 0,
			},
			2,
		)

		gocv.PutText(&img,
			fmt.Sprintf("On ground: %s", boolToYesNo[fd.OnGround]),
			image.Point{
				X: 10,
				Y: 160,
			},
			gocv.FontHersheyPlain,
			1.5,
			color.RGBA{
				R: 0,
				G: 255,
				B: 0,
				A: 0,
			},
			2,
		)

		gocv.PutText(&img,
			fmt.Sprintf("Hover: %s", boolToYesNo[fd.DroneHover]),
			image.Point{
				X: 10,
				Y: 180,
			},
			gocv.FontHersheyPlain,
			1.5,
			color.RGBA{
				R: 0,
				G: 255,
				B: 0,
				A: 0,
			},
			2,
		)

		gocv.PutText(&img,
			time.Now().Format(time.RFC1123),
			image.Point{
				X: 10,
				Y: frameY - 20,
			},
			gocv.FontHersheyPlain,
			1.5,
			color.RGBA{
				R: 0,
				G: 0,
				B: 255,
				A: 0,
			},
			2,
		)

		// Face detection
		// Enable that if you have a master race computer to perform image detection
		// because it is slow as shit and will introduce a 20+ seconds latency in the
		// feedback
		/*
			rects := classifier.DetectMultiScale(img)
			for _, r := range rects {
				gocv.Rectangle(&img, r, color.RGBA{
					R: 0,
					G: 0,
					B: 255,
					A: 0,
				}, 3)

				size := gocv.GetTextSize("Human", gocv.FontHersheyPlain, 1.2, 2)
				pt := image.Pt(r.Min.X+(r.Min.X/2)-(size.X/2), r.Min.Y-2)
				gocv.PutText(&img, "Human", pt, gocv.FontHersheyPlain, 1.2, color.RGBA{
					R: 0,
					G: 0,
					B: 255,
					A: 0,
				}, 2)
			}*/

		output.Write(img)

		jpg, err := gocv.IMEncode(gocv.JPEGFileExt, img)
		if err != nil {
			logrus.WithError(err).Error("Could not encode image to jpeg")
			continue
		}
		stream.Update(jpg)
		if err != nil {
			logrus.WithError(err).Error("Could not update stream")
			continue
		}
	}
}

func getLeftStick() stickPair {
	s := stickPair{x: 0, y: 0}
	s.x = leftX.Load().(float64)
	s.y = leftY.Load().(float64)
	return s
}

func getRightStick() stickPair {
	s := stickPair{x: 0, y: 0}
	s.x = rightX.Load().(float64)
	s.y = rightY.Load().(float64)
	return s
}
