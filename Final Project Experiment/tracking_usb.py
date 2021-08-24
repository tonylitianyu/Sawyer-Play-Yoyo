from goprocam import GoProCamera, constants
gopro = GoProCamera.GoPro(ip_address="172.28.128.51") # change
gopro.startWebcam(constants.Webcam.Resolution.R720p)
gopro.webcamFOV(constants.Webcam.FOV.Linear)
gopro.getWebcamPreview()