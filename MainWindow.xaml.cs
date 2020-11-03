namespace MaxHenkel.Kinect.GreenScreen
{
    using System.ComponentModel;
    using System.Windows;
    using System.Windows.Media;
    using System.Windows.Media.Imaging;
    using Microsoft.Kinect;

    /// <summary>
    /// Interaction logic for MainWindow
    /// </summary>
    public partial class MainWindow : INotifyPropertyChanged
    {
        /// <summary>
        /// Size of the RGB pixel in the bitmap
        /// </summary>
        private readonly int bytesPerPixel = (PixelFormats.Bgr32.BitsPerPixel + 7) / 8;

        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor kinectSensor;

        /// <summary>
        /// Coordinate mapper to map one type of point to another
        /// </summary>
        private CoordinateMapper coordinateMapper;

        /// <summary>
        /// Reader for depth/color/body index frames
        /// </summary>
        private MultiSourceFrameReader multiFrameSourceReader;

        /// <summary>
        /// Bitmap to display
        /// </summary>
        private WriteableBitmap bitmap;

        /// <summary>
        /// The size in bytes of the bitmap back buffer
        /// </summary>
        private uint bitmapBackBufferSize;

        /// <summary>
        /// Intermediate storage for the color to depth mapping
        /// </summary>
        private DepthSpacePoint[] colorMappedToDepthPoints;

        /// <summary>
        /// Current status text to display
        /// </summary>
        private string statusText = null;

        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {
            kinectSensor = KinectSensor.GetDefault();

            multiFrameSourceReader = kinectSensor.OpenMultiSourceFrameReader(FrameSourceTypes.Depth | FrameSourceTypes.Color | FrameSourceTypes.BodyIndex);

            multiFrameSourceReader.MultiSourceFrameArrived += Reader_MultiSourceFrameArrived;

            coordinateMapper = kinectSensor.CoordinateMapper;

            FrameDescription colorFrameDescription = kinectSensor.ColorFrameSource.FrameDescription;

            int colorWidth = colorFrameDescription.Width;
            int colorHeight = colorFrameDescription.Height;

            colorMappedToDepthPoints = new DepthSpacePoint[colorWidth * colorHeight];

            bitmap = new WriteableBitmap(colorWidth, colorHeight, 96.0, 96.0, PixelFormats.Bgra32, null);

            // Calculate the WriteableBitmap back buffer size
            bitmapBackBufferSize = (uint) (bitmap.BackBufferStride * (bitmap.PixelHeight - 1) + bitmap.PixelWidth * bytesPerPixel);

            kinectSensor.Open();

            DataContext = this;

            InitializeComponent();
        }

        /// <summary>
        /// INotifyPropertyChangedPropertyChanged event to allow window controls to bind to changeable data
        /// </summary>
        public event PropertyChangedEventHandler PropertyChanged;

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void WindowClose(object sender, CancelEventArgs e)
        {
            if (multiFrameSourceReader != null)
            {
                // MultiSourceFrameReder is IDisposable
                multiFrameSourceReader.Dispose();
                multiFrameSourceReader = null;
            }

            if (kinectSensor != null)
            {
                kinectSensor.Close();
                kinectSensor = null;
            }
        }

        /// <summary>
        /// Handles the depth/color/body index frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Reader_MultiSourceFrameArrived(object sender, MultiSourceFrameArrivedEventArgs e)
        {
            int depthWidth;
            int depthHeight;

            DepthFrame depthFrame = null;
            ColorFrame colorFrame = null;
            BodyIndexFrame bodyIndexFrame = null;
            bool isBitmapLocked = false;

            MultiSourceFrame multiSourceFrame = e.FrameReference.AcquireFrame();

            // If the Frame has expired by the time we process this event, return.
            if (multiSourceFrame == null)
            {
                return;
            }

            // We use a try/finally to ensure that we clean up before we exit the function.  
            // This includes calling Dispose on any Frame objects that we may have and unlocking the bitmap back buffer.
            try
            {
                depthFrame = multiSourceFrame.DepthFrameReference.AcquireFrame();
                colorFrame = multiSourceFrame.ColorFrameReference.AcquireFrame();
                bodyIndexFrame = multiSourceFrame.BodyIndexFrameReference.AcquireFrame();

                // If any frame has expired by the time we process this event, return.
                // The "finally" statement will Dispose any that are not null.
                if (depthFrame == null || colorFrame == null || bodyIndexFrame == null)
                {
                    return;
                }

                // Process Depth
                FrameDescription depthFrameDescription = depthFrame.FrameDescription;

                depthWidth = depthFrameDescription.Width;
                depthHeight = depthFrameDescription.Height;

                // Access the depth frame data directly via LockImageBuffer to avoid making a copy
                using (KinectBuffer depthFrameData = depthFrame.LockImageBuffer())
                {
                    coordinateMapper.MapColorFrameToDepthSpaceUsingIntPtr(depthFrameData.UnderlyingBuffer, depthFrameData.Size, colorMappedToDepthPoints);
                }

                // We're done with the DepthFrame 
                depthFrame.Dispose();
                depthFrame = null;

                // Process Color


                // Lock the bitmap for writing
                bitmap.Lock();
                isBitmapLocked = true;

                colorFrame.CopyConvertedFrameDataToIntPtr(bitmap.BackBuffer, bitmapBackBufferSize, ColorImageFormat.Bgra);

                // We're done with the ColorFrame 
                colorFrame.Dispose();
                colorFrame = null;

                // We'll access the body index data directly to avoid a copy
                using (KinectBuffer bodyIndexData = bodyIndexFrame.LockImageBuffer())
                {
                    unsafe
                    {
                        byte* bodyIndexDataPointer = (byte*) bodyIndexData.UnderlyingBuffer;

                        int colorMappedToDepthPointCount = colorMappedToDepthPoints.Length;

                        fixed (DepthSpacePoint* colorMappedToDepthPointsPointer = colorMappedToDepthPoints)
                        {
                            // Treat the color data as 4-byte pixels
                            uint* bitmapPixelsPointer = (uint*) bitmap.BackBuffer;

                            // Loop over each row and column of the color image
                            // Zero out any pixels that don't correspond to a body index
                            for (int colorIndex = 0; colorIndex < colorMappedToDepthPointCount; ++colorIndex)
                            {
                                float colorMappedToDepthX = colorMappedToDepthPointsPointer[colorIndex].X;
                                float colorMappedToDepthY = colorMappedToDepthPointsPointer[colorIndex].Y;

                                // The sentinel value is -inf, -inf, meaning that no depth pixel corresponds to this color pixel.
                                if (!float.IsNegativeInfinity(colorMappedToDepthX) && !float.IsNegativeInfinity(colorMappedToDepthY))
                                {
                                    // Make sure the depth pixel maps to a valid point in color space
                                    int depthX = (int) (colorMappedToDepthX + 0.5F);
                                    int depthY = (int) (colorMappedToDepthY + 0.5F);

                                    // If the point is not valid, there is no body index there.
                                    if (depthX >= 0 && depthX < depthWidth && depthY >= 0 && depthY < depthHeight)
                                    {
                                        int depthIndex = depthY * depthWidth + depthX;

                                        // If we are tracking a body for the current pixel, do not zero out the pixel
                                        if (bodyIndexDataPointer[depthIndex] != 0xFF)
                                        {
                                            continue;
                                        }
                                    }
                                }

                                bitmapPixelsPointer[colorIndex] = 0;
                            }
                        }

                        bitmap.AddDirtyRect(new Int32Rect(0, 0, bitmap.PixelWidth, bitmap.PixelHeight));
                    }
                }
            }
            finally
            {
                if (isBitmapLocked)
                {
                    bitmap.Unlock();
                }

                depthFrame?.Dispose();

                colorFrame?.Dispose();

                bodyIndexFrame?.Dispose();
            }
        }
    }
}