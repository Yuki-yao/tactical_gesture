//------------------------------------------------------------------------------
// <copyright file="MainWindow.xaml.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

namespace Microsoft.Samples.Kinect.BodyBasics
{
    using System;
    using System.Collections.Generic;
    using System.ComponentModel;
    using System.Diagnostics;
    using System.Globalization;
    using System.IO;
    using System.Windows;
    using System.Windows.Media;
    using System.Windows.Media.Media3D;
    using System.Windows.Media.Imaging;
    using System.Speech.Synthesis;
    using Microsoft.Kinect;

    /// <summary>
    /// Interaction logic for MainWindow
    /// </summary>
    public partial class MainWindow : Window, INotifyPropertyChanged
    {
        private readonly SpeechSynthesizer speech = null;
        /// <summary>
        /// Radius of drawn hand circles
        /// </summary>
        private const double HandSize = 30;

        /// <summary>
        /// Thickness of drawn joint lines
        /// </summary>
        private const double JointThickness = 3;

        /// <summary>
        /// Thickness of clip edge rectangles
        /// </summary>
        private const double ClipBoundsThickness = 10;

        /// <summary>
        /// Constant for clamping Z values of camera space points from being negative
        /// </summary>
        private const float InferredZPositionClamp = 0.1f;

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as closed
        /// </summary>
        private readonly Brush handClosedBrush = new SolidColorBrush(Color.FromArgb(128, 255, 0, 0));

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as opened
        /// </summary>
        private readonly Brush handOpenBrush = new SolidColorBrush(Color.FromArgb(128, 0, 255, 0));

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as in lasso (pointer) position
        /// </summary>
        private readonly Brush handLassoBrush = new SolidColorBrush(Color.FromArgb(128, 0, 0, 255));

        /// <summary>
        /// Brush used for drawing joints that are currently tracked
        /// </summary>
        private readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 68, 192, 68));

        /// <summary>
        /// Brush used for drawing joints that are currently inferred
        /// </summary>        
        private readonly Brush inferredJointBrush = Brushes.Yellow;

        /// <summary>
        /// Pen used for drawing bones that are currently inferred
        /// </summary>        
        private readonly Pen inferredBonePen = new Pen(Brushes.Gray, 1);

        /// <summary>
        /// Drawing group for body rendering output
        /// </summary>
        private DrawingGroup drawingGroup;

        /// <summary>
        /// Drawing image that we will display
        /// </summary>
        private DrawingImage imageSource;

        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor kinectSensor = null;

        /// <summary>
        /// Coordinate mapper to map one type of point to another
        /// </summary>
        private CoordinateMapper coordinateMapper = null;

        /// <summary>
        /// Reader for body frames
        /// </summary>
        private BodyFrameReader bodyFrameReader = null;

        /// <summary>
        /// Array for the bodies
        /// </summary>
        private Body[] bodies = null;

        /// <summary>
        /// definition of bones
        /// </summary>
        private List<Tuple<JointType, JointType>> bones;

        /// <summary>
        /// Width of display (depth space)
        /// </summary>
        private int displayWidth;

        /// <summary>
        /// Height of display (depth space)
        /// </summary>
        private int displayHeight;

        /// <summary>
        /// List of colors for each body tracked
        /// </summary>
        private List<Pen> bodyColors;

        /// <summary>
        /// Current status text to display
        /// </summary>
        private string statusText = null;

        private void speak(String s)
        {
            //if(speech.State == SynthesizerState.Ready)
                speech.SpeakAsync(s);
        }

        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {
            speech = new SpeechSynthesizer();
            speech.Rate = 5;
            speech.Volume = 100;
            //for(int i=0;i<10;i++) speak("喵");
            // one sensor is currently supported
            this.kinectSensor = KinectSensor.GetDefault();

            // get the coordinate mapper
            this.coordinateMapper = this.kinectSensor.CoordinateMapper;

            // get the depth (display) extents
            FrameDescription frameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;

            // get size of joint space
            this.displayWidth = frameDescription.Width;
            this.displayHeight = frameDescription.Height;

            // open the reader for the body frames
            this.bodyFrameReader = this.kinectSensor.BodyFrameSource.OpenReader();

            // a bone defined as a line between two joints
            this.bones = new List<Tuple<JointType, JointType>>();

            // Torso
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Head, JointType.Neck));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Neck, JointType.SpineShoulder));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.SpineMid));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineMid, JointType.SpineBase));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipLeft));

            // Right Arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderRight, JointType.ElbowRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowRight, JointType.WristRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.HandRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandRight, JointType.HandTipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.ThumbRight));

            // Left Arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderLeft, JointType.ElbowLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowLeft, JointType.WristLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.HandLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandLeft, JointType.HandTipLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.ThumbLeft));

            // Right Leg
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipRight, JointType.KneeRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeRight, JointType.AnkleRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleRight, JointType.FootRight));

            // Left Leg
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipLeft, JointType.KneeLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeLeft, JointType.AnkleLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleLeft, JointType.FootLeft));

            // populate body colors, one for each BodyIndex
            this.bodyColors = new List<Pen>();

            this.bodyColors.Add(new Pen(Brushes.Red, 6));
            this.bodyColors.Add(new Pen(Brushes.Orange, 6));
            this.bodyColors.Add(new Pen(Brushes.Green, 6));
            this.bodyColors.Add(new Pen(Brushes.Blue, 6));
            this.bodyColors.Add(new Pen(Brushes.Indigo, 6));
            this.bodyColors.Add(new Pen(Brushes.Violet, 6));

            // set IsAvailableChanged event notifier
            this.kinectSensor.IsAvailableChanged += this.Sensor_IsAvailableChanged;

            // open the sensor
            this.kinectSensor.Open();

            // set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.NoSensorStatusText;

            // Create the drawing group we'll use for drawing
            this.drawingGroup = new DrawingGroup();

            // Create an image source that we can use in our image control
            this.imageSource = new DrawingImage(this.drawingGroup);

            // use the window object as the view model in this simple example
            this.DataContext = this;

            // initialize the components (controls) of the window
            this.InitializeComponent();
        }

        /// <summary>
        /// INotifyPropertyChangedPropertyChanged event to allow window controls to bind to changeable data
        /// </summary>
        public event PropertyChangedEventHandler PropertyChanged;

        /// <summary>
        /// Gets the bitmap to display
        /// </summary>
        public ImageSource ImageSource
        {
            get
            {
                return this.imageSource;
            }
        }

        /// <summary>
        /// Gets or sets the current status text to display
        /// </summary>
        public string StatusText
        {
            get
            {
                return this.statusText;
            }

            set
            {
                if (this.statusText != value)
                {
                    this.statusText = value;

                    // notify any bound elements that the text has changed
                    if (this.PropertyChanged != null)
                    {
                        this.PropertyChanged(this, new PropertyChangedEventArgs("StatusText"));
                    }
                }
            }
        }

        /// <summary>
        /// Execute start up tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {
            if (this.bodyFrameReader != null)
            {
                this.bodyFrameReader.FrameArrived += this.Reader_FrameArrived;
            }
        }

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            if (this.bodyFrameReader != null)
            {
                // BodyFrameReader is IDisposable
                this.bodyFrameReader.Dispose();
                this.bodyFrameReader = null;
            }

            if (this.kinectSensor != null)
            {
                this.kinectSensor.Close();
                this.kinectSensor = null;
            }
        }
        private double stdLen(Dictionary<JointType, Point3D> stdJoints)
        {
            return (stdJoints[JointType.Neck] - stdJoints[JointType.SpineBase]).Length;
        }
        private bool dontKnow(Dictionary<JointType, Point3D> stdJoints)
        {
            Vector3D lrhand = stdJoints[JointType.HandLeft] - stdJoints[JointType.HandRight];
            Vector3D lrelbow = stdJoints[JointType.ElbowLeft] - stdJoints[JointType.ElbowRight];
            double elbowh = (stdJoints[JointType.ElbowLeft].Y + stdJoints[JointType.ElbowRight].Y) / 2;
            double handh = (stdJoints[JointType.HandLeft].Y + stdJoints[JointType.HandRight].Y) / 2;
            double stdl=stdLen(stdJoints);
            if (lrhand.Length < stdl)
                return false;
            if (Math.Abs(lrhand.Y) > stdl * 0.3)
                return false;
            if (Math.Abs(lrelbow.Y) > stdl * 0.25)
                return false;
            if (handh < elbowh)
                return false;
            if (stdJoints[JointType.SpineShoulder].Y * 0.8 + stdJoints[JointType.SpineBase].Y * 0.2 < elbowh)
                return false;
            return true;
        }
        private bool yanhu(Dictionary<JointType, Point3D> stdJoints)
        {
            Point3D leftHand = stdJoints[JointType.HandLeft];
            Point3D leftElbow = stdJoints[JointType.ElbowLeft];
            Point3D head = stdJoints[JointType.Head];
            Point3D leftShoulder = stdJoints[JointType.ShoulderLeft];
            double stdl = stdLen(stdJoints);
            if (Math.Abs(leftHand.X - head.X) > 0.2 * stdl)
                return false;
            if (leftHand.Y - head.Y < 0 || leftHand.Y - head.Y > 0.5 * stdl)
                return false;
            if (leftShoulder.X - leftElbow.X < 0.0 * stdl)
                return false;
            if (leftElbow.Y > leftHand.Y || leftElbow.Y < leftShoulder.Y)
                return false;
            return true;
        }
        private bool shang(Dictionary<JointType, Point3D> stdJoints, Body body)
        {
            if (body.HandLeftState != HandState.Open/* || body.HandLeftConfidence != TrackingConfidence.High*/)
                return false;
            if (stdJoints[JointType.HandRight].Y > stdJoints[JointType.ElbowRight].Y)
                return false;
            if (stdJoints[JointType.HandLeft].Y < stdJoints[JointType.ShoulderLeft].Y)
                return false;
            if (stdJoints[JointType.HandLeft].X > stdJoints[JointType.ShoulderLeft].X)
                return false;
            return true;
        }
        private bool mingbai(Dictionary<JointType, Point3D> stdJoints, Body body)
        {
            if (body.HandLeftState != HandState.Closed/* || body.HandLeftConfidence != TrackingConfidence.High*/)
                return false;
            if (stdJoints[JointType.HandRight].Y > stdJoints[JointType.ElbowRight].Y)
                return false;
            if (stdJoints[JointType.HandLeft].Y < stdJoints[JointType.Head].Y)
                return false;
            if (stdJoints[JointType.HandLeft].X > stdJoints[JointType.ShoulderLeft].X)
                return false;
            return true;
        }
        private bool xiandanup(Dictionary<JointType, Point3D> stdJoints, Body body)
        {
            if (body.HandLeftState != HandState.Closed/* || body.HandLeftConfidence != TrackingConfidence.High*/)
                return false;
            if (stdJoints[JointType.HandRight].Y > stdJoints[JointType.ElbowRight].Y)
                return false;
            if (stdJoints[JointType.HandLeft].Y > stdJoints[JointType.Head].Y)
                return false;
            if (stdJoints[JointType.HandLeft].X > stdJoints[JointType.ShoulderLeft].X)
                return false;
            if (stdJoints[JointType.HandLeft].Y < stdJoints[JointType.ElbowLeft].Y + stdLen(stdJoints) * 0.05)
                return false;
            return true;
        }
        private bool xiandandown(Dictionary<JointType, Point3D> stdJoints, Body body)
        {
            if (body.HandLeftState != HandState.Closed/* || body.HandLeftConfidence != TrackingConfidence.High*/)
                return false;
            if (stdJoints[JointType.HandRight].Y > stdJoints[JointType.ElbowRight].Y)
                return false;
            if (stdJoints[JointType.HandLeft].Y > stdJoints[JointType.Head].Y)
                return false;
            if (stdJoints[JointType.HandLeft].X > stdJoints[JointType.ShoulderLeft].X)
                return false;
            if (stdJoints[JointType.HandLeft].Y > stdJoints[JointType.ElbowLeft].Y - stdLen(stdJoints) * 0.05)
                return false;
            return true;
        }
        private bool nanxing(Dictionary<JointType, Point3D> stdJoints)
        {
            double stdl = stdLen(stdJoints);
            Point3D leftWrist = stdJoints[JointType.WristLeft];
            Point3D leftElbow = stdJoints[JointType.ElbowLeft];
            Point3D spineShoulder = stdJoints[JointType.SpineShoulder];
            Point3D neck = stdJoints[JointType.Neck];
            //Console.WriteLine((leftWrist - neck).Length / stdl);
            if ((leftWrist - neck).Length > 0.35 * stdl)
                return false;
            Console.WriteLine((spineShoulder.Y - leftElbow.Y) / stdl);
            if (spineShoulder.Y - leftElbow.Y < 0.1 * stdl)
                return false;
            return true;
        }
        private bool nvxing(Dictionary<JointType, Point3D> stdJoints)
        {
            if (stdJoints[JointType.HandLeft].Y < stdJoints[JointType.ElbowLeft].Y)
                return false;
            if (stdJoints[JointType.HandLeft].Y > stdJoints[JointType.SpineShoulder].Y)
                return false;
            if (stdJoints[JointType.ShoulderLeft].Y < stdJoints[JointType.ElbowLeft].Y)
                return false;
            if (stdJoints[JointType.HandLeft].X < stdJoints[JointType.ShoulderLeft].X)
                return false;
            if (stdJoints[JointType.HandLeft].X > stdJoints[JointType.SpineShoulder].X)
                return false;
            if (stdJoints[JointType.HandLeft].Y < stdJoints[JointType.SpineBase].Y)
                return false;
            if (stdJoints[JointType.HandRight].Y > stdJoints[JointType.ElbowRight].Y)
                return false;
            return true;
        }
        private bool zhihuiguan(Dictionary<JointType, Point3D> stdJoints)
        {
            double stdl = stdLen(stdJoints);
            Point3D leftHand = stdJoints[JointType.HandLeft];
            Point3D rightShoulder = stdJoints[JointType.ShoulderRight];
            Point3D rightElbow = stdJoints[JointType.ElbowRight];
            Point3D rightHand = stdJoints[JointType.HandRight];
            if ((rightShoulder - rightHand).Length < 0.8 * stdl)
                return false;
            if (Math.Abs(leftHand.X - rightElbow.X) > 0.2 * stdl)
                return false;
            if (leftHand.Y > rightShoulder.Y || leftHand.Y < rightElbow.Y)
                return false;
            return true;
        }
        private bool diren(Dictionary<JointType, Point3D> stdJoints)
        {
            double stdl = stdLen(stdJoints);
            Point3D leftHand = stdJoints[JointType.HandLeft];
            Point3D rightShoulder = stdJoints[JointType.ShoulderRight];
            Point3D rightElbow = stdJoints[JointType.ElbowRight];
            Point3D rightHand = stdJoints[JointType.HandRight];
            //Console.WriteLine((rightShoulder - rightHand).Length / stdl);
            if ((rightShoulder - rightHand).Length < 0.8 * stdl)
                return false;
            if (Math.Abs(leftHand.X - rightElbow.X) > 0.2 * stdl)
                return false;
            if (leftHand.Y > rightElbow.Y || leftHand.Y < rightHand.Y)
                return false;
            return true;
        }
        private bool zhuyiting(Dictionary<JointType, Point3D> stdJoints)
        {
            double stdl = stdLen(stdJoints);
            Point3D leftHand = stdJoints[JointType.HandLeft];
            Point3D spineShoulder = stdJoints[JointType.SpineShoulder];
            Point3D leftElbow = stdJoints[JointType.ElbowLeft];
            Point3D mid = stdJoints[JointType.Neck] + (stdJoints[JointType.Head] - stdJoints[JointType.Neck]) * 0.5;
            if ((leftHand - mid).Length > 0.2 * stdl)
                return false;
            if (spineShoulder.Y < leftElbow.Y - 0.2 * stdl)
                return false;
            return true;
        }
        private bool xialai(Dictionary<JointType, Point3D> stdJoints, Body body)
        {
            double stdl = stdLen(stdJoints);
            Point3D leftWrist = stdJoints[JointType.WristLeft];
            Point3D leftElbow = stdJoints[JointType.ElbowLeft];
            Point3D leftShoulder = stdJoints[JointType.ShoulderLeft];
            Point3D spineBase = stdJoints[JointType.SpineBase];
            if (leftElbow.X < leftWrist.X || leftElbow.X > leftShoulder.X)
                return false;
            if (leftElbow.Y < leftWrist.Y || leftElbow.Y > leftShoulder.Y)
                return false;
            if (Math.Abs(spineBase.Y - leftWrist.Y) > 0.3 * stdl)
                return false;
            if (spineBase.X - leftWrist.X < 0.5 * stdl)
                return false;
            if (body.HandLeftState == HandState.Closed)
                return false;
            return true;
        }
        int TOTAL_N = 15;
        Dictionary<int, List<double>> count = new Dictionary<int, List<double>>();
        string[] word={"","不明白","掩护","前进","明白","男性","女性","指挥官","敌人","Warning","下来","None","霰弹枪", "霰弹枪"};
        //string[] word={"","bumingbai","yanhu","qianjin","mingbai","nanxing","nyuxing","zhihuiguan","diren","zhuyiting","xialai"};

        double[] las = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
        private bool check(List<double> x, List<double> y)
        {
            int tot=0;
            for (int i = 0; i < x.Count; ++i)
                for (int j = 0; j < y.Count; ++j)
                    if (x[i] < y[j])
                        ++tot;
            if (x.Count * y.Count * 0.25 < tot && tot < x.Count * y.Count * 0.75)
                return true;
            return false;
        }
        private void term(int type)//type=0 is regist No. from 1~N
        {

            if (count.Count != TOTAL_N + 1)
            {
                for (int i = 0; i <= TOTAL_N; ++i)
                    count[i]=new List<double>();
            }
            double a = Environment.TickCount;
            //Console.WriteLine(a);
            for (int i = count[0].Count - 1; i >= 0; --i)
                if (a - count[0][i] > 1000)//1s
                {
                    count[0].RemoveRange(0, i + 1);
                    break;
                }
            for (int i = count[type].Count - 1; i >= 0; --i)
                if (a - count[type][i] > 1000)//1s
                {
                    count[type].RemoveRange(0, i + 1);
                    break;
                }
            count[type].Add(a);
            //if(type!=0)
            {
                Console.Write(count[type].Count);
                Console.Write("\t");
                Console.WriteLine(count[0].Count);
            }
            if (count[0].Count < 20)
                return;
            if (type > 0 && type <=10 && count[type].Count * 2.2 > count[0].Count && a - las[type] > 2000)
            {
                for (int i = las.Length - 1; i >= 0; --i)
                    las[i] = 0;
                las[type] = a;
                count[type].Clear();
                //speak("");
                speak(word[type]);
                Console.WriteLine(word[type]);
            }
            if (type > 10)
            {
                if ((count[type].Count + count[type ^ 1].Count) * 2.2 > count[0].Count &&
                    100 * count[type].Count / (count[type].Count + count[type ^ 1].Count) > 25 &&
                    100 * count[type].Count / (count[type].Count + count[type ^ 1].Count) < 75 &&
                    check(count[type],count[type^1]) &&
                    a - las[type] > 5000)
                {
                    for (int i = las.Length - 1; i >= 0; --i)
                        las[i] = 0;
                    las[type] = las[type & 1] = a;
                    count[type].Clear();
                    count[type^1].Clear();
                    speak(word[type]);
                    Console.WriteLine(word[type]);
                }
            }
        }
        /// <summary>
        /// Handles the body frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Reader_FrameArrived(object sender, BodyFrameArrivedEventArgs e)
        {
            bool dataReceived = false;

            using (BodyFrame bodyFrame = e.FrameReference.AcquireFrame())
            {
                if (bodyFrame != null)
                {
                    if (this.bodies == null)
                    {
                        this.bodies = new Body[bodyFrame.BodyCount];
                    }

                    // The first time GetAndRefreshBodyData is called, Kinect will allocate each Body in the array.
                    // As long as those body objects are not disposed and not set to null in the array,
                    // those body objects will be re-used.
                    bodyFrame.GetAndRefreshBodyData(this.bodies);
                    dataReceived = true;
                }
            }

            if (dataReceived)
            {
                using (DrawingContext dc = this.drawingGroup.Open())
                {
                    // Draw a transparent background to set the render size
                    dc.DrawRectangle(Brushes.Black, null, new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));

                    int penIndex = 0;
                    term(0);
                    foreach (Body body in this.bodies)
                    {
                        Pen drawPen = this.bodyColors[penIndex++];

                        if (body.IsTracked)
                        {
                            this.DrawClippedEdges(body, dc);
                            

                            IReadOnlyDictionary<JointType, Joint> rawJoints = body.Joints;
                            
                            // begin saffah

                            Dictionary<JointType, Point3D> stdJoints = new Dictionary<JointType, Point3D>();
                            foreach (JointType type in rawJoints.Keys)
                                stdJoints[type] = new Point3D(rawJoints[type].Position.X, rawJoints[type].Position.Y, rawJoints[type].Position.Z);
                            Vector3D dif = stdJoints[JointType.SpineBase] - stdJoints[JointType.Neck];
                            dif.Z = 0;
                            double dv = dif.Length;
                            //Console.WriteLine(dif);
                            //Console.WriteLine(dv);
                            foreach (JointType type in rawJoints.Keys)
                                stdJoints[type] = new Point3D(stdJoints[type].X / dv, stdJoints[type].Y / dv, stdJoints[type].Z / dv);

                            Dictionary<JointType, Joint> joints = new Dictionary<JointType, Joint>();
                            foreach (JointType jointType in rawJoints.Keys)
                            {
                                Joint newJoint = new Joint();
                                newJoint.JointType = rawJoints[jointType].JointType;
                                newJoint.Position = new CameraSpacePoint();
                                newJoint.Position.X = (float) stdJoints[jointType].X;
                                newJoint.Position.Y = (float) stdJoints[jointType].Y;
                                newJoint.Position.Z = (float) stdJoints[jointType].Z;
                                newJoint.TrackingState = rawJoints[jointType].TrackingState;
                                joints.Add(jointType, newJoint);
                            }
                            // end saffah
                            //Console.WriteLine("Time");
                            //Console.WriteLine(DateTime.Now);
                            if (dontKnow(stdJoints))
                            {
                                term(1);
                            }
                            if (yanhu(stdJoints))
                            {
                                term(2);
                            }
                            if (shang(stdJoints, body))
                            {
                                term(3);
                            }
                            if (mingbai(stdJoints, body))
                            {
                                term(4);
                            }
                            if (nanxing(stdJoints))
                            {
                                term(5);
                            }
                            if (nvxing(stdJoints))
                            {
                                term(6);
                            }
                            if (zhihuiguan(stdJoints))
                            {
                                term(7);
                            }
                            if (diren(stdJoints))
                            {
                                term(8);
                            }
                            /*if (zhuyiting(stdJoints))
                            {
                                term(9);
                            }*/
                            if (xiandanup(stdJoints, body))
                            {
                                term(12);
                                Console.WriteLine("***************");
                            }
                            if (xiandandown(stdJoints, body))
                            {
                                term(13);
                                Console.WriteLine("###############");
                            }
                            if (xialai(stdJoints, body))
                            {
                                term(10);
                            }
                            // convert the joint points to depth (display) space
                            Dictionary<JointType, Point> jointPoints = new Dictionary<JointType, Point>();

                            foreach (JointType jointType in joints.Keys)
                            {
                                // sometimes the depth(Z) of an inferred joint may show as negative
                                // clamp down to 0.1f to prevent coordinatemapper from returning (-Infinity, -Infinity)
                                CameraSpacePoint position = joints[jointType].Position;
                                if (position.Z < 0)
                                {
                                    position.Z = InferredZPositionClamp / (float) dv;
                                }

                                DepthSpacePoint depthSpacePoint = this.coordinateMapper.MapCameraPointToDepthSpace(position);
                                jointPoints[jointType] = new Point(depthSpacePoint.X, depthSpacePoint.Y);
                            }

                            this.DrawBody(joints, jointPoints, dc, drawPen);

                            this.DrawHand(body.HandLeftState, jointPoints[JointType.HandLeft], dc, joints[JointType.HandLeft].Position);
                            this.DrawHand(body.HandRightState, jointPoints[JointType.HandRight], dc, joints[JointType.HandRight].Position);
                        }
                    }

                    // prevent drawing outside of our render area
                    this.drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));
                }
            }
        }

        /// <summary>
        /// Draws a body
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// <param name="drawingPen">specifies color to draw a specific body</param>
        private void DrawBody(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, DrawingContext drawingContext, Pen drawingPen)
        {
            // Draw the bones
            foreach (var bone in this.bones)
            {
                this.DrawBone(joints, jointPoints, bone.Item1, bone.Item2, drawingContext, drawingPen);
            }

            // Draw the joints
            foreach (JointType jointType in joints.Keys)
            {
                Brush drawBrush = null;

                TrackingState trackingState = joints[jointType].TrackingState;

                if (trackingState == TrackingState.Tracked)
                {
                    drawBrush = this.trackedJointBrush;
                }
                else if (trackingState == TrackingState.Inferred)
                {
                    drawBrush = this.inferredJointBrush;
                }

                if (drawBrush != null)
                {
                    drawingContext.DrawEllipse(drawBrush, null, jointPoints[jointType], JointThickness, JointThickness);
                }
            }
        }

        /// <summary>
        /// Draws one bone of a body (joint to joint)
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="jointType0">first joint of bone to draw</param>
        /// <param name="jointType1">second joint of bone to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// /// <param name="drawingPen">specifies color to draw a specific bone</param>
        private void DrawBone(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, JointType jointType0, JointType jointType1, DrawingContext drawingContext, Pen drawingPen)
        {
            Joint joint0 = joints[jointType0];
            Joint joint1 = joints[jointType1];

            // If we can't find either of these joints, exit
            if (joint0.TrackingState == TrackingState.NotTracked ||
                joint1.TrackingState == TrackingState.NotTracked)
            {
                return;
            }

            // We assume all drawn bones are inferred unless BOTH joints are tracked
            Pen drawPen = this.inferredBonePen;
            if ((joint0.TrackingState == TrackingState.Tracked) && (joint1.TrackingState == TrackingState.Tracked))
            {
                drawPen = drawingPen;
            }

            drawingContext.DrawLine(drawPen, jointPoints[jointType0], jointPoints[jointType1]);
        }

        /// <summary>
        /// Draws a hand symbol if the hand is tracked: red circle = closed, green circle = opened; blue circle = lasso
        /// </summary>
        /// <param name="handState">state of the hand</param>
        /// <param name="handPosition">position of the hand</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawHand(HandState handState, Point handPosition, DrawingContext drawingContext, CameraSpacePoint csp)
        {
            //drawingContext.DrawText(new FormattedText(csp.X.ToString() + "," + csp.Y.ToString() + "," + csp.Z.ToString(), CultureInfo.CurrentCulture, FlowDirection.LeftToRight, new Typeface("Consolas"), 10, new SolidColorBrush(Color.FromArgb(255, 255, 255, 0))), handPosition);
            switch (handState)
            {
                case HandState.Closed:
                    drawingContext.DrawEllipse(this.handClosedBrush, null, handPosition, HandSize, HandSize);
                    break;

                case HandState.Open:
                    drawingContext.DrawEllipse(this.handOpenBrush, null, handPosition, HandSize, HandSize);
                    break;

                case HandState.Lasso:
                    drawingContext.DrawEllipse(this.handLassoBrush, null, handPosition, HandSize, HandSize);
                    break;
            }
        }

        /// <summary>
        /// Draws indicators to show which edges are clipping body data
        /// </summary>
        /// <param name="body">body to draw clipping information for</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawClippedEdges(Body body, DrawingContext drawingContext)
        {
            FrameEdges clippedEdges = body.ClippedEdges;

            if (clippedEdges.HasFlag(FrameEdges.Bottom))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, this.displayHeight - ClipBoundsThickness, this.displayWidth, ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Top))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, this.displayWidth, ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Left))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, ClipBoundsThickness, this.displayHeight));
            }

            if (clippedEdges.HasFlag(FrameEdges.Right))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(this.displayWidth - ClipBoundsThickness, 0, ClipBoundsThickness, this.displayHeight));
            }
        }

        /// <summary>
        /// Handles the event which the sensor becomes unavailable (E.g. paused, closed, unplugged).
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Sensor_IsAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        {
            // on failure, set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.SensorNotAvailableStatusText;
        }
    }
}
