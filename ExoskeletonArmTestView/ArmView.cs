using System;
using System.Collections.Generic;
using System.Drawing;
using System.Data;
using System.Linq;
using System.Windows.Forms;
using System.Diagnostics;
using System.IO.Ports;
using System.IO;

namespace ExoskeletonArmTestView {

    public partial class ArmView : UserControl {
        object paintingLock = new object();

        const float scale = 20;

        Vector3 startingPos = new Vector3(-scale / 2, 0, 0);

        float[] startingAngles = { 0, 0, 0 };
        Axis[] startingAngleAxes = { Axis.Y, Axis.Z, Axis.X };

        Axis[] segmentAngleAxes = new Axis[] { Axis.Y, Axis.X, Axis.X, Axis.Y, Axis.Y };
        public float[] segmentAngles = new float[] { 90.0f, 0.0f, 0.0f, 45.0f, 135.0f };
        Vector3[] segments = new Vector3[] { new Vector3(0, 0, 10), new Vector3(0, -5, 0), new Vector3(4, 0, 0), new Vector3(12, 0, 0), new Vector3(12, 0, 0) };

        bool mouseIsDown = false;
        private NumericUpDown angle0UpDown;
        private NumericUpDown angle1UpDown;
        private NumericUpDown angle3UpDown;
        private NumericUpDown angle4UpDown;
        Point lastMouseP;

        SerialPort serial = new SerialPort();
        int lowerLeft = 0;
        int lowerRight = 0;
        int lowerTop = 0;
        private TextBox targetXyzText;
        int lowerBottom = 0;

        bool receiveFromArduinoOnly = false;
        
        Vector3 targetXyz;

        Timer connectionTimer = new Timer();
        private CheckBox targetXyzEnableCheck;
        Timer motionTimer = new Timer();

        public ArmView() {
            InitializeComponent();

            angle0UpDown.Value = (decimal)segmentAngles[0];
            angle1UpDown.Value = (decimal)segmentAngles[1];
            angle3UpDown.Value = (decimal)segmentAngles[3];
            angle4UpDown.Value = (decimal)segmentAngles[4];

            SetStyle(
                ControlStyles.UserPaint |
                ControlStyles.AllPaintingInWmPaint |
                ControlStyles.OptimizedDoubleBuffer, true);
        }

        private void updateUI() {
            // Make UI modification thread-safe
            if (InvokeRequired) {
                Invoke((MethodInvoker)updateUI);
                return;
            }
            angle0UpDown.Value = (decimal)segmentAngles[0];
            angle1UpDown.Value = (decimal)segmentAngles[1];
            angle3UpDown.Value = (decimal)segmentAngles[3];
            angle4UpDown.Value = (decimal)segmentAngles[4];

            //PathPlanner.SolveForNextAnglesInPath(new Vector3(90, 135, 135), new Vector3(0, 10, 0), 1f);

            Invalidate();
        }

        void showForceData(string line) {
            if (!line.Contains("Top Force:")) {
                return;
            }
            string afterTop = line.Split(new string[] { "Top Force:" }, StringSplitOptions.None)[1];
            lowerTop = int.Parse(afterTop.Split(' ')[1]);

            string afterBottom = line.Split(new string[] { "Bottom Force:" }, StringSplitOptions.None)[1];
            lowerBottom = int.Parse(afterBottom.Split(' ')[1]);
        }

        void mirrorPotData(string line) {
            if (!line.Contains("Pots:")) {
                return;
            }

            string afterPotsString = line.Split(new string[] { "Pots:" }, StringSplitOptions.None)[1];
            string[] values = afterPotsString.Split(new char[] { ' ' }, StringSplitOptions.RemoveEmptyEntries);

            float[] maximums = new float[] { (float)angle0UpDown.Maximum, (float)angle1UpDown.Maximum, 0f,
                                             (float)angle3UpDown.Maximum, (float)angle4UpDown.Maximum };

            float[] minimums = new float[] { (float)angle0UpDown.Minimum, (float)angle1UpDown.Minimum, 0f,
                                             (float)angle3UpDown.Minimum, (float)angle4UpDown.Minimum };

            for (int i = 0; i < 2; i++) {
                float potAngle = float.Parse(values[i]);

                int segmentI = new int[] { 4, 3, 1, 0 }[i];
                segmentAngles[segmentI] = minimums[segmentI] + potAngle * (maximums[segmentI] - minimums[segmentI]);
            }
        }

        void remoteControlData(string line) {
            if (!line.StartsWith("V")) {
                return;
            }

            string[] parts = line.Substring(1).Split(' ');
            float[] vals = new float[3];

            for (int i = 0; i < 3; i++) {
                if (!float.TryParse(parts[i], out vals[i])) {
                    return;
                }
            }

            float[] maximums = new float[] { (float)angle0UpDown.Maximum,
                                             (float)angle3UpDown.Maximum,
                                             (float)angle4UpDown.Maximum };

            float[] minimums = new float[] { (float)angle0UpDown.Minimum,
                                             (float)angle3UpDown.Minimum,
                                             (float)angle4UpDown.Minimum };

            for (int i = 0; i < 3; i++) {
                int segmentI = new int[] { 0, 3, 4 }[i];
                segmentAngles[segmentI] = minimums[i] + vals[i] * (maximums[i] - minimums[i]);
            }
        }

        private void handleArduinoData(object sender, SerialDataReceivedEventArgs e) {
            string line = "";
            try {
                line = serial.ReadLine();
            } catch (IOException) {
                return;
            } catch (InvalidOperationException) {
                return;
            }

            mirrorPotData(line);
            showForceData(line);
            remoteControlData(line);

            segmentAngles[0] = constrain(segmentAngles[0], (float)angle0UpDown.Minimum, (float)angle0UpDown.Maximum);
            segmentAngles[1] = constrain(segmentAngles[1], (float)angle1UpDown.Minimum, (float)angle1UpDown.Maximum);
            segmentAngles[3] = constrain(segmentAngles[3], (float)angle3UpDown.Minimum, (float)angle3UpDown.Maximum);
            segmentAngles[4] = constrain(segmentAngles[4], (float)angle4UpDown.Minimum, (float)angle4UpDown.Maximum);

            updateUI();
        }

        private void attemptToConnectToArduino(object o, EventArgs e) {
            if (serial.IsOpen) {
                return;
            }

            serial.BaudRate = 9600;
            serial.Parity = Parity.None;
            serial.DataBits = 8;
            serial.StopBits = StopBits.One;
            serial.Handshake = Handshake.None;
            serial.DataReceived += handleArduinoData;

            string[] availablePorts = SerialPort.GetPortNames();
            foreach (string port in availablePorts) {
                if (port.Contains("COM")) {
                    try {
                        serial.PortName = port;
                        serial.Open();
                        if (serial.IsOpen) {
                            serial.DiscardInBuffer();
                            updateUI();
                            break;
                        }
                    } catch (IOException) {
                        // COM Port no longer available
                        updateUI();
                    } catch (UnauthorizedAccessException) {
                        updateUI();
                        try {
                            serial.Close();
                        } catch (IOException) {
                        }
                    }
                }
            }
        }

        private float constrain(float v, float min, float max) {
            if (v < min) {
                return min;
            } else if (v > max) {
                return max;
            }
            return v;
        }

        private void updateArmFromForce() {
            int threshold = 40;
            float slope = 0.0025f;

            int lowerTopBottomDiff = lowerTop - lowerBottom;
            if (lowerTopBottomDiff > threshold) {
                segmentAngles[4] -= slope * (lowerTopBottomDiff - threshold);
            } else if (lowerTopBottomDiff < -threshold) {
                segmentAngles[4] -= slope * (lowerTopBottomDiff + threshold);
            }
        }

        private void updateArmFromXyz() {
            Vector3 currentAngles = new Vector3(segmentAngles[0], segmentAngles[3], segmentAngles[4]);
            Vector3 nextAngles = PathPlanner.SolveForNextAnglesInPath(currentAngles, targetXyz, 1.0f);
            Vector3 nextAngles0to1 = nextAngles;

            float[] maximums = new float[] { (float)angle0UpDown.Maximum,
                                             (float)angle3UpDown.Maximum,
                                             (float)angle4UpDown.Maximum };

            float[] minimums = new float[] { (float)angle0UpDown.Minimum,
                                             (float)angle3UpDown.Minimum,
                                             (float)angle4UpDown.Minimum };

            for (int i = 0; i < 3; i++) {
                nextAngles0to1[i] = (nextAngles0to1[i] - minimums[i]) / (maximums[i] - minimums[i]);
            }
            if (serial.IsOpen) {
                try {
                    serial.Write("V" + nextAngles0to1[0] + " 0 " + nextAngles0to1[1] + ' ' + nextAngles0to1[2] + '\n');
                } catch (IOException) {
                    //
                } catch (InvalidOperationException) {
                    //
                }
            } else {
                Vector3 newAngles = currentAngles.Lerp(nextAngles, 0.2f);
                segmentAngles[0] = newAngles[0];
                segmentAngles[3] = newAngles[1];
                segmentAngles[4] = newAngles[2];
            }
        }

        private void updateArmMotion(object o, EventArgs e) {
            if (receiveFromArduinoOnly) {
                updateArmFromForce();
            } else if (targetXyzEnableCheck.Checked) {
                updateArmFromXyz();
            }

            segmentAngles[0] = constrain(segmentAngles[0], (float)angle0UpDown.Minimum, (float)angle0UpDown.Maximum);
            segmentAngles[1] = constrain(segmentAngles[1], (float)angle1UpDown.Minimum, (float)angle1UpDown.Maximum);
            segmentAngles[3] = constrain(segmentAngles[3], (float)angle3UpDown.Minimum, (float)angle3UpDown.Maximum);
            segmentAngles[4] = constrain(segmentAngles[4], (float)angle4UpDown.Minimum, (float)angle4UpDown.Maximum);

            updateUI();
        }

        Vector3 addVec(Vector3 a, Vector3 b) {
            return new Vector3(a.x + b.x, a.y + b.y, a.z + b.z);
        }

        PointF project3D(Vector3 a) {
            return new PointF(a.x, a.z);
        }

        protected override void OnPaint(PaintEventArgs pe) {
            // Safe with updating ui
            System.Threading.Monitor.Enter(paintingLock);

            Graphics graphics = pe.Graphics;
            Pen pen = new Pen(Color.White, 0.2f);

            graphics.Clear(Color.Black);

            graphics.ScaleTransform(Width / 2.0f / scale, -Height / 2.0f / scale);
            graphics.TranslateTransform(scale, -scale);

            //graphics.DrawLine(pen, new PointF(-scale, -scale), new PointF(scale, scale));

            float[] correctedSegmentAngles = new float[] {segmentAngles[0] - 90, segmentAngles[1], segmentAngles[2], 90 - segmentAngles[3], segmentAngles[4] - 180};

            Matrix3x3 runningRot = Matrix3x3.Rotate(startingAngles[0], startingAngleAxes[0]);
            Matrix3x3 userRot = Matrix3x3.Rotate(startingAngles[1] + correctedSegmentAngles[0], startingAngleAxes[1]);
            userRot = userRot.LeftMultiply(Matrix3x3.Rotate(startingAngles[2], startingAngleAxes[2]));

            Vector3 p1 = startingPos;
            for (int i = 0; i < segments.Length; i++) {
                if (i > 0) {
                    runningRot = runningRot.LeftMultiply(Matrix3x3.Rotate(correctedSegmentAngles[i], segmentAngleAxes[i]));
                }

                Matrix3x3 mat = runningRot;
                if (i > 2) {
                    mat = runningRot.LeftMultiply(Matrix3x3.Rotate(correctedSegmentAngles[1], segmentAngleAxes[1]));
                }
                mat = mat.LeftMultiply(userRot);

                Vector3 p2 = addVec(p1, mat.Multiply(segments[i]));
                graphics.DrawLine(pen, project3D(p1), project3D(p2));
                p1 = p2;
            }

            Font largeFont = new Font(FontFamily.GenericSerif, 18.0f);
            Font smallFont = new Font(FontFamily.GenericSerif, 15.0f);
            Brush b = Brushes.White;

            graphics.ResetTransform();

            graphics.DrawString("Lower", largeFont, b, Width - 210, 151);
            graphics.DrawString("Left: " + lowerLeft, smallFont, b, Width - 280, 189);
            graphics.DrawString("Right: " + lowerRight, smallFont, b, Width - 125, 189);
            graphics.DrawString("Top: " + lowerTop, smallFont, b, Width - 280, 250);
            graphics.DrawString("Bottom: " + lowerBottom, smallFont, b, Width - 125, 250);

            graphics.DrawString("Upper", largeFont, b, Width - 210, 327);
            graphics.DrawString("Left: ", smallFont, b, Width - 280, 365);
            graphics.DrawString("Right: ", smallFont, b, Width - 125, 365);
            graphics.DrawString("Top: ", smallFont, b, Width - 280, 426);
            graphics.DrawString("Bottom: ", smallFont, b, Width - 125, 426);

            Vector3 currentAngles = new Vector3(segmentAngles[0], segmentAngles[3], segmentAngles[4]);
            Vector3 xyz = PathPlanner.AnglesToXyz(currentAngles);
            graphics.DrawString("X:" + xyz.x.ToString("0.00") +
                                " Y: " + xyz.y.ToString("0.00") +
                                " Z: " + xyz.z.ToString("0.00"), smallFont, b, 20, Height - 24);

            graphics.DrawString(serial.IsOpen ? "Connected" : "Connecting to Arduino...", smallFont, b, Width - 260, 500);

            System.Threading.Monitor.Exit(paintingLock);
        }

        private void InitializeComponent() {
            this.angle0UpDown = new System.Windows.Forms.NumericUpDown();
            this.angle1UpDown = new System.Windows.Forms.NumericUpDown();
            this.angle3UpDown = new System.Windows.Forms.NumericUpDown();
            this.angle4UpDown = new System.Windows.Forms.NumericUpDown();
            this.targetXyzText = new System.Windows.Forms.TextBox();
            this.targetXyzEnableCheck = new System.Windows.Forms.CheckBox();
            ((System.ComponentModel.ISupportInitialize)(this.angle0UpDown)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.angle1UpDown)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.angle3UpDown)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.angle4UpDown)).BeginInit();
            this.SuspendLayout();
            // 
            // angle0UpDown
            // 
            this.angle0UpDown.Location = new System.Drawing.Point(461, 3);
            this.angle0UpDown.Maximum = new decimal(new int[] {
            170,
            0,
            0,
            0});
            this.angle0UpDown.Minimum = new decimal(new int[] {
            10,
            0,
            0,
            0});
            this.angle0UpDown.Name = "angle0UpDown";
            this.angle0UpDown.Size = new System.Drawing.Size(120, 20);
            this.angle0UpDown.TabIndex = 0;
            this.angle0UpDown.Value = new decimal(new int[] {
            10,
            0,
            0,
            0});
            this.angle0UpDown.ValueChanged += new System.EventHandler(this.angle0UpDown_ValueChanged);
            // 
            // angle1UpDown
            // 
            this.angle1UpDown.Location = new System.Drawing.Point(461, 29);
            this.angle1UpDown.Maximum = new decimal(new int[] {
            45,
            0,
            0,
            0});
            this.angle1UpDown.Minimum = new decimal(new int[] {
            45,
            0,
            0,
            -2147483648});
            this.angle1UpDown.Name = "angle1UpDown";
            this.angle1UpDown.Size = new System.Drawing.Size(120, 20);
            this.angle1UpDown.TabIndex = 1;
            this.angle1UpDown.ValueChanged += new System.EventHandler(this.angle1UpDown_ValueChanged);
            // 
            // angle3UpDown
            // 
            this.angle3UpDown.Location = new System.Drawing.Point(461, 81);
            this.angle3UpDown.Maximum = new decimal(new int[] {
            170,
            0,
            0,
            0});
            this.angle3UpDown.Minimum = new decimal(new int[] {
            10,
            0,
            0,
            -2147483648});
            this.angle3UpDown.Name = "angle3UpDown";
            this.angle3UpDown.Size = new System.Drawing.Size(120, 20);
            this.angle3UpDown.TabIndex = 3;
            this.angle3UpDown.Value = new decimal(new int[] {
            10,
            0,
            0,
            0});
            this.angle3UpDown.ValueChanged += new System.EventHandler(this.angle3UpDown_ValueChanged);
            // 
            // angle4UpDown
            // 
            this.angle4UpDown.Location = new System.Drawing.Point(461, 107);
            this.angle4UpDown.Maximum = new decimal(new int[] {
            1675,
            0,
            0,
            65536});
            this.angle4UpDown.Minimum = new decimal(new int[] {
            80,
            0,
            0,
            0});
            this.angle4UpDown.Name = "angle4UpDown";
            this.angle4UpDown.Size = new System.Drawing.Size(120, 20);
            this.angle4UpDown.TabIndex = 4;
            this.angle4UpDown.Value = new decimal(new int[] {
            80,
            0,
            0,
            0});
            this.angle4UpDown.ValueChanged += new System.EventHandler(this.angle4UpDown_ValueChanged);
            // 
            // targetXyzText
            // 
            this.targetXyzText.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.targetXyzText.Location = new System.Drawing.Point(3, 500);
            this.targetXyzText.Name = "targetXyzText";
            this.targetXyzText.Size = new System.Drawing.Size(182, 26);
            this.targetXyzText.TabIndex = 5;
            this.targetXyzText.KeyPress += new System.Windows.Forms.KeyPressEventHandler(this.targetXyzText_KeyPress);
            // 
            // targetXyzEnableCheck
            // 
            this.targetXyzEnableCheck.AutoSize = true;
            this.targetXyzEnableCheck.Font = new System.Drawing.Font("Microsoft Sans Serif", 12F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.targetXyzEnableCheck.ForeColor = System.Drawing.Color.White;
            this.targetXyzEnableCheck.Location = new System.Drawing.Point(192, 500);
            this.targetXyzEnableCheck.Name = "targetXyzEnableCheck";
            this.targetXyzEnableCheck.Size = new System.Drawing.Size(87, 24);
            this.targetXyzEnableCheck.TabIndex = 6;
            this.targetXyzEnableCheck.Text = "Enabled";
            this.targetXyzEnableCheck.UseVisualStyleBackColor = true;
            // 
            // ArmView
            // 
            this.BackColor = System.Drawing.Color.Black;
            this.Controls.Add(this.targetXyzEnableCheck);
            this.Controls.Add(this.targetXyzText);
            this.Controls.Add(this.angle4UpDown);
            this.Controls.Add(this.angle3UpDown);
            this.Controls.Add(this.angle1UpDown);
            this.Controls.Add(this.angle0UpDown);
            this.Name = "ArmView";
            this.Size = new System.Drawing.Size(584, 561);
            this.Load += new System.EventHandler(this.ArmView_Load);
            this.MouseDown += new System.Windows.Forms.MouseEventHandler(this.ArmView_MouseDown);
            this.MouseMove += new System.Windows.Forms.MouseEventHandler(this.ArmView_MouseMove);
            this.MouseUp += new System.Windows.Forms.MouseEventHandler(this.ArmView_MouseUp);
            this.Resize += new System.EventHandler(this.ArmView_Resize);
            ((System.ComponentModel.ISupportInitialize)(this.angle0UpDown)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.angle1UpDown)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.angle3UpDown)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.angle4UpDown)).EndInit();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        private void ArmView_Resize(object sender, EventArgs e) {
            updateUI();
        }

        private void ArmView_MouseDown(object sender, MouseEventArgs e) {
            mouseIsDown = true;
            lastMouseP = e.Location;
        }

        private void ArmView_MouseMove(object sender, MouseEventArgs e) {
            if (mouseIsDown) {
                startingAngles[1] += (e.X - lastMouseP.X) * 0.2f;
                startingAngles[2] += (e.Y - lastMouseP.Y) * 0.2f;
                lastMouseP = e.Location;
                updateUI();
            }
        }

        private void ArmView_MouseUp(object sender, MouseEventArgs e) {
            mouseIsDown = false;
        }

        private void angle0UpDown_ValueChanged(object sender, EventArgs e) {
            segmentAngles[0] = (float)angle0UpDown.Value;
            updateUI();
        }

        private void angle1UpDown_ValueChanged(object sender, EventArgs e) {
            segmentAngles[1] = (float)angle1UpDown.Value;
            updateUI();
        }

        private void angle3UpDown_ValueChanged(object sender, EventArgs e) {
            segmentAngles[3] = (float)angle3UpDown.Value;
            updateUI();
        }

        private void angle4UpDown_ValueChanged(object sender, EventArgs e) {
            segmentAngles[4] = (float)angle4UpDown.Value;
            updateUI();
        }

        private void ArmView_Load(object sender, EventArgs e) {
            connectionTimer.Tick += attemptToConnectToArduino;
            connectionTimer.Interval = 1000;
            connectionTimer.Start();

            motionTimer.Tick += updateArmMotion;
            motionTimer.Interval = 50;
            motionTimer.Start();
        }

        private void targetXyzText_KeyPress(object sender, KeyPressEventArgs e) {
            if (e.KeyChar == '\r') {
                string[] parts = targetXyzText.Text.Split(new char[] { ' ' }, StringSplitOptions.RemoveEmptyEntries);
                if (parts.Length != 3) {
                    return;
                }
                float[] vals = new float[3];
                for (int i = 0; i < 3; i++) {
                    if (!float.TryParse(parts[i], out vals[i])) {
                        return;
                    }
                }

                targetXyz = new Vector3(vals[0], vals[1], vals[2]);
                targetXyzEnableCheck.Checked = true;
            }
        }
    }

    public struct Vector3 {
        public float x;
        public float y;
        public float z;

        public Vector3(float x, float y, float z) {
            this.x = x;
            this.y = y;
            this.z = z;
        }

        public float this[int i] {
            get {
                switch (i) {
                    case 0:
                        return x;
                    case 1:
                        return y;
                    case 2:
                        return z;
                    default:
                        throw new IndexOutOfRangeException();
                }
            }

            set {
                switch (i) {
                    case 0:
                        x = value;
                        break;
                    case 1:
                        y = value;
                        break;
                    case 2:
                        z = value;
                        break;
                    default:
                        throw new IndexOutOfRangeException();
                }
            }
        }

        public float Mag() {
            return (float)Math.Sqrt(x * x + y * y + z * z);
        }

        public Vector3 Add(Vector3 b) {
            return new Vector3(x + b.x, y + b.y, z + b.z);
        }

        public Vector3 Sub(Vector3 b) {
            return new Vector3(x - b.x, y - b.y, z - b.z);
        }

        public Vector3 Mul(float b) {
            return new Vector3(x * b, y * b, z * b);
        }

        public Vector3 Lerp(Vector3 b, float factor) {
            return b.Mul(factor).Add(Mul(1 - factor));
        }

        public override string ToString() {
            return "X: " + x + " Y: " + y + " Z: " + z;
        }
    }

    public class Matrix3x3 {
        public float[,] data = new float[3, 3];

        public static Matrix3x3 RotateX(float angle) {
            Matrix3x3 m = new Matrix3x3();

            float sinA = (float)Math.Sin(angle * Math.PI / 180.0f);
            float cosA = (float)Math.Cos(angle * Math.PI / 180.0f);
            m.data = new float[,] { { 1, 0, 0 }, { 0, cosA, -sinA }, { 0, sinA, cosA } };
            return m;
        }

        public static Matrix3x3 RotateY(float angle) {
            Matrix3x3 m = new Matrix3x3();

            float sinA = (float)Math.Sin(angle * Math.PI / 180.0f);
            float cosA = (float)Math.Cos(angle * Math.PI / 180.0f);
            m.data = new float[,] { { cosA, 0, sinA }, { 0, 1, 0 }, { -sinA, 0, cosA } };
            return m;
        }

        public static Matrix3x3 RotateZ(float angle) {
            Matrix3x3 m = new Matrix3x3();

            float sinA = (float)Math.Sin(angle * Math.PI / 180.0f);
            float cosA = (float)Math.Cos(angle * Math.PI / 180.0f);
            m.data = new float[,] { { cosA, -sinA, 0 }, { sinA, cosA, 0 }, { 0, 0, 1 } };
            return m;
        }

        public static Matrix3x3 Rotate(float angle, Axis axis) {
            switch (axis) {
                case Axis.X:
                    return RotateX(angle);
                case Axis.Y:
                    return RotateY(angle);
                case Axis.Z:
                    return RotateZ(angle);
                default:
                    throw new InvalidOperationException("Invalid axis: " + axis);
            }
        }

        public Matrix3x3 Multiply(Matrix3x3 b) {
            Matrix3x3 c = new Matrix3x3();

            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    float v = 0;
                    for (int k = 0; k < 3; k++) {
                        v += data[i, k] * b.data[k, j];
                    }
                    c.data[i, j] = v;
                }
            }

            return c;
        }

        public Matrix3x3 LeftMultiply(Matrix3x3 a) {
            return a.Multiply(this);
        }

        public Vector3 Multiply(Vector3 v) {
            Vector3 res;

            res.x = data[0, 0] * v.x + data[0, 1] * v.y + data[0, 2] * v.z;
            res.y = data[1, 0] * v.x + data[1, 1] * v.y + data[1, 2] * v.z;
            res.z = data[2, 0] * v.x + data[2, 1] * v.y + data[2, 2] * v.z;

            return res;
        }
    }

    public enum Axis {
        X,
        Y,
        Z
    }

}
